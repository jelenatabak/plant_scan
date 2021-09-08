#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/filters/filter.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
//#include <pcl_ros/transforms.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/registration/gicp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
//#include <tf2_eigen/tf2_eigen.h>
#include <vector>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#define PI 3.14159265
#define DZ 0.057142857142857
#define H 54323.6074271
#define T 0.01352163461
//#define DZ 0.05

// definiranje klase cloudModel u kojoj se odvija registracija
class cloudModel{
  protected:
    ros::NodeHandle nh;
    tf::TransformListener listener;
    tf::StampedTransform tf_first;
    tf::StampedTransform tf_second;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_first;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_second;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> cloud_archive;
    std::vector<tf::StampedTransform> tf_archive;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_model;
    pcl::PointCloud<pcl::PointXYZRGB> tf_cloud_first;
    double vert = 0.0;
    double angle = 0.0;

  public:
    ros::Subscriber pcl_sub = nh.subscribe("/pcl", 1000, &cloudModel::cloudCB, this);
    ros::Publisher full_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_cloud", 1);
//    int vert = 0;
//    int angle = 0;
    int flag_init = 0;
    int flag_model = 0;
    int v_top = 0.7;
    int flag_pub = 0;

// cloud callback funkcija - čita cloudove i poziva funkcije za registraciju
    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> tf_cloud;
        tf::StampedTransform transform;
// dobivanje vremena i clouda

        if (flag_init == 0)
        {
          ros::Time time_cloud = input.header.stamp;
          pcl::fromROSMsg (input, cloud);
//          boost::shared_ptr<dynamixel_workbench_msgs::DynamixelStateList const> sharedPtr;
//          dynamixel_workbench_msgs::DynamixelStateList state;

//          sharedPtr = ros::topic::waitForMessage<dynamixel_workbench_msgs::DynamixelStateList>("/m1");
//          state = *sharedPtr;

//          double angle = state.dynamixel_state[0].present_position*T*PI/180;
//          double z_move = 0.5-state.dynamixel_state[1].present_position/H;

          cout << "vert: " << vert+1 << endl;
          cout << "angle: " << angle*180/PI << endl;
          double x_move = 0.225*cos(angle);
          double y_move = 0.225*sin(angle);
          double z_move = 0.5-vert*DZ;

          cout << "XYZ: " << x_move << ", " << y_move << ", " << z_move << endl;

          tf::Quaternion spin_f;
          tf::Quaternion spin_c;
          spin_f.setRPY(0.0, 0.0, angle);
          spin_f.normalize();
          //VAŽNO! KONFIGURACIJA ROTACIJE KAMERE JE UVIJEK RPY: -0.5*PI, 0.0, -0.5*PI
          spin_c.setRPY(-0.5*PI, 0.0, 0.5*PI);
          spin_c.normalize();

          tf::Quaternion spin_total = spin_f*spin_c;
          spin_total.normalize();

          cout << "XYZW: " << spin_total.getX() << ", " << spin_total.getY() << ", " << spin_total.getZ() << ", " << spin_total.getW() << endl;

          transform.setOrigin(tf::Vector3(x_move, y_move, z_move));
          transform.setRotation(spin_total);

          pcl_ros::transformPointCloud(cloud, tf_cloud, transform);
          tf_cloud.header.frame_id="map";
//          testPublish(tf_cloud);
          cloud_archive.push_back(tf_cloud);
          cout << "Archive size: " << cloud_archive.size() << endl;

          vert += 1;

          if (vert == 8)
          {
            vert = 0.0;
            angle += PI/6;
          }
        }
        if (cloud_archive.size() == 64 && flag_model == 0)
        {
          flag_init = 1;
          flag_model = 1;
          cout << "Full archive" << endl;
        }

// ako smo spremni za konstrukciju (model) ali model nije gotov (pub), počinjemo konstrukciju
      if (flag_model == 1 && flag_pub == 0)
      {
        cloud_model=cloud_archive.back();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloud_model, cloud_model, indices);
        if(cloud_model.width == 0)
        {
          cloud_archive.erase(cloud_archive.begin());
          cloud_model=cloud_archive.front();
          cout << "Empty" << endl;
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(cloud_model, cloud_model, indices);
        }
        // voxel downsampling
        pcl::VoxelGrid<pcl::PointXYZRGB> voxgrid;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_model(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_model));
        voxgrid.setInputCloud(ptr_model);
        voxgrid.setLeafSize(0.01f, 0.01f, 0.01f);
        voxgrid.filter(cloud_model);

        for (int i = cloud_archive.size()-2; i >= cloud_archive.size()-47; i-=1)
        {
          cout << "Cloud No. " << cloud_archive.size()-i << endl;
          tf_cloud=cloud_archive[i];

          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(tf_cloud, tf_cloud, indices);

          // voxel downsampling
          pcl::VoxelGrid<pcl::PointXYZRGB> voxgrid;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_tf(new pcl::PointCloud<pcl::PointXYZRGB>(tf_cloud));
          voxgrid.setInputCloud(ptr_tf);
          voxgrid.setLeafSize(0.01f, 0.01f, 0.01f);
          voxgrid.filter(tf_cloud);

          testAlign(cloud_model, tf_cloud);
          //cloud_model += tf_cloud;
        }
        flag_pub = 1;
      }
      if (flag_pub == 1)
      {
// ako smo spremni za slanje gotovog modela (pub), pozivamo funkciju testPublish s argumentom cloud_model
        cout << "Publishing" << endl;
        testPublish(cloud_model);
      }
  }

// funkcija za ICP i konstrukciju modela
    void testAlign(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_1, const pcl::PointCloud<pcl::PointXYZRGB> &cloud_2)
    {
      pcl::PointCloud<pcl::PointXYZRGB> cloud_aligned;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_first(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_1));
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_second(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_2));

// postavljamo parametre ICP funkcije
      icp.setMaximumIterations(800);
      icp.setTransformationEpsilon(1e-8);
      icp.setEuclideanFitnessEpsilon(1);
      icp.setMaxCorrespondenceDistance(1e-4);

      cout << "TCW1: " << cloud_1.width << endl;
      cout << "TCW2: " << cloud_2.width << endl;

// ukoliko koristimo ICP (nije potrebno za simulaciju), provodimo registraciju drugog clouda prema prvom
// (od/zakomentiraj ovisno o tome koji se uzorak koristi)
      icp.setInputSource(ptr_second);
      //icp.setInputCloud(ptr_second);
      //cout << "Cloud 1 set" << endl;
      icp.setInputTarget(ptr_first);
      //cout << "Cloud 2 set" << endl;
      icp.align(cloud_aligned);
      //cloud_aligned = cloud_2;

// novi cloud se dodaje modelu
      cloud_model += cloud_aligned;

      //cloud_model = cloud_1;

      cout << "CMW: "<< cloud_model.width << endl;
      //cout << "Model received" << endl;
//      testPublish(cloud_model);
    }

// funkcija slanja gotovog modela - model se pretvara u format PC2 i šalje na topic
    void testPublish(const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
    {
      // voxel downsampling
//      pcl::VoxelGrid<pcl::PointXYZRGB> voxgrid;
//      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_model(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_model));
//      voxgrid.setInputCloud(ptr_model);
//      voxgrid.setLeafSize(0.02f, 0.02f, 0.02f);
//      voxgrid.filter(cloud_model);
//      cout << "Filtered; CW: " << cloud_model.width << endl;

      sensor_msgs::PointCloud2 pub;
      pcl::toROSMsg(cloud, pub);
      full_pub.publish(pub);
    }
};

// glavni program
int main(int argc, char** argv)
{
    ros::init(argc, argv, "all_frames_sim");
    cloudModel cm;
    ros::spin();
    return 0;
}
