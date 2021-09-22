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
#include <pcl/filters/crop_box.h>
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
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> cloud_archive;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_model;

  public:
    ros::Subscriber pcl_sub = nh.subscribe("pcl", 1000, &cloudModel::cloudCB, this);
    ros::Publisher full_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_cloud", 1);
    int flag_model = 0;

// cloud callback funkcija - čita cloudove i poziva funkcije za registraciju
    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> tf_cloud;
        tf::StampedTransform transform;
// dobivanje vremena i clouda

        if (flag_model == 0)
        {
//          ros::Time time_cloud = input.header.stamp;
          pcl::fromROSMsg (input, cloud);

          boost::shared_ptr<dynamixel_workbench_msgs::DynamixelStateList const> sharedPtr;
          dynamixel_workbench_msgs::DynamixelStateList state;

          sharedPtr = ros::topic::waitForMessage<dynamixel_workbench_msgs::DynamixelStateList>("motor");
          state = *sharedPtr;

          double angle = state.dynamixel_state[0].present_position*T*PI/180;
          double z_move = 0.6-state.dynamixel_state[1].present_position/H;

          cout << "vert: " << z_move << endl;
          cout << "angle: " << angle*180/PI << endl;
          double x_move = 0.215*cos(angle)+0.023*sin(angle);
          double y_move = 0.215*sin(angle)-0.023*cos(angle);

          cout << "XYZ: " << x_move << ", " << y_move << ", " << z_move << endl;

          tf::Quaternion spin_f;
          tf::Quaternion spin_c;
          tf::Quaternion spin_total;
          spin_f.setRPY(0.0, 0.0, angle);
          spin_f.normalize();
          //VAŽNO! KONFIGURACIJA ROTACIJE KAMERE JE UVIJEK RPY: -0.5*PI, 0.0, 0.5*PI
          spin_c.setRPY(-0.5*PI, 0.0, 0.5*PI);
          spin_c.normalize();

          spin_total = spin_f*spin_c;
          spin_total.normalize();

          transform.setOrigin(tf::Vector3(x_move, y_move, z_move));
          transform.setRotation(spin_total);

          pcl_ros::transformPointCloud(cloud, tf_cloud, transform);
          tf_cloud.header.frame_id="map";

          pcl::PointCloud<pcl::PointXYZRGB> cloud_cropping;
          pcl::PointCloud<pcl::PointXYZRGB> cloud_publishing;
          cloud_cropping = tf_cloud;

          cout << "Tf CW: " << tf_cloud.width << endl;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_1(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_cropping));
          pcl::CropBox<pcl::PointXYZRGB> boxfilt;
          boxfilt.setMin(Eigen::Vector4f(-0.2,-0.2,0,1.0));
          boxfilt.setMax(Eigen::Vector4f(0.2,0.2,1,1.0));
          boxfilt.setInputCloud(ptr_1);
          boxfilt.filter(cloud_publishing);
          cout << "Cropped; CW: " << cloud_publishing.width << endl;

          testPublish(cloud_publishing);
          if (cloud_publishing.width > 0)
          {
              cloud_archive.push_back(cloud_publishing);
          }
          cout << "Archive size: " << cloud_archive.size() << endl;

          if (angle*180/PI > 300 && z_move < 0.37)
          {
            cout << "Modeling" << endl;
            flag_model = 1;
          }
        }


// ako smo spremni za konstrukciju (model) ali model nije gotov (pub), počinjemo konstrukciju
      if (flag_model == 1)
      {
        cloud_model=cloud_archive.front();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloud_model, cloud_model, indices);

        for (int i = 1; i < cloud_archive.size(); i+=1)
        {
          cout << "Cloud No. " << i+1 << endl;
          tf_cloud=cloud_archive[i];

          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(tf_cloud, tf_cloud, indices);

          testAlign(cloud_model, tf_cloud);
          cout << "Publishing" << endl;
          testPublish(cloud_model);
          //cloud_model += tf_cloud;
        }
      }
  }

// funkcija za ICP i konstrukciju modela
    void testAlign(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_1, const pcl::PointCloud<pcl::PointXYZRGB> &cloud_2)
    {
      pcl::PointCloud<pcl::PointXYZRGB> cloud_aligned;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_first(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_1));
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_second(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_2));

// postavljamo parametre ICP funkcije
      icp.setMaximumIterations(1000);
      icp.setTransformationEpsilon(1e-8);
      icp.setEuclideanFitnessEpsilon(1);
      icp.setMaxCorrespondenceDistance(1e-5);

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
//      cloud_aligned = cloud_2;

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
      pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_model(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
      pcl::VoxelGrid<pcl::PointXYZRGB> voxgrid;
      voxgrid.setInputCloud(ptr_model);
      voxgrid.setLeafSize(0.007f, 0.007f, 0.007f);
      voxgrid.filter(cloud_out);
      cout << "Filtered; CW: " << cloud_out.width << endl;

      sensor_msgs::PointCloud2 pub;
      pcl::toROSMsg(cloud_out, pub);
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
