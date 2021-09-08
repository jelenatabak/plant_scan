#!/usr/bin/env python3

import sys
import rospy
import rosbag
from dynamixel_workbench_msgs.msg import DynamixelStateList
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool
import geometry_msgs.msg
import tf
#import tf2_ros
import time


def ready_callback(msg):
#    ready_msg = msg.data
#    if ready_msg == True:
        try:
            #pcl_msg=rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            #img_msg=rospy.wait_for_message("/camera/color/image_raw", Image)
#            mtr_msg=rospy.wait_for_message("/dynamixel_workbench/dynamixel_state", DynamixelStateList)
            mtr_msg=rospy.wait_for_message("/m1", DynamixelStateList)
            m1=mtr_msg.dynamixel_state[0].present_position
            m2=mtr_msg.dynamixel_state[1].present_position


#            tf_msg = tfbuffer.lookup_transform('camera','world',rospy.Time())
            #pcl_msg.header.stamp = rospy.Time.now()
            #img_msg.header.stamp = rospy.Time.now()
            #pcl_bag.write("pcl", pcl_msg)
            #img_bag.write("image", img_msg)
            mtr_bag.write("m1", m1)
            mtr_bag.write("m2", m2)
#            tf_bag.write("trans", tf_msg)
            rospy.loginfo("Spremam podatke")
        except (rospy.ROSInterruptException, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        #try:
        #    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        #    b,g,r = cv2.split(cv_image)
        #    rgb_cv_image = cv2.merge([r,g,b])
        #except CvBridgeError as e:
        #    rospy.logerr("CvBridge Error: {0}".format(e))

#tfbuffer = tf2_ros.Buffer()
#tfready = tf2_ros.TransformListener(tfbuffer)
#img_bag = rosbag.Bag('imgdata.bag', 'w')
mtr_bag = rosbag.Bag('mtrdata.bag', 'w')
#pcl_bag = rosbag.Bag('pcldata.bag', 'w')
#tf_bag = rosbag.Bag('tfdata.bag', 'w')
ready =  rospy.Subscriber('/ready', Bool, ready_callback)

if __name__ == '__main__':
    rospy.init_node('baggins',anonymous=True)
    while not rospy.is_shutdown():
        rospy.spin()
    if rospy.is_shutdown():
#        img_bag.close()
#        pcl_bag.close()
        mtr_bag.close()
