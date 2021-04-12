#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError


bridge=CvBridge()
counter = 0


def save_image(img):
    global counter
    file='/home/andro/catkin_ws/src/plant_scan/images/img'+str(counter)+'.jpeg'
    cv2.imwrite(file, img)
    counter+=1


def ready_callback(msg):
    ready_msg = msg.data
    if ready_msg == True:
        rospy.loginfo('Spremam sliku...')
        try:
            img_msg=rospy.wait_for_message("/camera/color/image_raw", Image)
        except rospy.ROSInterruptException:
            pass
    
    
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
            b,g,r = cv2.split(cv_image)
            rgb_cv_image = cv2.merge([r,g,b])
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        
        save_image(rgb_cv_image)
        
        
    
ready=rospy.Subscriber('/ready',Bool, ready_callback)

if __name__ == '__main__':
    rospy.init_node('opencv_proba',anonymous=True)
    while not rospy.is_shutdown():
        rospy.spin()
