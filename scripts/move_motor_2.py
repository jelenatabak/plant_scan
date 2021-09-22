#!/usr/bin/env python3

import sys
import rospy
import rosbag
import time
import tf_conversions
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import Bool
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList


class Trans():
    def circ(self, old_theta, new_theta):  #računanje koordinata za novi kut
        #razlika kuta, sinus/kosinus razlike
        delta_theta=new_theta-old_theta
        delta_sine=math.sin(delta_theta)
        delta_cosine=math.cos(delta_theta)

        #bilježenje starih koordinata
        self.old_y=self.new_y
        self.old_x=self.new_x

        self.new_x=self.old_x*delta_cosine-self.old_y*delta_sine                       #računanje novih koordinata prema pravilu sinusa/kosinusa zbroja kuteva - ne znamo radijus
        self.new_y=self.old_x*delta_sine+self.old_y*delta_cosine
        self.new_q=tf_conversions.transformations.quaternion_from_euler(0,0,new_theta) #računanje kvaterniona izravno iz novog kuta

        #računanje transformacija x/y/q
        self.delta_x=self.new_x-self.old_x
        self.delta_y=self.new_y-self.old_y
        self.delta_q=self.new_q-self.old_q


    def vert(self, old_height, new_height):             #računanje koordinata za novu visinu
        self.old_z=self.max_z-old_height*self.delta_h               #bilježenje stare vrijednosti
        self.new_z=self.max_z-new_height*self.delta_h               #postavljanje nove vrijednosti
        self.delta_z=self.new_z-self.old_z  #računanje transformacije visine


    def __init__(self, height, max_height, min_height, delta_height, radius, theta): #postavljanje objekta
        #računanje početnih vrijednosti za x/y - planirano i za matricu rotacije, program ne prihvaća; preskočeno
        sine = math.sin(theta)
        cosine = math.cos(theta)

#        #računanje realnog kuta preko radijana
#        self.max_a=math.pi*2
#        self.min_a=0
#        self.delta_a=(self.max_a-self.min_a)/(num_theta-1)

        #računanje realne visine - max_h koristimo 50 cm, min_h koristimo 10 cm
        #napomena: robot spušta kameru od pozicije na vrhu okvira, treba obratno
        self.max_z=max_height
        self.min_z=min_height
        self.delta_h=delta_height

        self.old_x=radius*cosine
        self.old_y=radius*sine
        self.old_z=height
        self.old_q=tf_conversions.transformations.quaternion_about_axis(theta,(0,0,1))

        self.new_x=radius*cosine
        self.new_y=radius*sine
        self.new_z=height
        self.new_q=tf_conversions.transformations.quaternion_about_axis(theta,(0,0,1))


    def cam(self): #slanje novog stanja na primatelja transformacije
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp=rospy.Time.now()
        t.header.frame_id="world"
        t.child_frame_id="camera"

        t.transform.translation.x = self.new_x/100
        t.transform.translation.y = self.new_y/100
        t.transform.translation.z = self.new_z/100

        t.transform.rotation.x = self.new_q[0]
        t.transform.rotation.y = self.new_q[1]
        t.transform.rotation.z = self.new_q[2]
        t.transform.rotation.w = self.new_q[3]

        br.sendTransform(t)
        #tf_bag.write("tf", t)
        #print("Recorded transform")


class Move_motors():

    def pose_callback(self, data):
        if self.id == 2 :
            self.actual_position = data.dynamixel_state[0].present_position
        elif self.id == 3 :
            self.actual_position = data.dynamixel_state[1].present_position
        self.offset = abs(self.actual_position - self.goal_position)


    def __init__(self, command, id, addr_name, value):
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        self.command = command
        self.id = id
        self.addr_name = addr_name
        self.value = value
        self.goal_position = 0
        self.offset = 0

        self.actual_position = DynamixelStateList()
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, self.pose_callback)

        try:
            self.move_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            motor = self.move_motor(command, id, addr_name, value)
        except rospy.ServiceException as e:
            print("There was a problem with initializing motors: %s" %e)


    def move(self, value):
        self.goal_position = value
        self.move_motor(self.command, self.id, self.addr_name, value)
        while self.offset < 1:
            print("Offset")
            time.sleep(0.001)
        time.sleep(1)
        print(self.actual_position, self.goal_position)

#    def record(self):
#        if self.id == 2 :
#            tf_bag.write("mtr_2", self)
#        elif self.id == 3 :
#            tf_bag.write("mtr_3", self)
#        print('Recorded current positions')

    def calc_h(self, min_h, max_h, num_of_h):  #max_h u cm, num_of_h broj visina za slikanje
        return int((((max_h-min_h)/7.54)*4096)/(num_of_h-1))

    def calc_a(self, num_of_photo):    #broj slika po rotaciji
        return int((math.pi*2*4096)/num_of_photo)

#def cam(d,r,theta,alpha):
#    br = tf2_ros.TransformBroadcaster()
#    t = geometry_msgs.msg.TransformStamped()
#    t.header.stamp=rospy.Time.now
#    t.header.frame_id="world"
#    t.child_frame_id="camera"
#
#    s=math.sin(theta)
#    c=math.cos(theta)
#
#    t.transform.translation.x=r*c
#    t.transform.translation.y=r*s
#    t.transform.translation.z=d
#
##    rot=rotation_matrix(theta,(0, 0, 1))
#    q=tf_conversions.transformations.quaternion_from_euler(0,0,theta)
#    t.transform.rotation.x = q[0]
#    t.transform.rotation.y = q[1]
#    t.transform.rotation.z = q[2]
#    t.transform.rotation.w = q[3]
#
#    br.sendTransform(t)
tf_bag = rosbag.Bag('motordata.bag', 'w')


if __name__ == '__main__':
    try:
        rospy.init_node('motor_position')
        m1 = Move_motors('command', 3, 'Goal_Position', 0)
        m2 = Move_motors('command', 2, 'Goal_Position', 0)
        pub = rospy.Publisher('/ready', Bool, queue_size = 1)
        num_of_h = 7        # broj visina na kojima ce se slikati
        num_of_photo = 12    # broj slika po jednoj rotaciji
        top_h = 60          # visina u cm do koje će ići kamera, max je 70cm
        max_h = 60
        min_h = 36
        delta_h=(max_h-min_h)/(num_of_h-1)
        k=int((max_h-min_h)/delta_h)
        print('Visine:')
        for i in range (k+1):
            j=i*delta_h+min_h;
            print(str(j))
        print('Kutevi:')
        for i in range(num_of_photo):
            j=(360*i)/num_of_photo
            print(str(j))
        #input('Pritisni ENTER za početak...')
        tag = 0
        time.sleep(2)
        elev = 0
        rad = 32	     # radijus okvira skenera
        height_old = top_h
        value_old = 0
        cam_pose=Trans(height_old,max_h,min_h,delta_h,rad,value_old)
        cam_pose.cam()
        print('Počinjem')
        for angle in range (num_of_photo):
            value = m2.calc_a(num_of_photo) * (angle)
            cam_pose.circ(value_old/4096, value/4096)            #računanje novog x/y
            m2.move(value)
            cam_pose.cam()
            input("Angle changed; press Enter to continue")
            time.sleep(2)
            for height in range (num_of_h):
                print('Visina: ' + str(height))
                if tag == 0:
                    elev = m1.calc_h(min_h, max_h, num_of_h) * (height)
                else:
                    elev = m1.calc_h(min_h, max_h, num_of_h) * (num_of_h-height-1)
                cam_pose.vert(height_old, height)      #računanje nove visine
                m1.move(elev)
                cam_pose.cam()                         #slanje transformacije na primatelja
#                input("Height changed; press Enter to continue")
                if height == 0:
                    print('Returning to top')
                    time.sleep(5)
                time.sleep(3)
                pub.publish(True)
                #m1.record()
                #m2.record()
                print('Slikano ' + str(angle))
                time.sleep(2)
                height_old=height
            value_old=value
        if tag == 0:
            tag = 1
        else:
            tag = 0
        input("Scan over; press Enter to continue")
        m1.move(0)
        m2.move(0)
        cam_pose.circ(value_old/4096, 0)
        cam_pose.vert(height_old, 0)
        cam_pose.cam()
#        tf_bag.close()
    except rospy.ROSInterruptException:
        pass
