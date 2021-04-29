#!/usr/bin/env python3

import sys
import rospy
import time
#potrebni novi importi za računanje transformacija
import tf_conversions
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import math
#kraj novih importa
from std_msgs.msg import Bool
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList

#klasa koordinata kamere
class Trans():
    def circ(self, old_theta, new_theta):  #računanje koordinata za novi kut
        #razlika kuta, sinus/kosinus razlike
        delta_theta=new_theta-old_theta
        delta_sine=math.sin(delta_theta)
        delta_cosine=math.cos(delta_theta)

        #bilježenje starih koordinata
        self.old_y=self.new_y
        self.old_x=self.new_x

        #računanje novih koordinata - budući da je radijus nepoznat, koristi se pravilo ko/sinusa zbroja kuteva
        self.new_x=self.old_x*delta_cosine-self.old_y*delta_sine
        self.new_y=self.old_x*delta_sine+self.old_y*delta_cosine
        self.new_q=tf_conversions.transformations.quaternion_from_euler(0,0,new_theta) #računanje kvaterniona izravno iz novog kuta

        #računanje transformacija x/y/q
        self.delta_x=self.new_x-self.old_x
        self.delta_y=self.new_y-self.old_y
        self.delta_q=self.new_q-self.old_q

    #računanje koordinata nove visine - Oprez! Kamera se spušta; potrebna prilagodba u slučaju dizanja
    def vert(self, old_height, new_height):
        self.old_z=self.max_z-old_height*self.delta_h           #bilježenje stare vrijednosti
        self.new_z=self.max_z-new_height*self.delta_h           #postavljanje nove vrijednosti
        self.delta_z=self.new_z-self.old_z                      #računanje transformacije visine

    #definiranje objekta transformacije - potrebna početna visina, najviša točka, najniža točka, srednja udaljenost između vertikalnih točaka, radijus i početni kut
    def __init__(self, height, max_height, min_height, delta_height, radius, theta):
        #računanje početnih vrijednosti za x/y - planirano i za matricu rotacije, program ne prihvaća; preskočeno
        sine = math.sin(theta)
        cosine = math.cos(theta)

        #bilježenje početnih točaka za visinu
        self.max_z=max_height
        self.min_z=min_height
        self.delta_h=delta_height

        #računanje transformacije; za inicijalni slučaj uzima se da su stare i nove vrijednosti iste
        self.old_x=radius*cosine
        self.old_y=radius*sine
        self.old_z=height
        self.old_q=tf_conversions.transformations.quaternion_about_axis(theta,(0,0,1)) #stara matrica rotacije

        self.new_x=radius*cosine
        self.new_y=radius*sine
        self.new_z=height
        self.new_q=tf_conversions.transformations.quaternion_about_axis(theta,(0,0,1)) #nova matrica rotacije

    #slanje novog stanja na listener node
    def cam(self):
        #definiranje objekta tipa 'transform'
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        #osnovni podaci: trenutno vrijeme, prozor prostora, prozor kamere
        t.header.stamp=rospy.Time.now()
        t.header.frame_id="world"
        t.child_frame_id="camera"

        #translacija; budući da su vrijednosti selfa u centimetrima, pretvaramo u metre
        t.transform.translation.x = self.new_x/100
        t.transform.translation.y = self.new_y/100
        t.transform.translation.z = self.new_z/100

        #rotacija; čitamo iz kvaterniona
        t.transform.rotation.x = self.new_q[0]
        t.transform.rotation.y = self.new_q[1]
        t.transform.rotation.z = self.new_q[2]
        t.transform.rotation.w = self.new_q[3]

        br.sendTransform(t)


class Move_motors():

    def pose_callback(self, data): #napomena: krivo postavljeni self.id, ispravljeno
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
        while self.offset > 10 :
            time.sleep(0.001)
        time.sleep(1)
        print(self.actual_position, self.goal_position)


    def calc_h(self, min_h, max_h, num_of_h):  #min_h u cm, max_h u cm, num_of_h broj visina za slikanje
        return int(((max_h-min_h/7.54)*4096)/(num_of_h-1)) #napomena: za preniske pozicije nit se odmota i počne ponovo namotavati (yo-yo efekt), dodana minimalna visina kako bi se to spriječilo

    def calc_a(self, num_of_photo):    #broj slika po rotaciji
        return int((math.pi*2*4096)/num_of_photo) #napomena: umjesto originalnih 6.5, koristimo 2*math.pi




if __name__ == '__main__':
    try:
        rospy.init_node('motor_position')
        m1 = Move_motors('command', 3, 'Goal_Position', 0)
        m2 = Move_motors('command', 2, 'Goal_Position', 0)
        pub = rospy.Publisher('/ready', Bool, queue_size = 1)
        num_of_h = 4        # broj visina na kojima ce se slikati
        num_of_photo = 7    # broj slika po jednoj rotaciji
        max_h = 50          # najviša točka u cm do koje će ići kamera, max je 70cm
        min_h = 30          # najniža točka u cm do koje će ići kamera, stvarni min je 0 cm, preporuča se 10 cm
        tag = 0
        time.sleep(2)

        elev = 0                            # trenutna vertikalna pozicija kamere
        rad = 32                            # radijus okvira skenera - napomena: ponovo izmjeriti
        height_old = max_h                  # 'stara' visina; za slučaj da se kamera spušta, uzima se najviša točka
        value_old = 0                       # 'stari' kut; uzima se početna vrijednost
        delta_h=(max_h-min_h)/(num_of_h-1)  # razlika u visini između vertikalnih pozicija na kojima se kamera zaustavlja

        # inicijaliziranje varijable za transformaciju pozicije kamere i slanje početne transformacije
        cam_pose=Trans(height_old,max_h,min_h,delta_h,rad,value_old)
        cam_pose.cam()

        for angle in range (num_of_photo):
            value = m2.calc_a(num_of_photo) * (angle)
            cam_pose.circ(value_old/4096, value/4096)   # računanje novog x/y
            m2.move(value)                              # pomak na novi x/y
            cam_pose.cam()                              # slanje nove pozicije
            for height in range (num_of_h):
                # računanje nove visine - koristi se varijabla elev za računanje pozicije kamere
                if tag == 0:
                    elev = (m1.calc_h(min_h, max_h, num_of_h) * (height))
                else:
                    elev = (m1.calc_h(min_h, max_h, num_of_h) * (num_of_h-height-1))
                cam_pose.vert(height_old, height)       # računanje nove visine
                m1.move(elev)                           # pomak na novu visinu
                cam_pose.cam()                          # slanje nove pozicije
                pub.publish(True)
                print('Slikano na kutu br. ' + str(angle))
                time.sleep(2)
                height_old=height                       # bilježenje zastarjele Visine
            value_old=value_old                         # bilježenje zastarjelog kuta
        if tag == 0:
            tag = 1
        else:
            tag = 0
        m1.move(0)
        m2.move(0)
        # računanje transformacije za povratak na početnu poziciju
        cam_pose.circ(value_old/4096, 0)
        cam_pose.vert(height_old, 0)
        cam_pose.cam()
    except rospy.ROSInterruptException:
        pass
