#!/usr/bin/env python3

import sys
import rospy
import time
from std_msgs.msg import Bool
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList


class Move_motors():
    
    def pose_callback(self, data):
        if self.id == 3 :
            self.actual_position = data.dynamixel_state[0].present_position
        elif self.id == 2 :
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


    def calc_h(self, max_h, num_of_h):  #max_h u cm, num_of_h broj visina za slikanje
        return int(((max_h/7.54)*4096)/(num_of_h-1))

    def calc_a(self, num_of_photo):    #broj slika po rotaciji
        return int((6.5*4096)/num_of_photo)




if __name__ == '__main__':
    try:
        rospy.init_node('motor_position')
        m1 = Move_motors('command', 3, 'Goal_Position', 0)
        m2 = Move_motors('command', 2, 'Goal_Position', 0)
        pub = rospy.Publisher('/ready', Bool, queue_size = 1)
        num_of_h = 4        # broj visina na kojima ce se slikati
        num_of_photo = 7    # broj slika po jednoj rotaciji
        max_h = 50          # visina u cm do koje će ići kamera, max je 70cm
        tag = 0
        time.sleep(2)
        for angle in range (num_of_photo):
            value = m2.calc_a(num_of_photo) * (angle)
            m2.move(value)
            for height in range (num_of_h): 
                if tag == 0:
                    m1.move(m1.calc_h(max_h, num_of_h) * (height))
                else:
                    m1.move(m1.calc_h(max_h, num_of_h) * (num_of_h-height-1))
                pub.publish(True)
                print('slikal sam ' + str(angle))
                time.sleep(2)
        if tag == 0:
            tag = 1
        else:
            tag = 0
        m1.move(0)
        m2.move(0)
    except rospy.ROSInterruptException:
        pass
