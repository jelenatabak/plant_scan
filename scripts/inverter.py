#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import math
import tf_conversions

class Inverter:
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1) #definiran tip statične transformacije za slanje

        while not rospy.is_shutdown():
            rospy.sleep(0.01) #čekaj 0.01 sekundu

            t = geometry_msgs.msg.TransformStamped() #definiran tip poruke prije pretvaranja u transformaciju
            t.header.frame_id = "camera" #uzimaju se koordinate primljene sa topica "camera"
            t.header.stamp = rospy.Time.now() #uzima se trenutno vrijeme
            t.child_frame_id = "camera_optical" #definira se rezultat transformacije

            #translacije na novu poziciju
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = 0

            #rotacije oko nove pozicije - uzimaju se konstantne vrijednosti budući da je transformacija statična
            t.transform.rotation.x = -0.5
            t.transform.rotation.y = -0.5
            t.transform.rotation.z = 0.5
            t.transform.rotation.w = 0.5

            #pretvaranje poruke u transformaciju i slanje
            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('inverter') #inicijalizacija 'inverter' node
    tfb = Inverter() #definiranje tipa podatka
    rospy.spin() #ponavljaj beskrajno
