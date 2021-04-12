#!/usr/bin/env python3

from std_msgs.msg import Bool
import rospy


class Finish_Listener:

	###Callback funckija u kojoj pratimo /finished
	def finish_callback(self,data):
		print("The node has finished its job? {}".format(data.data))
		if(data.data==True):
				print("Launch meshroom with the images in bin/images");
				rospy.signal_shutdown("Success")
			


	###Inicijalizacija Subscribera
	def __init__(self):
		self.listener=rospy.Subscriber("finished",Bool,self.finish_callback)
		self.finished_listen=Bool()

	###Running 
	def run(self):

		while not rospy.is_shutdown():
			rospy.spin()

	###Main function
if __name__=='__main__':
	rospy.init_node('finish_listener')
	try:
		finish=Finish_Listener()
		finish.run()
	except rospy.ROSInterruptException: pass
