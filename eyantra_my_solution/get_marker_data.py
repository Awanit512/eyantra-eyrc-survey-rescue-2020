#!/usr/bin/env python
#final file/////////////////////////////////////SUCCCCESSSFULLLLLLLL//////////////////////////
import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
#from geometry_msgs.msg import Pose

#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = {}	# Declaring dictionaries
	
		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
	# Callback for /whycon/poses
	# Please fill in the function
	def whycon_data(self,msg):
		for i in range(5):
			self.whycon_marker[i]=list()
			self.whycon_marker[i].append(round(msg.poses[i].position.x,3))
			self.whycon_marker[i].append(round(msg.poses[i].position.y,3))
			self.whycon_marker[i].append(round(msg.poses[i].position.z,3))

		#self.whycon_marker[str(1)]=msg.pose.position.x


		# Printing the detected markers on terminal
		#pose.msg.position.x , pose.msg.position.y , pose.msg.position.z
		
		print(self.whycon_marker)


if __name__=="__main__":
	marker = Marker_detect()
	while not rospy.is_shutdown():
		rospy.spin()