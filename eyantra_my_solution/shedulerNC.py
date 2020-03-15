#!/usr/bin/env python
from __future__ import print_function
import roslib
import time
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
#THE SCRIPT IS HARDCODED EXCEPT WE HAVE TO FILL THE BASE CELL LOCATION BEFORE RUNNING THE SCRIPT IN def __init__(self): SELF.BASE_CELL_LOCATION
#also do the same in position_hold.py script
class sr_scheduler():

	def __init__(self):
		rospy.Subscriber('/detection_info',SRInfo,self.detection_callback)	
		rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		rospy.Subscriber('/stats_sr', SRDroneStats,self.stats_callback)
		self.decision_pub = rospy.Publisher('/decision_info',SRInfo,queue_size=4)
		self.decided_msg = SRInfo()
		self.previous_decided_msg = SRInfo()
		self.detectedloc=[]
		self.detectedinfo=[]
		self.detected_event_status=[]
		self.detected_event_time=[]
		self.alarm_rescue=[]
		self.no_of_rescue=0
		self.event_count=0
		self.prev_event_loc=0
		self.prev_event_info=0
		self.deadline={"FOOD": 30,"MEDICINE": 30 ,"RESCUE": 10}
		self.startflag=0
		self.gotoflag=0#if base is scheduled then made this 1
		self.escapeflag=0
		self.stats=None
		self.base_cell_location="B3"#if rescue is scheduled made this 1 
		#DEPENDING UPON CONFIGURATION SET THE BASE LOCATION IN SCRIPT JUST FOR CONVENIENCE ASSUMING BASE IS B3


	def stats_callback(self, msg):###########################################
		self.stats.food_on_board=msg.foodOnboard
		self.stats.med_on_board=msg.medOnboard


		
	def detection_callback(self, msg):
		if self.startflag==0:
			self.decided_msg.location = msg.location
			self.decided_msg.info = msg.info

			self.event_count+=1
			if self.decided_msg.info=="RESCUE":#this tells us that rescue event has arrived so where is it in ready queue it means index of ready queue
				self.alarm_rescue.append(self.event_count-1)#since index of ready queue stats from 0 and this store the index of rescues events
				self.no_of_rescue+=1#this tells us that number of rescue events in ready queue
			self.detectedloc.append(self.decided_msg.location )
			self.detectedinfo.append(self.decided_msg.info)
			self.detected_event_status.append(0)
			self.detected_event_time.append(time.time())


		if self.startflag==1:
			if self.previous_decided_msg.location!=msg.location or self.previous_decided_msg.info!=msg.info:
				self.decided_msg.location = msg.location
				self.decided_msg.info = msg.info
				self.event_count+=1
				if self.decided_msg.info=="RESCUE":#this tells us that rescue event has arrived so where is it in ready queue it means index of ready queue
					self.alarm_rescue.append(self.event_count-1)#since index of ready queue stats from 0
					self.no_of_rescue+=1#this tells us that number of rescue events in ready queue
				self.detectedloc.append(self.decided_msg.location )
				self.detectedinfo.append(self.decided_msg.info)
				self.detected_event_status.append(0)
				self.detected_event_time.append(time.time())
				self.previous_decided_msg.location=msg.location
				self.previous_decided_msg.info=msg.info







		
		if self.startflag==0:
			self.decision_pub.publish(self.decided_msg)
			self.previous_decided_msg.location=msg.location
			self.previous_decided_msg.info=msg.info
			self.startflag=1


	def serviced_callback(self,msg):
		k=0
		self.gotoflag=0
		self.escapeflag=0
		self.serviced_msg.location = msg.location
		self.serviced_msg.info = msg.info

		if self.serviced_msg.info=="SUCCESS" or self.serviced_msg.info=="FAILURE":
			for j in range(self.event_count):
				if self.detectedloc[j]==self.serviced_msg.location and self.detected_event_status[j]!=1 :
					self.detected_event_status[j]=1# event which is either succedd or get fAILED HAVE STAUS ==1
					if self.detectedinfo[j]=="RESCUE":
						self.no_of_rescue-=1
						self.decided_msg.location = self.base_cell_location
						self.decided_msg.info = "BASE"
						self.decision_pub.publish(self.decided_msg)
						self.gotoflag=1


					break




			self.serviced_msg.info=="INVALID"

			 #########################512512512512
			if self.gotoflag==0:
				for i in range(self.event_count):
					if self.no_of_rescue>0:
						for k in self.alarm_rescue:# for each such k calculate naruto-------------.....--------------\/
							#naruto=.....                                                                            ||
							################time reqired to go there is represented by naruto---...----------------\/\/
							if self.detected_event_status[k]!=1 and (((time.time())- self.detected_event_time[k])+naruto+5)<=self.deadline["RESCUE"]:
								self.decided_msg.location =self.detectedloc[k]
								self.decided_msg.info = self.detectedinfo[k]
								self.decision_pub.publish(self.decided_msg)
								self.escapeflag=1
								break
					if self.escapeflag ==  1 :
						break
					else :
						if self.self.detected_event_status[i]==1:
							continue


						elif self.detectedinfo[i]=="FOOD" and (((self.stats.food_on_board>0) and self.self.detected_event_status[i]!=1) and (((time.time())- self.detected_event_time[i])+naruto+3)<=self.deadline[self.detectedinfo[i]]):
							self.decided_msg.location =self.detectedloc[i]
							self.decided_msg.info = self.detectedinfo[i]
							self.decision_pub.publish(self.decided_msg)
							break
						elif self.detectedinfo[i]=="MEDICINE" and (self.stats.med_on_board>0) and self.self.detected_event_status[i]!=1 and (((time.time())- self.detected_event_time[i])+naruto+3)<=self.deadline[self.detectedinfo[i]]:
							self.decided_msg.location =self.detectedloc[i]
							self.decided_msg.info = self.detectedinfo[i]
							self.decision_pub.publish(self.decided_msg)
							break
						elif self.detectedinfo[i]=="RESCUE" and self.self.detected_event_status[i]!=1 and (((time.time())- self.detected_event_time[i])+naruto+5)<=self.deadline[self.detectedinfo[i]]:
							self.decided_msg.location =self.detectedloc[i]
							self.decided_msg.info = self.detectedinfo[i]
							self.decision_pub.publish(self.decided_msg)
							break

						elif self.detected_event_status[i]!=1 and (((time.time())- self.detected_event_time[i])+naruto+3)<=self.deadline[self.detectedinfo[i]] and ((self.stats.med_on_board==0) or (self.stats.food_on_board==0)):
							self.decided_msg.location = self.base_cell_location
							self.decided_msg.info = "BASE"
							self.decision_pub.publish(self.decided_msg)
							break



		# Take appropriate action when either service SUCCESS or FAILIURE is recieved from monitor.pyc
		

	
	def shutdown_hook(self):
		# This function will run when the ros shutdown request is recieved.
		# For instance, when you press Ctrl+C when this is running
		pass



def main(args):
	
	sched = sr_scheduler()
	rospy.init_node('sr_scheduler', anonymous=False)
	rospy.on_shutdown(sched.shutdown_hook)
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)