#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from cv_bridge import CvBridge, CvBridgeError
import randoms
import pickle
import imutils
import copy





class sr_determine_colors():

	def __init__(self):
		self.detect_info_msg = SRInfo()
		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=10) 
 		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
 		self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
 		self.img = None
 		self.lis=["A1","B1","C1","D1","E1","F1","A2","B2","C2","D2","E2","F2","A3","B3","C3","D3","E3","F3","A4","B4","C4","D4","E4","F4","A5","B5","C5","D5","E5","F5","A6","B6","C6","D6","E6","F6"]
 		#x1=[]
 		#y1=[]
 		#w1=[]
 		#h1=[]
 		self.detected_message = SRInfo()
 		self.information=["RESCUE","MEDICINE","FOOD"]
 		self.img_roi=[]
 		self.result=None
 		self.redflag=[0,0,0,0,0,0,0,0]
 		self.blueflag=[0,0,0,0,0,0,0,0]
 		self.greenflag=[0,0,0,0,0,0,0,0]
 		self.countours_center={}
 		self.key=[0,0,0,0,0,0,0,0]
 		self.debug=0
 		self.look=0

 		# 8 modules 
 		self.selected_rois=[6,13,2,11,21,35,26,31] #statr from 1and goes to 32

 		

	def load_rois(self):
		try:
			self.rois = np.load("/home/awanit/catkin_ws/src/survey_and_rescue/scripts/roi_coords_numpy.npy")
		

		except IOError, ValueError:
			print("File doesn't exist or is corrupted")
		


 	def image_callback(self, data):
 		try:
 			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
 		except CvBridgeError as e:
 			print(e)


 	def serviced_callback(self, msg):
 		pass
 	
	def detect_color_contour_centers(self):
		p=0
		
		for i in range(8):
			if (self.greenflag[i]==1 or self.redflag[i]==1 or self.blueflag[i]==1) and self.key[i]==0:
				M = cv2.moments(self.rois[self.selected_rois[i]])
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				self.countours_center[self.lis[self.selected_rois[i]]]=[cX,cY]
				if i==0 and p==0:
					print(cX,cY)
					p=1

				self.key[i]=1


			else:
				continue
			


		

	def check_whether_lit(self):

	
		self.img_copy=self.img
		n=np.zeros((1,4),np.uint8)
		j=0
		c=0
	
		for i in self.selected_rois:
			x,y,w,h = cv2.boundingRect(self.rois[i]) # offsets - with this you get 'mask'
			#cv2.rectangle(self.img_copy,(x,y),(x+w,y+h),(0,255,0),2)
			#cv2.imshow('cutted contour',self.img_copy[y:y+h,x:x+w])
			#n=np.array(cv2.mean(self.img_copy[y:y+h,x:x+w])).astype(np.uint8)
			t=0
			#print('Average color (BGR): ',np.array(cv2.mean(img[y:y+h,x:x+w])).astype(np.uint8))
			while t<500:
				n=np.array(cv2.mean(self.img_copy[y:y+h,x:x+w])).astype(np.uint8)

				if n[0]>=100 and self.blueflag[j]==0:#blue
				#print("cnts[{}]".format(i))
					print("medicine and i={} and j={} and t={}".format(i,j,t))
					#print(self.greenflag)
				
					self.detected_message.location=self.lis[i]
					self.detected_message.info = self.information[1]
					self.detect_pub.publish(self.detected_message)
					self.blueflag[j]=1
					self.redflag[j]=0
					self.greenflag[j]=0
				elif n[1]>=95 and self.greenflag[j]==0:#green
				#print("cnts[{}]".format(i))
					print("food and i={} and j={} and t={}".format(i,j,t))
					
				
					self.detected_message.location=self.lis[i]
					self.detected_message.info = self.information[2]
					self.detect_pub.publish(self.detected_message)
					self.greenflag[j]=1
					self.redflag[j]=0
					self.blueflag[j]=0
					print(self.greenflag)

				elif n[2]>=100 and self.redflag[j]==0:

				#red
				#print("cnts[{}]".format(i))
					print("rescue and i={} and j={} and t={}".format(i,j,t))
					
			
					self.detected_message.location=self.lis[i]
					self.detected_message.info = self.information[0]
					self.detect_pub.publish(self.detected_message)
					self.redflag[j]=1
					self.blueflag[j]=0
					self.greenflag[j]=0
					

				elif n[1]<95 and n[0]<100 and n[2]<100:
					if j==7 and self.redflag[7]==1:
						self.debug=1
					
					
					self.greenflag[j]=0
					self.blueflag[j]=0
					self.redflag[j]=0
					
				t+=1
			if n[0]>=100 and self.blueflag[7]==1 and self.look==0:
				self.look=1
				c=0
				print("flag of A2 greenflag={}".format(self.greenflag[0]))
				while c<5:
					x,y,w,h = cv2.boundingRect(self.rois[6])
					n=np.array(cv2.mean(self.img_copy[y:y+h,x:x+w])).astype(np.uint8)
					print('Average color (BGR) of A2: ',np.array(cv2.mean(self.img_copy[y:y+h,x:x+w])).astype(np.uint8))
					M = cv2.moments(self.rois[6])
					cX = int(M["m10"] / M["m00"])
					cY = int(M["m01"] / M["m00"])
					print(cX,cY)
					
					c+=1	





			j+=1
	cv2.waitKey(1)


def main(args):
	
	try:
		p=0
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		'''You may choose a suitable rate to run the node at.
		Essentially, you will be proceesing that many number of frames per second.
		Since in our case, the max fps is 30, increasing the Rate beyond that
		will just lead to instances where the same frame is processed multiple times.'''
		rate = rospy.Rate(30)
		# rate = rospy.Rate(5)
		s.load_rois()
		while s.img is None:
			pass
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			

			s.check_whether_lit()
			
			s.detect_color_contour_centers()
			rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)