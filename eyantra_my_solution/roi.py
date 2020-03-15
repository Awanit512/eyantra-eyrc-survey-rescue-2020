#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pickle
from imutils import contours
import imutils
import copy
import numpy as np
import itertools
import time
import json

class sr_determine_rois():


	def __init__(self):

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect",Image,self.image_callback)
		self.img = None
		self.flag=0
		
		self.lis=["A1","B1","C1","D1","E1","F1","A2","B2","C2","D2","E2","F2","A3","B3","C3","D3","E3","F3","A4","B4","C4","D4","E4","F4","A5","B5","C5","D5","E5","F5","A6","B6","C6","D6","E6","F6"]

	
	# CV_Bridge acts as the middle layer to convert images streamed on rostopics to a format that is compatible with OpenCV
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)

  	def detect_rois(self) :
  		
  		
  		self.img_copy=self.img

  		if self.flag == 0:
  			self.countours = []


  			blur= cv2.medianBlur(self.img_copy, 5)

  		

  			sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
  			sharpen = cv2.filter2D(blur, -1, sharpen_kernel)
  			ret,thresh = cv2.threshold(sharpen,127,255, cv2.THRESH_BINARY)
  			kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
  			close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=23)



  			_ ,cnts, _= cv2.findContours(close, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

  			#print is jst for debugging
  			print("no of cnts="+str(len(cnts)))

  			for i in range(1,37):
  				self.countours.append(cnts[i])
  			print("Number of contours = " + str(len(self.countours)))


  			#############above is final cnts list
  			#for c in cnts:
  				#approx=cv2.approxPolyDP(c,0.01* cv2.arcLength(cnts, True), True)
  				#cv2.drawContours(self.img_copy,[approx], -1, (255, 255, 255), 3)




  			##############drawinfg cnts
  			cv2.drawContours(self.img_copy,self.countours, -1, (0, 255, 0), 3)




  			###########sorting,.................
  			#self.countours.sort(key=lambda x:self.countour_precedence(x,self.img_copy.shape[1]))




  			############drawing celll names
  			#for i in xrange(len(self.countours)) :
  				#x,y,w,h=cv2.boundingRect(self.countours[i])
  				
  				#self.img_copy=cv2.putText(self.img_copy,str(i+1),(x+w/4,y+3*h/4),cv2.FONT_HERSHEY_COMPLEX,1,[255,255,255],True)



  			cv2.imshow("Detected ROIs", self.img_copy)
  			cv2.waitKey(1)
  			#cv2.imshow("Detected ROIs", self.img_copy)

  			#for i in range(len(cnts)):
  				#cv2.drawContours(self.img_copy,cnts, i, (0, 255, 0), 3)
  				#cv2.imshow("Detected ROIs", self.img_copy)
  				#time.sleep(10)
  			#cnts = sorted(cnts, key=cv2.contourArea)[:5]
  			
  			#for (i, c) in enumerate(cnts):
  				#orig = draw_contour(orig, c, i)

  			#(cnts, boundingBoxes) = self.sort_rois(cnts)

  			#for (i, c) in enumerate(cnts):
  				#self.draw_cell_names(self.img_copy, c, i)
  			#cnts = imutils.grab_contours(cnts)
  			#for (i, c) in enumerate(cnts):
  				#self.img_copy = contours.label_contour(self.img_copy, c, i, color=(255, 0, 159))





  			#for c in cnts:
  				#approx=cv2.approxPolyDP(c,0.01* cv2.arcLength(cnts, True), True)
  				#cv2.drawContours(self.img_copy,[approx], -1, (255, 255, 255), 3)
  				#x = approx.ravel()[0]
  				#y = approx.ravel()[1]
  				#cv2.putText(img, str(q), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255))
  				#q+=1














 



  	
  			#cv2.imshow("Detected ROIs", self.img_copy)

			#cv2.imshow('sharpen', sharpen)
			#cv2.imshow('blur', blur)
			#cv2.imshow('thresh', thresh)
			self.flag=1



  		#A copy is chosen because self.img will be continuously changing due to the callback function
  		cv2.waitKey(1)
		



		# Add your Code here


	def countour_precedence(self,cnts,cols) :
		t_factor=10
		origin = cv2.boundingRect(cnts)
		return ((origin[1]// t_factor)*t_factor)*cols+origin[0]

		
	def sort_rois(self):
		self.countours.sort(key=lambda x:self.countour_precedence(x,self.img_copy.shape[1]))
		#print("sorting wait u")
		#i=0
		#boundingBoxes = [cv2.boundingRect(c) for c in cnts]
		#(cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes),key=lambda b:b[1][i]))
		#return (cnts, boundingBoxes)
		

		# Add your Code here

	

	def query_yes_no(self, question, default=None):
		valid = {"yes": True, "y": True, "ye": True,"no": False, "n": False}
		if default is None :


			prompt = " [Y/N]:\t"
		elif default == "yes" : 

			prompt = " [Y/N]:\t"
		elif default == "no":
			prompt = " [Y/N]:\t"
		else:
			raise ValueError("Invalid default answer: '%s'" % default)

		while True:
			sys.stdout.write(question + prompt)
			choice = raw_input().lower()
			if default is not None and choice == '':
				return valid[default]
			elif choice in valid:
				return valid[choice]
			else:
				sys.stdout.write("\nPlease respond with 'yes' or 'no' ""(or 'y' or 'n').\n")

	'''	You may save the list using anymethod you desire
	 	The most starightforward way of doing so is staright away pickling the objects
		You could also save a dictionary as a json file, or, if you are using numpy you could use the np.save functionality
		Refer to the internet to find out more '''
	def save_rois(self,file_path1 = '/home/awanit/catkin_ws/src/survey_and_rescue/scripts/roi_coords.json',file_path2 = '/home/awanit/catkin_ws/src/survey_and_rescue/scripts/roi_coords_pickle.txt',file_path3 = '/home/awanit/catkin_ws/src/survey_and_rescue/scripts/roi_coords_numpy.npy'):
		
		np.save(file_path3,self.countours)
		print("save succesfully as numpy file\n")



		with open(file_path2, mode='wb') as fg:
			pickle.dump(self.countours,fg)
		print("save succesfully as pickle file\n")







		#roi_dict = dict(zip(self.lis, self.countours))
		float_lis=np.array(self.countours)
		float_lis=np.array(float_lis) + 0.
		roi_dict = dict(zip(self.lis, float_lis))


		with open(file_path1, mode='w') as outfiles:
			json.dump(roi_dict, outfiles)
		print("save succesfully as json file\n")




		
		#Add your code here

	#You may optionally implement this to display the image as it is displayed in the Figure given in the Problem Statement
	def draw_cell_names(self):
		for i in xrange(len(self.countours)) :
  				x,y,w,h=cv2.boundingRect(self.countours[i])
  				
  				self.img_copy=cv2.putText(self.img_copy,self.lis[i],(x+w/4,y+3*h/4),cv2.FONT_HERSHEY_COMPLEX,1,[255,255,255],True)
  		cv2.imwrite('/home/awanit/catkin_ws/src/survey_and_rescue/scripts/roi_detect.png',self.img_copy)

		#print(i)
		#M = cv2.moments(c)
		#if  M["m00"] != 0 :
		#	cX = int(M["m10"] /( M["m00"]))
		#	cY = int(M["m01"] /( M["m00"]))
		#else :
		#	print(str(i), str(M["m00"]))
		#	print("\n")
		#	return image
		#cv2.putText(image, "#{}".format(i + 1), (cX - 20, cY), cv2.FONT_HERSHEY_SIMPLEX,2.0, (255, 255, 255), 2)
		#print("\n")
		#return image
		
		#Add your code here

def main(args):
	#Sample process flow
	try:
		rospy.init_node('sr_roi_detector', anonymous=False)
		r =	sr_determine_rois()
		while True:
			if r.img is not None:
				r.detect_rois()
				if(r.flag == 0):
					new_thresh_flag = r.query_yes_no("WHAT U think 36 cells are  detected ?")
					if(new_thresh_flag):
						r.flag=1
						
						#Change settings as per your desire
					else:
						continue
				else:
					satis_flag = r.query_yes_no("Are you satisfied with the currently detected ROIs?")
					if(satis_flag):
						#print("want to take one more tym y or n")
						#choice = raw_input().lower()
						#if (choice == "y"):
							#r.flag=0
						#else:
							#break

						r.sort_rois()
						r.draw_cell_names()
						r.save_rois()
						cv2.destroyAllWindows()
						break
				
					
					else:
						r.flag=0
						#Change more settings
		# r.draw_cell_names(r.img) # Again, this is optional
		cv2.destroyAllWindows()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)