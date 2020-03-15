#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import json
import sys
import os
from tf.transformations import euler_from_quaternion
#THE SCRIPT IS HARDCODED EXCEPT WE HAVE TO FILL THE BASE CELL LOCATION BEFORE RUNNING THE SCRIPT IN def __init__(self): SELF.BASE_CELL_LOCATION
#also do the same in position_hold.py script



class Edrone():

	'''coordinates={}
	with open('square_coords.json') as json_file:
			coordinates=json.load(json_file)
			for coords in coordinates:
				coordinates[coords][2]=35
				coordinates[coords].append(-3)'''

	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		
		#with open('square_coords.json') as json_file:
		#	coordinates=json.load(json_file)
		#for coords in coordinates:
				#coordinates[coords][2]=35
				#coordinates[coords].append(-3)
		'''coordinates={}
		with open('square_coords.json') as json_file:
			coordinates=json.load(json_file)
			for coords in coordinates:
				coordinates[coords][2]=35
				coordinates[coords].append(-3)'''

		self.drone_position = [0,0,0,0,0]
		self.quaternion = [0,0,0,0]
		self.orientation_list=[0,0,0,0]
		#self.setpoint = [2,2,20,-3] 
		self.setpoint = [0, 0,32,0]# whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [57.66,57.66,105,0]
		self.Ki = [0.416,0.416,0.856,0]
		self.Kd = [41.4,41.7,35.1,0]

		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.prev_error=[0,0,0,0]
		self.max_values=[1530,1530,1900,2000]
		self.min_values=[1430,1430,1100,1000]
		self.iterm=[0,0,0,0]
		self.error_value=[0,0,0,0]
		self.values =[0,0,0,0]
		self.sample_time = 0.0333
		self.decided_msg = None
		self.base_cell_location=None
		self.cell_name_list = []
		self.led_seq_list = []

		








		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.value_pub1 = rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.value_pub2 = rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.value_pub3 = rospy.Publisher('/roll_error',Float64,queue_size=1)
		self.value_pub4 = rospy.Publisher('/yaw_error',Float64,queue_size=1)








		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/decision_info', SRInfo, self.decision_callback)






		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	 #Disarming condition of the drone
	 #512 json file editing


	#def reading_json_file(self):
		#with open('square_coords.json') as json_file:
		#	coordinates=json.load(json_file)
		#for coords in coordinates:
				#coordinates[coords][2]=20
				#coordinates[coords].append(-3)
			




	'''def getJson(filename):
		with open(filename,'r') as json_file:
			return json.load(json_file)'''

	def disarm(self):

		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def read_tsv_config(self, file_path=os.path.expanduser('~/catkin_ws/src/survey_and_rescue/scripts/LED_Config.tsv')):
		 with open(file_path, 1='r') as (infile):
			 reader = csv.reader(infile, 3='\t')
			if os.path.split(file_path)[1] == 'LED_Config.tsv':
				 for row in reader:
					if all(y.strip() for y in row):
						row[1] = string.upper(row[1]).strip()
						row[0] = string.upper(row[0]).strip()
						cell_input_res = re.findall('^[A-F][1-6]$', row[0])
						if cell_input_res:
							if row[1] != 'BASE':
								self.cell_name_list.append(row[0])
								self.led_seq_list.append(int(row[1]) - 1)
							else:
								 self.base_cell_location = row[0]
						else:
							raise EnvironmentError('sorry incorrect. Coordinate')


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def decision_callback(self,msg):
		self.decided_msg.location=msg.location
		self.decided_msg.info=msg.info


	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] =msg.poses[0].position.y
		self.drone_position[2] =msg.poses[0].position.z
		self.quaternion =msg.poses[0].orientation
		self.orientation_list=[self.quaternion.x,self.quaternion.y,self.quaternion.z,self.quaternion.w]
		(self.roll1,self.pitch1,self.drone_position[3])=euler_from_quaternion(self.orientation_list)
		#print self.drone_position[3],self.roll1,self.pitch1







		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp *0.06
		self.Ki[1] = pitch.Ki *0.008
		self.Kd[1] = pitch.Kd *0.3

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp *0.06
		self.Ki[0] = roll.Ki *0.008
		self.Kd[0] = roll.Kd *0.3
	def yaw_set_pid(self,yaw):
		self.Kp[0] = yaw.Kp *0.06
		self.Ki[0] = yaw.Ki *0.008
		self.Kd[0] = yaw.Kd *0.3














	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
		for i in range(0,4):

			self.error_value[i]=self.setpoint[i]-self.drone_position[i]
		self.out_roll=self.error_value[0]*self.Kp[0]+self.iterm[0]+self.Kd[0]*(self.error_value[0]-self.prev_error[0])/self.sample_time
		self.out_pitch=self.Kp[1]*self.error_value[1]+self.iterm[1]+self.Kd[1]*(self.error_value[1]-self.prev_error[1])/self.sample_time
		self.out_alt=self.Kp[2]*self.error_value[2]+self.iterm[2]+self.Kd[2]*(self.error_value[2]-self.prev_error[2])/self.sample_time
		self.out_yaw=self.Kp[3]*self.error_value[3]+self.iterm[3]+self.Kd[3]*(self.error_value[3]-self.prev_error[3])/self.sample_time
		self.cmd.rcRoll =1500+self.out_roll
		self.cmd.rcPitch=1500-self.out_pitch
		self.cmd.rcThrottle=1500-self.out_alt
		self.cmd.rcYaw=1500-self.out_yaw
		if self.cmd.rcRoll>self.max_values[0]:
			self.cmd.rcRoll=self.max_values[0]
		elif self.cmd.rcRoll<self.min_values[0]:
			self.cmd.rcRoll=self.min_values[0]
		if self.cmd.rcPitch>self.max_values[1]:
			self.cmd.rcPitch=self.max_values[1]
		elif self.cmd.rcPitch<self.min_values[1]:
			self.cmd.rcPitch=self.min_values[1]
		if self.cmd.rcThrottle>self.max_values[2]:
			self.cmd.rcThrottle=self.max_values[2]
		elif self.cmd.rcThrottle<self.min_values[2]:
			self.cmd.rcThrottle=self.min_values[2]
		if self.cmd.rcYaw>self.max_values[3]:
			self.cmd.rcYaw=self.max_values[3]
		elif self.cmd.rcYaw<self.min_values[3]:
			self.cmd.rcYaw=self.min_values[3]
		for j in range(0,4):
			self.prev_error[j]=self.error_value[j]
			self.iterm[j]=(self.iterm[j]+self.error_value[j]*self.sample_time)*self.Ki[j]

		self.command_pub.publish(self.cmd)	

			

			
			
			
			
			


		
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum









	#------------------------------------------------------------------------------------------------------------------------




		

		self.value_pub1.publish(self.error_value[2])
		self.value_pub2.publish(self.error_value[1])
		self.value_pub3.publish(self.error_value[0])
		self.value_pub4.publish(self.error_value[3])
				




if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(30)
	lis=[]
	ps={}
	with open('/home/awanit/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json') as json_file:
		coordinates=json.load(json_file)
		for coords in coordinates:
			coordinates[coords][2]=32   #making height -32 at 1.5 feet from ground
			coordinates[coords].append(0)   #making yaw to 0
			lis.append(coords)
	lis.sort()


	e_drone.read_tsv_config(os.path.join(path, 'LED_Config.tsv'))

	e_drone.setpoint=coordinates[self.base_cell_location]
	start=0# statring from or whenever come to  base start is  kept at =0 delse at other set point i.e cell other than base start==1
	time_for_hover=5



	


	#print(coordinates)





	#specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	#while not rospy.is_shutdown():
	#	e_drone.pid()
	k=0
	i=0
	h=0
	m=0
	l=0
	a=1
	j=0
	s=0
	g=1




	#specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		#timeR________________________________________________________________--_____________________________
		if abs(e_drone.error_value[0])<=0.5 and abs(e_drone.error_value[1])<=0.5 and abs(e_drone.error_value[2])<=1 :




			if m==0:
				l= time.time()
				m=1
			a=0

            if start==0:
				print("\rAt {} for {} Seconds".format(self.base_cell_location,str(round(time.time()-l-j,1))))
				s=time.time()-l-j
				os.system('clear')
				g=0
			else:
				print("\rAt {} for {} Seconds".format(self.decided_msg.location,str(round(time.time()-l-j,1))))
				s=time.time()-l-j
				os.system('clear')
				g=0

		else :
			if g==0 :
				j=time.time()-l-s


		#timer ends ______________________________--------------------________X-----X----X---_______________-
			
			
		








		if (s>=time_for_hover) and start==0 :
			e_drone.setpoint=coordinates[self.decided_msg.location]
			if self.decided_msg.info=="FOOD" or self.decided_msg.info=="MEDICINE":
				time_for_hover=3
				start=1
			elif self.decided_msg.info=="RESCUE":
				time_for_hover=5
				start=1
			elif self.decided_msg.info=="BASE":
				time_for_hover=5
				start=0






			m=0
			a=1
			s=0
			j=0
			g=1
			

			
			



			i+=1
			#k=0	
		#if i>=35 and s>=3:
			#e_drone.disarm()





		r.sleep()
