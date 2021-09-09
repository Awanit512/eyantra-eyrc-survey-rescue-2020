 #!/usr/bin/env python

import rospy
import time
from edrone_client.msg import edrone_msgs
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
#import rospy
#import time
class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [5,5,15] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


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
		self.Kp = [57.66,57.66,105]
		self.Ki = [0.416,0.416,0.856]
		self.Kd = [41.4,41.7,35.1]
		self.prev_error = [0,0,0]
		self.max_values = [2000,2000,2000]
		self.min_values = [1000,1000,1000]
		self.iterm = [0,0,0]
		self.error_value = [0,0,0]
		self.values = [0,0,0]
		self.sample_time = 0.0333




		#512 -----------------------Add other required variables for pid here ----------------------------------------------








		# 512  Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#	512	 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # 512 This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# 512 self.sample_time = 0.060 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.value_pub1 = rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.value_pub3 = rospy.Publisher('/roll_error',Float64,queue_size=1)
		self.value_pub2 = rospy.Publisher('/pitch_error',Float64,queue_size=1)


		#----------512 --------------Add other ROS Publishers here-----------------------------------------------------







		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll_',PidTune,self.roll_set_pid)

		#-------------------------Add other ROS Subscribers here----------------------------------------------------






		self.arm()
		#------------------------------------------------------------------------------------------------------------
		'''for i in range(4):
			if i==0:
				
				self.setpoint = [3,3,15]
				self.arm()
			if i==1:
				self.setpoint = [2,2,20]
				self.arm()
			if i==2:
				
				self.setpoint = [3,3,23]
				self.arm()
			if i==3:
				self.setpoint = [2,2,10]
				self.arm()
			#self.arm()
		
		  ARMING THE DRONE


		

		
				for i in range(0,3):
					self.error_value[i]=self.setpoint[i]-self.drone_position[i]
				self.out_roll = self.error_value[0]*self.Kp[0]+self.iterm[0]+self.Kd[0]*(self.error_value[0]-self.prev_error[0])/self.sample_time
				self.out_pitch = self.error_value[1]*self.Kp[1]+self.iterm[1]+self.Kd[1]*(self.error_value[1]-self.prev_error[1])/self.sample_time
				self.out_alt = self.error_value[2]*self.Kp[2]+self.iterm[2]+self.Kd[2]*(self.error_value[2]-self.prev_error[2])/self.sample_time
				self.cmd.rcRoll = 1500 + self.out_roll
				self.cmd.rcPitch = 1500-self.out_pitch
				self.cmd.rcThrottle = 1500-self.out_alt

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

				for j in range(0,3):
					self.prev_error[j]=self.error_value[j]
					self.iterm[j]=(self.iterm[j]+self.error_value[j]*self.sample_time)*self.Ki[j]









	#------------------//512//-----------Write the PID algorithm here--------------------------------------------------------------

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


		
				self.command_pub.publish(self.cmd)
				self.value_pub1.publish(self.error_value[2])
				self.value_pub2.publish(self.error_value[1])
				self.value_pub3.publish(self.error_value[0])
				

			if k==1 :
				time.sleep(5)
				self.setpoint = [-4,-4,23]
				
				self.iterm = [0,0,0]
				self.error_value = [0,0,0]
				self.values = [0,0,0]
				self.cmd.rcRoll = 1500
				self.cmd.rcRoll = 1500
				self.cmd.rcPitch = 1500
				self.cmd.rcYaw = 1500
				self.cmd.rcThrottle = 1500
				self.cmd.rcAUX1 = 1500
				self.cmd.rcAUX2 = 1500
				self.cmd.rcAUX3 = 1500
				self.cmd.rcAUX4 = 1500
				for i in range(0,3):
					self.error_value[i]=self.setpoint[i]-self.drone_position[i]
				self.out_roll = self.error_value[0]*self.Kp[0]+self.iterm[0]+self.Kd[0]*(self.error_value[0]-self.prev_error[0])/self.sample_time
				self.out_pitch = self.error_value[1]*self.Kp[1]+self.iterm[1]+self.Kd[1]*(self.error_value[1]-self.prev_error[1])/self.sample_time
				self.out_alt = self.error_value[2]*self.Kp[2]+self.iterm[2]+self.Kd[2]*(self.error_value[2]-self.prev_error[2])/self.sample_time
				self.cmd.rcRoll = 1500 + self.out_roll
				self.cmd.rcPitch = 1500-self.out_pitch
				self.cmd.rcThrottle = 1500-self.out_alt

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

				for j in range(0,3):
					self.prev_error[j]=self.error_value[j]
					self.iterm[j]=(self.iterm[j]+self.error_value[j]*self.sample_time)*self.Ki[j]









	#------------------//512//-----------Write the PID algorithm here--------------------------------------------------------------

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


		
				self.command_pub.publish(self.cmd)
				self.value_pub1.publish(self.error_value[2])
				self.value_pub2.publish(self.error_value[1])
				self.value_pub3.publish(self.error_value[0]) 
			
				
				#self.pid
			
			if k==2 :
				time.sleep(5)
				self.setpoint = [-5,-5,23]
				
				
				self.cmd.rcRoll = 1500
				self.cmd.rcRoll = 1500
				self.cmd.rcPitch = 1500
				self.cmd.rcYaw = 1500
				self.cmd.rcThrottle = 1500
				self.cmd.rcAUX1 = 1500
				self.cmd.rcAUX2 = 1500
				self.cmd.rcAUX3 = 1500
				self.cmd.rcAUX4 = 1500 
				
				
				self.pid'''


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
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
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


		#----------512//512 own thoughti thuk this call back function update the new coordinateeach time callback is callrd//512----------Set the remaining co-ordinates of the drone from msg----------------------------------------------





		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp*0.06
		self.Ki[1] = pitch.Ki*0.008
		self.Kd[1] = pitch.Kd*.03

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp*0.06
		self.Ki[0] = roll.Ki*0.008
		self.Kd[0] = roll.Kd*0.3 

    
	#def pitch_set_pid(self,pitch):
    	#self.Kp[1] = pitch.Kp * 0.06
    	#self.Ki[1] = pitch.Ki * 0.008
    	#self.Kd[1] = pitch.Kd * 0.3


	#def roll_set_pid(self,roll):
    	#self.Kp[0] = roll.Kp * 0.06
    	#self.Ki[0] = roll.Ki * 0.008
    	#self.kd[0] = roll.Kd * 0.3

       
	#-----------------------////512////-----Define callback function like altitide_set_pid to tune pitch, roll--------------


















	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
		for i in range(0,3):
			self.error_value[i]=self.setpoint[i]-self.drone_position[i]
		self.out_roll = self.error_value[0]*self.Kp[0]+self.iterm[0]+self.Kd[0]*(self.error_value[0]-self.prev_error[0])/self.sample_time
		self.out_pitch = self.error_value[1]*self.Kp[1]+self.iterm[1]+self.Kd[1]*(self.error_value[1]-self.prev_error[1])/self.sample_time
		self.out_alt = self.error_value[2]*self.Kp[2]+self.iterm[2]+self.Kd[2]*(self.error_value[2]-self.prev_error[2])/self.sample_time
		self.cmd.rcRoll = 1500 + self.out_roll
		self.cmd.rcPitch = 1500-self.out_pitch
		self.cmd.rcThrottle = 1500-self.out_alt

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

		for j in range(0,3):
			self.prev_error[j]=self.error_value[j]
			self.iterm[j]=(self.iterm[j]+self.error_value[j]*self.sample_time)*self.Ki[j]









	#------------------//512//-----------Write the PID algorithm here--------------------------------------------------------------

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


		
		self.command_pub.publish(self.cmd)
		self.value_pub1.publish(self.error_value[2])
		self.value_pub2.publish(self.error_value[1])
		self.value_pub3.publish(self.error_value[0])





if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():

		for k in range(0,2):
			if k==0:
				e_drone.pid()
			if k==1:
				time.sleep(5)
				e_drone.setpoint = [-4,-4,23]
				
				e_drone.iterm = [0,0,0]
				e_drone.error_value = [0,0,0]
				e_drone.values = [0,0,0]
				'''e_drone.cmd.rcRoll = 1500
				e_drone.cmd.rcRoll = 1500
				e_drone.cmd.rcPitch = 1500
				e_drone.cmd.rcYaw = 1500
				e_drone.cmd.rcThrottle = 1500
				e_drone.cmd.rcAUX1 = 1500
				e_drone.cmd.rcAUX2 = 1500
				e_drone.cmd.rcAUX3 = 1500
				e_drone.cmd.rcAUX4 = 1500'''

				e_drone.pid()
		r.sleep()
