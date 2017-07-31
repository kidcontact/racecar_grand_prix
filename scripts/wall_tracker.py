
#!/usr/bin/python

import rospy
import numpy as np
import time
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class wallController:
	def __init__(self):
		# Subscription and Publication
		rospy.Subscriber("/scan", LaserScan, self.wall_controller)
		self.cmd_pub = rospy.Publisher("/wall_error", Float32, queue_size=10)
		
		# Desired distance from the wall
		self.d_des = 0.4 #left_.4 right_.9
		self.speed_des = 0

		# Controller gains
		self.Kp = .04
		self.Kd = .03

		# Error variables
		self.error = 0
		self.error_rate = 0

		# Time variables
		self.old_time = time.time()
		self.old_error = 0

		# Right(0) or left(1)
		self.wall_orientation = 0

	def wall_controller(self, scan):

	#bangbang_one_pt
		#turn_msg = AckermannDriveStamped()
		#if scan.ranges[170] > 0.4:
			#turn_msg.drive.steering_angle = -0.4
			#turn_msg.drive.speed = 1
			#self.cmd_pub.publish(turn_msg)
			#print 'RIGHT'
		#else:
			#turn_msg.drive.steering_angle = 0.4
			#turn_msg.drive.speed = 1
			#self.cmd_pub.publish(turn_msg)
			#print 'LEFT'
		#print scan.ranges[170]

	# Proportional

		#if self.wall_orientation == 0:
			#d60 = scan.ranges[300] #780, 300
			#d90 = scan.ranges[172] #908, 172
		#elif self.wall_orientation == 1:
			#d60 = scan.ranges[780]
			#d90 = scan.ranges[908]
		#C = np.sqrt(d90**2 + d60**2 - (2*d90*d60*np.cos(np.pi/6)))
		#s_prop = np.sin(np.pi/6)/C
		#a = np.arcsin(d90*s_prop)
		#true_perp = np.pi*5/6 - a
		#true_perp_ind = 540 - true_perp/scan.angle_increment
		#d_est = scan.ranges[int(true_perp_ind)]
		#self.error = self.d_des - d_est
		#print '90:', d90
		#print '60:', d60

 		smallestDistance = 300

		if self.wall_orientation == 0:
			a = 172
			b = 300
		elif self.wall_orientation == 1:
			a = 780
			b = 908

		for i in range(a, b):
			if scan.ranges[i] < smallestDistance:
				smallestDistance = scan.ranges[i]
 				smallestIndex = i
		self.error = self.d_des - smallestDistance
		print 'error: ', self.error
		self.cmd_pub.publish(self.error)

	# Derivative
	'''	new_time = time.time()
		new_error = self.error
		delta_time = new_time - self.old_time
		delta_error = new_error - self.old_error
		self.old_time = time.time()
		self.old_error = self.error
		print 'time: ', delta_time
		print 'error: ', delta_error
		self.error_rate = delta_error/delta_time
		print 'error rate: ', self.error_rate
'''
	# Send error values to position method
		'''self.run_position_controller(self.error, self.error_rate)'''

'''	def run_position_controller(self, error, error_rate):
		p_control = self.Kp * self.error
		d_control = self.Kd * self.error_rate

		saturation = 0.3

		if self.wall_orientation == 0:
			u_steer = p_control #+ d_control

		elif self.wall_orientation == 1:
			u_steer = -(p_control)# + d_control)
			
		if u_steer > saturation:
			u_steer = 0.3 
		elif u_steer < -saturation:
			u_steer = -0.3

		u_steer_msg = AckermannDriveStamped()
		u_steer_msg.drive.steering_angle = u_steer
		u_steer_msg.drive.speed = self.speed_des
		u_steer_msg.header.stamp = rospy.Time.now()
		self.cmd_pub.publish(u_steer_msg)
'''

if __name__ == "__main__":
    rospy.init_node("wall_controller")
    node = wallController()
    rospy.spin()
