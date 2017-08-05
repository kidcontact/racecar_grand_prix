#!/usr/bin/python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class wallController:
	def __init__(self):
		# Subscription and Publication
		rospy.Subscriber("/scan", LaserScan, self.wall_controller)
		self.cmd_pub = rospy.Publisher("wall_error", Float32, queue_size=10)
		
		# Desired distance from the wall
		self.d_des = 0.7#0.68 #left_.4 right_.9

		# Error variables
		self.error = 0

		# Right(0) or left(1)
		self.wall_orientation = 1

                self.start = int((np.pi / 8) * (720 / np.pi))
                self.end = int((np.pi / 2) * (720 / np.pi))

	def wall_controller(self, scan):

 		smallestDistance = 300
                
                r_ranges = scan.ranges[self.start:self.end]
                l_ranges = scan.ranges[-self.end:-self.start]
                
                right_mean = np.mean(r_ranges)
                left_mean = np.mean(l_ranges)
                
                if right_mean > left_mean:
                    closest_point = np.min(l_ranges)
                    scale = -1
                    side = 'left'
                else:
                    closest_point = np.min(r_ranges)
                    index = np.argmin(r_ranges)
                    scale = 1
                    side = 'right'

                #closest_right_point = np.min(r_ranges)
                #closest_left_point = np.min(l_ranges)
                
		self.error = self.d_des - closest_point
		
		
                self.error *= scale
		
                #print 'error: ' + str(self.error) + ' -- ' + side + ' -- ' + str(closest_point)
                self.cmd_pub.publish(self.error)

if __name__ == "__main__":
    rospy.init_node("wall_tracker")
    node = wallController()
    rospy.spin()
