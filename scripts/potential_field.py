#!/usr/bin/python2
import rospy
import numpy as np
import math
import heapq
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray

class MotionPlannerNode:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.scanner_callback)
        self.cmd_pub = rospy.Publisher('potential_field_error', Float32MultiArray, queue_size=10)
        
        self.force_scale_factor = 0.001
        self.minimum_repel_force = 0.02
        self.forward_force = 3
        self.num_samples = 10 # n largest magnitudes will be used
        self.max_speed = 1.2
        self.max_steering = 0.32

        self.K_d = 0.05
        self.prev = 0

    def scanner_callback(self, msg):
        cutoff = np.pi / 4
        ranges = msg.ranges[int(cutoff / msg.angle_increment): -int(cutoff / msg.angle_increment)]
        ranges = np.array(ranges, dtype=np.float32)

        # calculate repulsive forces for all points found by scanner
        ranges = ranges ** -2
        ranges[ranges < self.minimum_repel_force] = 0 
        ranges = ranges * self.force_scale_factor

        # x and y components for force vectors
        angles = np.arange(msg.angle_min + cutoff, msg.angle_max - cutoff, msg.angle_increment)
        x = np.sin(angles) * ranges
        y = np.cos(angles) * ranges
        magnitudes = np.sqrt(x ** 2 + y ** 2)
        samples = heapq.nlargest(len(magnitudes), xrange(len(magnitudes)), magnitudes.take)
        
        # compute combined force vector (-np.sum to make the forces repulsive)
        x_sum = -np.sum(x[samples])
        y_sum = -np.sum(y[samples])
        y_sum += self.forward_force
        magnitude = math.sqrt(x_sum ** 2 + y_sum ** 2)
        theta = math.atan2(x_sum, y_sum)

        if theta > np.pi / 2 or theta < -np.pi / 2:
            magnitude = -magnitude
        
        msg = Float32MultiArray()
        msg.data = [theta, magnitude]
        self.cmd_pub.publish(msg)
        
'''
        theta += self.K_d * (self.prev - theta)

        # minimum and maximum bounding for speed/steering
        speed = max(-self.max_speed, min(magnitude, self.max_speed))
        steering_angle = max(-self.max_steering, min(theta, self.max_steering))
        
        self.prev = theta

        if msg.ranges[len(msg.ranges) / 2] < 0.75:
            steering_angle = self.max_steering
        
        if speed < 0.85:
            pass#return

        # publish
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.drive.speed = speed
        cmd.drive.steering_angle = steering_angle
        self.cmd_pub.publish(cmd)

        rospy.loginfo('x_sum: %s -- y_sum: %s -- magnitude: %s -- theta: %s',
                x_sum, y_sum, magnitude, theta)
        rospy.loginfo('speed: %s -- steering: %s',
                speed, steering_angle)
'''

if __name__ == '__main__':
    rospy.init_node('potential_field')
    node = MotionPlannerNode()
    rospy.spin()

