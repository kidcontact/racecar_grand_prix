#!/usr/bin/python2
import rospy
import numpy as np
import math
import heapq
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray, Int32
from ar_localization import TrackPosition

class MotionPlannerNode:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.scanner_callback)
        rospy.Subscriber('track_position', Int32, self.position_callback)
        self.cmd_pub = rospy.Publisher('potential_field_error', Float32MultiArray, queue_size=10)
        
        self.force_scale_factor = 0.002
        self.minimum_repel_force = 0.02
        self.forward_force = 5
        self.num_samples = 10 # n largest magnitudes will be used
        self.track_position = None
        
    def scanner_callback(self, msg):
        cutoff = np.pi / 4
        ranges = msg.ranges[int(cutoff / msg.angle_increment): -int(cutoff / msg.angle_increment)]
        ranges = np.array(ranges, dtype=np.float32)

        # calculate repulsive forces for all points found by scanner
        ranges = ranges ** -1
        ranges[ranges < self.minimum_repel_force] = 0 
        ranges = ranges * self.force_scale_factor

        # x and y components for force vectors
        angles = np.arange(msg.angle_min + cutoff, msg.angle_max - cutoff, msg.angle_increment)
        x = np.sin(angles) * ranges
        y = np.cos(angles) * ranges
        
        fov = np.pi / 6 # 22.5 degrees on each side for 45 degrees total
        front = np.arange(int((len(x) / 2) - (fov / msg.angle_increment)), int((len(x) / 2) + (fov / msg.angle_increment)))
        x[front] = x[front] * 4
        y[front] = y[front] * 4
        
        if self.track_position == TrackPosition.MESH_WALL:
            side_scale = 0

            x[:front[0]] = x[:front[0]] * side_scale
            x[front[-1]:] = x[front[-1:]] * side_scale
            y[:front[0]] = y[:front[0]] * side_scale
            y[front[-1]:] = y[front[-1:]] * side_scale

        #magnitudes = np.sqrt(x ** 2 + y ** 2)
        #samples = heapq.nlargest(len(magnitudes), xrange(len(magnitudes)), magnitudes.take)
        
        # compute combined force vector (-np.sum to make the forces repulsive)
        x_sum = -np.sum(x)
        y_sum = -np.sum(y)
        y_sum += self.forward_force
        magnitude = math.sqrt(x_sum ** 2 + y_sum ** 2)
        theta = math.atan2(x_sum, y_sum)

        if theta > np.pi / 2 or theta < -np.pi / 2:
            magnitude = -magnitude
            theta = -theta
        rospy.loginfo('x_sum: %s -- y_sum: %s -- magnitude: %s -- theta: %s',
                x_sum, y_sum, magnitude, theta)
        msg = Float32MultiArray()
        msg.data = [theta, magnitude]
        self.cmd_pub.publish(msg)
        
    def position_callback(self, msg):
        self.track_position = TrackPosition(msg.data)

if __name__ == '__main__':
    rospy.init_node('potential_field')
    node = MotionPlannerNode()
    rospy.spin()

