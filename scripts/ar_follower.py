#!/usr/bin/python2
#

import rospy
import math
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from ackermann_msgs.msg import AckermannDriveStamped

class ArFollowerNode:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_callback)
        self.frame_timeout = 3
        self.frames_since_seen = 0

    def marker_callback(self, msg):
        markers = msg.markers
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        if len(markers) > 0:
            self.frames_since_seen = 0
            marker = None
            for m in markers:
                if m.id == 7:
                    marker = m
            if marker is None:
                return

            position = marker.pose.pose.position
            position.x -= 0.18
            orientation = marker.pose.pose.orientation


            self.speed = 0.7
            drive_cmd.drive.speed = self.speed
            drive_cmd.drive.acceleration = 0.5
            self.steering_angle = -math.atan2(position.x, position.z) * 0.75
            drive_cmd.drive.steering_angle = self.steering_angle

        else:
            self.frames_since_seen += 1
            if self.frames_since_seen > self.frame_timeout:
                drive_cmd.drive.speed = 0
                drive_cmd.drive.steering_angle = 0
            else:
                try:
                    drive_cmd.drive.speed = self.speed
                    drive_cmd.drive.steering_angle = self.steering_angle
                except:
                    pass
        
        #drive_cmd.drive.speed = 0
        self.cmd_pub.publish(drive_cmd)

if __name__ == '__main__':
    rospy.init_node('ar_follower')
    node = ArFollowerNode()
    rospy.spin()

