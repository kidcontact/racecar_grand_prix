#!/usr/bin/python2
#

import rospy
from aenum import Enum
from namedlist import namedlist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, Float32, Float32MultiArray
from sensor_msgs.msg import Joy
from ar_localization import TrackPosition

class PrixControllerNode:
    def __init__(self):
        self.track_position = TrackPosition.NOT_STARTED
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        
        PidValues= namedlist('PidValues', ['p', 'd', 'i', ('prev', 0), ('derivator', 0), ('integrator', 0)])
        self.K_vision    = PidValues(p = -0.01, d = 0, i = 0)
        self.K_wall      = PidValues(p = 1.5, d = 0, i = 0)
        self.K_potential = PidValues(p = 4.0, d = 0.2, i = 0.0)
        # working potential field values: 4, 2
        self.speed = 3.5
        self.min_speed = 3.5
        self.max_steering = 0.34
        self.drive_enabled = False

        rospy.Subscriber('track_position', Int32, self.track_position_callback)
        rospy.Subscriber('vision_error', Float32, self.vision_error_callback)
        rospy.Subscriber('wall_error', Float32, self.wall_error_callback)
        rospy.Subscriber('potential_field_error', Float32MultiArray, self.potential_field_error_callback)
        rospy.Subscriber('/joy', Joy, self.joy_callback) 

    def pid_control(self, error, speed, K):
        K.integrator += error
        K.derivator = K.prev - error
        K.prev = error
        steering_angle = K.p * error + K.i * K.integrator + K.d * K.derivator

        # bound steering angle and speed to saturation value
        steering_cap = self.max_steering
        steering_angle = max(-steering_cap, min(steering_angle, steering_cap))
        if steering_angle != 0:
            speed = 0.4 / abs(steering_angle)
        if speed < self.min_speed:
            speed = self.min_speed
        speed = max(-self.speed, min(speed, self.speed))
        
        #if abs(steering_angle) > 0.15:
        #    speed *= 0.5

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = steering_angle
        if self.drive_enabled:
            self.cmd_pub.publish(drive_cmd)
    
    def joy_callback(self, msg):
        buttons = msg.buttons
        if buttons[2] == 1: # x
            self.K_potential.integrator = 0
            self.K_potential.derivator = 0
            self.K_potential.prev = 0
            self.drive_enabled = True
        elif buttons[3] == 1: # y
            self.drive_enabled = False

    def track_position_callback(self, msg):
        self.track_position = TrackPosition(msg.data)

    def vision_error_callback(self, msg):
        if self.track_position == TrackPosition.YELLOW_BRICK:
            self.pid_control(msg.data, self.speed, self.K_vision)

    def wall_error_callback(self, msg):
        if self.track_position == TrackPosition.BRIDGE or self.track_position == TrackPosition.AFTER_WATER:
            self.pid_control(msg.data, self.speed, self.K_wall)

    def potential_field_error_callback(self, msg):
        if self.track_position == TrackPosition.START:
            self.pid_control(msg.data[0], msg.data[1], self.K_potential)
    

if __name__ == '__main__':
    rospy.init_node('prix_controller')
    node = PrixControllerNode()
    rospy.spin()

