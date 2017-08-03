#!/usr/bin/python2
#

import rospy
from aenum import Enum
from namedlist import namedlist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, Float32, Float32MultiArray
from sensor_msgs.msg import Joy
from ar_localization import TrackPosition

class ControllerMode(Enum):
    POTENTIAL = 0
    WALL_FOLLOW = 1
    VISION = 2

class PrixControllerNode:
    def __init__(self):
        self.track_position = TrackPosition.NOT_STARTED
        self.controller_mode = self.position_to_controller_mode(self.track_position)
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        
        self.max_speed = 1
        self.min_speed = 1
        self.max_steering = 0.34

        DriveValues= namedlist('DriveValues', ['p', 'd', 'i',
            ('max_speed', self.max_speed),
            ('min_speed', self.min_speed),
            ('prev', 0),
            ('derivator', 0),
            ('integrator', 0)])
        self.K_vision    = DriveValues(p = -0.01, d = 0, i = 0, max_speed = 1, min_speed = 1)
        self.K_wall      = DriveValues(p = 0.1, d = 0, i = 0, max_speed = 3, min_speed = 3)
        self.K_potential = DriveValues(p = 4.0, d = 0.2, i = 0.0, max_speed = 3, min_speed = 3)
        
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
        steering_angle = max(-self.max_steering, min(steering_angle, self.max_steering))
        if steering_angle != 0:
            speed = 0.4 / abs(steering_angle)
        
        if speed < K.min_speed:
            speed = K.min_speed
        speed = max(-K.max_speed, min(speed, K.max_speed))
        
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = steering_angle
        if self.drive_enabled:
            self.cmd_pub.publish(drive_cmd)
    
    def position_to_controller_mode(self, track_position):
        if track_position == TrackPosition.START or track_position == TrackPosition.BRIDGE:
            return ControllerMode.POTENTIAL
        elif track_position == TrackPosition.AFTER_WATER:
            return ControllerMode.WALL_FOLLOW
        elif track_position == TrackPosition.YELLOW_BRICK:
            return ControllerMode.VISION

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
        self.controller_mode = self.position_to_controller_mode(self.track_position)

    def vision_error_callback(self, msg):
        if self.controller_mode == ControllerMode.VISION:
            self.pid_control(msg.data, self.K_vision.max_speed, self.K_vision)

    def wall_error_callback(self, msg):
        if self.controller_mode == ControllerMode.WALL_FOLLOW:
            self.pid_control(msg.data, self.K_wall.max_speed, self.K_wall)

    def potential_field_error_callback(self, msg):
        if self.controller_mode == ControllerMode.POTENTIAL:
            self.pid_control(msg.data[0], msg.data[1], self.K_potential)


if __name__ == '__main__':
    rospy.init_node('prix_controller')
    node = PrixControllerNode()
    rospy.spin()

