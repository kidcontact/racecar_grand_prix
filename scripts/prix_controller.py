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
    NONE        = 0
    POTENTIAL   = 1
    WALL_FOLLOW = 2
    VISION      = 3

class PrixControllerNode:
    def __init__(self):
        self.track_position = TrackPosition.NOT_STARTED
        self.mode_at_position = {
                TrackPosition.NOT_STARTED: ControllerMode.POTENTIAL,
                TrackPosition.ROLLING_WEAVE: ControllerMode.POTENTIAL,
                TrackPosition.HAIRPIN_TURN: ControllerMode.POTENTIAL,
                TrackPosition.OVERPASS: ControllerMode.POTENTIAL,
                TrackPosition.WATER_HAZARD: ControllerMode.VISION,
                TrackPosition.UNDERPASS: ControllerMode.POTENTIAL,
                TrackPosition.BOA_BASIN: ControllerMode.POTENTIAL,
                TrackPosition.MESH_WALL: ControllerMode.POTENTIAL,
                TrackPosition.FINISH_LINE: ControllerMode.POTENTIAL
        }
        self.danger_dist_at_position = {
                TrackPosition.NOT_STARTED: 1,
                TrackPosition.ROLLING_WEAVE: 0.5,
                TrackPosition.HAIRPIN_TURN: 0.5,
                TrackPosition.OVERPASS: 0.5,
                TrackPosition.WATER_HAZARD: 1,
                TrackPosition.UNDERPASS: 1,
                TrackPosition.BOA_BASIN: 1,
                TrackPosition.MESH_WALL: 0.25,
                TrackPosition.FINISH_LINE: 1
        }
        self.controller_mode = self.mode_at_position[self.track_position]
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        
        self.max_speed = 1
        self.min_speed = 1
        self.max_steering = 0.34
        rospy.set_param('danger_distance', # safety controller    
                self.danger_dist_at_position[self.track_position])

        DriveValues= namedlist('DriveValues', ['p', 'd', 'i',
            ('max_speed', self.max_speed),
            ('min_speed', self.min_speed),
            ('prev', 0),
            ('derivator', 0),
            ('integrator', 0)])
        self.K_vision    = DriveValues(p = 0.002, d = 0.000032, i = 0, max_speed = 2.4, min_speed = 2.4)
        self.K_wall      = DriveValues(p = 0.4, d = 0.001, i = 0, max_speed = 2, min_speed = 2)
        self.K_potential = DriveValues(p = 3.8, d = 0.2, i = 0, max_speed = 4, min_speed = 2)
        
        self.drive_enabled = False

        rospy.Subscriber('track_position', Int32, self.track_position_callback)
        rospy.Subscriber('vision_error', Float32, self.vision_error_callback)
        rospy.Subscriber('wall_error', Float32, self.wall_error_callback)
        rospy.Subscriber('potential_field_error', Float32MultiArray, self.potential_field_error_callback)
        rospy.Subscriber('/joy', Joy, self.joy_callback) 

    def pid_control(self, error, speed, K):
        if self.controller_mode == ControllerMode.VISION and error < 130 and error > -130:
            error = 0
        K.integrator += error
        K.derivator = K.prev - error
        K.prev = error

        steering_angle = K.p * error + K.i * K.integrator + K.d * K.derivator
        
        min_speed = K.min_speed
        max_speed = K.max_speed
        if self.track_position == TrackPosition.ROLLING_WEAVE or self.track_position == TrackPosition.HAIRPIN_TURN:
            min_speed = 1.25
            max_speed = 3.5
        elif self.track_position == TrackPosition.OVERPASS:
            max_speed = 3
        
        if self.track_position == TrackPosition.HAIRPIN_TURN:
            min_speed = 2

        # bound steering angle and speed to saturation value
        max_steering = self.max_steering
        if self.track_position == TrackPosition.OVERPASS:
            max_steering = 0.15
        steering_angle = max(-max_steering, min(steering_angle, max_steering))
        sign = speed / abs(speed)
        if abs(steering_angle) > 0.1:
            speed = 0.4 / abs(steering_angle)
        speed *= sign

        print('SPEED_BEFORE: ' + str(speed)) 
        if self.controller_mode == ControllerMode.POTENTIAL:
            if speed > 0 and speed < min_speed:
                speed = min_speed
        elif speed < min_speed:
            speed = min_speed
        speed = max(-max_speed, min(speed, max_speed))
        
        print('SPEED: ' + str(speed))

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = steering_angle
        if self.drive_enabled:
            self.cmd_pub.publish(drive_cmd)
    
    def joy_callback(self, msg):
        buttons = msg.buttons
        if buttons[2] == 1: # x
            for K in (self.K_potential, self.K_wall, self.K_vision):
                K.prev = 0
                K.derivator = 0
                K.integrator = 0
            self.drive_enabled = True
        elif buttons[3] == 1: # y
            self.drive_enabled = False

    def track_position_callback(self, msg):
        self.track_position = TrackPosition(msg.data)
        self.controller_mode = self.mode_at_position[self.track_position]
        rospy.set_param('danger_distance', # safety controller    
                self.danger_dist_at_position[self.track_position])

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

