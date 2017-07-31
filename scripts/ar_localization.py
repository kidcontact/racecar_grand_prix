#!/usr/bin/python2
#

import rospy
from aenum import Enum
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int32

class TrackPosition(Enum):
    NOT_STARTED = -1
    START = 0
    BRIDGE = 1
    YELLOW_BRICK = 2
    AFTER_WATER = 3

tag_ids = {
    7: TrackPosition.BRIDGE,
    6: TrackPosition.YELLOW_BRICK,
    5: TrackPosition.AFTER_WATER
}

class ArLocalizationNode:
    def __init__(self):
        self.track_position = TrackPosition.NOT_STARTED
        self.position_pub = rospy.Publisher('track_position', Int32, queue_size=1)
        self.position_pub.publish(self.track_position.value)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback)

    def marker_callback(self, msg):
        markers = msg.markers
        for m in markers:
            if m.id in tag_ids:
                self.track_position = tag_ids[m.id]
                self.position_pub.publish(tag_ids[m.id].value)

if __name__ == '__main__':
    rospy.init_node('ar_localization')
    node = ArLocalizationNode()
    rospy.spin()

