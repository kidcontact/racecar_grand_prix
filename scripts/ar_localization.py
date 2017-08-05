#!/usr/bin/python2
#

import rospy
from aenum import Enum
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int32
import threading

class TrackPosition(Enum):
    NOT_STARTED   = 0
    ROLLING_WEAVE = 1
    HAIRPIN_TURN  = 2
    OVERPASS      = 3
    WATER_HAZARD  = 4
    UNDERPASS     = 5
    BOA_BASIN     = 6
    MESH_WALL     = 7
    FINISH_LINE   = 8

tag_ids = {
        16: TrackPosition.FINISH_LINE,
        17: TrackPosition.ROLLING_WEAVE,
        18: TrackPosition.HAIRPIN_TURN,
        19: TrackPosition.OVERPASS,
        20: TrackPosition.WATER_HAZARD,
        21: TrackPosition.UNDERPASS,
        22: TrackPosition.BOA_BASIN,
        23: TrackPosition.MESH_WALL
}

class ArLocalizationNode:
    def __init__(self):
        self.track_position = TrackPosition.FINISH_LINE

        self.position_pub = rospy.Publisher('track_position', Int32, queue_size=1)
        self.position_pub.publish(self.track_position.value)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback)
        
        self.last_seen = None

        t = threading.Thread(target=self.publish_position)
        t.daemon = True
        t.start()

    def publish_position(self):
        r = rospy.Rate(10)
        while True:
            self.position_pub.publish(self.track_position.value)
            r.sleep()

    def marker_callback(self, msg):
        markers = msg.markers
        found = None
        for m in markers:
            if m.id in tag_ids and m.pose.pose.position.z < 3.5:
                if tag_ids[m.id] == TrackPosition.WATER_HAZARD:
                    if m.pose.pose.position.z < 3:
                        self.last_seen = m
                        found = m
                elif tag_ids[m.id] == TrackPosition.UNDERPASS:
                    if m.pose.pose.position.z < 3:
                        self.last_seen = m
                else:
                    #self.track_position = tag_ids[m.id]
                    self.last_seen = m
                    found = m
                break

        if found is None and self.last_seen is not None:
            self.track_position = tag_ids[self.last_seen.id]
            self.last_seen = None

if __name__ == '__main__':
    rospy.init_node('ar_localization')
    node = ArLocalizationNode()
    rospy.spin()

