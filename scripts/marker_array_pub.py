#!/usr/bin/env python
# Test publication of marker arrays into rosbags, particularly those with many points forming lines

import math
import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class MarkerArrayPub:
    def __init__(self):
        self.period = rospy.get_param("~period", 0.1)
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=2)
        self.phase = 0.0
        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def update(self, event):
        msg = MarkerArray()
        for i in range(8):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = event.current_real
            marker.type = Marker.LINE_STRIP
            marker.ns = 'test'
            marker.id = i
            marker.pose.orientation.w = 1.0
            marker.color.r = 0.5
            marker.color.b = 1.0 - i * 0.1
            marker.color.g = i * 0.2
            marker.color.a = 1.0
            marker.scale.x = 0.01
            marker.scale.y = 0.0
            marker.scale.z = 0.0
            for j in range(3000):
                pt = Point()
                fr = float(j) / 200.0
                # spiral
                sc = 0.8
                pt.x = sc * math.cos(fr * math.pi + self.phase) + i * 2.
                pt.y = sc * math.sin(fr * math.pi + self.phase)
                pt.z = fr * 0.15
                marker.points.append(pt)
            msg.markers.append(marker)
        self.marker_pub.publish(msg)
        self.phase += self.period * 0.3333

if __name__ == '__main__':
    rospy.init_node('marker_array_pub')
    marker_array_pub = MarkerArrayPub()
    rospy.spin()
