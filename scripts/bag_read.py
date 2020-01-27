#!/usr/bin/env python

import rosbag
import rospy
import sys


rospy.init_node('bag_read')
bag_name = sys.argv[1]

rospy.loginfo('loading {}'.format(bag_name))
t0 = rospy.Time.now()
bag = rosbag.Bag(bag_name)
t1 = rospy.Time.now()
dt = (t1 - t0).to_sec()
rospy.loginfo('{} seconds to load {}'.format(dt, bag_name))

info = bag.get_type_and_topic_info()
rospy.loginfo(info)
count = 0
for topic, msg, stamp in bag.read_messages():
    if rospy.is_shutdown():
        break
    if count % 300 == 0:
        # for marker in msg.markers:
        print('{} {} {} {}'.format(count, stamp, topic, type(msg)))
    count += 1
bag.close()
