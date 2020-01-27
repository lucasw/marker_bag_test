#!/usr/bin/env python

import rosbag
import sys


print(sys.argv)
bag = rosbag.Bag(sys.argv[1])
info = bag.get_type_and_topic_info()
print(info)
count = 0
for topic, msg, stamp in bag.read_messages():
    if count % 500 == 0:
        # for marker in msg.markers:
        print('{} {} {}'.format(stamp, topic, type(msg)))
    count += 1
bag.close()
