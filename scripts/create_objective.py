#!/usr/bin/env python3

import rosbag
from std_msgs.msg import String

bag = rosbag.Bag('kuka_buy_consumables_objective.bag', 'w')

bag.write('/brushes', String("1"))
bag.write('/canvases', String("5"))
bag.write('/paint', String("1"))

bag.close()
