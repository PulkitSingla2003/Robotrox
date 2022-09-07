#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.79, Y + 0.66).
#you need to name this node "omnibase_controller"

import rospy
import numpy as np
from nav_msgs.msg import Odometry

rospy.init_node('tb3_controller')

sub1 = rospy.Subscriber('/odom', Odometry)