#!/usr/bin/env python2

from geometry_msgs.msg import Twist, Vector3, PointStamped, Point
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from neato_node.msg import Bump
import numpy as np
import tf
import math


import rospy

class WallFollower(object):
    def __init__(self):
        super(WallFollower, self).__init__()
        rospy.init_node('wall_follower')
        self.point_publisher = rospy.Publisher('/wall', PointStamped, queue_size=10)

    def publish_point(self, x, y):
        self.point_publisher.publish(
            PointStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
                point=Point(x, y, 0)
            )
        )

    def run(self):
        r = rospy.Rate(50)

        while True:
            self.publish_point(1, 0)
            r.sleep()

WallFollower().run()

