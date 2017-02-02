#!/usr/bin/env python2

from geometry_msgs.msg import Twist, Vector3, PointStamped, Point
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import numpy as np
import tf
import math


import rospy

class WallFollower(object):
    def __init__(self):
        super(WallFollower, self).__init__()
        rospy.init_node('wall_follower')
        self.point_publisher = rospy.Publisher('/wall', PointStamped, queue_size=10)
        self.movement_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)

        self.got_first_message = False
        self.offset_angle = 15
        self.base_angle = 90
        self.left_wall_point = 0
        self.right_wall_point = 0
        self.k = 5.0

        rospy.on_shutdown(self.stop)

    def stop(self):
        self.movement_publisher.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def process_scan(self, msg):
        self.left_wall_point = msg.ranges[(self.base_angle + self.offset_angle) % 360]
        self.right_wall_point = msg.ranges[(self.base_angle - self.offset_angle) % 360]

        if not self.got_first_message:
            self.got_first_message = True

    def publish_point(self, x, y):
        self.point_publisher.publish(
            PointStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
                point=Point(x, y, 0)
            )
        )

    # def publish_wall_points(publisher, base_angle, offset_angle):
    #     publish_point()

    def run(self):
        r = rospy.Rate(50)

        while not self.got_first_message:
            r.sleep()

        while not rospy.is_shutdown():
            print 'Left Point: ' + str(self.left_wall_point)
            print 'Right Point: ' + str(self.right_wall_point)
            print 'angular: ' + str(self.right_wall_point - self.left_wall_point)

            fwd_msg = Twist(linear=Vector3(0.75, 0.0, 0.0), 
                            angular=Vector3(0.0, 0.0, self.k * (self.right_wall_point - self.left_wall_point)))
            self.movement_publisher.publish(fwd_msg)

            r.sleep()





WallFollower().run()

