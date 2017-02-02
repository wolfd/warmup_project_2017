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
        self.offset_angle = 45
        self.base_angle = 90
        self.left_wall_point = 0
        self.right_wall_point = 0
        self.k = 2.0

        rospy.on_shutdown(self.stop)

    def stop(self):
        self.movement_publisher.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def process_scan(self, msg):
        temp_L = msg.ranges[(self.base_angle + self.offset_angle) % 360]
        if temp_L != 0.0:
          self.left_wall_point = temp_L
        temp_R = msg.ranges[(self.base_angle - self.offset_angle) % 360]
        if temp_R != 0.0:
            self.right_wall_point = temp_R

        if not self.got_first_message:
            self.got_first_message = True

    def publish_point(self, x, y):
        self.point_publisher.publish(
            PointStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
                point=Point(x, y, 0)
            )
        )

    def run(self):
        r = rospy.Rate(50)

        while not self.got_first_message:
            r.sleep()

        while not rospy.is_shutdown():
            print 'Left Point: ' + str(self.left_wall_point)
            print 'Right Point: ' + str(self.right_wall_point)

            angle = math.pi/4 - math.atan2(self.left_wall_point, self.right_wall_point)
            print 'angle: ' + str(angle)

            fwd_msg = Twist(linear=Vector3(0.75, 0.0, 0.0), 
                            angular=Vector3(0.0, 0.0, self.k * angle))
            self.movement_publisher.publish(fwd_msg)

            r.sleep()





WallFollower().run()

