#!/usr/bin/env python2

from geometry_msgs.msg import Pose, Twist, Vector3, PointStamped, Point
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import tf
import math

import rospy

class WallFollower(object):
    """
    Class that encapsulates Wall Following ROS node.
    """
    def __init__(self):
        super(WallFollower, self).__init__()
        rospy.init_node('wall_follower')
        self.point_publisher = rospy.Publisher('/wall', PointStamped, queue_size=10)
        self.movement_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('/visualization_messages/Marker', Marker, queue_size=10)
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)

        self.got_first_message = False # Tell whether we recieved our first message
        self.offset_angle = 45 # the angle that each laser point is offset from center

        self.base_angle = 90 # The angle between the robot and the wall
        self.left_wall_point = 0 # Initial left point
        self.right_wall_point = 0 # Initial right point
        self.k = 2.0 # Turning constant for proportional control

        self.position = None
        self.orientation = None

        rospy.on_shutdown(self.stop)

    def stop(self):
        """
        Tells the robot to stop moving.
        """
        self.movement_publisher.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def process_scan(self, msg):
        """
        Processes a stabilized_lazer_scan message. Saves the distance to the left 
        and right wall points.
        """
        self.left_wall_point = msg.ranges[(self.base_angle + self.offset_angle) % 360]
        self.right_wall_point = msg.ranges[(self.base_angle - self.offset_angle) % 360]

        if not self.got_first_message:
            self.got_first_message = True

    def publish_points(self, xs, ys):
        """
        Helper function that masks marker publishing. Publishes a list of points.

        Args:
            xs: list of x points
            ys: list of y points 
        """
        marker = Marker(
            type=Marker.POINTS,
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='base_link'
            ),
            points=[Point(x, y, 0) for (x, y) in zip(xs, ys)]
            ,
            scale=Vector3(0.2, 0.2, 0.2),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.5)
        )
        self.marker_publisher.publish(marker)

    def publish_wall_points(self):
        """
        Publishes markers at the left and right wall point. 
        """
        d_l = self.left_wall_point
        theta_l = math.radians(self.base_angle + self.offset_angle)
        x_l = d_l * math.cos(theta_l)
        y_l = d_l * math.sin(theta_l)

        d_r = self.right_wall_point
        theta_r = math.radians(self.base_angle - self.offset_angle)
        x_r = d_r * math.cos(theta_r)
        y_r = d_r * math.sin(theta_r)

        self.publish_points([x_l, x_r], [y_l, y_r])

    def run(self):
        """
        Function that contains the logic of the main function.
        """
        r = rospy.Rate(50)

        # Don't move until we get the first message
        while not self.got_first_message:
            r.sleep()

        while not rospy.is_shutdown():
            print 'Left Point: ' + str(self.left_wall_point)
            print 'Right Point: ' + str(self.right_wall_point)

            # Calculate the angle between the robot and the wall
            angle = math.pi/4 - math.atan2(self.left_wall_point, self.right_wall_point)
            print 'angle: ' + str(angle)

            # Dont move while we're getting faulty points
            if self.left_wall_point == 0.0 or self.right_wall_point == 0.0:
                print 'not communicating properly... stopping'
                self.stop()
            else:
                self.publish_wall_points()
                fwd_msg = Twist(linear=Vector3(0.75, 0.0, 0.0), 
                            angular=Vector3(0.0, 0.0, self.k * angle))
                self.movement_publisher.publish(fwd_msg)

            r.sleep()

WallFollower().run()

