#!/usr/bin/env python2

from __future__ import division

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point, Vector3, Pose
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import rospy
import tf
import time
import numpy as np

class PersonFinder(object):
    def __init__(self):
        super(PersonFinder, self).__init__()
        rospy.init_node('person_finder')
        self.listener = tf.TransformListener(10)

        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan)
        self.box_publisher = rospy.Publisher('/visualization_messages/Box', Marker, queue_size=10)
        self.point_publisher = rospy.Publisher('/visualization_messages/Marker', Marker, queue_size=10)
        self.centroid_publisher = rospy.Publisher('/visualization_messages/Centroid', Marker, queue_size=10)
        
        self.got_first_message = False
        self.scan_msg = None
        self.detection_length = 1
        self.detection_width = 1

    def process_scan(self, msg):
        # Get the angles of the lazer scan data
        thetas = np.array(msg.channels[1].values)
        # We only want what's in front of us
        target_indices = np.logical_or(thetas < 90, thetas > 270)

        target_points = np.array(msg.points)[target_indices]
        msg.points = target_points

        self.scan_msg = msg

        if not self.got_first_message:
            self.got_first_message = True

    def publish_box(self, length, width):
        far_left = Point(length, -1 * width/2.0, 0)
        far_right = Point(length,     width/2.0, 0)
        close_left = Point(0, -1 * width/2.0, 0)
        close_right = Point(0, width/2.0, 0)
        box = [far_left, far_right, close_right, close_left, far_left]
        marker = Marker(
            type=Marker.LINE_STRIP,
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='base_link'
            ),
            points=box,
            scale=Vector3(0.01, 0.2, 0.2),
            color=ColorRGBA(0.0, 1.0, 0.0, 1.0)
        )
        self.box_publisher.publish(marker)

    def publish_points(self, points):
        marker = Marker(
            type=Marker.POINTS,
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='base_link'
            ),
            points=points,
            scale=Vector3(0.05, 0.05, 0.05),
            color=ColorRGBA(1.0, 1.0, 0.0, 1.0)
        )
        self.point_publisher.publish(marker)

    def publish_centroid(self, point):
        marker = Marker(
            type=Marker.SPHERE,
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='base_link'
            ),
            pose=Pose(
                position=point
            ),
            scale=Vector3(0.05, 0.05, 0.05),
            color=ColorRGBA(0.0, 1.0, 1.0, 1.0)
        )
        self.centroid_publisher.publish(marker)

    def in_box(self, point, box_length, box_width):

        # The x direction in baselink points forward from the robot
        x_lower_lim = 0
        x_upper_lim = box_length
        y_lower_lim = -1 * box_width/2.0
        y_upper_lim = box_width/2.0

        in_x_bounds = point.x > x_lower_lim and point.x < x_upper_lim
        in_y_bounds = point.y > y_lower_lim and point.y < y_upper_lim
        return in_x_bounds and in_y_bounds

    def to_base_link(self, msg):
        now = rospy.Time.now()

        new_msg = PointCloud(
            header=Header(
                stamp=now, 
                frame_id='odom'
            ), 
            points = msg.points)

        try:
            self.listener.waitForTransform('base_link', 'odom', now, rospy.Duration(4.0))
            return self.listener.transformPointCloud('base_link', new_msg)
        except (tf.LookupException, tf.ConnectivityException):
            print 'Time lookup Exception!'

    def get_centroid(self, points):
        if len(points) == 0:
            return None

        return Point(
            x = sum([p.x for p in points]) / len(points),
            y = sum([p.y for p in points]) / len(points),
            z = sum([p.z for p in points]) / len(points)
        )

    def run(self):
        r = rospy.Rate(50)

        while not self.got_first_message:
            r.sleep()

        time.sleep(1)

        while not rospy.is_shutdown():
            
            scan_msg_base_link = self.to_base_link(self.scan_msg)

            in_scan_range = lambda p: self.in_box(p, self.detection_length, self.detection_width)
            points_in_range = filter(in_scan_range, scan_msg_base_link.points)

            self.publish_points(points_in_range)
            self.publish_box(self.detection_length, self.detection_width)

            self.publish_centroid(
                self.get_centroid(points_in_range)
            )

            r.sleep()

PersonFinder().run()