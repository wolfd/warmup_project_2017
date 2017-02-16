#!/usr/bin/env python2

from geometry_msgs.msg import Twist, Vector3, PointStamped, Point
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from neato_node.msg import Bump
import numpy as np
import tf
import math

import smach
import rospy

class SquareDance(smach.State):
    """
    SquareDance is a 
    """
    def __init__(self, outcomes=['reached_end', 'hit_something']):
        super(SquareDance, self).__init__(outcomes=outcomes)
        self.left_front_triggered = 0
        self.right_front_triggered = 0

        self.position = None
        self.orientation = None

        self.starting_position = None
        self.starting_orientation = None

        self.running = False

        rospy.Subscriber('/bump', Bump, self.detect_bump)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.destination_publisher = rospy.Publisher('/destination', PointStamped, queue_size=10)

        rospy.on_shutdown(self.stop)

    def publish_destination(self, x, y, z):
        self.destination_publisher.publish(
            PointStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='odom'),
                point=Point(x, y, z)
            )
        )

    def convert_to_euler(self, x, y, z, w):
        quaternion = (x, y, z, w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return np.array([roll, pitch, yaw])

    def stop(self):
        self.publisher.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def detect_bump(self, msg):
        self.left_front_triggered = msg.leftFront
        self.right_front_triggered = msg.rightFront

        if self.left_front_triggered or self.right_front_triggered:
            print('stopping due to bump')
            self.running = False

    def update_odometry(self, msg):
        if self.position is None or self.orientation is None:
            pos = msg.pose.pose.position
            self.starting_position = np.array([pos.x, pos.y, pos.z])
            quat = msg.pose.pose.orientation
            self.starting_orientation = self.convert_to_euler(quat.x, quat.y, quat.z, quat.w)
        current_pos = msg.pose.pose.position
        self.position = np.array([current_pos.x, current_pos.y, current_pos.z])
        current_quat = msg.pose.pose.orientation
        self.orientation = self.convert_to_euler(current_quat.x, current_quat.y, current_quat.z, current_quat.w)

        if not self.running:
            self.running = True
    
    def get_angle(self):
        return -self.orientation[2]

    def transform_to_odom(self, destination_base_link):
        theta = self.get_angle()
        rot = np.matrix([[np.cos(theta), -1*np.sin(theta), 0],
                        [np.sin(theta) ,    np.cos(theta), 0], 
                        [0             ,  0              , 1]])
        # Should this be position or starting position?
        destination_odom = np.matrix(destination_base_link) * rot + self.position
        return destination_odom

    def distance_to(self, point):
        return np.linalg.norm(point - self.position)

    def go_forward(self, distance=1.0):
        r = rospy.Rate(50)
        destination_odom = self.transform_to_odom(np.array([distance, 0.0, 0.0]))

        move_starting_position = self.position

        while not rospy.is_shutdown() and self.running:
            self.publish_destination(destination_odom[0, 0], destination_odom[0, 1], destination_odom[0, 2])
            if self.distance_to(move_starting_position) < distance:
                fwd_msg = Twist(linear=Vector3(1.0, 0.0, 0.0))
                self.publisher.publish(fwd_msg)
            else:
                self.stop()
                return True
            r.sleep()
        return False

    def delta_angle(self, a, b):
        return ((b - a) + math.pi) % (math.pi * 2.0) - math.pi

    def rotate(self, angle):
        r = rospy.Rate(50)

        starting_angle = self.get_angle()
        final_angle = starting_angle + angle

        while not rospy.is_shutdown() and self.running:
            delta = self.delta_angle(self.get_angle(), final_angle)
            if abs(delta) >= math.pi / 100.0:
                turn_msg = Twist(angular=Vector3(0.0, 0.0, -delta * 1.0))
                self.publisher.publish(turn_msg)
            else:
                self.stop()
                return
            r.sleep()

    def execute(self, userdata):
        return self.run()

    def run(self):
        r = rospy.Rate(50)

        # Wait for the first odometry position update to come in
        while not self.running:
            r.sleep()

        for i in range(4):
            success = self.go_forward(distance=1.0)
            if not success:
                return 'hit_something'
            self.rotate(-math.pi / 2.0)
        return 'reached_end'

        print('done!')

if __name__ == '__main__':
    rospy.init_node('square_dance')
    SquareDance().run()
