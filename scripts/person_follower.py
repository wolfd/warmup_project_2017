#!/usr/bin/env python2

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Twist, Vector3
import math


class PersonFollower(object):
    def __init__(self):
        super(PersonFollower, self).__init__()
        rospy.init_node('perosn_follower')
        rospy.Subscriber('/visualization_messages/Centroid', Marker, self.get_target)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.target = None
        self.got_first_message = False

        rospy.on_shutdown(self.stop)

    def get_target(self, msg):
        self.target = msg.pose.position

        if not self.got_first_message:
            self.got_first_message = True

    def stop(self):
        self.publisher.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def run(self):
        r = rospy.Rate(40)

        while not self.got_first_message:
            r.sleep()

        while not rospy.is_shutdown():
            angle = math.atan2(self.target.y, self.target.x)
            distance = math.sqrt(self.target.y**2 + self.target.x**2)
            print 'distance: ' + str(distance)
            print 'angle: ' + str(angle)

            optimal_dist = 0.5
            # This makes it so robot doesn't back/forward if there is no
            # object detected.
            if distance == 0.0:
                forward_vec = Vector3(0.0, 0.0, 0.0)
            else:
                forward_vec = Vector3(distance - optimal_dist, 0.0, 0.0)

            turn_msg = Twist(
                linear=forward_vec,
                angular=Vector3(0.0, 0.0, angle)
            )
            self.publisher.publish(turn_msg)

            r.sleep()

PersonFollower().run()

