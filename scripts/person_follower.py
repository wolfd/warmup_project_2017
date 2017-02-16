#!/usr/bin/env python2

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Twist, Vector3
import math


class PersonFollower(object):
    """
    The node that gets a position message and moves towards that position
    """
    def __init__(self):
        super(PersonFollower, self).__init__()
        rospy.init_node('perosn_follower')
        rospy.Subscriber('/visualization_messages/Centroid', Marker, self.get_target)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.target = None # The target that a the robot will move towards
        self.got_first_message = False # We don't want to move until we get the message where to go

        rospy.on_shutdown(self.stop)

    def get_target(self, msg):
        """
        Assign the class target object whenever we get a message from person_finder
        """
        self.target = msg.pose.position

        if not self.got_first_message:
            self.got_first_message = True

    def stop(self):
        """
        Tells the robot to stop moving.
        """
        self.publisher.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def run(self):
        """
        Main control loop of the robot.
        """
        # We run this slightly slower than person_finder
        r = rospy.Rate(40)

        # Dont do anything until we get a target message.
        while not self.got_first_message:
            r.sleep()


        while not rospy.is_shutdown():
            # Calc angle to target
            angle = math.atan2(self.target.y, self.target.x)
            # Calc distance to target
            distance = math.sqrt(self.target.y**2 + self.target.x**2)
            print 'distance: ' + str(distance)
            print 'angle: ' + str(angle)

            # The distance that the robot wants to be from the target
            optimal_dist = 0.5
            # This makes it so robot doesn't back/forward if there is no
            # object detected.
            if distance == 0.0:
                forward_vec = Vector3(0.0, 0.0, 0.0)
            else:
                # Speed is proportional to distance from optimal_dist
                forward_vec = Vector3(distance - optimal_dist, 0.0, 0.0)

            turn_msg = Twist(
                linear=forward_vec,
                angular=Vector3(0.0, 0.0, angle)
            )
            self.publisher.publish(turn_msg)

            r.sleep()

PersonFollower().run()

