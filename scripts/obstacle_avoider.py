#!/usr/bin/env python2

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Twist, Vector3
from nav_msgs.msg import Odometry
from obstacles import Goal
import tf
import math
import rospy

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

class ObstacleAvoider(object):
    """Avoids obstacles"""
    def __init__(self):
        super(ObstacleAvoider, self).__init__()
        
        rospy.init_node('obstacle_avoider')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)

        # Coordinates of goal
        x_g = 1
        y_g = 0
        r = 0.2
        s = 3
        alpha = 0.2

        self.goal = Goal(x_g, y_g, r, s, alpha)

        self.x = None
        self.y = None
        self.angle = None

        self.got_first_message = False

        rospy.on_shutdown(self.stop)

    def update_odometry(self, msg):
        current_pos = msg.pose.pose.position
        self.x = current_pos.x
        self.y = current_pos.y

        current_quat = msg.pose.pose.orientation
        orientation = tf.transformations.euler_from_quaternion(
            [current_quat.x, current_quat.y, current_quat.z, current_quat.w])
        self.angle = orientation[2]

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
            
            print 'x: ' + str(self.x)
            print 'y: ' + str(self.y)
            print 'angle: ' + str(self.angle)
            mag = self.goal.get_mag(self.x, self.y)
            theta = self.goal.get_angle(self.x, self.y)
            print 'theta: ' + str(theta)
            dtheta = angle_diff(theta,self.angle)
            print 'dtheta: ' + str(dtheta)

            if mag != 0:
                forward_msg = Twist(
                    # div by dtheta + 0.01 -> robot goes slower when turning
                    # 0.01 is to avoid division by 0
                    linear=Vector3(mag/ abs(dtheta + 0.01), 0, 0),
                    angular=Vector3(0.0, 0.0, 0.6 *dtheta)
                )
                self.publisher.publish(forward_msg)
            else:
                self.stop()
            
            r.sleep()

ObstacleAvoider().run()


        