#!/usr/bin/env python2

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from obstacles import Goal, Obstacle
from std_msgs.msg import Header, ColorRGBA

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
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan)
        self.marker_publisher = rospy.Publisher('/obstacle_vector', MarkerArray, queue_size=10)
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
        self.got_first_scan = False

        rospy.on_shutdown(self.stop)

    def process_scan(self, msg):
        self.scan_msg = msg

        if not self.got_first_scan:
            self.got_first_scan = True

    def create_sphere(self, number, point):
        marker = Marker(
            type=Marker.SPHERE,
            id = number,
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='odom'
            ),
            pose=Pose(
                position=point
            ),
            scale=Vector3(self.goal.radius, self.goal.radius, self.goal.radius),
            color=ColorRGBA(0.0, 1.0, 1.0, 1.0)
        )
        return marker

    def create_arrow(self, number, pos, quat, mag, color=ColorRGBA(1.0, 1.0, 0.0, 0.75)):
        marker = Marker(
            type=Marker.ARROW,
            id = number,
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='odom'
            ),
            pose=Pose(
                position = pos,
                orientation = quat
            ),
            scale=Vector3(mag, 0.05, 0.05),
            color=color
        )
        return marker
            

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

    def to_quat(self, theta):
        quat = tf.transformations.quaternion_from_euler(
            0, 0, theta)
        return Quaternion(quat[0], quat[1], quat[2], quat[3])

    def sum_forces(self, x, y, obstacles):
        y_mag = 0
        x_mag = 0
        for item in obstacles:
            angle = item.get_angle(x, y)
            mag = item.get_mag(x, y)
            dx = mag * math.cos(angle)
            dy = mag * math.sin(angle)
            x_mag += dx
            y_mag += dy

        final_mag = math.sqrt(x_mag**2 + y_mag**2)
        final_angle = math.atan2(y_mag, x_mag)

        return final_mag, final_angle

    def run(self):
        r = rospy.Rate(40)

        while not self.got_first_message or not self.got_first_scan:
            r.sleep()

        while not rospy.is_shutdown():
            
            # populate obstacles
            points = self.scan_msg.points
            obstacles = [self.goal]
            for point in points:
                obstacles.append(Obstacle(point.x, point.y, 0.0001, 0.5, 0.5))
            
            print 'x: ' + str(self.x)
            print 'y: ' + str(self.y)
            print 'angle: ' + str(self.angle)
            res_mag, res_angle = self.sum_forces(self.x, self.y, obstacles)
            print 'force: ' + str(res_mag)
            print 'force_angle: ' + str(res_angle)
            dtheta = angle_diff(res_angle,self.angle)
            print 'dtheta: ' + str(dtheta)

            # Visualize force
            arrow = self.create_arrow(
                0, 
                Point(self.x, self.y, 0),
                self.to_quat(res_angle),
                res_mag
            )
            # Visualize Goal
            sphere = self.create_sphere(
                1,
                Point(self.goal.x, self.goal.y, 0))

            
            if dtheta > 0.05:
                forward_mag = 0
            else:
                forward_mag = 0.1 * res_mag / abs(dtheta + 0.01)
            turn_mag = 0.6 * dtheta

           
            # Visualize instructions being sent
            arrow2 = self.create_arrow(
                2, 
                Point(self.x, self.y, 0),
                self.to_quat(self.angle),
                forward_mag,
                color=ColorRGBA(0.0, 1.0, 1.0, 0.75)
            )

            marker_array = MarkerArray(markers=[arrow, sphere, arrow2])
            self.marker_publisher.publish(marker_array)


            if res_mag != 0:
                forward_msg = Twist(
                    # div by dtheta + 0.01 so robot goes slower when turning
                    # 0.01 is to avoid division by 0
                    linear=Vector3(forward_mag, 0, 0),
                    angular=Vector3(0.0, 0.0, turn_mag)
                )
                self.publisher.publish(forward_msg)
            else:
                self.stop()
            
            r.sleep()

ObstacleAvoider().run()


        