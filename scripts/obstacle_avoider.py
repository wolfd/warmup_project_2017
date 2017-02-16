#!/usr/bin/env python2

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from obstacles import Goal, Obstacle
from std_msgs.msg import Header, ColorRGBA

import smach

import tf
import math
import rospy
import numpy as np

def angle_normalize(z):
    """
    convenience function to map an angle to the range [-pi,pi]
    """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """
    Calculates the difference between angle a and angle b (both should be in radians)
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

class ObstacleAvoider(smach.State):
    """
    ObstacleAvoider is a ROS node or smach state that instructs the NeatoAvoids obstacles
    """
    def __init__(self, outcomes=['reached_goal', 'timed_out']):
        super(ObstacleAvoider, self).__init__(outcomes=outcomes)
        
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan)
        self.marker_publisher = rospy.Publisher('/obstacle_vector', MarkerArray, queue_size=10)
        
        # Coordinates of goal
        goal_x = 1.0 # meters
        goal_y = 0.0
        goal_radius = 0.2
        goal_limit = 3.0 # meters, after which the robot will have a constant pull toward the goal
        goal_strength = 0.3 # multiplicitive factor that determines the strength of the pull

        self.goal = Goal(x_g, y_g, goal_radius, s, goal_strength)

        # Neato coordinates
        self.x = None
        self.y = None
        self.angle = None

        # Flags to determine when to not try to use data we don't have
        self.got_first_odom = False
        self.got_first_scan = False

        # on SIGINT & node shutdown, tell the Neato to stop
        rospy.on_shutdown(self.stop)

    def process_scan(self, msg):
        """
        Callback for the laser scanner data, sets the data on the object
        """
        self.scan_msg = msg

        if not self.got_first_scan:
            self.got_first_scan = True

    def create_sphere(self, number, point):
        """
        Helper method to create a sphere marker at the current ROS time
        """
        return Marker(
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

    def create_arrow(self, number, pos, quat, mag, color=ColorRGBA(1.0, 1.0, 0.0, 0.75)):
        """
        Helper method to create an arrow marker at the current ROS time
        """
        return Marker(
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

    def update_odometry(self, msg):
        """
        Callback for odometry information. Converts the quaternion into Euler angles for easier use by mere mortals
        """
        current_pos = msg.pose.pose.position
        self.x = current_pos.x
        self.y = current_pos.y

        current_quat = msg.pose.pose.orientation
        orientation = tf.transformations.euler_from_quaternion(
            [current_quat.x, current_quat.y, current_quat.z, current_quat.w])
        self.angle = orientation[2]

        if not self.got_first_odom:
            self.got_first_odom = True

    def stop(self):
        """
        Publishes a message to the Neato to stop moving
        """
        self.publisher.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def to_quat(self, theta):
        """
        Takes a yaw theta value, and makes a quaternion
        """
        quat = tf.transformations.quaternion_from_euler(
            0, 0, theta)
        return Quaternion(quat[0], quat[1], quat[2], quat[3])

    def sum_forces(self, x, y, obstacles):
        """
        Takes all of the obstacles and the goals and sums them into a single 2D
        vector
        """
        y_mag = 0
        x_mag = 0
        for item in obstacles:
            angle = item.get_angle(x, y)
            mag = item.get_mag(x, y)
            dx = mag * math.cos(angle)
            dy = mag * math.sin(angle)
            x_mag += dx
            y_mag += dy

        return np.array([x_mag, y_mag])

    def to_mag_and_angle(self, vector):
        """
        Takes a 2D vector and returns a tuple of the magnitude and the angle
        """
        return np.linalg.norm(vector), math.atan2(vector[1], vector[0])

    def to_vector(self, mag, angle):
        """
        Takes a magnitude and an angle and returns the 2D vector representation
        """
        return np.array([
            math.cos(angle) * mag,
            math.sin(angle) * mag
        ])

    def get_motion_vector(self, obstacle_vector):
        """
        Takes the summed obstacle vector, and returns a Twist object of what the
        obstacle avoider "thinks" that the Neato's motion should be.

        Uses a dot product of the robot's direction and the summed obstacle
        vector to figure out if the robot should back up or go forward, and
        turns according to the angle difference between the robot and the
        obstacle vector, multiplied by a constant
        """
        robot_direction = self.to_vector(1.0, self.angle)

        obstacle_force_mag, obstacle_force_angle = self.to_mag_and_angle(obstacle_vector)
        dtheta = angle_diff(obstacle_force_angle, self.angle)
        turn_mag = 0.6 * dtheta

        return Twist(
            linear=Vector3(
                np.dot(robot_direction, obstacle_vector),
                0.0,
                0.0
            ),
            angular=Vector3(
                0.0,
                0.0,
                turn_mag
            )
        )

    def execute(self, userdata):
        """
        For smach, this calls the run method with a set timeout so it will give
        up and move onto another state if that has been set up in the state
        machine.
        """
        return self.run(timeout=30)

    def run(self, timeout=None):
        """
        The main loop of this node/state.

        Takes scan data, processes it, creates visualizations for rviz, and
        publishes the motion vector to the robot, and determines if the robot
        has failed or reached its goal.
        """
        r = rospy.Rate(40)

        time_started = rospy.get_rostime()

        # Don't really start until we have data
        while not self.got_first_odom or not self.got_first_scan:
            r.sleep()

        while not rospy.is_shutdown():
            # populate obstacles with laser scan data
            points = self.scan_msg.points
            obstacles = [self.goal]
            for point in points:
                obstacles.append(Obstacle(point.x, point.y, 0.0001, 0.5, 0.5))
            
            obstacle_vector = self.sum_forces(self.x, self.y, obstacles)
            obstacle_force_mag, obstacle_force_angle = self.to_mag_and_angle(obstacle_vector)

            # get motion instructions for robot
            motion_vector = self.get_motion_vector(obstacle_vector)

            # Visualize Goal
            goal_viz = self.create_sphere(
                1,
                Point(self.goal.x, self.goal.y, 0)
            )

            # Visualize obstacle "force"
            obtacle_force_viz = self.create_arrow(
                0, 
                Point(self.x, self.y, 0),
                self.to_quat(obstacle_force_angle),
                obstacle_force_mag
            )

            # Visualize linear instructions being sent
            motion_instruction_viz = self.create_arrow(
                2, 
                Point(self.x, self.y, 0),
                self.to_quat(self.angle),
                motion_vector.linear.x,
                color=ColorRGBA(0.0, 1.0, 1.0, 0.75)
            )

            marker_array = MarkerArray(
                markers=[obtacle_force_viz, goal_viz, motion_instruction_viz]
            )
            self.marker_publisher.publish(marker_array)

            if obstacle_force_mag != 0:
                self.publisher.publish(motion_vector)
            else:
                self.stop()

            # exit (and return smach outcome) if we're within an acceptable
            # radius of the goal
            if self.goal.get_distance(self.x, self.y) < self.goal.radius * 2.0:
                return 'reached_goal'
            elif timeout is not None and (rospy.get_rostime() - time_started).secs > timeout:
                return 'timed_out'
            
            r.sleep()

if __name__ == '__main__':
    # we aren't in a state machine, init node
    rospy.init_node('obstacle_avoider')
    ObstacleAvoider().run()

