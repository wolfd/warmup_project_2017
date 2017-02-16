#!/usr/bin/env python2

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA
from obstacles import Goal, Obstacle
from sensor_msgs.msg import PointCloud

import math
import numpy as np
import tf
import rospy

class FieldPublisher(object):
    """Displays a vector field"""
    def __init__(self):
        super(FieldPublisher, self).__init__()
        rospy.init_node('field_publisher')

        self.publisher = rospy.Publisher('/vector_field', MarkerArray, queue_size=10)
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan)

        self.got_first_message = False
        self.can_msg = None
        x_g = 1
        y_g = 0
        r = 0.2
        s = 3
        alpha = 0.2

        self.goal = Goal(x_g, y_g, r, s, alpha)

    def process_scan(self, msg):
        self.scan_msg = msg

        if not self.got_first_message:
            self.got_first_message = True
            

    def create_arrow(self, number, pos, quat, mag):
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
            color=ColorRGBA(1.0, 1.0, 0.0, 1.0)
        )
        return marker


    def to_quat(self, theta):
        quat = tf.transformations.quaternion_from_euler(
            0, 0, theta)
        return Quaternion(quat[0], quat[1], quat[2], quat[3])


    def generate_arrow_field(self, rows, columns, obstacles):
        magnitudes = np.zeros([len(rows),len(columns)])
        index = 0
        arrows = []
        for (row, col), mag in np.ndenumerate(magnitudes):
            y_mag = 0
            x_mag = 0
            # Obstacles also includes the goals
            for item in obstacles:
                angle = item.get_angle(rows[row], columns[col])
                mag = item.get_mag(rows[row], columns[col])
                dx = mag * math.cos(angle)
                dy = mag * math.sin(angle)
                x_mag += dx
                y_mag += dy

            final_mag = math.sqrt(x_mag**2 + y_mag**2)
            final_angle = math.atan2(y_mag, x_mag)

            marker = self.create_arrow(
                index, 
                Point(rows[row], columns[col], 0),
                self.to_quat(final_angle),
                final_mag
            )
            arrows.append(marker)
            index += 1
        return arrows   

    def run(self):
        r = rospy.Rate(50)

        while not self.got_first_message:
            r.sleep()

        while not rospy.is_shutdown():
            
            points = self.scan_msg.points
            obstacles = [self.goal]
            for point in points:
                obstacles.append(Obstacle(point.x, point.y, 0.0001, 0.5, 2))
            markers = self.generate_arrow_field(
                np.linspace(-3,3,10),
                np.linspace(-3,3,10), 
                obstacles)
            marker_array = MarkerArray(markers=markers)
            self.publisher.publish(marker_array)
            
            r.sleep()

FieldPublisher().run()   