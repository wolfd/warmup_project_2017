#!/usr/bin/env python2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA

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

    def get_mag(self, r, s, xg, yg, x, y):
        d = math.sqrt((xg - x)**2 + (yg - y)**2)
        alpha = 2
        if d < r:
            return 0
        if d >= r and (s+r) >= d:
            return alpha * (d - r)
        else:
            return alpha * s

    def get_angle(self, xg, yg, x, y):
        theta = math.atan2(yg - y, xg - x)
        quat = tf.transformations.quaternion_from_euler(
            0, 0, theta)
        return Quaternion(quat[0], quat[1], quat[2], quat[3])

    def generate_arrow_field(self, rows, columns):
        magnitudes = np.zeros([rows,columns])
        index = 0
        arrows = []
        r = 0.75
        s = 3
        for (row, col), mag in np.ndenumerate(magnitudes):
            marker = self.create_arrow(
                index, 
                Point(row - rows/2, col - columns/2, 0),
                self.get_angle(5, 0, row - rows/2, col - columns/2,),
                0.1 * self.get_mag(r, s, 5, 0, row - rows/2, col - columns/2)
            )
            arrows.append(marker)
            index += 1
        return arrows   


        

    def run(self):
        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            pos = Point(1,1,0)
            quat = Quaternion(1,0,0,1)
            markers = self.generate_arrow_field( 11,11)
            marker_array = MarkerArray(markers=markers)
            self.publisher.publish(marker_array)
            


            r.sleep()

FieldPublisher().run()   