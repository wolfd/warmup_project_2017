#!/usr/bin/env python2

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import rospy

class PersonFollower(object):
    def __init__(self):
    	super(PersonFollower, self).__init__()
        rospy.init_node('person_follower')

        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan)
        self.marker_publisher = rospy.Publisher('/visualization_messages/Marker', Marker, queue_size=10)

        self.got_first_message = False
        self.points = []

    def process_scan(self, msg):
        self.points = msg.points

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
            points=box
            ,
            scale=Vector3(0.01, 0.2, 0.2),
            color=ColorRGBA(0.0, 1.0, 0.0, 1.0)
        )
        self.marker_publisher.publish(marker)

    def run(self):
    	r = rospy.Rate(50)

    	while not self.got_first_message:
            r.sleep()

        while not rospy.is_shutdown():
        	self.publish_box(1, 0.75)
        	r.sleep()

PersonFollower().run()
