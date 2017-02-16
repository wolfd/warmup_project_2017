#!/usr/bin/env python2

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy

class PublishMarker(object):
    def __init__(self):
        super(PublishMarker, self).__init__()
        rospy.init_node('publish_marker')

        self.pub_marker = rospy.Publisher('/visualization_messages/Marker', Marker, queue_size=10)

        self.marker = Marker(
            type=Marker.SPHERE,
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='odom'
            ),
            pose=Pose(
                position=Point(x=1.0, y=2.0)
            ),
            scale=Vector3(1.0, 1.0, 1.0),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.5)
        )

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_marker.publish(self.marker)

            r.sleep()

PublishMarker().run()
