#!/usr/bin/env python2

from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump

import rospy

class EmergencyStop(object):
	def __init__(self):
		super(EmergencyStop, self).__init__()
		self.left_front_triggered = 0
		self.right_front_triggered = 0

		self.running = True

		rospy.init_node('emergency_stop')

		rospy.Subscriber('/bump', Bump, self.detect_bump)
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		rospy.on_shutdown(self.stop)

	def stop(self):
		self.publisher.publish(
			Twist(linear=Vector3(0.0, 0.0, 0.0))
		)

	def detect_bump(self, msg):
		self.left_front_triggered = msg.leftFront
		self.right_front_triggered = msg.rightFront
	
	def run(self):
		r = rospy.Rate(50)
		while not rospy.is_shutdown() and self.running:
			fwd_msg = Twist(linear=Vector3(1.0, 0.0, 0.0))
			self.publisher.publish(fwd_msg)
`
			if self.left_front_triggered == 1 or self.right_front_triggered == 1:
				self.running = False

			print(self.running, self.left_front_triggered, self.right_front_triggered)
			r.sleep()
		
		self.stop()

EmergencyStop().run()
