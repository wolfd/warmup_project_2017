#!/usr/bin/env python2

import tty
import select
import sys
import termios

from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump

import rospy

class TeleOp(object):
	def __init__(self):
		super(TeleOp, self).__init__()
		self.settings = termios.tcgetattr(sys.stdin)

		rospy.init_node('my_tele_op')

		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		rospy.on_shutdown(self.stop)

	def stop(self):
		self.publisher.publish(
			Twist(linear=Vector3(0.0, 0.0, 0.0))
		)

	def get_key(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)

		key = sys.stdin.read(1)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

		return key
			
	def select_movement(self, key):
					# Go forward
		commands = {'i': Twist(linear = Vector3(1.0, 0.0, 0.0)),
					# Turn left
					'j': Twist(angular = Vector3(0.0, 0.0, 1.0)),
					# Turn right
					'l': Twist(angular = Vector3(0.0, 0.0, -1.0)),
					# Go backwards
					',': Twist(linear = Vector3(-1.0, 0.0, 0.0)),
					# Stop
					'k': Twist(linear = Vector3(0.0, 0.0, 0.0))}
		if key in commands:
			return commands[key]


	def run(self):
		r = rospy.Rate(50)
		key = self.get_key()
		while not rospy.is_shutdown() and key != '\x03':
			key = self.get_key()
			movement_msg = self.select_movement(key)
			self.publisher.publish(movement_msg)
			r.sleep()
		
		self.stop()

TeleOp().run()





