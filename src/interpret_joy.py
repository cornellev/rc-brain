#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def joy_to_twist(data):
	global max_turning_angle
	global max_vel

	turn = data.axes[3] * max_turning_angle  # Positive angle turns left, so counterclockwise
	drive = -.5 * (data.axes[2] - 1) * max_vel  # By default axis is 1, and goes to -1, so transform

	twist = Twist()
	twist.linear.x = drive  # LT
	twist.angular.z = turn  # R Stick
	pub.publish(twist)

# Intializes everything
def start():
	global pub
	global max_vel
	global max_turning_angle

	max_vel = 5
	max_turning_angle = 50

	pub = rospy.Publisher('rc_movement_twist', Twist, queue_size=10)

	rospy.Subscriber("joy", Joy, joy_to_twist)

	rospy.init_node('joy_interpreter')
	rospy.spin()

if __name__ == '__main__':
	start()
