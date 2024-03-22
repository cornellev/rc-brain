#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def scale(axis_value: float):
	global max_vel
	global min_vel

	# Scale to max_velocity if positive, scale to min_velocity if negative
	if axis_value > 0:
		return min(axis_value ** 1.5 * max_vel, max_vel)
	return max(abs(axis_value) ** 1.5 * min_vel, min_vel)

def joy_to_twist(data):
	rospy.loginfo("hello")
	twist = Twist()
	twist.linear.x = scale(data.axes[1])  # Left stick
	twist.angular.z = scale(data.axes[0])  # Right stick
	pub.publish(twist)

# Intializes everything
def start():
	global pub
	global max_vel
	global min_vel

	max_vel = 5
	min_vel = -5

	pub = rospy.Publisher('rc_movement_twist', Twist, queue_size=10)

	rospy.Subscriber("joy", Joy, joy_to_twist)

	rospy.init_node('joy_interpreter')
	rospy.spin()

if __name__ == '__main__':
	start()
