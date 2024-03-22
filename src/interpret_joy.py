#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

def joy_to_twist(data):
	global max_turning_angle
	global max_vel

	turn = data.axes[3] * max_turning_angle  # Positive angle turns left, so counterclockwise
	drive = -.5 * (data.axes[2] - 1) * max_vel  # By default axis is 1, and goes to -1, so transform

	msg = AckermannDriveStamped()
	msg.header.stamp = rospy.Time.now()
	msg.header.steering_angle = turn  # LT
	msg.header.speed = drive  # R Joy
	pub.publish(msg)

# Intializes everything
def start():
	global pub
	global max_vel
	global max_turning_angle

	max_vel = 5
	max_turning_angle = 50

	pub = rospy.Publisher('rc_movement_msg', AckermannDriveStamped, queue_size=10)

	rospy.Subscriber("joy", Joy, joy_to_twist)

	rospy.init_node('joy_interpreter')
	rospy.spin()

if __name__ == '__main__':
	start()
