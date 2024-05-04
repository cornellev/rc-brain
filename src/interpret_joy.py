#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

from gamepads import LogitechRead

class JoyInterpreter:
	def __init__(self, joystick_read_type):
		self.joystick_read_type = joystick_read_type
		self.movement_pub = rospy.Publisher('rc_movement_msg', AckermannDrive, queue_size=10)
		rospy.Subscriber("joy", Joy, self.joy_to_twist)

		self.deadzone = rospy.get_param(param_name='joy_interpreter/teleop/deadzone', default=0.1)
		self.max_turning_angle = rospy.get_param(param_name='joy_interpreter/teleop/max_turning_angle', default=20.0)
		self.max_velocity = rospy.get_param(param_name='joy_interpreter/teleop/max_velocity', default=1.5)

		rospy.loginfo(f"Parameter max_turning_angle={self.max_turning_angle}")
		rospy.loginfo(f"Parameter max_velocity={self.max_velocity}")
		rospy.loginfo(f"Parameter deadzone={self.deadzone}")

	def joy_to_twist(self, data):
		gamepad_data = self.joystick_read_type(data.axes, data.buttons)
		turn = -gamepad_data.get_right_stick_x() * self.max_turning_angle  # Positive angle turns left, so counterclockwise

		if abs(turn) < self.deadzone:   
			turn = 0.0

		drive = (gamepad_data.get_right_trigger() - gamepad_data.get_left_trigger()) * self.max_velocity  # By default axis is 1, and goes to -1, so transform

		msg = AckermannDrive()
		msg.steering_angle = turn
		msg.speed = drive 

		self.movement_pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('joy_interpreter')
	ji = JoyInterpreter(joystick_read_type=LogitechRead)
	rospy.spin()

