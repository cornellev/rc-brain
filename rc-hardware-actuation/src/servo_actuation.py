#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDrive
from gpiozero import AngularServo


def read_param(param, default):
	
	if not rospy.has_param(param):
		rospy.logwarn(f"Did not find {param}, using default {default}")
		return default
    
	val = rospy.get_param(param)
	rospy.loginfo(f"Using parameter {param}=\"{val}\"")
	return val

max_pwm = read_param('max_steer_pwm', 0.30)
min_pwm = read_param('min_steer_pwm', -0.15)
forward_pwm = read_param('forward_steer_pwm', 0.08)

min_angle = read_param('min_steer_angle', -90)
max_angle = read_param('max_steer_angle', 90)

# PWM zero signal is not forwards, offset the PWM signal to be forwards
bias_angle = forward_pwm * max_angle
max_steering_angle = max_pwm * max_angle - bias_angle
min_steering_angle = min_pwm * max_angle - bias_angle

rospy.loginfo(f"Max Reachable Steering Angle {max_steering_angle}")
rospy.loginfo(f"Min Reachable Steering Angle {min_steering_angle}")

servo = AngularServo(
	12,
	min_angle=min_angle, max_angle=max_angle, initial_angle=bias_angle,
	min_pulse_width=0.5/1000.0, max_pulse_width=2.5/1000.0
)

rospy.loginfo(f"Max Steering Angle: {max_steering_angle}")
rospy.loginfo(f"Min Steering Angle: {min_steering_angle}")

def drive_ackermann(msg):
	value = msg.steering_angle
	if value < min_steering_angle:
		rospy.logwarn(f"Warning, {value} too low, below {min_steering_angle}. Using {min_steering_angle} instead.")
		value = min_steering_angle
	elif value > max_steering_angle:
		rospy.logwarn(f"Warning, ${value} too high, above {max_steering_angle}. Using {max_steering_angle} instead.")
		value = max_steering_angle

	value += bias_angle
	servo.angle = value

if __name__ == '__main__':
	rospy.init_node('servo_controller')
	rospy.Subscriber("rc_movement_msg", AckermannDrive, drive_ackermann)
	rospy.spin()

