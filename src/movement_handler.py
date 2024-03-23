#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDrive

def drive_ackermann(msg):
	rospy.loginfo("RECEIVED:\n\n")
	rospy.loginfo(msg.steering_angle)
	rospy.loginfo(msg.speed)

def start():
	rospy.init_node('movement_handler')
	rospy.Subscriber("rc_movement_msg", AckermannDrive, drive_ackermann)
	rospy.spin()

if __name__ == '__main__':
	start()
