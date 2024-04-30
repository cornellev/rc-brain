#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import math

min_angle_range_to_register = math.radians(10)
angle_min = math.radians(150)
angle_max = math.radians(210)
min_distance_to_register = .3 # m


def convert_polar_dist_to_vertical_dist(dist, angle):
  return dist * math.sin(angle)

def check_data_forward(data, start, i):
  end = start + i

  while end < len(data):
    curr = end - i
    found_obstacle = True
    while curr < end:
      if data[curr] > min_distance_to_register:
        found_obstacle = False
        break
      curr += 1
    if found_obstacle:
      return True

    end += 1

  return False

def interpret_scan(data):
  increment = data.angle_increment
  step_count = int(min_angle_range_to_register / increment)

  start = int(angle_min / increment)
  end = int(angle_max / increment)

  ranges = data.ranges[start:] + data.ranges[:end]

#   rospy.loginfo("Increment: " + str(increment))
#   rospy.loginfo("Step count: " + str(step_count))
#   rospy.loginfo("Start: " + str(start))
#   rospy.loginfo("End: " + str(end))
  # rospy.loginfo("Original len: " + str(len(data.ranges)))
#   rospy.loginfo("New len: " + str(len(ranges)))

  if check_data_forward(ranges, 0, step_count):
    rospy.loginfo("DETECTED OBSTACLE. AUTOBRAKING.")
    brake.data = True
    pub.publish(brake)
  else:
    brake.data = False
    pub.publish(brake)

if __name__ == '__main__':
  brake = Bool()
  brake.data = False

  rospy.init_node('autobrake')

  sub = rospy.Subscriber('scan', LaserScan, interpret_scan)
  pub = rospy.Publisher('autobrake', Bool, queue_size=1)

  rospy.spin()


