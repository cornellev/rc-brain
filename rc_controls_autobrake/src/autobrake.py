#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import math
from rc_localization_odometry.msg import SensorCollect

min_angle_range_to_register = math.radians(10)
angle_min = math.radians(160)
angle_max = math.radians(200)
min_distance_to_register = 1.2 # m


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

  # ranges = data.ranges[start:] + data.ranges[:end]
  
  ranges = data.ranges[start:end]
  # rospy.loginfo(ranges)



#   rospy.loginfo("Increment: " + str(increment))
#   rospy.loginfo("Step count: " + str(step_count))
  # rospy.loginfo("Start: " + str(start))
  # rospy.loginfo("End: " + str(end))
  # rospy.loginfo("Original len: " + str(len(data.ranges)))
#   rospy.loginfo("New len: " + str(len(ranges)))

  if check_data_forward(ranges, 0, step_count):
    rospy.loginfo("DETECTED OBSTACLE. AUTOBRAKING.")
    brake.data = True
    pub.publish(brake)
  else:
    brake.data = False
    pub.publish(brake)

def set_range(data):
  global min_distance_to_register
  min_distance_to_register = max(data.velocity/2, .3)

  rospy.loginfo("New min_distance_to_register: " + str(min_distance_to_register))

if __name__ == '__main__':
  brake = Bool()
  brake.data = False

  rospy.init_node('autobrake')

  sub = rospy.Subscriber('scan', LaserScan, interpret_scan)
  pub = rospy.Publisher('autobrake', Bool, queue_size=1)
  sub = rospy.Subscriber('sensor_collect', SensorCollect, set_range)

  rospy.loginfo("Autobrake node initialized.")

  rospy.spin()


