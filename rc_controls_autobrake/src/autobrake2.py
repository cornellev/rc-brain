#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import math
from rc_localization_odometry.msg import SensorCollect

VEHICLE_LENGTH = .3
VEHICLE_WIDTH = 0.25
AUTOBRAKE_TIME = .3

MIN_COLLISIONS_FOR_BRAKE = 5

LIDAR_START_ANGLE = math.pi

def turning_radius(steering_angle):
  if steering_angle == 0:
    return 1000000000000

  return VEHICLE_LENGTH / math.tan(steering_angle)

def test_collision(data):
  check_collision(data)

def check_collision(data):
  global velocity
  global steering_angle

  steering_angle = -steering_angle
  velocity = velocity
  brake = Bool()
  brake.data = False

  if velocity < 0:
    pub.publish(brake)
    return


  collisions = 0
  flag = 1

  if (steering_angle < 0):
    flag = -1
    steering_angle = -steering_angle

  turning_radius_center = turning_radius(steering_angle)
  turning_radius_left_wheel = turning_radius_center - VEHICLE_WIDTH/2
  turning_radius_right_wheel = turning_radius_center + VEHICLE_WIDTH/2
  circle_center = (-turning_radius_center, 0)

  rospy.loginfo("VELOCITY: " + str(velocity))
  rospy.loginfo("STEERING ANGLE: " + str(steering_angle))
  rospy.loginfo("TURNING RADIUS: " + str(turning_radius_center))

  increment = data.angle_increment

  for obs in range(len(data.ranges)):
    obstacle = (data.ranges[obs], obs * increment + LIDAR_START_ANGLE)
    obstacle_x = flag * obstacle[0] * math.cos(obstacle[1])
    obstacle_y = obstacle[0] * math.sin(obstacle[1])
    obstacle_center_dist = math.sqrt((obstacle_x - circle_center[0])**2 + (obstacle_y - circle_center[1])**2)
    obstacle_center_angle = math.atan2(obstacle_y - circle_center[1], obstacle_x - circle_center[0])

    if obstacle_center_angle < 0:
      obstacle_center_angle = 2*math.pi + obstacle_center_angle

    if turning_radius_right_wheel < obstacle_center_dist < turning_radius_left_wheel or turning_radius_left_wheel < obstacle_center_dist < turning_radius_right_wheel:
      circum_dist_to_obstacle_angle = turning_radius_center * obstacle_center_angle
      time_to_collision = (circum_dist_to_obstacle_angle / velocity) if velocity != 0 else float('inf')

      if time_to_collision < max(AUTOBRAKE_TIME * velocity * 2, AUTOBRAKE_TIME):
        collisions += 1

      if time_to_collision < 2:
        rospy.loginfo("TIME TO COLLISION: " + str(time_to_collision))

  if collisions > MIN_COLLISIONS_FOR_BRAKE:
    brake.data = True
    rospy.loginfo("DETECTED OBSTACLE. AUTOBRAKING.")

  pub.publish(brake)

def set_vars(data):
  global velocity
  global steering_angle

  velocity = data.velocity
  steering_angle = data.steering_angle

if __name__ == '__main__':
  steering_angle = 0
  velocity = 0

  rospy.init_node('autobrake')

  sub = rospy.Subscriber('scan', LaserScan, test_collision)
  sub = rospy.Subscriber('sensor_collect', SensorCollect, set_vars)

  pub = rospy.Publisher('autobrake', Bool, queue_size=1)

  rospy.loginfo("Autobrake node initialized.")

  rospy.spin()