#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import math
from rc_localization_odometry.msg import SensorCollect
from ackermann_msgs.msg import AckermannDrive

VEHICLE_LENGTH = .3
VEHICLE_WIDTH = 0.25
AUTOBRAKE_TIME = .2 # .45

MIN_COLLISIONS_FOR_BRAKE = 1

LIDAR_START_ANGLE = math.pi

def turning_radius(steering_angle):
  if abs(steering_angle) < .01:
    return 1000000000000

  return VEHICLE_LENGTH / math.tan(steering_angle)

def test_collision(data):
  check_collision(data)

def check_collision(data):
  global velocity
  global steering_angle
  global target_velocity
  global brake

  steering_angle = steering_angle
  velocity = velocity
  target_velocity = target_velocity
  brake = brake

  if target_velocity < 0:
    brake.data = False
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

  # rospy.loginfo("VELOCITY: " + str(velocity))
  # rospy.loginfo("STEERING ANGLE: " + str(steering_angle))
  # rospy.loginfo("TURNING RADIUS: " + str(turning_radius_center))

  angle_start = data.angle_min
  increment = data.angle_increment

  # min_obstacle = float('inf')

  for obs in range(len(data.ranges)):
    obstacle = (data.ranges[obs], obs * increment + angle_start)
    obstacle_x = flag * obstacle[0] * math.cos(obstacle[1])
    obstacle_y = obstacle[0] * math.sin(obstacle[1])
    obstacle_center_dist = math.sqrt((obstacle_x - circle_center[0])**2 + (obstacle_y - circle_center[1])**2)
    obstacle_center_angle = math.atan2(obstacle_y - circle_center[1], obstacle_x - circle_center[0])

    if obstacle_center_angle < 0:
      obstacle_center_angle = 2*math.pi + obstacle_center_angle

    if turning_radius_right_wheel < obstacle_center_dist < turning_radius_left_wheel or turning_radius_left_wheel < obstacle_center_dist < turning_radius_right_wheel:
      circum_dist_to_obstacle_angle = turning_radius_center * obstacle_center_angle
      time_to_collision = (circum_dist_to_obstacle_angle / max(velocity, target_velocity)) if max(velocity, target_velocity) != 0 else float('inf')

      # if circum_dist_to_obstacle_angle < min_obstacle:
      #   min_obstacle = circum_dist_to_obstacle_angle

      # rospy.loginfo("TIME TO COLLISION: " + str(time_to_collision))

      if time_to_collision < max(AUTOBRAKE_TIME * max(velocity, target_velocity), AUTOBRAKE_TIME):
      # # if time_to_collision < AUTOBRAKE_TIME:
        collisions += 1

      if circum_dist_to_obstacle_angle < 1:
        collisions += 1

      if collisions >= MIN_COLLISIONS_FOR_BRAKE:
        brake.data = True
        pub.publish(brake)
        rospy.loginfo("DETECTED OBSTACLE. AUTOBRAKING.")
        return
        pass

      # if time_to_collision < 2:
      #   rospy.loginfo("TIME TO COLLISION: " + str(time_to_collision))

  # if collisions > MIN_COLLISIONS_FOR_BRAKE:
  #   brake.data = True
  #   rospy.loginfo("DETECTED OBSTACLE. AUTOBRAKING.")

  brake.data = False

  # rospy.loginfo("MIN OBSTACLE: " + str(min_obstacle))
  pub.publish(brake)

def set_vars(data):
  global velocity
  global steering_angle

  velocity = data.velocity
  steering_angle = -data.steering_angle

def set_targets(data):
  global target_velocity

  target_velocity = data.speed

if __name__ == '__main__':
  steering_angle = 0
  velocity = 0
  target_velocity = 0

  brake = Bool()
  brake.data = False
  rospy.init_node('autobrake')

  sub = rospy.Subscriber('scan', LaserScan, test_collision)
  sub = rospy.Subscriber('sensor_collect', SensorCollect, set_vars)
  sub = rospy.Subscriber('rc_movement_msg', AckermannDrive, set_targets)

  pub = rospy.Publisher('autobrake', Bool, queue_size=1)

  rospy.loginfo("Autobrake node initialized.")

  rate = rospy.Rate(10) 

  while not rospy.is_shutdown():
    pub.publish(brake)
    rate.sleep()