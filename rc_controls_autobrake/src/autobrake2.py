#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Bool
# from sensor_msgs.msg import LaserScan
import math
# from rc_localization_odometry.msg import SensorCollect
import matplotlib.pyplot as plt

VEHICLE_LENGTH = 1
VEHICLE_WIDTH = 0.8
AUTOBRAKE_TIME = .7

LIDAR_START_ANGLE = 0

def turning_radius(steering_angle):
  if steering_angle == 0:
    return 1000000000000

  return VEHICLE_LENGTH / math.tan(steering_angle)

def check_collision(data, steering_angle, velocity):
  collisions = 0
  flag = 1

  if (steering_angle < 0):
    flag = -1
    steering_angle = -steering_angle

  turning_radius_center = turning_radius(steering_angle)
  turning_radius_left_wheel = turning_radius_center - VEHICLE_WIDTH/2
  turning_radius_right_wheel = turning_radius_center + VEHICLE_WIDTH/2
  circle_center = (-turning_radius_center, 0)

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

      if time_to_collision < AUTOBRAKE_TIME:
        collisions += 1

  return collisions

class Data:
  ranges = []
  angle_increment = 0

data = Data()
data.ranges = [1, 1.2, 1, .6, 1]
data.angle_increment = math.pi / 5

# Steering Circle Plot
steering_angle = math.pi / 4
turning_radius_center = turning_radius(steering_angle)
circle = plt.Circle((-turning_radius_center, 0), turning_radius_center, color='r', fill=False)
circle2 = plt.Circle((-turning_radius_center, 0), turning_radius_center + VEHICLE_WIDTH / 2, color='r', fill=False)
circle3 = plt.Circle((-turning_radius_center, 0), turning_radius_center - VEHICLE_WIDTH / 2, color='r', fill=False)

# Obstacles Plot
angles = [i * data.angle_increment + LIDAR_START_ANGLE for i in range(len(data.ranges))]

distances = data.ranges
obstacle_x = [distances[i] * math.cos(angles[i]) for i in range(len(distances))]
obstacle_y = [distances[i] * math.sin(angles[i]) for i in range(len(distances))]

plt.figure(figsize=(8, 6))
ax = plt.gca()
ax.add_patch(circle)
ax.add_patch(circle2)
ax.add_patch(circle3)
plt.plot(obstacle_x, obstacle_y, 'bo')
ax.set_aspect('equal', adjustable='box')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Steering Circle and Obstacles')
plt.grid(True)
plt.show()

print(check_collision(data, math.pi/4, 2))