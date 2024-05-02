#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Bool
# from sensor_msgs.msg import LaserScan
import math
# from rc_localization_odometry.msg import SensorCollect
import matplotlib.pyplot as plt

velocity = 1
steering_angle = -math.pi/4
VEHICLE_LENGTH = 1
VEHICLE_WIDTH = 0.5

def turning_radius(steering_angle):
  if steering_angle == 0:
    return 1000000000000

  return VEHICLE_LENGTH / math.tan(steering_angle)

def calculate_closest(obstacle, turning_radius, velocity):
  if turning_radius < 0:
    turning_radius = -turning_radius
    obstacle = (-obstacle[0], obstacle[1])

  angle_to_obstacle = math.tan(obstacle[1] / (obstacle[0] + turning_radius))
  traveled_dist_to_collision = angle_to_obstacle * turning_radius

  x_at_collision = turning_radius * math.cos(angle_to_obstacle) - turning_radius
  y_at_collision = obstacle[1]*(x_at_collision + turning_radius) / (obstacle[0] + turning_radius)

  dist_from_obstacle_to_intersection = math.sqrt((obstacle[0] - x_at_collision)**2 + (obstacle[1] - y_at_collision)**2)
  time_to_collision = (velocity / traveled_dist_to_collision) if traveled_dist_to_collision > 0 else 0
  
  return dist_from_obstacle_to_intersection, time_to_collision

obstacle = (1.7, 7*math.pi/4)  # r, theta
flag = 1

if (steering_angle < 0):
  flag = -1
  steering_angle = -steering_angle

turning_radius_center = turning_radius(steering_angle)
turning_radius_left_wheel = turning_radius_center - VEHICLE_WIDTH/2
turning_radius_right_wheel = turning_radius_center + VEHICLE_WIDTH/2
circle_center = (-turning_radius_center, 0)

obstacle_x = flag * obstacle[0] * math.cos(obstacle[1])
obstacle_y = obstacle[0] * math.sin(obstacle[1])

obstacle_center_dist = math.sqrt((obstacle_x - circle_center[0])**2 + (obstacle_y - circle_center[1])**2)
obstacle_center_angle = math.atan2(obstacle_y - circle_center[1], obstacle_x - circle_center[0])

print(obstacle_center_angle)

if obstacle_center_angle < 0:
  obstacle_center_angle = 2*math.pi + obstacle_center_angle

print(math.degrees(obstacle_center_angle))

if turning_radius_right_wheel < obstacle_center_dist < turning_radius_left_wheel or turning_radius_left_wheel < obstacle_center_dist < turning_radius_right_wheel:
  print("Obstacle is in the path of the vehicle")

circum_dist_to_obstacle_angle = turning_radius_center * obstacle_center_angle
time_to_collision = (circum_dist_to_obstacle_angle / velocity) if velocity != 0 else float('inf')
print(f"TIME TO COLLISION: {time_to_collision} s", )



### PLOTTING SEPARATOR


# Plot both circles
circle1 = plt.Circle((-turning_radius_center, 0), turning_radius_left_wheel, color='r', fill=False)
circle2 = plt.Circle((-turning_radius_center, 0), turning_radius_right_wheel, color='b', fill=False)
circle3 = plt.Circle((-turning_radius_center, 0), turning_radius_center, color='g', fill=False)
fig, ax = plt.subplots()

# Patch
ax.add_patch(circle1)
ax.add_patch(circle2)
ax.add_patch(circle3)

# Plot point at origin
ax.plot(0, 0, 'bo')

# Plot circle center
ax.plot(circle_center[0], circle_center[1], 'go')

# Plot vec from circle center to obstacle
ax.plot([circle_center[0], obstacle_x], [circle_center[1], obstacle_y], 'r')

plt.axis('equal')
plt.show()

print(turning_radius(steering_angle))