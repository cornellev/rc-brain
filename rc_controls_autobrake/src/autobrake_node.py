#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import math
from rc_localization_odometry.msg import SensorCollect
from ackermann_msgs.msg import AckermannDrive
import math

"""
R = L / tan(steering_angle)
Circle Center = (-R, 0)

Obstacle = (rsin(theta), rcos(theta))

Angle from center to obstacle = tan(O_y / (O_x + R))
Dist from center to obstacle = sqrt((O_x + R)^2 + O_y^2)

Circum dist to obstacle = R * angle from center to obstacle
Time to hit obstacle = circum dist to obstacle / velocity

NOTE: LIDAR'S 0 is forward, and angles increment clockwise
"""

VEHICLE_LENGTH = .3
VEHICLE_WIDTH = 0.2
AUTOBRAKE_TIME = .7 # .45
AUTOBRAKE_DISTANCE = .2

MIN_COLLISIONS_FOR_BRAKE = 3
LIDAR_ROTATIONAL_OFFSET = math.pi
LIDAR_HORIZONTAL_OFFSET = .035 # From center of front axle

steering_angle = 0
velocity = 0
target_velocity = 0
brake = Bool()
brake.data = False

def autobrake_time(vel):
    return AUTOBRAKE_TIME + vel * .1 + (.3 if vel > 1.8 else 0)

def check_collision(data: LaserScan):
    invert_flag = 1
    num_collisions = 0
    vel = max(velocity, target_velocity)  # Set velocity to the max of the current velocity and the target velocity to be safe

    if steering_angle < 0:  # If the steering angle negative, flip all coordinates across the x-axis to keep calculations consistent
        invert_flag = -1

    turning_radius = VEHICLE_LENGTH / math.tan(invert_flag * steering_angle) if abs(steering_angle) > .01 else float('inf')  # Calculate turning radius

    # Calculate turning radii of inside and outside wheels
    inner_wheel_radius = turning_radius - VEHICLE_WIDTH/2
    outer_wheel_radius = turning_radius + VEHICLE_WIDTH/2

    for i in range(len(data.ranges)):
        if data.ranges[i] < data.range_min or data.ranges[i] > data.range_max:
            continue

        dist_to_obstacle = float('inf')
        time_to_hit = float('inf')

        theta = (LIDAR_ROTATIONAL_OFFSET + data.angle_min + i * data.angle_increment)
        r = data.ranges[i]
        x = invert_flag * (r * math.sin(theta) + LIDAR_HORIZONTAL_OFFSET)
        y = r * math.cos(theta)

        if turning_radius == float('inf'):  # Straight line forward
            if abs(x) < VEHICLE_WIDTH/2:  # If the obstacle is within the width of the vehicle, then record dist and time to collision
                dist_to_obstacle = y  # Calculate distance to obstacle
                time_to_hit = dist_to_obstacle / vel if vel != 0 else float('inf')  # Calculate time to hit obstacle
            else:  # Otherwise, continue to next obstacle
                continue
        else:
            radius_dist_to_obstacle = math.sqrt((x + turning_radius)**2 + y**2)  # Calculate radius from center of movement circle to obstacle

            if outer_wheel_radius < radius_dist_to_obstacle < inner_wheel_radius or inner_wheel_radius < radius_dist_to_obstacle < outer_wheel_radius:  # Check if obstacle within bounds of wheel circles
                angle_from_center_to_obstacle = math.atan2(y, turning_radius + x)  # Calculate angle from center of movement circle to obstacle

                if angle_from_center_to_obstacle < 0:
                    angle_from_center_to_obstacle += 2 * math.pi
                
                angle_from_center_to_obstacle %= 2 * math.pi

                dist_to_obstacle = turning_radius * angle_from_center_to_obstacle  # Calculate circumferential distance to obstacle
                time_to_hit = dist_to_obstacle / vel if vel != 0 else float('inf')  # Calculate time to hit obstacle
            else:
                continue

        if dist_to_obstacle >= 0 and time_to_hit >= 0 and (dist_to_obstacle <= AUTOBRAKE_DISTANCE or time_to_hit <= autobrake_time(vel)):  # If the obstacle is within the autobrake distance or time, then increment the collision counter
            num_collisions += 1

    brake.data = num_collisions >= MIN_COLLISIONS_FOR_BRAKE  # If the number of collisions is greater than the minimum number of collisions for braking, then set the brake flag to True


def set_vars(data):
  global velocity
  global steering_angle

  velocity = data.velocity
  steering_angle = math.radians(data.steering_angle)

def set_targets(data):
  global target_velocity

  target_velocity = data.speed


if __name__ == '__main__':
    rospy.init_node('autobrake')

    sub = rospy.Subscriber('scan', LaserScan, check_collision)
    sub = rospy.Subscriber('sensor_collect', SensorCollect, set_vars)
    sub = rospy.Subscriber('rc_movement_msg', AckermannDrive, set_targets)

    pub = rospy.Publisher('autobrake', Bool, queue_size=1)

    rospy.loginfo("Autobrake node initialized.")

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        pub.publish(brake)
        rate.sleep()

