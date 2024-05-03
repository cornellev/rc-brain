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
VEHICLE_WIDTH = 0.25
AUTOBRAKE_TIME = .25 # .45
AUTOBRAKE_DISTANCE = .2

MIN_COLLISIONS_FOR_BRAKE = 1
LIDAR_HORIZONTAL_OFFSET = .035 # From center of front axle

steering_angle = 0
velocity = 0
brake = Bool()
brake.data = False

def check_collision(data: LaserScan):
    invert_flag = 1
    num_collisions = 0

    if steering_angle < 0:
        invert_flag = -1

    turning_radius = VEHICLE_LENGTH / math.tan(invert_flag * steering_angle) if abs(steering_angle) > .01 else float('inf')

    rospy.loginfo("TURNING RADIUS: " + str(turning_radius))

    if turning_radius == float('inf'):
        for i in range(len(data.ranges)):
            theta = (data.angle_min + i * data.angle_increment)
            r = data.ranges[i]
            x = r * math.sin(theta) + LIDAR_HORIZONTAL_OFFSET
            y = r * math.cos(theta)

            if abs(x) < VEHICLE_WIDTH/2:
                dist_to_obstacle = y
                time_to_hit = dist_to_obstacle / velocity if velocity != 0 else float('inf')
                
                if dist_to_obstacle <= AUTOBRAKE_DISTANCE or time_to_hit <= AUTOBRAKE_TIME:
                    num_collisions += 1
    else:
        left_wheel_radius = turning_radius - VEHICLE_WIDTH/2
        right_wheel_radius = turning_radius + VEHICLE_WIDTH/2

        for i in range(len(data.ranges)):
            theta = (data.angle_min + i * data.angle_increment)
            r = data.ranges[i]
            x = invert_flag * (r * math.sin(theta) + LIDAR_HORIZONTAL_OFFSET)
            y = r * math.cos(theta)

            dist_to_obstacle = math.sqrt((x + turning_radius)**2 + y**2)

            if right_wheel_radius < dist_to_obstacle < left_wheel_radius or left_wheel_radius < dist_to_obstacle < right_wheel_radius:
                angle_from_center_to_obstacle = math.atan2(y, turning_radius + x)

                if angle_from_center_to_obstacle < 0:
                    angle_from_center_to_obstacle += 2 * math.pi
                
                angle_from_center_to_obstacle %= 2 * math.pi

                circum_dist_to_obstacle = turning_radius * angle_from_center_to_obstacle
                time_to_hit = circum_dist_to_obstacle / velocity if velocity != 0 else float('inf')

                if circum_dist_to_obstacle <= AUTOBRAKE_DISTANCE or time_to_hit <= AUTOBRAKE_TIME:
                    num_collisions += 1

    if num_collisions >= MIN_COLLISIONS_FOR_BRAKE:
        brake.data = True
    else:
        brake.data = False


def set_vars(data):
  global velocity
  global steering_angle

  velocity = data.velocity
  steering_angle = -math.radians(data.steering_angle)

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


    

