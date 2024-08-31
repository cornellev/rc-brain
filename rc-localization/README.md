<<<<<<< HEAD
# rc-telelop

## Requires:

 - ros-noetic-joy
 - ros-noetic-ackermann-msgs

## Instructions:

Please follow the tutorial at
 - http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick


sudo chmod a+rw /dev/input/js0  
rosparam set joy_node/dev "/dev/input/js0"  
rosrun joy joy_node  
rostopic echo joy  
roslaunch rc-teleop teleop.launch  

## Controls

*Make sure gamepad is in x-mode so triggers are analog input.*

 - Left Trigger is forward and Right Trigger is reverse
 - Right Joystick X-axis is steering angle

### Publish
 - Publishes an ackermann_msgs/AckermannDrive message to 'rc_movement_msg'
=======
# rc_localization

There are two packages in this repository:
1. `rc_localization_imu` - random imu config stuff
2. `rc_localization_odometry` - encoder odometry code
3. `rc_localization_icp_benchmark` - tests for icp with the lidar
>>>>>>> localization/master
