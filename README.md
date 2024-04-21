# rc-telelop

## Requires:

 - ros-noetic-joy
 - ros-noetic-ackermann-msgs

## Instructions:

Please follow the tutorial at
 - http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick


On your computer:
 - export ROS_IP='your_ip_on_LAN'
 - export ROS_MASTER_URI=http://mini-uno:11311
 - sudo chmod a+rw /dev/input/js0
 - rosparam set joy_node/dev "/dev/input/js0"
 - rosrun joy joy_node
On the pi:
 - rostopic echo joy

## Controls

*Make sure gamepad is in x-mode so triggers are analog input.*

 - Left Trigger is forward and Right Trigger is reverse
 - Right Joystick X-axis is steering angle

### Publish
 - Publishes an ackermann_msgs/AckermannDrive message to 'rc_movement_msg'
