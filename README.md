Dependencies:
In the catkin_ws/src directory please run:
 - sudo apt-get install libqt5serialport5-dev
 - sudo apt-get install ros-noetic-robot-localization
 - git clone --recursive https://github.com/ElettraSciComp/witmotion_IMU_ros.git src/witmotion_ros
Then run:
 - mv src/witmotion_ros .

When connecting the wit-motion IMU:
 - It interfaces over USB-RS232. As in, Ground -> Ground, VCC -> 5V, TX -> 232R, RX -> 232T.
 - Make sure the /dev/ttyUSB0 serial port is open to your user (so the wit-motion package can access serial output).
	 - sudo usermod -a $(whoami) -G dialout
	 - sudo chgrp dialout /dev/ttyUSB0		
	 - sudo chmod 660 /dev/ttyUSB0
To get info from the IMU:
 - roslaunch witmotion_ros witmotion.launch

To launch localization:
 - roslaunch rc-localization localization.launch

To launch the controller:
 - export ROS_MASTER_URI=http://mini-uno:11211
 - roslaunch rc-localization controller.launch # Not implemented yet
