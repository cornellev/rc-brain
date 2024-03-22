Please follow the tutorial at
	http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

On your computer:
	sudo chmod a+rw /dev/input/js0
	rosparam set joy_node/dev "/dev/input/js0"
	rosrun joy joy_node
On the pi:
	rostopic echo joy
