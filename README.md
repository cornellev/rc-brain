# rc-hardware

Hardware and sensor interfacing for RC cars. 

## Installation

This module requires a recursive clone to build. The encoder firmware is in a separate repo so that it can be isolated and put on the arduino.

```git clone --recursive git@github.com:cornellev/rc-hardware.git```

### Dependencies
`rc_hardware_sensors` depends on `rc_localization_odometry` for certain message types.
You must have `rc_localization_odometry` cloned/installed before you begin.

You also must have `rosserial`, `rosserial_arduino`, and `ackermann_msgs` installed.
```sh
sudo apt-get install ros-noetic-rosserial
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-ackermann-msgs
```

### Building 
1. `catkin_make` as usual. 
2. Run `catkin_make rc_hardware_sensors_firmware_firmware-upload` to upload the firmware.

## Running
1. Make sure all of your shells are sourced with `devel/setup.bash` in your workspace.
2. Make sure the port you are using is accessible and that you have read/write permissions `sudo chmod a+rw /dev/ttyUSB0` (if running on a rpi, this step may be unnecessary.)
3. `rosrun rosserial_python serial_node.py /dev/ttyUSB0` to connect the Nano to `roscore` running on your computer.