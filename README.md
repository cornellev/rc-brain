# rc-hardware

Hardware and sensor interfacing for RC cars. 

## Installation

### Dependencies
`rc_hardware_sensors` depends on `rc_localization` for certain message types.
You must have `rc_localization` cloned/installed before you begin.

You also must have `rosserial` and `rosserial_arduino` installed.
```sh
sudo apt-get install ros-noetic-rosserial
sudo apt-get install ros-noetic-rosserial-arduino
``` 

### Building 
1. `catkin_make` as usual. 
2. Run `catkin_make rc_hardware_sensors_firmware_firmware-upload` to upload the firmware.

## Running
1. Make sure all of your shells are sourced.
2. Make sure the port you are using is accessible and that you have read/write permissions `sudo chmod a+rw /dev/ttyUSB0`
3. `rosrun rosserial_python serial_node.py /dev/ttyUSB0` to connect the Nano to `roscore` running on your computer.
