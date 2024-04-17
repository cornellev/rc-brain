# rc-hardware

Hardware and sensor interfacing for RC cars. 

## Installation

### Dependencies
`rc_hardware_sensors` depends on `rc_localization` for certain message types.
You must have `rc_localization` cloned/installed before you begin.

You also must have `rosserial` and `rosserial_arduino` installed. You can follow the
second section [here](https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).

### Building 
1. `catkin_make` before you start to make sure the required message types are generated.
2. Run `rosrun rosserial_client make_library.py ~/Arduino/libraries rc_localization` to copy header files for `rc_localization` messages to a directory where the Arduino CLI/IDE/whatever can access them.
3. You should be able to verify and upload `onboard.ino`.
