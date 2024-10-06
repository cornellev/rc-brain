One repo to rule them all

### TELEOP
The joystick is connected to `/dev/input/event0` by default. To be able to access it, the user launching ros must have the `input` group.

## SERIAL
Arduino Uno seems to connect to `/dev/ttyACM0`
To run, the `serial-ros2` library must be cloned to the parent workspace.
The user launching ros must have the `dialout` group.

## CONTROLS
To run, the `sllidar_ros2` library must be cloned to the parent workspace