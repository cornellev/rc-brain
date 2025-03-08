#!/bin/sh

arduino-cli compile --fqbn arduino:avr:uno ~/src/rc-brain/hardware/Onboard
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno ~/src/rc-brain/hardware/Onboard
