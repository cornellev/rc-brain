#!/bin/sh

arduino-cli compile --fqbn arduino:avr:uno ~/src/rc-brain/hardware/Onboard
arduino-cli upload -p /dev/arduino --fqbn arduino:avr:uno ~/src/rc-brain/hardware/Onboard
