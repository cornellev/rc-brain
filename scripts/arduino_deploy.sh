#!/bin/sh

arduino-cli compile --fqbn arduino:avr:uno ~/src/brain/hardware/Onboard
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno ~/src/brain/hardware/Onboard
