#!/bin/bash

cd ~
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv ~/bin/arduino-cli /bin
echo "alias arduino-cli='/bin/arduino-cli'" >> ~/.bashrc
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install Servo
arduino-cli lib install Encoder
