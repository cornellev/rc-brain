project(firmware)
cmake_minimum_required(VERSION 2.8.3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

include_directories(Encoder/)
include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(firmware
  SRCS onboard.cpp ${ROS_LIB_DIR}/time.cpp Encoder/Encoder.cpp
  BOARD nano328
  PORT /dev/ttyUSB0
)