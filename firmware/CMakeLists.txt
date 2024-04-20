project(firmware)
cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

include_directories(Encoder/)
include_directories(${ROS_LIB_DIR})

add_definitions(-DUSB_CON)

generate_arduino_firmware(firmware
  SRCS onboard.cpp ${ROS_LIB_DIR}/time.cpp Encoder/Encoder.cpp
  BOARD nano328
  PORT /dev/ttyUSB0
)