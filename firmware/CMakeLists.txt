cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(rover
  SKETCH speed_control
  LIBS PID_v1
  BOARD uno
  PORT /dev/ttyACM0
)
