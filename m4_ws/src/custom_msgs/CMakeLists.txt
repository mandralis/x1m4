cmake_minimum_required(VERSION 3.5)
project(custom_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add the custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TiltVel.msg"
  "msg/DriveVel.msg"
  "msg/InputRc.msg"
  "msg/TiltAngle.msg"
)

ament_package()
