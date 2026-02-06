cmake_minimum_required(VERSION 3.8)
project(parking_system_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages to build messages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Define the message and service files to be built
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ParkingSpot.msg"
  "msg/ParkingStatusArray.msg"
  "srv/BookSpot.srv"
  "srv/BookTimeSlot.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
