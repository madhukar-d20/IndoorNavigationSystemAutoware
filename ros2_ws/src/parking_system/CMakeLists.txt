cmake_minimum_required(VERSION 3.8)
project(parking_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER_IS_GNU)
  set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -Werror")
  set(CMAKE_C_FLAGS "-Wall -Wextra -Wpedantic -Werror")
endif()

# Find other required packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # Add if your msgs/srvs use geometry_msgs types
find_package(nav_msgs REQUIRED) # Add if your msgs/srvs use nav_msgs types
find_package(autoware_auto_vehicle_msgs REQUIRED) # Add if your msgs/srvs use autoware_auto_vehicle_msgs types

# --- IMPORTANT ADDITIONS FOR CUSTOM MESSAGES/SERVICES ---
find_package(rosidl_default_generators REQUIRED)

# This command tells ROS 2 to generate interfaces for your .msg and .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ParkingSpot.msg"
  "msg/ParkingStatusArray.msg"
  "srv/BookSpot.srv"
  # List any standard messages used within your custom msgs/srvs that are NOT already listed as find_package above.
  # For example, if ParkingSpot.msg used 'builtin_interfaces/msg/Time', you'd add 'builtin_interfaces' here.
  # Based on your current structure, geometry_msgs, nav_msgs, autoware_auto_vehicle_msgs are already found above.
  # So, 'DEPENDENCIES' might not be strictly needed here if your custom messages only use primitive types or types from packages already found.
)
# --------------------------------------------------------

# Install Python packages
ament_python_install_package(${PROJECT_NAME})

# Ensure dependencies for custom messages/services are linked
# This ensures that generated Python message files can be found at runtime
ament_target_dependencies(${PROJECT_NAME} rosidl_default_runtime)

ament_package()
