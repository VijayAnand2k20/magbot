cmake_minimum_required(VERSION 3.8)
project(interfaces_pkg)

find_package(ament_cmake REQUIRED)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define the message files
set(msg_files
  "msg/ElectricalMeasurements.msg"
  "msg/Angle.msg"
  "msg/JointSpace.msg"
  "msg/TaskSpace.msg"
)

# Generate the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs # Add std_msgs as a dependency
)

ament_package()


