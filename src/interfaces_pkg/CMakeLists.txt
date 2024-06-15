cmake_minimum_required(VERSION 3.8)
project(interfaces_pkg)

find_package(ament_cmake REQUIRED)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define the message files
set(msg_files
  "msg/ElectricalMeasurements.msg"
)

# Generate the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_package()


