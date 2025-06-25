cmake_minimum_required(VERSION 3.8)
project(BinPickingCellControl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(
  DIRECTORY include/BinPickingCellControl
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules (scripts)
install(PROGRAMS
  scripts/cell_sensors.py
  scripts/hmi.py 
  scripts/client.py 
  scripts/server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()