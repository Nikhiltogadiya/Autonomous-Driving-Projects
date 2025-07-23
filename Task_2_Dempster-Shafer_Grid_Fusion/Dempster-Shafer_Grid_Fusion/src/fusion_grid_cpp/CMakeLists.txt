cmake_minimum_required(VERSION 3.5)
project(fusion_grid_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# The rest is standard ROS 2 packaging description
add_executable(grid   
  src/lidar_grid.cpp
)
ament_target_dependencies(grid 
  rclcpp 
  sensor_msgs 
  geometry_msgs
  cv_bridge
)

target_link_libraries(grid
  ${OpenCV_LIBS}
)

install(TARGETS
  grid
  DESTINATION lib/${PROJECT_NAME})

ament_package()

