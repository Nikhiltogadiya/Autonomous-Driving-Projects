cmake_minimum_required(VERSION 3.5)
project(acc_control_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(webots_interfaces REQUIRED)

# The rest is standard ROS 2 packaging description
add_executable(acc_agent
  src/acc_agent.cpp
)
ament_target_dependencies(acc_agent 
  rclcpp 
  std_msgs 
  webots_interfaces
)


install(TARGETS
  acc_agent 
  DESTINATION lib/${PROJECT_NAME})

ament_package()

