cmake_minimum_required(VERSION 3.8)
project(turtlebot3_full_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# === BUILD FILTER NODE ===


install(TARGETS

  DESTINATION lib/${PROJECT_NAME}
)

# === INSTALL NON-C++ ASSETS ===
install(DIRECTORY
  launch
  maps
  rviz
  scripts
  test
  turtlebot3_full_bringup
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
include_directories(${EIGEN3_INCLUDE_DIRS})
ament_package()
