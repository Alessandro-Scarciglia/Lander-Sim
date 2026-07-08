cmake_minimum_required(VERSION 3.8)
project(lander_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the necessary libraries
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Thrusters
add_library(gazebo_ros_thruster SHARED src/gazebo_ros_thruster.cpp)
ament_target_dependencies(gazebo_ros_thruster gazebo_ros rclcpp std_msgs gazebo_dev)


# Initial Velocity
add_library(InitialVelocityPlugin SHARED src/InitialVelocityPlugin.cc)
ament_target_dependencies(InitialVelocityPlugin gazebo_dev gazebo_ros rclcpp)

# Dynamic Terrain
add_library(DynamicTerrainPlugin SHARED src/Dynamic_terrain_plugin.cc)
ament_target_dependencies(DynamicTerrainPlugin gazebo_dev gazebo_ros rclcpp)

# Crushable Shock Absorbers
add_library(CrushableJointPlugin SHARED src/crushable_shockabs.cpp)
ament_target_dependencies(CrushableJointPlugin gazebo_dev gazebo_ros rclcpp geometry_msgs)


install(TARGETS  
  gazebo_ros_thruster  
  InitialVelocityPlugin
  DynamicTerrainPlugin
  CrushableJointPlugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
