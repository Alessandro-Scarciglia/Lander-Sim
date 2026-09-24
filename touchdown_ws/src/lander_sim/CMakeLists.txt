cmake_minimum_required(VERSION 3.8)
# Build the Gazebo plugins that connect the SDF model to ROS and custom physics.
# Each shared library below is loaded by name from a world or model SDF file.

project(lander_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies shared by the plugin targets
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# ROS-controlled thrust force plugin
add_library(gazebo_ros_thruster SHARED src/gazebo_ros_thruster.cpp)
ament_target_dependencies(gazebo_ros_thruster gazebo_ros rclcpp std_msgs gazebo_dev)


# Initial condition plugin
add_library(InitialVelocityPlugin SHARED src/InitialVelocityPlugin.cc)
ament_target_dependencies(InitialVelocityPlugin gazebo_dev gazebo_ros rclcpp)

# Depth-dependent terrain contact plugin
add_library(DynamicTerrainPlugin SHARED src/Dynamic_terrain_plugin.cc)
ament_target_dependencies(DynamicTerrainPlugin gazebo_dev gazebo_ros rclcpp)

# Landing-leg crush and rebound plugin
add_library(CrushableJointPlugin SHARED src/crushable_shockabs.cpp)
ament_target_dependencies(CrushableJointPlugin gazebo_dev gazebo_ros rclcpp geometry_msgs)


# Install plugin libraries where Gazebo can discover them
install(TARGETS  
  gazebo_ros_thruster  
  InitialVelocityPlugin
  DynamicTerrainPlugin
  CrushableJointPlugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
