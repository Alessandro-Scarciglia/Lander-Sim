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

# Create the first library (Mass Updater)
add_library(gazebo_ros_mass_updater SHARED src/gazebo_ros_mass_updater.cpp)
ament_target_dependencies(gazebo_ros_mass_updater rclcpp gazebo_ros gazebo_dev std_msgs geometry_msgs)

# Create the second library (Thrusters)
add_library(gazebo_ros_thruster SHARED src/gazebo_ros_thruster.cpp)
ament_target_dependencies(gazebo_ros_thruster gazebo_ros rclcpp std_msgs gazebo_dev)


# Create the third library (Initial Velocity)
add_library(InitialVelocityPlugin SHARED src/InitialVelocityPlugin.cc)
ament_target_dependencies(InitialVelocityPlugin gazebo_dev gazebo_ros rclcpp)


install(TARGETS
  gazebo_ros_mass_updater
  gazebo_ros_thruster  
  InitialVelocityPlugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
