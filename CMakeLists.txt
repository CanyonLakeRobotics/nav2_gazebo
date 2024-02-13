cmake_minimum_required(VERSION 3.5)
project(nav2_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

# Ignition
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

include_directories(
  include
)


# Add cart name extractor
add_executable(cart_contact_detector src/cart_name_extractor.cpp)
ament_target_dependencies(
  cart_contact_detector
  rclcpp
  std_msgs
  ros_gz_interfaces
)

# Add fake latch plugin lib
add_library(FakeLatchPlugin SHARED src/fake_latch_plugin.cpp)
target_link_libraries(FakeLatchPlugin PUBLIC
                      ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
                      ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
                      ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
)
install(
  TARGETS FakeLatchPlugin
  DESTINATION lib/nav2_gazebo)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")


install(
  DIRECTORY description scripts launch rviz config world models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/reach_goal.py
  scripts/follow_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    cart_contact_detector
  RUNTIME DESTINATION lib/nav2_gazebo
  INCLUDES DESTINATION include/nav2_gazebo
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
