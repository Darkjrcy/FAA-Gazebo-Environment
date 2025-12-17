cmake_minimum_required(VERSION 3.8)
project(airplane_gazebo_plugin)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(gnss_multipath_plugin REQUIRED)
find_package(rosidl_default_generators REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Make the message avaliable for the plugin:
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AirplaneKineticModelInfo.msg"
  "msg/AvoidanceStates.msg"
  "msg/AdsbData.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

# Build the airplane_3D_movement plugin
add_library(airplane_3d_movement_gazebo_plugin SHARED
  src/airplane_3d_movement_gazebo_plugin.cpp)
target_include_directories(airplane_3d_movement_gazebo_plugin PUBLIC include)

ament_target_dependencies(airplane_3d_movement_gazebo_plugin
  "gazebo_dev"
  "gazebo_ros"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(airplane_3d_movement_gazebo_plugin "${cpp_typesupport_target}")


# Build the real-time factor plugin
add_library(real_time_factor_plugin SHARED src/real_time_factor_publisher.cpp)
ament_target_dependencies(real_time_factor_plugin
  "gazebo_ros"
  "rclcpp" 
  "std_msgs" # Add dependency on the message package
)


# Built the Waypoint follower executable:
add_executable(airplane_waypoint_follower
  src/airplane_waypoint_follower.cpp
)
ament_target_dependencies(airplane_waypoint_follower
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "std_srvs"
  "Eigen3"
  "rosidl_default_runtime"
)
add_dependencies(airplane_waypoint_follower
  ${PROJECT_NAME}__cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(airplane_waypoint_follower "${cpp_typesupport_target}")
target_include_directories(airplane_waypoint_follower PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${EIGEN3_INCLUDE_DIRS}
)
install(TARGETS airplane_waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)

# Build the DAA executor for an airplane:
add_executable(geometric_wp_detect_and_avoidance
  src/geometric_wp_detect_and_avoidance.cpp
)
ament_target_dependencies(geometric_wp_detect_and_avoidance
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "std_srvs"
  "Eigen3"
  "rosidl_default_runtime"
)
add_dependencies(geometric_wp_detect_and_avoidance
  ${PROJECT_NAME}__cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(geometric_wp_detect_and_avoidance "${cpp_typesupport_target}")
target_include_directories(geometric_wp_detect_and_avoidance PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${EIGEN3_INCLUDE_DIRS}
)
install(TARGETS geometric_wp_detect_and_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

# Build the ADSB sensor
add_executable(adsb_sensor
  src/adsb_sensor.cpp
)
ament_target_dependencies(adsb_sensor
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "Eigen3"
  "gnss_multipath_plugin"
  "rosidl_default_runtime"
)
add_dependencies(adsb_sensor
  ${PROJECT_NAME}__cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(adsb_sensor "${cpp_typesupport_target}")
target_include_directories(adsb_sensor PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${EIGEN3_INCLUDE_DIRS}
)
install(TARGETS adsb_sensor
  DESTINATION lib/${PROJECT_NAME}
)

# Install targets
install(TARGETS
  airplane_3d_movement_gazebo_plugin
  real_time_factor_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install share directories
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()