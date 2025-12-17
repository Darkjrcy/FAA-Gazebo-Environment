cmake_minimum_required(VERSION 3.8)
project(plane_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add models into the installation folder:
install(
  DIRECTORY 
  AirplaneCharacteristics
  Detection_encounters
  launch 
  worlds
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
