cmake_minimum_required(VERSION 3.8)
project(hydrobatic_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sam_msgs REQUIRED)
find_package(smarc_msgs REQUIRED)
find_package(dead_reckoning_msgs REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(message_filters REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_library(CPPUNITLITE_LIBRARY NAMES CppUnitLite PATHS /usr/local/lib)
find_package(piml_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# GTSAM stuff
# find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
message(GTSAM_INCLUDE_DIR="${GTSAM_INCLUDE_DIR}")
# CppTest
#include the headers
include_directories(${PROJECT_SOURCE_DIR}/include/)
# include launch files
include_directories(${PROJECT_SOURCE_DIR}/launch/)
#include GeographicLib
include_directories(${GeographicLib_INCLUDE_DIRS})
include_directories(${Python3_INCLUDE_DIRS})
# set python interpreter path 

add_executable(state_estimator src/state_estimator.cpp src/gtsam_graph.cpp src/BodyVelocityFactor.cpp src/BarometerFactor.cpp src/DvlFactor.cpp src/SamMotionModel.cpp src/SamMotionModelFactor.cpp)
ament_target_dependencies(state_estimator
  rclcpp
  ament_index_cpp
  sensor_msgs
  dead_reckoning_msgs
  std_msgs
  nav_msgs
  sam_msgs
  smarc_msgs
  tf2 
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  piml_msgs
  pybind11
  Python3
)
target_link_libraries(state_estimator
  gtsam
  Eigen3::Eigen
  GeographicLib::GeographicLib
  yaml-cpp   
  tf2_geometry_msgs::tf2_geometry_msgs
  Python3::Python

  )
add_executable(motion_model  src/motion_model.cpp src/SamMotionModel.cpp 
 )
ament_target_dependencies(motion_model
  rclcpp
  sensor_msgs
  piml_msgs
  dead_reckoning_msgs
  std_msgs
  nav_msgs
  sam_msgs
  smarc_msgs
  tf2 
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  piml_msgs
)

target_link_libraries(motion_model Python3::Python)

# add logger node executable
add_executable(logger src/loggerNode.cpp)
ament_target_dependencies(logger
  rclcpp
  nav_msgs
  tf2 
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  message_filters
  )
# Insatll the executablese, headers and launch files

install(PROGRAMS
${PROJECT_SOURCE_DIR}/scripts/sinus_pub.py
${PROJECT_SOURCE_DIR}/scripts/pressure_pub.py
DESTINATION lib/${PROJECT_NAME}
)



install(TARGETS
  # hydrobatic_localization
  state_estimator
  logger
  motion_model
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

 install(
   DIRECTORY config/
   DESTINATION share/${PROJECT_NAME}/config
 )




# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # add_executable(test_motion_model_factor
#     # test/SamMotionModelFactorTest.cpp
#     # src/SamMotionModelFactor.cpp
#     # src/SamMotionModel.cpp
#   # )
#   add_executable(test_barometer_factor
#     test/BarometerFactorTest.cpp
#     src/BarometerFactor.cpp
#   )
#   add_executable(test_dvl_factor
#     test/DvlFactorTest.cpp
#     src/DvlFactor.cpp
#   )
#   add_executable(test_bodyvelocity_factor
#     test/BodyVelocityFactorTest.cpp
#     src/BodyVelocityFactor.cpp
#   )
#   # TODO inlcude CppUnitLite headers in a better way
#   target_include_directories(test_barometer_factor PUBLIC /usr/local/include)
#   find_library(CPPUNITLITE_LIBRARY NAMES cppunitlite PATHS /usr/local/lib)
#   if(NOT CPPUNITLITE_LIBRARY)
#     message(FATAL_ERROR "Could not find the CppUnitLite library in /usr/local/lib")
#   endif()
#   target_link_libraries(test_dvl_factor
#     gtsam
#     ${CPPUNITLITE_LIBRARY}
#   )
#   target_link_libraries(test_barometer_factor
#     gtsam
#     ${CPPUNITLITE_LIBRARY}
#   )
#   # target_link_libraries(test_motion_model_factor
#   #   gtsam
#   #   ${CPPUNITLITE_LIBRARY}
#   # )
#   target_link_libraries(test_bodyvelocity_factor
#     gtsam
#     ${CPPUNITLITE_LIBRARY}
#   )
#   # ament_target_dependencies(test_motion_model_factor
#   #   rclcpp
#   # )
#   ament_add_test(test_dvl_factor COMMAND test_dvl_factor)
#   ament_add_test(test_barometer_factor COMMAND test_barometer_factor)
#   # ament_add_test(test_motion_model_factor COMMAND test_motion_model_factor)
#   ament_add_test(test_bodyvelocity_factor COMMAND test_bodyvelocity_factor)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
