cmake_minimum_required(VERSION 3.10)
project(examples_rclcpp_minimal_publisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(perception_pcl REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(publisher_lambda lambda.cpp )
ament_target_dependencies(publisher_lambda rclcpp std_msgs sensor_msgs)

target_link_libraries(publisher_lambda yaml-cpp libhps3d641719_arm.so ${PCL_LIBRARIES})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  lib
  ${PCL_LIBRARY_DIRS}
  
)
add_definitions(
  ${PCL_DEFINITIONS}
)

install(TARGETS
  publisher_lambda
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
