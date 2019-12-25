cmake_minimum_required(VERSION 3.8)
project(hps_camera)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             sensor_msgs
             pcl_conversions
             pcl_ros
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy
)

set(INCLUDE_DIRS
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(PRIVATE_LINK_LIBS
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  libhps3d
)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/libhps3d)

set(CAMERA_CLIENT "ros_camera_client")
add_executable(${CAMERA_CLIENT}
  ${PROJECT_SOURCE_DIR}/src/CameraClientROSMain.cpp
  ${PROJECT_SOURCE_DIR}/src/CameraClientROS.cpp
  ${PROJECT_SOURCE_DIR}/src/DeviceHandler.cpp
)
target_include_directories(${CAMERA_CLIENT}
  PRIVATE ${INCLUDE_DIRS}
)

target_link_libraries(${CAMERA_CLIENT}
  PRIVATE ${PRIVATE_LINK_LIBS}
)
