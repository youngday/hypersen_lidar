cmake_minimum_required(VERSION 3.8)

project(libhps3d)

add_library(${PROJECT_NAME} SHARED IMPORTED GLOBAL)

set_target_properties(
  ${PROJECT_NAME} PROPERTIES IMPORTED_LOCATION
                             ${PROJECT_SOURCE_DIR}/lib/libhps3d.so)

set_target_properties(${PROJECT_NAME} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                 ${PROJECT_SOURCE_DIR}/include)
