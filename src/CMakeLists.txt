#-------------------------------------------------------------------------------
# @file: CMakeLists.txt
# @created on: March 18th, 2019
# @author: Ivana Collado
#
# @brief: CMake file for data_augmentation lib.
#-------------------------------------------------------------------------------
# Project name 
project(data_augmentation)
message(STATUS "VANTTEC: Building ${PROJECT_NAME} Library")

# External required packages 
find_package(OpenCV REQUIRED)
if(${OpenCV_FOUND})
  message("-- Found OpenCV version: " ${OpenCV_VERSION})
else()
  message("ERROR -- Could not find OpenCV")
endif()

###########
## Build ##
###########

# Internal required headers 
include_directories( 
  ${OpenCV_INCLUDE_DIRS}
  ${BUILD_INCLUDE_OUTPUT_DIRECTORY}
)
## Declare a C++ library
add_library(data_augmentation_lib
  include/data_augmentation.cpp
  include/data_augmentation.h
)
target_link_libraries(data_augmentation_lib
  ${OpenCV_LIBS}
)

# Add tests here
add_executable(data_augmentation_test 
  test/data_augmentation_test.cpp
)
target_link_libraries(data_augmentation_test
  data_augmentation_lib 
  ${OpenCV_LIBS}
)

# Add excecutable
add_executable(data_augmentation data_augmentation.cpp)
target_link_libraries(data_augmentation
  data_augmentation_lib
  ${OpenCV_LIBS}
)


