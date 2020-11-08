#-------------------------------------------------------------------------------
# @file: CMakeLists.txt
# @created on: March 18th, 2019
# @author: Ivana Collado
#
# @brief: CMake file for vanttec_classic_vision.
#-------------------------------------------------------------------------------
# Project name 
project(vanttec_classic_vision)
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
	include
)
## Declare a C++ library
add_library(data_augmentation_lib
  include/data_augmentation/data_augmentation.cpp
  include/data_augmentation/data_augmentation.h
)
target_link_libraries(data_augmentation_lib
  ${OpenCV_LIBS}
)

add_library(shape_detection_lib
  include/shape_detection/shape_detection.cpp
  include/shape_detection/shape_detection.h
)
target_link_libraries(shape_detection_lib
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

add_executable(shape_detection_test 
  test/shape_detection_test.cpp
)
target_link_libraries(shape_detection_test
  shape_detection_lib 
  ${OpenCV_LIBS}
)

# Add excecutable
add_executable(data_augmentation data_augmentation.cpp)
target_link_libraries(data_augmentation
  data_augmentation_lib
  ${OpenCV_LIBS}
)

# Add executable
add_executable(rename_images rename_augmented_images.cpp)
target_link_libraries(rename_images
  data_augmentation_lib
  ${OpenCV_LIBS}
)
