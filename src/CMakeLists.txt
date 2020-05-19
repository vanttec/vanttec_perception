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

# Internal required headers 
include_directories( 
  ${OpenCV_INCLUDE_DIRS}
  ${BUILD_INCLUDE_OUTPUT_DIRECTORY}
)

set(VANTTEC_HEADERS 
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}.h
)

set(VANTTEC_SOURCES 
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}.cpp
)

set(VANTTEC_LIBRARIES
  data_augmentation
)

add_library(${PROJECT_NAME} SHARED ${VANTTEC_SOURCES} ${VANTTEC_HEADERS})

target_link_libraries( ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${XLAB_LIBRARIES}
)


# Add tests here
set(TEST_EXEC data_augmentation_test)
set(TEST ${CMAKE_CURRENT_SOURCE_DIR}/test/${TEST_EXEC}.cpp)

# Unit tests
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(${TEST_EXEC} ${TEST})
target_link_libraries(${TEST_EXEC}
  ${PROJECT_NAME} 
  ${OpenCV_LIBS}
)


