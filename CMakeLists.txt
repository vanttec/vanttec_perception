#------------------------------------------------------------------------------
# @file: CMakeLists.txt
# @created on: March 18th, 2019
# @author: Ivana Collado
#
#------------------------------------------------------------------------------
#Minimum CMake required 
cmake_minimum_required(VERSION 3.5)

# Set the project name and version
project(data_augmentation VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set path to binaries
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
message("----- VANTTEC: Binaries path: " ${EXECUTABLE_OUTPUT_PATH})

# Enable test / app compilation
set(VANTTEC_TEST True)

# Add subdirectories
add_subdirectory(src)