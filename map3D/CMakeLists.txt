cmake_minimum_required(VERSION 3.10)

project(Extrude)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create the executable
add_executable(extrude extrude.cpp)
target_link_libraries(extrude ${OpenCV_LIBS} ${PCL_LIBRARIES})
