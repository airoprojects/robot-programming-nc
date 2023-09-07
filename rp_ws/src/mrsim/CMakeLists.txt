cmake_minimum_required(VERSION 3.16)
project(mrsim)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# In case you want to use a JSON format for the configuration file, feel free to use the jsoncpp lib
# find_package(jsoncpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS

    # Add required ROS packages to use in this project (ie. sensor_msgs, nav_msgs, etc)
    roscpp
)

catkin_package(
    INCLUDE_DIRS
    "include/"
    CATKIN_DEPENDS
    roscpp

    # Add required ROS dependencies
)

add_library(mrsim SHARED src/robot.cpp src/world.cpp src/lidar.cpp)
target_include_directories(mrsim PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(mrsim PUBLIC ${OpenCV_LIBRARIES} Eigen3::Eigen)

# If you want to use jsoncpp library, remember to link it!
# target_link_libraries(<your_module> PUBLIC/PRIVATE jsoncpp_lib)
