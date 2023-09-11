cmake_minimum_required(VERSION 3.16)
project(mrsim)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

# In case you want to use a JSON format for the configuration file, feel free to use the jsoncpp lib
# find_package(jsoncpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    # Add required ROS packages to use in this project (ie. sensor_msgs, nav_msgs, etc)
    roscpp
    std_msgs
    geometry_msgs
    message_generation
)


add_message_files(
    FILES
    rodom.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS "include/"
    CATKIN_DEPENDS  message_runtime 
    roscpp

    # Add required ROS dependencies
)    


add_library(mrsim SHARED src/robot.cpp src/world.cpp src/lidar.cpp src/utils.cpp) 
target_include_directories(mrsim PUBLIC 
    include 
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(mrsim PUBLIC 
    ${OpenCV_LIBRARIES} 
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    jsoncpp_lib
)

# Add mrsim_node
add_executable(mrsim_node bin/mrsim_node.cpp)
target_include_directories(mrsim_node PUBLIC 
    include 
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(mrsim_node 
    mrsim
    ${catkin_LIBRARIES}
)

# Add robot_node
add_executable(opkey_node bin/opkey_node.cpp)
target_include_directories(opkey_node PUBLIC 
    include 
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(opkey_node
    mrsim
    ${catkin_LIBRARIES}
)