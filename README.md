# Multi Robot ROS Simulator
Robot Programming Project, Francesco Nocera, Leonardo Colosi - Sapienza (AIRO) 2022/2023

---
## Table of content
1. [Introduction](#introduction)
2. [Requirements](#requirements)
3. [How to compile](#how-to-compile)
4. [How to run](#how-to-run)
5. [Project Structure](#project-structure)
6. [License](#license)
---

# Introduction
In this project we have built a 2D multi robot simulator with C++ and ROS. The application can simulate multiple robots and give the user the ablity to control each of them one at the time. Each robot is equiped whit a lidar sensor whose base scan can be visualized in RViz as a point cloud.    

# Requirements
In order to compile and execute this project it is required to install [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) on a [Ubuntu 20.04](https://releases.ubuntu.com/focal/) machine. This version of ubuntu should came with a `C++` and `CMake` compiler out of the box. If not it is required to install both. Also it is suggested to install the ` ros-noetic-desktop-full` version in order to have access to all the necessary ROS module used in the project.

# How to compile
There are several ways to complile this project. The easiest one, after cloning this repo on a local machine...
```code 
git clone https://github.com/airoprojects/robot-programming-nc.git
```
is to move inside the project workspace and run the bash script:
```code
cd ./rp_ws
./prj --c
```
If it is preferred to manually compile the code it is possible to use both `catkin_make` or `cmake`.
In the first case it is sufficient to run:
```code
cd ./rp_ws
source /opt/ros/noetic/setup.bash
catkin_make
source /deve/setup.bash
```
For the second case it is necessary to crete a build directory bedore compiling
```code
# In a CMake project
mkdir build
cd build
cmake ..
make
make install  # (optionally)
```
This last methos is discouraged because while it works perfectly fine for compiling a CMake project it will not automatically make a devel directory necessary to run the application.

# How to run
There are several ways to run the code as well. Once agein the easyest one would be to run:
```code
./prj --e
```
This command will automatically run the primary ROS node and all the secondary node. It will also provide the simulation node with a map and a list of items, both extarcted form a default copnfiguration file `config.json` contained inside the config directory.
To execute the simulation with a different configuration it is possible either to edit the `config.json` file or tu run the code as follow:
```code
# In a separate instace of the terminal
source /opt/ros/noetic/setup.bash
roscore
# In the terminal window used to run the code
rosrun mrsim mrsim_node my_config.json
```
Note that `my_config.json` should be placed inside the config directory. Also the name map provided in the configuration file should not contain path to the file, just the name, but the map file (png, JPG, JPEG) must be inside the map directory. 

# Project Structure
The core of the project is containded in `rp_ws/src/mrsim/`:

- `bin`: Directory containing node source files
  - `mrsim_node.cpp`: Source file for the mrsim node
  - `opkey_node.cpp`: Source file for the opkey node
- `CMakeLists.txt`: CMake file to manage the build process of the project
- `include`: Directory containing header files
  - `lidar.h`: Header file for the Lidar module
  - `robot.h`: Header file for the Robot module
  - `types.h`: Header file defining various types used in the project
  - `utils.h`: Header file containing utility functions
  - `world.h`: Header file for the World module
- `msg`: Directory containing message definition files
  - `rodom.msg`: Message definition file for odom messages
- `package.xml`: Package information file
- `src`: Directory containing source files for various modules
  - `lidar.cpp`: Source file for the Lidar module
  - `robot.cpp`: Source file for the Robot module
  - `utils.cpp`: Source file containing utility functions
  - `world.cpp`: Source file for the World module


# License
This project is licensed under the [GPL-3.0 License](LICENSE). Feel free to use and modify the code according to your needs.
