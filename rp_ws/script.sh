#! /bin/bash
catkin_make
source devel/setup.bash


rosrun mrsim mrsim_node config.json
