#pragma once

#include <string>
#include <vector>
#include <map> 
#include <tuple>
#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "mrsim/rodom.h"
#include "geometry_msgs/Twist.h"



using namespace std;


using IntPoint = Eigen::Vector2i;
using Point = Eigen::Vector2f;
using Pose = Eigen::Isometry2f;

