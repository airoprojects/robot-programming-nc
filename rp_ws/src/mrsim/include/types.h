#pragma once

#include <map> 
#include <tuple>
#include <string>
#include <vector>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

using namespace std;

using IntPoint = Eigen::Vector2i;
using Point = Eigen::Vector2f;
using Pose = Eigen::Isometry2f;