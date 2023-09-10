#include <iostream>
#include <fstream>
#include <cstdio>
#include <array>
#include <jsoncpp/json/json.h>
#include <string>
#include <vector>
#include <map> 
#include <tuple>
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "types.h"


int makeLaunchFile(std::string in_path, std::string out_path);
Json::Value readJson(std::string in_path);

std::string getGitRootPath();