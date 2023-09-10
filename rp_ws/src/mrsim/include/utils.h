#include <iostream>
#include <fstream>
#include <cstdio>
#include <array>
#include <jsoncpp/json/json.h> 
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "types.h"

// custom tuple of dictionaries
using WorldItemMap = std::map<int, std::shared_ptr<WorldItem>>;
using StringToWorldItemVectorMap = std::map<std::string, std::vector<std::shared_ptr<WorldItem>>>;
using InnerTuple = std::tuple<WorldItemMap, StringToWorldItemVectorMap>;
using DictTuple = std::tuple<InnerTuple>;


Json::Value readJson(std::string in_path);
DictTuple initSimEnv(Json::Value root, std::shared_ptr<World> w);
int makeLaunchFile(std::string in_path, std::string out_path);
std::string getGitRootPath();