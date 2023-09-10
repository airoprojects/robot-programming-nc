#include <iostream>
#include <fstream>
#include <cstdio>
#include <array>
#include <jsoncpp/json/json.h> 
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "types.h"

using namespace std;
using WorldPointer = shared_ptr<World>;
using WorldItemMap = map<int, shared_ptr<WorldItem>>;
using RobotLidarMap = map<string, vector< shared_ptr<WorldItem>>>;
// using InnerTuple =  tuple<WorldItemMap, StringToWorldItemVectorMap>;
// using DictTuple =  tuple<InnerTuple>;


Json::Value readJson(string in_path);
RobotLidarMap initSimEnv(Json::Value root, shared_ptr<World> w, int& robot_counter);
int makeLaunchFile(string in_path,  string out_path);
string getGitRootPath();