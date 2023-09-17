#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"

#include <jsoncpp/json/json.h> 
#include <optional>


// Complex custom types
using WorldPointer = shared_ptr<World>;
using RobotPointer = shared_ptr<Robot>;
using LidarPointer = shared_ptr<Lidar>;
using RobotsVector = vector<RobotPointer>;
using LidarsVector = vector<LidarPointer>;
// using IdRobotSharedMap = map<int, shared_ptr<Robot>>;
using IdRobotSharedMap = map<int, RobotPointer>;
using RobotsAndLidarsVector = tuple<vector<RobotPointer>,vector<LidarPointer>>;

// using RobotsAndLidarsVector = tuple<vector<Robot*>, vector<Lidar*>>;
// using WorldItemPointer = shared_ptr<WorldItem>;
// //using RobotsAndLidarsVector = tuple<vector<Robot*>,vector<Lidar*>>;
// using IdRobotMap = map<int, Robot*>;
// using RobotLidarMap = map<string, vector< shared_ptr<WorldItem>>>;


// Functions definitions
void killTerminal();
void clearTerminal();
string getGitRootPath();
int runShellCommand(string command);
Json::Value readJson(string in_path);
RobotsAndLidarsVector initSimEnv(Json::Value root, WorldPointer w, int& robot_counter);
