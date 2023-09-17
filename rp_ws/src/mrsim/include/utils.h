#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"

#include <jsoncpp/json/json.h> 
#include <optional>
#include <variant>

// Complex custom types
using WorldPointer = shared_ptr<World>;
using WorldItemPointer = shared_ptr<WorldItem>;
using RobotPointer = shared_ptr<Robot>;
using LidarPointer = shared_ptr<Lidar>;
using RobotsVector = vector<RobotPointer>;
using LidarsVector = vector<LidarPointer>;
using IdRobotSharedMap = map<int, RobotPointer>;
using RobotsAndLidarsVector = tuple<vector<RobotPointer>,vector<LidarPointer>>;

// Functions definitions
void killTerminal();
void clearTerminal();
string getGitRootPath();
int runShellCommand(string command);
Json::Value readJson(string in_path);
RobotsAndLidarsVector initSimEnv(Json::Value root, WorldPointer w, int& robot_counter);
