#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"

#include <jsoncpp/json/json.h> 
#include <optional>

using WorldPointer = shared_ptr<World>;
using WorldItemPointer = shared_ptr<WorldItem>;
using RobotPointer = shared_ptr<Robot>;
using LidarPointer = shared_ptr<Lidar>;
using WorldItemPointer = shared_ptr<WorldItem>;
//using RobotsAndLidarsVector = tuple<vector<Robot*>,vector<Lidar*>>;
using RobotsAndLidarsVector = tuple<vector<RobotPointer>,vector<LidarPointer>>;

// using IdWorldItemTuple =  tuple<int, Robot*>;
// using IdItemTupleVector = vector<IdWorldItemTuple>;

using RobotsVector = vector<RobotPointer>;

using LidarsVector = vector<LidarPointer>;
using LidarsVectors = vector<shared_ptr<Lidar>>;
using IdRobotSharedMap = map<int, shared_ptr<Robot>>;
using IdRobotMap = map<int, Robot*>;

using RobotLidarMap = map<string, vector< shared_ptr<WorldItem>>>;



Json::Value readJson(string in_path);
RobotsAndLidarsVector initSimEnv(Json::Value root, shared_ptr<World> w, int& robot_counter);
string getGitRootPath();
double timeMillisec();