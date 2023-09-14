#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"

#include <jsoncpp/json/json.h> 
#include <optional>

using WorldPointer = shared_ptr<World>;
using WorldItemPointer = shared_ptr<WorldItem>;
using RobotPointer = shared_ptr<Robot>;
using WorldItemPointer = shared_ptr<WorldItem>;

// using IdWorldItemTuple =  tuple<int, Robot*>;
// using IdItemTupleVector = vector<IdWorldItemTuple>;

using RobotsVector = vector<Robot*>;
using LidarsVector = vector<Lidar*>;
using IdRobotMap = map<int, Robot*>;
using RobotLidarMap = map<string, vector< shared_ptr<WorldItem>>>;



Json::Value readJson(string in_path);
vector<Robot*>  initSimEnv(Json::Value root, shared_ptr<World> w, int& robot_counter);
string getGitRootPath();
double timeMillisec();