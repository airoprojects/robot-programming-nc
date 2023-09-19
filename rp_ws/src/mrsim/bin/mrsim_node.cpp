#include <ros/ros.h>  
 
#include <sys/time.h>
#include <thread>
#include <chrono>
#include <cmath>

#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "utils.h"

int main(int argc, char** argv) {

  if (argc < 2) {
    cerr << "Error: no config.jsosn file provided:\n" 
    << "Please insert your configuration file in the project config directory.\n"
    << "Then pass the name of the file as arg to the node.\n" << endl;
    return 1;
  }

  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  // LC: get git root directory path
  string git_root_path;
  try {
    git_root_path = getGitRootPath();
  }
  catch (const runtime_error& e) {
    cerr << "Exception: " << e.what() << '\n';
  }

  // Load configuration file
  string config_path = git_root_path + "/config/" + argv[1];
  Json::Value root = readJson(config_path);
  string map = root["map"].asString();
  cout << "Map -> " << map << endl;
  string image_path = git_root_path + "/map/" +  map;

  // LC: new instance of World with shared pointer
  World world(42); 
  WorldPointer world_pointer(&world, [](World*){ });   // is a lambda function 

  world.loadFromImage(image_path);  
 
  // LC: environment configuration
  int NUM_ROBOTS = 0;
  auto robots_and_lidars =  initSimEnv(root, world_pointer, NUM_ROBOTS);
  if (robots_and_lidars == RobotsAndLidarsVector({},{})){
    cerr << "Error: incorrect configuration \n" << endl;
    return 1;
  } 

  // LC: check that each items has been correctly added to the world
  cout << "World Items: " << endl;
  for (const auto item: world._items) {cout << item->item_frame_id << endl;}

  world.draw();
  cv::waitKey(1);

  // config robot in rviz
  string command = "rosparam set robot_description --textfile " + git_root_path + "/config/simple_mobile_robot.urdf";
  if (runShellCommand(command)) return 1; // exit if the command fails

  // LC: run rviz
  command = "gnome-terminal -- bash -c 'rosrun rviz rviz -d" + git_root_path + "/config/rviz_basic_config.rviz'";
  if (runShellCommand(command)) return 1; // exit if the command fails
  sleep(3);

  // LC: run opkey controller node 
  command = "gnome-terminal -- bash -c 'rosrun mrsim opkey_node " + to_string(NUM_ROBOTS) + " ; exec bash'";
  if (runShellCommand(command)) return 1; // exit if the command fails

  // LC: simulation parameters
  float delay = 0.08;
  ros::Rate loop_rate(10);

  cout << "Running primary node" << endl;

  while (ros::ok()) {

    world.draw();
    int k = cv::waitKey(1);
    if (k == 27) break;

    ros::spinOnce();
    world.timeTick(delay);
    this_thread::sleep_for(chrono::milliseconds(10)); // sleep for x milliseconds

  }

  cv::destroyAllWindows();
  return 0;
} 
