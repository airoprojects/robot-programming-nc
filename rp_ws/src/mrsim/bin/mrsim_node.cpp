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
  world.draw();
  cv::waitKey(1);
 
  // LC: environment configuration
  int NUM_ROBOTS = 2;
  auto robots_and_lidars =  initSimEnv(root, world_pointer, NUM_ROBOTS);
  // if (!robots_and_lidars){
  //   cerr << "Error: incorrect configuration \n" << endl;
  //   return 1;
  // } 

  // LC: check that each items has been correctly added to the world
  for (const auto robot: world._items) {cout << robot->_namespace << endl;}

  // LC: run opkey controller node 
  string command = "gnome-terminal -- bash -c 'rosrun mrsim opkey_node " + to_string(NUM_ROBOTS) + " ; exec bash'";
  int result = system(command.c_str());
  if (result != 0) {
    ROS_ERROR("Failed to execute roslaunch command");
    return 1;
  }


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