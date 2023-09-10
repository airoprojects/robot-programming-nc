#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

// Custom lib
#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "utils.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  // LC: get git root directory path
  string git_root_path;
  try {
        git_root_path = getGitRootPath();
        //cerr << "Git Root Path: " << git_root_path << '\n';
    } catch(const runtime_error& e) {
        cerr << "Exception: " << e.what() << '\n';
  }

  // Load the configuration file and initialize the simulator
  // TODO :
  /*
    0. Initialize a instance of world
    1. Import json config file                                                DONE
    2. Read json config file and extract:                                     DONE
      2.1 NUM_ROBOTS: number of robots in the simulation (N)  
      2.2 NUM_LIDARS: number of lidars in the simulation (M)  
      2.3 INFO about robots                                                   DONE
      2.4 INFO about lidars                                                   DONE
    3. Make a launch file to run N robot_nodes and M lidars_nodes             DONE
    4. Launch the launch file
    5. Initialize an array of publisher objects for each robot to 
       allow mrsim_node to publish on specific topics for each robot
  */

  string config_path = git_root_path + "/config/" + argv[1];
  Json::Value root = readJson(config_path);
  string map = root["map"].asString();
  cout << "Map -> " << map << endl;
  string image_path = git_root_path + "/map/" +  map;

  // LC: pointer new instance of World
  shared_ptr<World> world_pointer = make_shared<World>(42);
  w_ptr->loadFromImage(image_path); //THE MOST STUPID FUNCTION IN THE UNIVERSE, BASTARD FUNCTION.
  
  int NUM_ROBOT = 0;
  RobotLidarMap  robots_and_lidars =  initSimEnv(root, world_pointer, NUM_ROBOT);
  
  // // LC: make a launch based on config.json to run multiple robot/lidar nodes
  // int NUM_ROBOT = makeLaunchFile(
  //                 git_root_path + "/config/config.json", 
  //                 git_root_path + "/rp_ws/src/mrsim/launch/simulation.launch");

  // // LC: launch the simulation launch file from this node
  // int result = system("roslaunch mrsim simulation.launch");
  // if (result != 0) {
  //     ROS_ERROR("Failed to execute roslaunch command");
  // }

  // LC: no robot is selected to be controlled at the beginning
  bool select_robot = true; 
  int robot_index = -1;

  // LC: keypress log
  // ofstream keylog("./key.log");

  while (ros::ok()) {

    // LC: this function update the status of each world item
    // world.timeTick(delay); 
    // world.draw();

  }

  // // Destroy the created window
  // cv::destroyWindow("Window");
  // keylog.close();
  // return 0;
}