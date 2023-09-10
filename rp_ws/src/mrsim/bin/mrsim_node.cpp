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
 
  
  // test world instance
  // test load image
  //w_ptr->loadFromImage(image_path); THE MOST STUPID FUNCTION IN THE UNIVERSE, BASTARD FUNCTION.
  
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

    // LC: Select the index of the robot you wnat to control
    // if (select_robot) {

    //   // This should be temporary, just to test key captures
    //   cv::namedWindow("Window");

    //   while (true) {

    //     cout << "\nWhat robot do you want to control? " << endl; 
    //     cout << "Press a numebr beween 0 and " << NUM_ROBOT-1 << ": ";
    //     cin >> robot_index;

    //     // LC: check for user error in the input
    //     if (cin.fail() || robot_index < 0 || robot_index > NUM_ROBOT-1) {
    //       cin.clear(); // reset the fail state
    //       cin.ignore(numeric_limits<streamsize>::max(), '\n'); // discard invalid input
    //       cout << "Invalid input. Please try again." << endl;
    //     } 
    //     else {
    //       cout << "You select robot " << robot_index << endl;
    //       cout << "Press 'c' to change robot\n" << endl;
    //       cout << "Press 'ESC' to exit the simulation\n" << endl;
    //       break;
    //     }
    //   }
    //   // reset the index to keep controlling the same robot
    //   select_robot = false;
    // }

    // // Switch case to control robot motion
    // int k = cv::waitKey(0);
    // keylog << "\nKey pressed with decimal value: " << k << endl;
    // switch (k) {
    //     case 81: cout << "robot_" << robot_index << " left\n"; break; // arow left
    //     case 82: cout << "robot_" << robot_index << " up\n"; break; // arow up
    //     case 83: cout << "robot_" << robot_index << " right\n"; break; // arow right
    //     case 84: cout << "robot_" << robot_index << " down\n"; break; // arow dw
    //     case 32: cout << "\nspacebar"; break;// spacebar
    //     case 99: select_robot = true; break; // c key
    //     case 27: cout << "\n"; return 0; // esc
    //     default: break;
    // }

    // // B.F.N: this if controll if you want change 
    // if (!select_robot) {
    //   ros::spinOnce();
    // }
    // else {
    //   cout << "Change robot\n" << endl;
    //   cv::destroyWindow("Window");
    // }
  }

  // // Destroy the created window
  // cv::destroyWindow("Window");
  // keylog.close();
  // return 0;
}