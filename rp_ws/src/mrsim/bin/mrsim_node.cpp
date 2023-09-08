#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <opencv2/highgui.hpp>

// Custom lib
#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "getgit.h"

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  // LC: get git root directory path
  std::string git_root_path;
  try {
        git_root_path = getGitRootPath();
        cout << "Git Root Path: " << git_root_path << '\n';
    } catch(const std::runtime_error& e) {
        cerr << "Exception: " << e.what() << '\n';
  }

  // Load the configuration file and initialize the simulator
  // TODO :
  /*
    1. Import json config file
    2. Read json config file and extract:
      2.1 NUM_ROBOTS: number of robots in the simulation (N)
      2.2 NUM_LIDARS: number of lidars in the simulation (M)
      2.3 INFO about robots
      2.4 INFO about lidars
    3. Make a launch file to run N robot_nodes and M lidars_nodes
    4. Launch the launch file
    5. Initialize an array of publisher objects for each robot to allow mrsim_node to publish on specific topics for each robot
  */

  // 1
  Json::Value root; // B.F.N object to hold the parsed json file
  Json::CharReaderBuilder readerBuilder; // B.F.N reader of json file
  std::ifstream file(git_root_path + "/config.json", std::ifstream::binary); // B.F.N file to read
  std::string errs; // B.F.N where the error are memorized


  // int NUM_ROBOT = 2;

  // // tmp def of world
  // World w;
  // w.loadFromImage("/home/leeoos/Projects/robot-programming-nc/map3D/map2D.png");


  // tmp def of lidar
  // float delay = 0.1;
  // int k;
  // int select_robot = 0;
  // std::vector<ros::Publisher> publishers;

  /*
  for robot_i in jaso...:
    ros::Publisher pub = n.advertise<geometry_msgs/Twist>(robot_i+"/cmd_vel", 1000);
    
  */

  //for create every robot!!!
  while (ros::ok()) {

    // this function update the status of each world item
    // w.timeTick(delay); // 

    // // draw the world
    // w.draw();

    // // LC: Select the index of the robot you wnat to control (switch case)
    // if (select_robot == 0){
    //   int i = -1;
    //   while (true) {
    //     cout << "What robot do you want to control? " << endl; 
    //     cout << "Insert anumebr beween 0 and " << NUM_ROBOT
    //     cin >> i;
        
    //     if (std::cin.fail() || i < -1 || i > NUM_ROB -1) {
    //         std::cin.clear(); // reset the fail state
    //         std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard invalid input
    //         std::cout << "Invalid input. Please try again." << std::endl;
    //     } 
    //     else {
    //       cout << "You select robot " << i << endl;
    //       cout << "press c to change the robot to controll anytime" << endl;
    //       break;
    //   }
    //   // reset the index to keep controlling the same robot
    //   select_robot = 1=
    // }

    // // Switch case to control robot motion
    // k = cv::waitKeyEx(0)&255;
    // switch (k) {
    //   case 81: r.rv+=0.05; break; // arow left
    //   case 82: r.tv+=0.1; break; // arow up
    //   case 83: r.rv-=0.05; break; // arow right
    //   case 84: r.tv-=0.1; break; // arow dw
    //   case 32: r.tv=0; r.rv=0; break;// spacebar
    //   case c: select_robot = -1; // 'c' key
    //   case 27: return 0; // esc
    //   default:;
    // }


    // // B.F.N: this if controll if you want change 
    // if select_robot == -1{
    //   select_robot = 0;
    // }
    // else:
    // publisher[i].publish(msg)
    // /robot_0/cmd_vel
    
      
    // pub.publish(msg)
      
    ros::spinOnce();
  }

  return 0;
}