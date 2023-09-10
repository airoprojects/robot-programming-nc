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

  ros::init(argc, argv, "opkey_node");
  ros::NodeHandle nh("/");

  // LC: get the number of robots in the simulation
  int NUM_ROBOTS = -1;
  if (argc < 1) {
    NUM_ROBOTS = argv[1].asInt();
  }
  else nh.getParam("numrobots", NUM_ROBOTS)
  } std::string namespace;
  nh.getParam("numrobots", namespace);


  // LC make a vector of publishers
  vector<ros::Publisher> publishers_vector;
  for (int i=0; i < NUM_ROBOTS; i++) {
    ros::Publisher foo_pub  = nh.advertise<geometry_msgs::Twist>("robot_" + to_string(i) + "/cmd_vel", 1000);
    publisher_vectro.push_back(foo_pub);
  }

  // LC: key press actions log
  ofstream keylog("./key.log");

  ros::Rate rate(2);
  while (ros::ok()) {

    // LC: message definition
    geometry_msgs::Twist msg;
    msg.linear.x = 1.0;
    msg.angular.z = 1.0;

    // LC: Select the index of the robot you wnat to control
    if (select_robot) {

      // This should be temporary, just to test key captures
      cv::namedWindow("Window");

      while (true) {

        std::cout << "\nWhat robot do you want to control? " << std::endl; 
        std::cout << "Press a numebr beween 0 and " << NUM_ROBOT-1 << ": ";
        std::cin >> robot_index;

        // LC: check for user error in the input
        if (std::cin.fail() || robot_index < 0 || robot_index > NUM_ROBOT-1) {
          std::cin.clear(); // reset the fail state
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard invalid input
          std::cout << "Invalid input. Please try again." << std::endl;
        } 
        else {
          std::cout << "You select robot " << robot_index << std::endl;
          std::cout << "Press 'c' to change robot\n" << std::endl;
          std::cout << "Press 'ESC' to exit the simulation\n" << std::endl;
          break;
        }
      }
      // LC: reset the index to keep controlling the same robot
      select_robot = false;
    }

    // Switch case to control robot motion
    int k = cv::waitKey(0);
    keylog << "\nKey pressed with decimal value: " << k << std::endl;
    switch (k) {
        case 81: msg.angular.z = 0.5;; break; // arow left
        case 82: msg.linear.x = 1.0; break; // arow up
        case 83: msg.angular.z = -0.5; break; // arow right
        case 84: msg.linear.x = -1.0; ; break; // arow dw
        case 99: select_robot = true; break; // c key
        case 27: std::cout << "\n"; return 0; // esc
        default: break;
    }

    // LC: publish on robot_{robot_index}/cmd_vel topic
    if (!select_robot) {
      publisher_vectro[robot_index].publish(msg);
      ros::spinOnce();
    }
    // LC: or change robot
    else {
      std::cout << "Change robot\n" << std::endl;
      cv::destroyWindow("Window");
    }
  }

  // LC: destroy the created window
  cv::destroyWindow("Window");
  keylog.close();
  return 0;
}