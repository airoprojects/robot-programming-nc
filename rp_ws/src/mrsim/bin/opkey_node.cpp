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

void writeOn(cv::Mat image, string text) {
  cv::putText(image,                           // Target image
            text,                // Text to be displayed
            cv::Point(100, 250),             // Bottom-left corner of the text string in the image
            cv::FONT_HERSHEY_SIMPLEX,        // Font type
            1,                               // Font scale
            cv::Scalar(0, 255, 0),           // Font color
            2);                              // Line thickness
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "opkey_node");
  ros::NodeHandle nh("/");
 
  // LC: get the number of robots in the simulation
  int NUM_ROBOTS = -1;
  NUM_ROBOTS = stoi(argv[1]);

  cout << "NUM_ROBOTS " << NUM_ROBOTS << endl;


  // LC: make a vector of publishers
  vector<ros::Publisher> publishers_vector;
  for (int i=0; i < NUM_ROBOTS; i++) {
    ros::Publisher foo_pub  = nh.advertise<geometry_msgs::Twist>("robot_" + to_string(i) + "/cmd_vel", 1000);
    publishers_vector.push_back(foo_pub);
  }

  // LC: no robot is selected to be controlled at the beginning
  bool select_robot = true; 
  int robot_index = -1;

  // LC: key press actions log
  ofstream keylog("./key.log");
  ros::Rate rate(2);

  // LC: controller matrix
  cv::Mat ctrl_window = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);

  while (ros::ok()) {

    // LC: Select the index of the robot you wnat to control
    if (select_robot) {

      // This should be temporary, just to test key captures
      // cv::namedWindow("Window");
      // cv::destroyWindow("");
      // cv::destroyAllWindows();

      while (true) {

        std::cout << "\nWhat robot do you want to control? " << std::endl; 
        std::cout << "Type a numebr beween 0 and " << NUM_ROBOTS-1 << ": ";
        std::cin >> robot_index;

        // LC: check for user error in the input
        if (std::cin.fail() || robot_index < 0 || robot_index > NUM_ROBOTS-1) {
          std::cin.clear(); // reset the fail state
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard invalid input
          std::cout << "Invalid input. Please try again." << std::endl;
        } 
        else {
          std::cout << "\nControlling robot " << robot_index << "\n" << std::endl;
          std::cout << "Press 'c' to change robot\n" << std::endl;
          std::cout << "Press 'ESC' to exit the simulation\n" << std::endl;
          break;
        }
      }
      // LC: reset the index to keep controlling the same robot
      select_robot = false;
    }

    cv::imshow("Control Window", ctrl_window);

    // LC: message definition
    geometry_msgs::Twist msg;
    msg.linear.x = 1.0;
    msg.angular.z = 1.0;

    // Switch case to control robot motion
    int k = cv::waitKey(0);
    keylog << "\nKey pressed with decimal value: " << k << std::endl;
    switch (k) {
        case 81: writeOn(ctrl_window, "left"); msg.angular.z = 0.5;; break; // arow left
        case 82: cout <<"\nup\n"; msg.linear.x = 1.0; break; // arow up
        case 83: cout <<"\nright\n"; msg.angular.z = -0.5; break; // arow right
        case 84: cout <<"\ndown\n"; msg.linear.x = -1.0; ; break; // arow down
        case 99: cout <<"\nrobot_"<<robot_index<<"\n" ; select_robot = true; break; // c key
        case 27: std::cout << "\n"; return 0; // esc
        default: break;
    }

    // LC: publish on robot_{robot_index}/cmd_vel topic
    if (!select_robot) {
      publishers_vector[robot_index].publish(msg);
      ros::spinOnce();
    }
  }

  // LC: destroy the created window
  // cv::destroyWindow("Window");
  //cv::destroyAllWindows();
  keylog.close();
  return 0;
}
