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

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        // Define the button's region (here, a rectangle at the top-left corner of the window)
        if (x > 50 && x < 200 && y > 50 && y < 100) {
            std::cout << "Button clicked!" << std::endl;
        }
        cv::destroyWindow("ButtonsWindow");
    }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  // LC: get git root directory path
  std::string git_root_path;
  try {
        git_root_path = getGitRootPath();
        //std::cerr << "Git Root Path: " << git_root_path << '\n';
    } catch(const std::runtime_error& e) {
        std::cerr << "Exception: " << e.what() << '\n';
  }

  // Load the configuration file and initialize the simulator
  // TODO :
  /*
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
  int NUM_ROBOT = makeLaunchFile(
                  git_root_path + "/config/config.json", 
                  git_root_path + "/rp_ws/launch/simulation.launch");

  // LC: no robot is selected to be controlled at the beginning
  int select_robot = 1; // 1 == true 
  int robot_index = -1;

  // LC: keypress log
  std::ofstream keylog("./key.log");

  while (ros::ok()) {

    // LC: this function update the status of each world item
    // w.timeTick(delay); 
    // w.draw();

    // LC: Select the index of the robot you wnat to control
    if (select_robot == 1) {

      // This should be temporary, just to test key captures
      cv::namedWindow("Window");

      while (true) {

        std::cout << "What robot do you want to control? " << std::endl; 
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
      // reset the index to keep controlling the same robot
      select_robot = 0;
    }

    // Switch case to control robot motion
    int k = cv::waitKey(0);
    keylog << "\nKey pressed with decimal value: " << k << std::endl;
    switch (k) {
        case 81: std::cout << "robot_" << robot_index << " left\n"; break; // arow left
        case 82: std::cout << "robot_" << robot_index << " up\n"; break; // arow up
        case 83: std::cout << "robot_" << robot_index << " right\n"; break; // arow right
        case 84: std::cout << "robot_" << robot_index << " down\n"; break; // arow dw
        case 32: std::cout << "\nspacebar"; break;// spacebar
        case 99: select_robot = 1; break; // c key
        case 27: std::cout << "\n"; return 0; // esc
        default: break;
    }

    // B.F.N: this if controll if you want change 
    if (select_robot == 0) {
      ros::spinOnce();
    }
    else {
      std::cout << "Change robot\n" << std::endl;
      cv::destroyWindow("Window");
    }
  }

  // Destroy the created window
  cv::destroyWindow("Window");
  keylog.close();
  return 0;
}