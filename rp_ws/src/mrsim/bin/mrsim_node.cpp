#include <ros/ros.h>
#include <opencv2/highgui.hpp>

// Custom include 
#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"

using namespace std

int main(int argc, char** argv) {
  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  // Load the configuration file and initialize the simulator 
  // TODO
  /*
    1. Count numebr of robots / lidar
    2. create a vector of publisher
    3. lauch file to run all the robots nodes:
        rosrun mrsim robot_node "robot_i" 
  */

  int NUM_ROBOT = 2;

  // tmp def of world
  World w;
  w.loadFromImage("/home/leeoos/Projects/robot-programming-nc/map3D/map2D.png");


  // tmp def of lidar
  float delay = 0.1;
  int k;
  int select_robot = 0;
  std::vector<ros::Publisher> publishers;

  /*
  for robot_i in jaso...:
    ros::Publisher pub = n.advertise<geometry_msgs/Twist>(robot_i+"/cmd_vel", 1000);
    
  */

  //for create every robot!!!
  while (ros::ok()) {

    // this function update the status of each world item
    w.timeTick(delay); // 

    // draw the world
    w.draw();

    // LC: Select the index of the robot you wnat to control (switch case)
    if (select_robot == 0){
      int i = -1;
      while (true) {
        cout << "What robot do you want to control? " << endl; 
        cout << "Insert anumebr beween 0 and " << NUM_ROBOT
        cin >> i;
        
        if (std::cin.fail() || i < -1 || i > NUM_ROB -1) {
            std::cin.clear(); // reset the fail state
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard invalid input
            std::cout << "Invalid input. Please try again." << std::endl;
        } 
        else {
          cout << "You select robot " << i << endl;
          cout << "press c to change the robot to controll anytime" << endl;
          break;
      }
      // reset the index to keep controlling the same robot
      select_robot = 1=
    }

    // Switch case to control robot motion
    k = cv::waitKeyEx(0)&255;
    switch (k) {
      case 81: r.rv+=0.05; break; // arow left
      case 82: r.tv+=0.1; break; // arow up
      case 83: r.rv-=0.05; break; // arow right
      case 84: r.tv-=0.1; break; // arow dw
      case 32: r.tv=0; r.rv=0; break;// spacebar
      case c: select_robot = -1; // 'c' key
      case 27: return 0; // esc
      default:;
    }


    // B.F.N: this if controll if you want change 
    if select_robot == -1{
      select_robot = 0;
    }
    else:
    publisher[i].publish(msg)
    /robot_0/cmd_vel
    
      
    pub.publish(msg)
      
    ros::spinOnce();
  }

  return 0;
}