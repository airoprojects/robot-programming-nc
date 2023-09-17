#include <ros/ros.h>

#include <thread>  
#include <chrono>

#include <termios.h>
#include <unistd.h>
#include <signal.h>

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
  NUM_ROBOTS = stoi(argv[1]);

  // LC: make a vector of publishers
  vector<ros::Publisher> publishers_vector;
  for (int i=0; i < NUM_ROBOTS; i++) {
    ros::Publisher foo_pub  = nh.advertise<geometry_msgs::Twist>("robot_" + to_string(i) + "/cmd_vel", 1000);
    publishers_vector.push_back(foo_pub);
  }

  // LC: this code snippet configures the terminal to non-canonical mode 
  // disables the echoing of input characters ,input will be available 
  // immediately and will not be displayed on the terminal. 
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // LC: simulation parameters
  bool select_robot = true; // no robot is selected to be controlled at the beginning
  int robot_index = -1;
  bool clear = false;
  string input;
  geometry_msgs::Twist msg;
  ros::Rate rate(10);

  cout << "Running operational key controller node" << endl;

  while (ros::ok()) {

    // LC: Select the index of the robot you want to control
    if (select_robot) {
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // deactivate getchar terminal functionalities

      if (clear) {
        system("cls||clear"); // clear terminal screen
        clear = false;
      }

      while (true) {

        cout << "\nWhat robot do you want to control? " << endl; 
        cout << "Type a numebr beween 0 and " << NUM_ROBOTS-1 << " then press Enter: ";
        cin >> input;

        // LC: exit condiction
        if (input == "exit") killTerminal();

        // LC: check for user error in the input
        try {
          robot_index = stoi(input);
          if (robot_index < 0 || robot_index > NUM_ROBOTS-1) {
            system("cls||clear"); 
            cout << "Invalid input. Please try again." << endl;
          }
          else {
            cout << "\nControlling robot " << robot_index << "\n" << endl;
            cout << "Press 'c' to change robot\n" << endl;
            cout << "Press 'ESC' two times to exit the simulation\n" << endl;
            break;
          }
        } 
        catch (const exception& e) {
          system("cls||clear"); 
          cout << "Invalid input. Please try again." << endl;
        }
      }
      // LC: reset the index to keep controlling the same robot
      select_robot = false;
      char none = getchar(); // just to ignore Enter signal
      tcsetattr(STDIN_FILENO, TCSANOW, &newt); // reactivate getchar term functionalities
    }

    // LC: message definition
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;

    // LC: publish on robot_{robot_index}/cmd_vel topic
    this_thread::sleep_for(chrono::milliseconds(25)); // sleep for x milliseconds
    publishers_vector[robot_index].publish(msg);
    ros::spinOnce();
 
    // LC: capture user action: keypress
    char ch = getchar();

    if (ch == 27) {
      char next_ch = getchar(); // get the char after 'ESC'

      if(next_ch == '[') {
        ch = getchar(); // get the character after '['
        switch (ch) {
          case 'A': clearTerminal(); cout << "up\n"; msg.linear.x = 1.0; cout.flush(); break;
          case 'B': clearTerminal(); cout << "down\n"; msg.linear.x = -1.0; cout.flush(); break;
          case 'C': clearTerminal(); cout << "right\n"; msg.angular.z = -0.5; cout.flush(); break;
          case 'D': clearTerminal(); cout << "left\n"; msg.angular.z = 0.5; cout.flush(); break;
          default: clearTerminal(); cerr << "Invalid command: " << ch << endl; cout.flush(); break;
        }
      }
      else break;
    } 
    else if (ch == 'c')  {
      select_robot = true; 
      clear = true;
    }
    else if (ch != 'A' && ch != 'B' && ch != 'C' && ch != 'D') {
      clearTerminal(); cout << "Invalid command: " << ch << endl; cout.flush();
    }
    else ;

    // LC: publish on robot_{robot_index}/cmd_vel topic
    publishers_vector[robot_index].publish(msg);
    ros::spinOnce();

  }
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // deactivate getchar terminal functionalities
  killTerminal();
  return 0;
}
