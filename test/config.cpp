  // Load the configuration file and initialize the simulator 
  // config confi.json
  // TODO
  /*
    0. Read from json file
    1. Count numebr of items
    2. Create a vector of publisher
    3. Create lauch file to run all the robots nodes:
        rosrun mrsim robot_node "robot_i" 
  */

#include <iostream>
#include <jsoncpp/json/json.h>
#include <fstream>

int main(){
    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::ifstream file("./config.json", std::ifstream::binary);
    std::string errs;

    bool parsingSuccessful = Json::parseFromStream(readerBuilder, file, &root, &errs);
    if (!parsingSuccessful) {
        std::cout << "Failed to parse JSON file: " << errs << std::endl;
        return 1;
    }

    const std::string map = root["map"].asString();
    const double radius = root["items"][0]["radius"].asDouble();

    std::cout << "Map: " << map << std::endl;
    std::cout << "Radius: " << radius << std::endl;

    return 0;

}

















