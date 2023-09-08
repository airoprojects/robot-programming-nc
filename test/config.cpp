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


//B.F.N -> to remove the prints

int main(){
  
    Json::Value root; // B.F.N object to hold the parsed json file
    Json::CharReaderBuilder readerBuilder; // B.F.N reader of json file
    std::ifstream file("./config.json", std::ifstream::binary); // B.F.N file to read
    std::string errs; // B.F.N where the errors are memorized

    // B.F.N check if the parse is successfull
    bool parsingSuccessful = Json::parseFromStream(readerBuilder, file, &root, &errs);
    if (!parsingSuccessful) {
        std::cout << "Failed to parse JSON file: " << errs << std::endl;
        return 1;
    }

    // start the parse of the file
    const std::string map = root["map"].asString();
    std::cout << "Map: " << map << std::endl;

    int contRobot = 0; // B.F.N a counter to keep trace of number of robots

    // iterate items in the json:
    if(root["items"].isArray()) {

      for(const Json::Value item: root["items"]){
        
        const int id = item["id"].asInt();
        const std::string type = item["type"].asString();

        std::cout << "ID: " << id << std::endl;
        std::cout << "Type" << type << std::endl;


        if(type == "robot"){
          // B.F.N insert here all params about robot:
          const std::string frame_id_r = item["frame_id"].asString();
          const std::string namespace_r = item["namespace"].asString();
          const double radius = item["radius"].asDouble();
          const double max_rv = item["max_rv"].asDouble();
          const double max_tv = item["max_tv"].asDouble();
          const Json::Value pose_array_r = item["pose"];
          const std::vector<double> pose_r = {pose_array_r[0].asDouble(),pose_array_r[1].asDouble(),pose_array_r[2].asDouble()};
          const int parent_r = item["parent"].asInt();
          std::cout << "Frame ID: " << frame_id_r << ", Namespace: " << namespace_r << ", Radius: " << radius << ", Max RV: " << max_rv << ", Max TV: " << max_tv << ", Parent: " << parent_r << std::endl;
          for(int i = 0; i < 3; ++i) {
            std::cout << "pose_r[" << i << "] = " << pose_r[i] << std::endl;
          }
         

        }else if(type == "lidar"){
          const std::string frame_id_l = item["frame_id"].asString();
          const std::string namespace_l = item["namespace"].asString();
          const double fov_l = item["fov"].asDouble();
          const double max_range_l = item["max_range"].asDouble();
          const int num_beams_l = item["num_beams"].asInt();
          const Json::Value pose_array_l = item["pose"];
          const std::vector<double>  pose_l = {pose_array_l[0].asDouble(),pose_array_l[1].asDouble(),pose_array_l[2].asDouble()}; 
          const int parent_l = item["parent"].asInt();


          std::cout << "Frame ID: " << frame_id_l << std::endl
          << "Namespace: " << namespace_l << std::endl
          << "FOV: " << fov_l << std::endl
          << "Max Range: " << max_range_l << std::endl
          << "Number of Beams: " << num_beams_l << std::endl
          << "Parent: " << parent_l << std::endl;

          for(int i = 0; i < 3; ++i) {
            std::cout << "pose_r[" << i << "] = " << pose_l[i] << std::endl;
          }



        }else{
          std::cout << "Type error reading JSON file" << std::endl;// to finish
        }

      

      }

    }




    return 0;

}

















