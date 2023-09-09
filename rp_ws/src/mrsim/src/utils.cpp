// Custom lib
#include "utils.h"

// LC: thsi function thake as input a config.jason file and makes a ros launch file to start nodes in a simulation
int makeLaunchFile(std::string in_path, std::string out_path) {

  std::cout << "Configuration file path:" << in_path << std::endl;
  Json::Value root; // B.F.N object to hold the parsed json file
  Json::CharReaderBuilder readerBuilder; // B.F.N reader of json file
  std::ifstream file(in_path, std::ifstream::binary); // B.F.N file to read
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

  // Robot counter initialization
  int robot_counter = 0;

  // Open the launch file
  std::ofstream outfile(out_path);

  std::cout << "Launch file path:" << out_path <<std::endl;

  // Check if the file was opened successfully
  if (!outfile) {
      std::cout << "Error path:" << out_path <<std::endl;
      std::cerr << "Error opening file for writing" << std::endl;
      return 1;
  }
  outfile << "<launch> \n\n";
 
  if (root["items"].isArray()) {
    for(const Json::Value& item: root["items"]) {

      const int id = item["id"].asInt();
      const std::string type = item["type"].asString();

      if(type == "robot"){
       
        outfile << "<node pkg='mrsim'" << " name='" << item["namespace"].asString()
                << "' type='" << type << "'" 
                << "/>\n"
                << "  <param id='" << id << "' frame_id='" << item["frame_id"].asString()
                << "' radious='" << item["radius"].asDouble() 
                << "' max_rv='" << item["max_rv"].asDouble() 
                << "' max_tv='" << item["max_tv"].asDouble() 
                << "' parent='" << item["parent"].asInt() << "'"
                
                << "/>\n</node> \n";

        robot_counter++;
      }
      else if (type == "lidar") {
        
        outfile << "<node pkg='mrsim'"<< " name='" << item["namespace"].asString() 
                << "' type='" << type << "'"
                << "/>\n"
                << "  <param id='" << id << "' frame_id='" << item["frame_id"].asString()
                << "' fov='" << item["fov"].asDouble()
                << "' num_beams='" << item["num_beams"].asInt()
                << "' parent='" << item["parent"].asInt() << "'"

                << "/>\n</node> \n";
      }
      else std::cerr << "Type error reading JSON file" << std::endl;
      outfile << "\n";
    }
  }
  outfile << "\n</launch>";
  outfile.close();
  std::cout << "Launch file created successfully!!!" << std::endl;
  return robot_counter;
}
       

// LC: this function return the path to the root directory of the current git project
std::string getGitRootPath() {

  std::array<char, 128> buffer;
  std::string root_path;

  // A pipe is opened to run the command git rev-parse --show-toplevel
  FILE* pipe = popen("git rev-parse --show-toplevel", "r");

  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }

  // Read the output of the command from the pipe buffer line by line
  // accumulate it in the root_path variable
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    root_path += buffer.data();
  }

  // Close the pipe and check the return code to be on the safe side
  auto return_code = pclose(pipe);

  if (return_code != 0) {
    throw std::runtime_error("Error finding git root directory");
  }

  // Remove the newline character at the end of the result
  if (!root_path.empty() && root_path[root_path.size() - 1] == '\n') {
    root_path.erase(root_path.size() - 1);
  }

  return root_path;
}