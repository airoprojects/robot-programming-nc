// Custom lib
#include "utils.h"

// LC: thsi function thake as input a config.jason file and makes a ros launch file to start nodes in a simulation
void MakeLaunchFile(std::string path) {

    
  // Open the launch file
  ofstream outfile(path);

  // Check if the file was opened successfully
  if (!outfile) {
      std::cerr << "Error opening file for writing" << std::endl;
      return 1;
  }

  outfile << '<launch> \n'

  // insert code 
  outfile << '<node pkg="mrsim" name="' << item.name << '" type="' << item.type << '"/> \n';

  outfile << '</launch>';

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