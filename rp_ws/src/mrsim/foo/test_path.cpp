#include <iostream>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

std::string getGitRootPath() {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("git rev-parse --show-toplevel", "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

int main() {
    try {
        std::string gitRootPath = getGitRootPath();
        gitRootPath.erase(std::remove(gitRootPath.end() - 1, gitRootPath.end(), '\n'), gitRootPath.end());  // Remove the trailing newline
        std::cout << "Git project root directory: " << gitRootPath << '\n';
    } catch (const std::runtime_error &e) {
        std::cerr << "Error: " << e.what() << '\n';
    }
    return 0;
}
