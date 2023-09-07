#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <opencv2/highgui.hpp>



int main() {
    while (true) {
        int key = cv::waitKeyEx(0)&255;
        // std::cout << "Key pressed: " << key << " (" << char(key) << ")" << std::endl;
        std::cout << "Key pressed: " << key << std::endl;
    }
    return 0;
}
