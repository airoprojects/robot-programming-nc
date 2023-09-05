#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/io/pcd_io.h>


int main(int argc, char** argv) {
    // Load the image
    cv::Mat img = cv::imread("/home/leeoos/Projects/robot-programming-nc/map3D/map2D.png", cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
        std::cerr << "Failed to load the image." << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Set the cloud dimensions based on the image size
    cloud->width = img.cols;
    cloud->height = img.rows;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
          
            // Scale the height based on the pixel value
            float height = img.at<uchar>(i, j);

            // Set the XYZ coordinates of the point
            pcl::PointXYZ& point = cloud->at(j, i);
            point.x = j;
            point.y = i;
            point.z = (-1)*height;
        }
    }

    // Save the point cloud
    pcl::io::savePCDFileASCII("/home/leeoos/Projects/robot-programming-nc/map3D/map3D.pcd", *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to test_pcd.pcd." << std::endl;

    return 0;
}
