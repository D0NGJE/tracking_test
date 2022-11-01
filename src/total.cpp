#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "opencv_version_test");
    ros::NodeHandle nh;
    
    /////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "OpenCV Version : " << CV_VERSION << std::endl;
    cv::Mat ref_img;
    ref_img = cv::imread("/home/dj/paper_ws/src/tracking_test/img_data/base_map.png");
    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
    cv::imshow("test", ref_img);
    cv::waitKey(0);
    cv::destroyWindow("test");
    ///////////////////////////////////////////////////////////////////////////////////////// 
    return 0;
}