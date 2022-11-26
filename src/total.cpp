#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "opencv_version_test");
    ros::NodeHandle nh;

    std::istringstream ss("4,-1,1261.51,492.931,66.1,162.723,0.842982,-1,-1,-1");
    char ch;
    float tpx, tpy, tpw, tph;
    int frame;
    int id;

    std::cout << ss.str() << std::endl;
    ss >> frame >> ch >> id >> ch;
    ss >> tpx >> ch >> tpy >> ch >> tpw >> ch >>tph;
    ss.str("");
    std::cout << frame << std::endl; // 4
    std::cout << ch << std::endl; // ,
    std::cout << id << std::endl; // -1
    std::cout << tpx << std::endl; // 1261.51
    std::cout << tpy << std::endl; // 492.931
    std::cout << tpw << std::endl; // 66.1
    std::cout << tph << std::endl; // 162.723
    /////////////////////////////////////////////////////////////////////////////////////////
    std::vector<double> x = {-29.083, -28.149, -27.665, -27.216, -26.714, -26.286, -25.861, -25.375, -24.886};
    double sum_x = 0;
    for(int i=0; i < x.size(); i++){
        sum_x += x[i];
    }
    std::cout << sum_x << std::endl;
    std::cout << sum_x / x.size() << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat_<float> A;
    A = cv::Mat_<float>(7, 7) <<
		1, 0, 0, 0, 1, 0, 0,
		0, 1, 0, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 0, 1,
		0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 1;

    A = cv::Mat::eye(4,7, CV_32F);
    A[0][4] = 1;
    A[1][5] = 1;
    A[2][6] = 1;

    std::cout << A << std::endl;
    /////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "OpenCV Version : " << CV_VERSION << std::endl;
    cv::Mat ref_img;
    ref_img = cv::imread("/home/dj/paper_ws/src/tracking_test/img_data/base_map.png");
    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
    cv::imshow("test", ref_img);
    cv::waitKey(0);
    cv::destroyWindow("test");
    /////////////////////////////////////////////////////////////////////////////////////////

    cv::Mat ref1;
    ref1 = cv::imread("/home/dj/paper_ws/src/tracking_test/img_2/636995fa3fe19c69e0e41ade.bmp");

    cv::Mat ref2;
    ref2 = cv::imread("/home/dj/paper_ws/src/tracking_test/img_2/gen_map2.bmp");

    cv::Mat enc;
    enc = cv::imread("/home/dj/paper_ws/src/tracking_test/img_2/6369960c3fda3927c35e05be.bmp");

    cv::Mat add_map;
    cv::add(ref1, ref2, add_map);
    cv::imwrite("/home/dj/paper_ws/src/tracking_test/img_2/add_map1.bmp", add_map);

    cv::Mat add_map2;
    cv::add(add_map, enc, add_map2);
    cv::imwrite("/home/dj/paper_ws/src/tracking_test/img_2/add_map2.bmp", add_map2);

    return 0;
}