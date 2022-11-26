#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class MAKEBAG{
    public:
    MAKEBAG();
    private:    
    int frame_ = 0;
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher lidar_pub_;
    void Callback(const sensor_msgs::PointCloud2ConstPtr input_msg);
};

MAKEBAG::MAKEBAG() : nh_("~"){
  lidar_sub_ = nh_.subscribe("/lidar_data", 1, &MAKEBAG::Callback, this);
  lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_data_0", 1);
}

Eigen::Matrix4f GetTranslation(double tr_x, double tr_y, double tr_z){
	Eigen::Matrix4f transformation;
    transformation << 1, 0, 0, tr_x, 
                      0, 1, 0, tr_y, 
                      0, 0, 1, tr_z, 
                      0, 0, 0, 1;
  	return transformation;
}

Eigen::Matrix4f GetRotateY(double deg){
	Eigen::Matrix4f transformation;
    double rad = deg * M_PI / 180;
    transformation << 1, 0, 0, 0, 
                      0, cos(rad), -sin(rad), 0, 
                      0, sin(rad), cos(rad), 0, 
                      0, 0, 0, 1;
  	return transformation;
}

Eigen::Matrix4f GetRotateX(double deg){
	Eigen::Matrix4f transformation;
    double rad = deg * M_PI / 180;
    transformation << cos(rad), 0, sin(rad), 0, 
                      0, 1, 0, 0, 
                      -sin(rad), 0, cos(rad), 0, 
                      0, 0, 0, 1;
  	return transformation;
}

Eigen::Matrix4f GetRotateZ(double deg){
	Eigen::Matrix4f transformation;
    double rad = deg * M_PI / 180;
    transformation << cos(rad), -sin(rad), 0, 0, 
                      sin(rad), cos(rad), 0, 0, 
                      0, 0, 0, 0, 
                      0, 0, 0, 1;
  	return transformation;
}

void MAKEBAG::Callback(const sensor_msgs::PointCloud2ConstPtr input_msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *input);

    std::string filepath_1 = ""; //"/home/dj/Downloads/test_11041700/tr_1_";
    std::string filepath_2 = ""; //"/home/dj/Downloads/test_11041700/tr_2_"; 

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr tr_cloud_1 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tr_cloud_2 (new pcl::PointCloud<pcl::PointXYZI>);

    // filepath_1 = "/home/dj/Downloads/test_11041700/tr_1_" + std::to_string(frame_) + ".pcd";
    // filepath_2 = "/home/dj/Downloads/test_11041700/tr_2_" + std::to_string(frame_) + ".pcd";

    filepath_1 = "/home/dj/Downloads/output/test/tr_1_" + std::to_string(frame_) + ".pcd";
    filepath_2 = "/home/dj/Downloads/output/test/tr_2_" + std::to_string(frame_) + ".pcd";

    pcl::io::loadPCDFile<pcl::PointXYZI> (filepath_1, *cloud_1);
    pcl::io::loadPCDFile<pcl::PointXYZI> (filepath_2, *cloud_2);

    Eigen::Matrix4f coor_tr_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f coor_tr_2 = Eigen::Matrix4f::Identity();
    coor_tr_1 = GetTranslation(0.0,0.75,0) * GetRotateX(-3) * GetRotateY(-4);
    coor_tr_2 = GetTranslation(0.0,0.75,0) * GetRotateX(-3) * GetRotateY(1); // 1.8,0.1,0

    pcl::transformPointCloud(*cloud_1, *tr_cloud_1, coor_tr_1);
    pcl::transformPointCloud(*cloud_2, *tr_cloud_2, coor_tr_2);
    /**********************************************************************************************/
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_combine(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto &p : *tr_cloud_1){
        cloud_combine->push_back(p);
    }
    for(auto &p : *tr_cloud_2){
        cloud_combine->push_back(p);
    }


    sensor_msgs::PointCloud2 lidar_msg;
    pcl::toROSMsg(*cloud_combine, lidar_msg);
    lidar_msg.header.frame_id = "/rslidar";
    lidar_pub_.publish(lidar_msg);

    frame_++;

    if(frame_ >165){
        ros::shutdown();
    }
}
int main(int argc, char** argv){
    ros::init(argc, argv, "make_rosbag");
    MAKEBAG node;
    ros::spin();
    return 0;
}
