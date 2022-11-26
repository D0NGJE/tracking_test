#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

class TOPIC{
    public:
    TOPIC();
    private:    
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher lidar_pub_;
    Eigen::Matrix4f GetTranslation(double tr_x, double tr_y, double tr_z);
    Eigen::Matrix4f GetRotateX(double deg);
    Eigen::Matrix4f GetRotateY(double deg);
    Eigen::Matrix4f GetRotateZ(double deg);
    void TransformPC(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, Eigen::Matrix4f TR);
    
    void Callback(const sensor_msgs::PointCloud2ConstPtr input_msg);
};

TOPIC::TOPIC() : nh_("~"){
  lidar_sub_ = nh_.subscribe("/lidar_data_0", 10, &TOPIC::Callback, this);
  lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/change_lidar_points", 10);
}

Eigen::Matrix4f TOPIC::GetTranslation(double tr_x, double tr_y, double tr_z){
	Eigen::Matrix4f transformation;
    transformation << 1, 0, 0, tr_x, 
                      0, 1, 0, tr_y, 
                      0, 0, 1, tr_z, 
                      0, 0, 0, 1;
  	return transformation;
}

Eigen::Matrix4f TOPIC::GetRotateX(double deg){
	Eigen::Matrix4f transformation;
    double rad = deg * M_PI / 180;
    transformation << 1, 0, 0, 0, 
                      0, cos(rad), -sin(rad), 0, 
                      0, sin(rad), cos(rad), 0, 
                      0, 0, 0, 1;
  	return transformation;
}

Eigen::Matrix4f TOPIC::GetRotateY(double deg){
	Eigen::Matrix4f transformation;
    double rad = deg * M_PI / 180;
    transformation << cos(rad), 0, sin(rad), 0, 
                      0, 1, 0, 0, 
                      -sin(rad), 0, cos(rad), 0, 
                      0, 0, 0, 1;
  	return transformation;
}

Eigen::Matrix4f TOPIC::GetRotateZ(double deg){
	Eigen::Matrix4f transformation;
    double rad = deg * M_PI / 180;
    transformation << cos(rad), -sin(rad), 0, 0, 
                      sin(rad), cos(rad), 0, 0, 
                      0, 0, 1, 0, 
                      0, 0, 0, 1;
  	return transformation;
}

void TOPIC::TransformPC(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, Eigen::Matrix4f TR){
	for(int i = 0; i < input->points.size(); i++){
		output->points[i].x = TR(0,0) * input->points[i].x + TR(0,1) * input->points[i].y + TR(0,2) * input->points[i].z + TR(0,3);
		output->points[i].y = TR(1,0) * input->points[i].x + TR(1,1) * input->points[i].y + TR(1,2) * input->points[i].z + TR(1,3);
		output->points[i].z = TR(2,0) * input->points[i].x + TR(2,1) * input->points[i].y + TR(2,2) * input->points[i].z + TR(2,3);
	}
}


void TOPIC::Callback(const sensor_msgs::PointCloud2ConstPtr input_msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *input);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    ptr_transformed->resize(input->points.size());
    Eigen::Matrix4f coor_rt_1 = Eigen::Matrix4f::Identity();
    double angle = 90;
    coor_rt_1(0,0) = cos(angle * M_PI / 180);
    coor_rt_1(0,1) = -sin(angle * M_PI / 180);
    coor_rt_1(1,0) = sin(angle * M_PI / 180);
    coor_rt_1(1,1) = cos(angle * M_PI / 180);        
    Eigen::Matrix4f TR1 = Eigen::Matrix4f::Identity();
    TR1 = GetRotateZ(0) * GetRotateY(4.6) * GetRotateX(-3); // 축 회젆 후 기준이라 GR_X가 Y축 기준 회전
    TransformPC(input, ptr_transformed, coor_rt_1 * TR1 *  GetTranslation(0.8, 0, 0));

    sensor_msgs::PointCloud2 lidar_msg;
    pcl::toROSMsg(*ptr_transformed, lidar_msg);
    lidar_msg.header.frame_id = "/rslidar";
    lidar_pub_.publish(lidar_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "change_topic");
    TOPIC node;
    ros::spin();
    return 0;
}