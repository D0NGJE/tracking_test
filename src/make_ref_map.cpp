#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Time.h>
#include <bits/stdc++.h>
#include <sstream>
#include <time.h>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <exception>
#include <opencv4/opencv2/opencv.hpp>

double XY2Theta(const double &x, const double &y){
    auto atan_value = atan2(y,x);
    return atan_value > 0 ? atan_value : atan_value + 2*M_PI;
}

double XY2Radius(const double &x, const double &y){
    return sqrt(pow(x, 2) + pow(y, 2));
}

Eigen::Matrix4f GetTranslation(double tr_x, double tr_y, double tr_z){
	Eigen::Matrix4f transformation;
    transformation << 1, 0, 0, tr_x, 
                      0, 1, 0, tr_y, 
                      0, 0, 1, tr_z, 
                      0, 0, 0, 1;
  	return transformation;
}

Eigen::Matrix4f GetRotateX(double deg){
	Eigen::Matrix4f transformation;
    double rad = deg * M_PI / 180;
    transformation << 1, 0, 0, 0, 
                      0, cos(rad), -sin(rad), 0, 
                      0, sin(rad), cos(rad), 0, 
                      0, 0, 0, 1;
  	return transformation;
}

Eigen::Matrix4f GetRotateY(double deg){
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
                      0, 0, 1, 0, 
                      0, 0, 0, 1;
  	return transformation;
}

void TransformPC(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4f TR){
    for(int i = 0; i < input->points.size(); i++){
        output->points[i].x = TR(0,0) * input->points[i].x + TR(0,1) * input->points[i].y + TR(0,2) * input->points[i].z + TR(0,3); 
        output->points[i].y = TR(1,0) * input->points[i].x + TR(1,1) * input->points[i].y + TR(1,2) * input->points[i].z + TR(1,3); 
        output->points[i].z = TR(2,0) * input->points[i].x + TR(2,1) * input->points[i].y + TR(2,2) * input->points[i].z + TR(2,3); 
    }
}

void RoatateNorth(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, double degree, double e, double n){
  output_cloud->resize(input_cloud->points.size());
  if(degree <= 180){
    degree = degree * (-1);
  }
  else{
    degree = 360 - degree;
  }
  double rad = degree * M_PI / 180;
  for(int i=0; i<input_cloud->points.size(); i++){
    output_cloud->points[i].x = input_cloud->points[i].x * cos(rad) - input_cloud->points[i].y * sin(rad) + e;
    output_cloud->points[i].y = input_cloud->points[i].x * sin(rad) + input_cloud->points[i].y * cos(rad) + n; 
    output_cloud->points[i].z = input_cloud->points[i].z;
  }
}

void XY2Global(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, double degree, double e, double n){
    if(degree <= 180){
        degree = degree;
    }
    else{
        degree = (360 - degree) * (-1);
    }
    double rad = degree * M_PI / 180;
    double rotate_angle = M_PI - rad;

    for(auto &point : *input_cloud){
        pcl::PointXYZ temp;
        double i = point.x;
        double j = - point.y;
        double r = sqrt(i * i + j * j);
        double theta = atan2(j, i);
        double jR = r * sin(rotate_angle + theta);
        double iR = r * cos(rotate_angle + theta);

        temp.x = jR;
        temp.y = iR;
        temp.z = point.z;
        output_cloud->push_back(temp);
    }
}

void WGS2UTM(double lat, double lon, double& easting, double& northing){
  int zone;
  bool northp;
  GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, easting, northing);
  std::string zonestr = GeographicLib::UTMUPS::EncodeZone(zone, northp);
}

std::vector<u_int8_t> flatten(const std::vector<std::vector<int>> &before_v){
    std::vector<u_int8_t> flatten_v;
    for(auto &element : before_v){
        flatten_v.insert(flatten_v.end(), element.begin(), element.end());
    }
    return flatten_v;
}

void make2DMap(double heading, float leaf_size, const Eigen::Vector2f &offset, const Eigen::Vector2f &map_size){
    double ppm = 0.948;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string ref_map_path = "/home/dj/paper_ws/src/tracking_test/lidar_data/ref_map_1_out.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (ref_map_path, *map_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr north_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    north_map_cloud->resize(map_cloud->points.size());

    if(heading <= 180){
        heading = heading * (-1);
    }
    else{
        heading = 360 - heading;
    }
    double rad = heading * M_PI / 180;
    for(int i=0; i<map_cloud->points.size(); i++){
        north_map_cloud->points[i].x = map_cloud->points[i].x * cos(rad) - map_cloud->points[i].y * sin(rad);
        north_map_cloud->points[i].y = map_cloud->points[i].x * sin(rad) + map_cloud->points[i].y * cos(rad); 
        north_map_cloud->points[i].z = 0.0;
    }

    std::vector<std::vector<int>> map;
    map.resize(map_size(0,0)/leaf_size);
    for(auto& cell : map) cell.resize(map_size(1,0)/leaf_size);

    for(auto& point : north_map_cloud->points){
        Eigen::Vector2f p;
        p << static_cast<int>(point.x / leaf_size / ppm) - offset(0,0), static_cast<int>(point.y / leaf_size / ppm) - offset(1,0);
        if( (p(0,0) >= 0 && p(0,0) < map.size()) && (p(1,0) >= 0 && p(1,0) < map.size()) ){
            map[(map.size() - 1) - p(0,0)][(map.size() - 1) - p(1,0)] = 255;
        }
    }

    auto flatten_map = flatten(map);

    cv::Mat mat(map.size(), map.size(), CV_8U, flatten_map.data());
    cv::resize(mat,mat, cv::Size(400,400));
    cv::imwrite("/home/dj/paper_ws/src/tracking_test/lidar_data/2d_map.bmp", mat);
    std::string title_name = "Grey image";
    // cv::namedWindow(title_name, cv::WINDOW_AUTOSIZE);
    // cv::imshow(title_name, mat);
}

int main(int argc, char** argv){
    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_map_1 (new pcl::PointCloud<pcl::PointXYZ>);
    std::string ref_map_path = "/home/dj/paper_ws/src/tracking_test/lidar_data/ref_map_1_out.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (ref_map_path, *ref_map_1);
    double ref_map_1_lat = 34.99157190;
    double ref_map_1_lon = 128.676800;
    double ref_map_1_heading = 270; // deg
    double ref_1_e = 0.0;
    double ref_1_n = 0.0;
    WGS2UTM(ref_map_1_lat, ref_map_1_lon, ref_1_e, ref_1_n);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_map_1_north (new pcl::PointCloud<pcl::PointXYZ>);
    RoatateNorth(ref_map_1, ref_map_1_north, ref_map_1_heading, ref_1_e, ref_1_n);
    // XY2Global(ref_map_1, ref_map_1_north, ref_map_1_heading, ref_1_e, ref_1_n);
    writer.write("/home/dj/paper_ws/src/tracking_test/lidar_data/north_ref_map_1.pcd", *ref_map_1_north, false);
    /*******************************************************************************************************************************************/
    Eigen::Vector2f offset, map_size;
    map_size << 400, 400;
    offset << -(map_size(0,0) * 0.5), -(map_size(1,0) * 0.5); 
    make2DMap(ref_map_1_heading, 1.0, offset, map_size);
    /*******************************************************************************************************************************************/
    std::fstream file2;
    file2.open("/home/dj/paper_ws/src/tracking_test/lidar_data/ref_north_global.txt", std::ios::out);
    for(int i = 0;i < ref_map_1_north->points.size(); i++){
        if(!isnan(ref_map_1_north->points[i].x) && !isnan(ref_map_1_north->points[i].y)){
            pcl::PointXYZ temp = ref_map_1_north->points[i];
            file2 << std::setprecision(11)<<  temp.x << ", " << temp.y << std::endl;
        }
    }
    file2.close();
    /*******************************************************************************************************************************************/
    return 0;
}