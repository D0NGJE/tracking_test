#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <std_msgs/Time.h>
#include <bits/stdc++.h>
#include <sstream>
#include <time.h>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <exception>
#include <opencv2/opencv.hpp>

struct gps_data{
  std::string time_stamp;
  double latitude;
  double longitude;
  double heading;
  double velocity;
  double roll;
  double pitch;
};

std::vector<std::string> lines_;
std::vector<gps_data> gps_que_;

class LIDAR2UTM{
    public:
    LIDAR2UTM();
    private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
};

LIDAR2UTM::LIDAR2UTM() : nh_("~"){
}

void ReadTxt(void){
  std::fstream file;
  std::string file_path = "/home/dj/bag/seadronix/0823_Busan/gps_test.txt";

  std::string line;
  file.open(file_path, std::ios::in);
  if(!file.is_open()){
    std::cout << "error can't open file" << std::endl;
  }
  else{
    while(getline(file, line)){
      lines_.push_back(line);
    }
  }
  file.close();
}

void GetData(void){
  gps_data temp;
  std::string lat, lon, head, vel, r, p, gps_time;
  double latitude, longitude, heading, velocity, roll, pitch;

  for(int i = 32; i < lines_.size(); i+=3){
    if(lines_[i].find("lon") != std::string::npos){
      gps_time = lines_[i].substr(lines_[i].find("I"), 14);
      lon = lines_[i].substr(lines_[i].find("lon") + 5, 12);
      lat = lines_[i].substr(lines_[i].find("lat") + 5, 11);
      head = lines_[i+1].substr(lines_[i+1].find("heading") + 9, 11);
      vel = lines_[i+1].substr(lines_[i+1].find("velocity") + 10, 10);
      r = lines_[i+2].substr(lines_[i+2].find("roll") + 6, 11);
      p = lines_[i+2].substr(lines_[i+2].find("pitch") + 7, 10);
      longitude = std::stod(lon);
      latitude = std::stod(lat);
      heading = std::stod(head);
      velocity = std::stod(vel);
      roll = std::stod(r);
      pitch = std::stod(p);

      temp.time_stamp = gps_time;
      temp.latitude = latitude;
      temp.longitude = longitude;
      temp.time_stamp = gps_time;
      temp.heading = heading;
      temp.velocity = velocity;
      temp.roll = roll;
      temp.pitch = pitch;
      gps_que_.push_back(temp);
    }

    if(i >= lines_.size()-2){
      break;
    }
  }
}

void DataCheck(){
  int cnt;
  for(int i=0; i < gps_que_.size(); i++){
    std::string temp = gps_que_[i].time_stamp;
    if(temp.compare(gps_que_[i+1].time_stamp) == 0){
      gps_que_.erase(gps_que_.begin() + i+1);
      i--;
    }
    else{
      temp = gps_que_[i+1].time_stamp;
    }
  }
}

int main(int argc ,char** argv){
    ReadTxt();
    GetData(); // txt파일 앞의 00 부분 줄 확인하고 자를것. 32번줄까지 0이면 i=32
    DataCheck();
    ros::init(argc, argv, "lidar2utm");
    return 0;
}