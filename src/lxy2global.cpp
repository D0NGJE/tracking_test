#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <ros/ros.h>
// #include <opencv2/opencv.hpp>
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

struct gps_data{
  std::string time_stamp;
  double latitude;
  double longitude;
  double heading;
  double velocity;
  double roll;
  double pitch;
};

std::vector<std::vector<bool>> colors_;
std::vector<std::string> lines_;
std::vector<gps_data> gps_que_;

class LXY2UTM{
  public:
  LXY2UTM();
  private:  
  ros::NodeHandle nh_;
  ros::Subscriber lidar_sub_;
  ros::Publisher water_lidar_pub_;

//   cv::Point2d map_base_point_ = cv::Point2d(35.116426, 128.894902);
  double ref_lat_ = 35.116426;
  double ref_lon_ = 128.894902;
  double ppm_ = 0.474; // ppm = pixel per meters
  gps_data cur_gps_;

  double XY2Theta(const double &x, const double &y);
  double XY2Radius(const double &x, const double &y);
  Eigen::Matrix4f GetTR(double tr_x, double tr_y, double tr_z, double rot_x, double rot_y, double rot_z);
  pcl::PointCloud<pcl::PointXYZI> TransformationPc(pcl::PointCloud<pcl::PointXYZI> input_cloud, Eigen::Matrix4f tr_mat);
  pcl::PointCloud<pcl::PointXYZI> RotatePcNorth(pcl::PointCloud<pcl::PointXYZI> input_cloud, double degree);
  pcl::PointCloud<pcl::PointXYZI> XY2Global(pcl::PointCloud<pcl::PointXYZI> input_cloud, double degree, double e, double n);
  void WGS2UTM(double lat, double lon, double& easting, double& northing);
  long int GetStamp(const sensor_msgs::PointCloud2 lidar_msg);
  std::string UnixTimeConverter(long int seconds);
  void MatchingTime(std::string unix_time);
  void LidarCallBack(const sensor_msgs::PointCloud2ConstPtr input_lidar_msg);
};

LXY2UTM::LXY2UTM() : nh_("~"){
  lidar_sub_ = nh_.subscribe("/rslidar_points", 10, &LXY2UTM::LidarCallBack, this);
  water_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/water_lidar_points", 10);
}

double LXY2UTM::XY2Theta(const double &x, const double &y){
    auto atan_value = atan2(y,x);
    return atan_value > 0 ? atan_value : atan_value + 2*M_PI;
}

double LXY2UTM::XY2Radius(const double &x, const double &y){
    return sqrt(pow(x, 2) + pow(y, 2));
}

Eigen::Matrix4f LXY2UTM::GetTR(double tr_x, double tr_y, double tr_z, double rot_x, double rot_y, double rot_z){
    Eigen::Matrix4f transformation;
	if(rot_x == 0 && rot_y == 0 && rot_z ==0){
		transformation << 1, 0, 0, tr_x, 0, 1, 0, tr_y, 0, 0, 1, tr_z, 0, 0, 0, 1;
	}
	else{
		double gamma = rot_x * M_PI / 180; //roll
		double beta = rot_y * M_PI / 180; //pitch
		double alpha = rot_z * M_PI / 180; //yaw
		double a11 = cos(alpha) * cos(beta);
		double a12 = cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma);
		double a13 = cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
		double a14 = tr_x;
		double a21 = sin(alpha) * cos(beta);
		double a22 = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
		double a23 = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma);
		double a24 = tr_y;
		double a31 = -sin(beta);
		double a32 = cos(beta) * sin(gamma);
		double a33 = cos(beta) * cos(gamma);
		double a34 = tr_z;
		transformation << a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34, 0, 0, 0, 1;
	}
	// std::cout << transformation << std::endl;
  	return transformation;
}

pcl::PointCloud<pcl::PointXYZI> LXY2UTM::TransformationPc(pcl::PointCloud<pcl::PointXYZI> input_cloud, Eigen::Matrix4f tr_mat){
	pcl::PointCloud<pcl::PointXYZI> output;
	for(int i = 0; i < input_cloud.points.size(); i++){
		pcl::PointXYZI temp;
		pcl::PointXYZI cur = input_cloud.points[i];
		temp.x = tr_mat(0,0) * cur.x + tr_mat(0,1) * cur.y + tr_mat(0,2) * cur.z + tr_mat(0,3);
		temp.y = tr_mat(1,0) * cur.x + tr_mat(1,1) * cur.y + tr_mat(1,2) * cur.z + tr_mat(1,3);
		temp.z = tr_mat(2,0) * cur.x + tr_mat(2,1) * cur.y + tr_mat(2,2) * cur.z + tr_mat(2,3);
        temp.intensity = cur.intensity;
		output.push_back(temp);
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZI> LXY2UTM::RotatePcNorth(pcl::PointCloud<pcl::PointXYZI> input_cloud, double degree){
    double rad = degree * M_PI / 180;
    pcl::PointCloud<pcl::PointXYZI> output;
    for(int i = 0; i < input_cloud.points.size(); i++){
		pcl::PointXYZI temp;
		pcl::PointXYZI cur = input_cloud.points[i];
    double theta = XY2Theta(cur.x, cur.y);
		temp.x = cos(rad + theta) * cur.x - sin(rad + theta) * cur.y;
		temp.y = sin(rad + theta) * cur.x + cos(rad + theta) * cur.y;
		temp.z = cur.z;
    temp.intensity = cur.intensity;
		output.push_back(temp);
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZI> LXY2UTM::XY2Global(pcl::PointCloud<pcl::PointXYZI> input_cloud, double degree, double e, double n){
  double rad = degree * M_PI / 180;
  pcl::PointCloud<pcl::PointXYZI> output;
  for(auto &point : input_cloud){
    pcl::PointXYZI temp;
    double i = point.x;
    double j = - point.y;
    double r = sqrt(i * i + j * j);
    double theta = atan2(j, i);
    double rotate_angle = M_PI - rad + theta;
    double jR = r * sin(rotate_angle);
    double iR = r * cos(rotate_angle);

    temp.x = jR + e;
    temp.y = iR + n;
    temp.z = point.z;
    temp.intensity = point.intensity;
    output.push_back(temp);
  }
  return output;
}

void LXY2UTM::WGS2UTM(double lat, double lon, double& easting, double& northing){
  int zone;
  bool northp;
  GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, easting, northing);
  std::string zonestr = GeographicLib::UTMUPS::EncodeZone(zone, northp);
}

long int LXY2UTM::GetStamp(const sensor_msgs::PointCloud2 lidar_msg){
  std::string ss = "";
  uint32_t rawtime_sec = lidar_msg.header.stamp.sec;
  uint32_t rawtime_nsec = lidar_msg.header.stamp.nsec;
  ss += std::to_string(rawtime_sec);
  long int output = std::stoi(ss);
  return output;
}

std::string LXY2UTM::UnixTimeConverter(long int seconds){
  std::string ans = "I";
  int daysOfMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  long int currYear, daysTillNow, extraTime, extraDays, index, date, month, hours, minutes, secondss, flag = 0;

  // Calculate total days unix time T
  daysTillNow = seconds / (24 * 60 * 60);
  extraTime = seconds % (24 * 60 * 60);
  currYear = 1970;

  // Calculating current year
  while(daysTillNow >= 365){
    if(currYear % 400 == 0 || (currYear % 4 == 0 && currYear % 100 != 0)){
      daysTillNow -= 366;
    }
    else{
      daysTillNow -= 365;
    }
    currYear += 1;
  }

  // Updating extradays because it
  // will give days till previous day
  // and we have include current day
  extraDays = daysTillNow + 1;

  if (currYear % 400 == 0 || (currYear % 4 == 0 && currYear % 100 != 0)){
    flag = 1;
  }

  // Calculating MONTH and DATE
  month = 0;
  index = 0;
  if(flag == 1){
    while(true){
      if(index == 1){
        if(extraDays - 29 < 0){
          break;
        }
        month += 1;
        extraDays -= 29;
      }
      else{
        if(extraDays - daysOfMonth[index] < 0) {
          break;
        }
        month += 1;
        extraDays -= daysOfMonth[index];
      }
      index += 1;
    }
  }
  else{
    while(true){
      if(extraDays - daysOfMonth[index] < 0){
        break;
      }
      month += 1;
      extraDays -= daysOfMonth[index];
      index += 1;
    }
  }

  // Current Month
  if(extraDays > 0){
    month += 1;
    date = extraDays;
  }
  else{
    if(month == 2 && flag == 1){
      date = 29;
    }
    else{
      date = daysOfMonth[month - 1];
    }
  }

  // Calculating HH:MM:YYYY
  hours = extraTime / 3600;
  minutes = (extraTime % 3600) / 60;
  secondss = (extraTime % 3600) % 60;

  if(month < 10){
    ans += "0";
    ans += std::to_string(month);
  }
  else{
    ans += std::to_string(month);
  }

  if(date < 10){
    ans += "0";
    ans += std::to_string(date);
  }
  else{
    ans += std::to_string(date);
  }
  ans += " ";

  if(hours + 9 < 10){
    ans += "0";
    ans += std::to_string(hours + 9);
  }
  else{
    ans += std::to_string(hours + 9);
  }
  ans += ":";

  if(minutes < 10){
    ans += "0";
    ans += std::to_string(minutes);
  }
  else{
    ans += std::to_string(minutes);
  }
  ans += ":";

  if(secondss < 10){
    ans += "0";
    ans += std::to_string(secondss);
  }
  else{
    ans += std::to_string(secondss);
  }
  // strptime();
  // ans += std::to_string(currYear);
  // ans += "/";
  return ans;
}

void LXY2UTM::MatchingTime(std::string unix_time){
  gps_data pre_gps;
  for(int i = 0; i<gps_que_.size(); i++){
    if(unix_time.compare(gps_que_[i].time_stamp) == 0){
      // std::cout << "success "<< gps_que_[i].time_stamp << ", " << unix_time << std::endl;
      cur_gps_.latitude = gps_que_[i].latitude;
      cur_gps_.longitude = gps_que_[i].longitude;
      cur_gps_.time_stamp = gps_que_[i].time_stamp;
      cur_gps_.heading = gps_que_[i].heading;
      cur_gps_.pitch = gps_que_[i].pitch;
      cur_gps_.roll = gps_que_[i].roll;
      cur_gps_.velocity = gps_que_[i].velocity;
      // std::cout << std::setprecision(9) << "lat : " << gps_msg_.latitude << ", lon : " << gps_msg_.longitude << std::endl;
      pre_gps = cur_gps_;
    }
    else{
      // std::cout << "fail "<< gps_que_[i].time_stamp << ", " << unix_time << std::endl;
      cur_gps_ = pre_gps;
    }
  }
}

void LXY2UTM::LidarCallBack(const sensor_msgs::PointCloud2ConstPtr lidar_input_msg){
  long int second = GetStamp(*lidar_input_msg);
  std::string stamp = UnixTimeConverter(second);
  MatchingTime(stamp);
  double cur_E, cur_N = 0;
  WGS2UTM(cur_gps_.latitude, cur_gps_.longitude, cur_E, cur_N);
  double ref_E, ref_N = 0;
  WGS2UTM(ref_lat_, ref_lon_, ref_E, ref_N);
  // std::cout << std::setprecision(11)  << cur_gps_.time_stamp << ", "
  //           << cur_gps_.latitude << ", " << cur_gps_.longitude << ", "
  //           << cur_gps_.heading << ", " << cur_gps_.velocity << ", "
  //           << cur_gps_.roll << ", " << cur_gps_.pitch << ", " 
  //           << cur_E << ", " << cur_N << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_input(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_north(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_output(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*lidar_input_msg, *lidar_input);

  std::fstream file;
  file.open("/home/dj/paper_ws/src/tracking_test/img_data/lidar_local.txt", std::ios::out);
  for(int i = 0;i < lidar_input->points.size(); i++){
    if(!isnan(lidar_input->points[i].x) && !isnan(lidar_input->points[i].y)){
        file << std::setprecision(11)<<  lidar_input->points[i].x << ", " << lidar_input->points[i].y << std::endl;
    }
  }
  std::cout << "file1 write" << std::endl;
  file.close();

  double heading_rad = cur_gps_.heading * M_PI / 180;
  Eigen::Matrix4f transform_north = Eigen::Matrix4f::Identity();
  *lidar_north = XY2Global(*lidar_input, 180 - cur_gps_.heading, 0, 0);
  std::cout << cur_gps_.heading << std::endl;
  std::fstream file2;
  file2.open("/home/dj/paper_ws/src/tracking_test/img_data/lidar_global.txt", std::ios::out);
  for(int i = 0;i < lidar_north->points.size(); i++){
    if(!isnan(lidar_north->points[i].x) && !isnan(lidar_north->points[i].y)){
      pcl::PointXYZI temp = lidar_north->points[i];
      pcl::PointXYZI global_temp = temp;
      global_temp.x = temp.x + cur_E;
      global_temp.y = temp.x + cur_N;
      file2 << std::setprecision(11)<<  temp.x << ", " << temp.y << std::endl;
    }
  }
  file2.close();
  std::cout << "file2 write" << std::endl;
    //   double diff_E = ref_E - global_temp.x;
    //   double diff_N = ref_N - global_temp.y;
      
    //   int cor_diff_e, cor_diff_n = 0;
    //   if(diff_E > 0){cor_diff_e = ceil(diff_E * ppm_);}
    //   else{cor_diff_e = -ceil(abs(diff_E * ppm_));}
    //   if(diff_N > 0){cor_diff_n = ceil(diff_N * ppm_);}
    //   else{cor_diff_n = -ceil(abs(diff_N * ppm_));}
      
    //   // std::cout << sea << std::endl;      
    //   if(abs(diff_E) < 500 * ppm_ && abs(diff_N) < 500 * ppm_){ // base map 안 이면
    //     if(colors_[500 - cor_diff_e - 1][500 - cor_diff_n - 1]){
    //       lidar_output->push_back(lidar_north->points[i]);
    //       sea++;
    //     }
    //     else{
    //       ground++;
    //     }
    //   }

  sensor_msgs::PointCloud2 water_lidar_msg;
  pcl::toROSMsg(*lidar_output, water_lidar_msg);
  water_lidar_msg.header.frame_id = lidar_input_msg->header.frame_id;
  water_lidar_pub_.publish(water_lidar_msg);
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

int main(int argc, char** argv){
    ReadTxt();
    GetData(); // txt파일 앞의 00 부분 줄 확인하고 자를것. 32번줄까지 0이면 i=32
    DataCheck();
    std::cout << "read txt ok" << std::endl;

    ros::init(argc, argv, "lxy2global");
    LXY2UTM node;
    ros::spin();
    return 0;
}