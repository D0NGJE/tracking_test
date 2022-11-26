#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>


typedef struct Point_{
    pcl::PointXYZI p;
    int clusterID;
}Point;

bool point_compare(pcl::PointXYZI a, pcl::PointXYZI b){
    return a.z < b.z;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr g_seeds_pc(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr g_ground_pc(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr g_not_ground_pc(new pcl::PointCloud<pcl::PointXYZI>);

class DBSCAN{
    public:
    DBSCAN();
    typedef std::vector<pcl::PointCloud<pcl::PointXYZI> > Ring;
    typedef std::vector<Ring>                     Zone;
    private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher colorize_lidar_pub_;

    // GPF parameter
    int sensor_mode_ = 32;
    double sensor_height_ = 2.2;
    int num_min_pts_ = 10;
    int num_seg_ = 1;
    int num_iter_ = 2;
    int num_lpr_ = 30;
    double th_seeds_ = 0.1;
    double th_dist_ = 0.1;
    float d_;
    Eigen::MatrixXf normal_;
    float th_dist_d_;

    // Patchwork parameter
    double max_range_ = 100.0;
    double min_range_ = 3.2;
    double uprightness_thr_ = 0.866;
    double min_range_z2_ = 7.7;
    double min_range_z3_ = 27.1;
    double min_range_z4_ = 36.6;
    double min_range_z5_ = 70.0;
    int num_zones_ = 5;
    std::vector<int> num_sectors_each_zone_ = {32,64,54,32,16};
    std::vector<int> num_rings_each_zone_ = {4,6,6,4,4};
    std::vector<double> sector_sizes_ = {2 * M_PI / num_sectors_each_zone_.at(0), 2 * M_PI / num_sectors_each_zone_.at(1),
                                        2 * M_PI / num_sectors_each_zone_.at(2),
                                        2 * M_PI / num_sectors_each_zone_.at(3)};
    std::vector<double> ring_sizes_ = {(min_range_z2_ - min_range_) / num_rings_each_zone_.at(0),
                                      (min_range_z3_ - min_range_z2_) / num_rings_each_zone_.at(1),
                                      (min_range_z4_ - min_range_z3_) / num_rings_each_zone_.at(2),
                                      (max_range_ - min_range_z4_) / num_rings_each_zone_.at(3)};
    std::vector<double> min_ranges_ = {min_range_, min_range_z2_, min_range_z3_, min_range_z4_, min_range_z5_};
    std::vector<Zone> ConcetricZoneModel_;

    pcl::PointCloud<pcl::PointXYZI> regionwise_ground_;
    pcl::PointCloud<pcl::PointXYZI> regionwise_nonground_;

    // DBSCAN parameter
    std::vector<Point> Points_;
    double epsilon_ = 2.2;
    int min_pts_ = 8;
    int color_arr_[6][3] =
    {
        {255,0,0},
        {0,255,0},
        {0,0,255},
        {255,255,0},
        {255,0,255},
        {0,255,255}
    };

    // Functions
    void RotateZ(double degree, pcl::PointCloud<pcl::PointXYZI>& laserCloudIn, pcl::PointCloud<pcl::PointXYZI>& laserCloudTurn);
    void RemoveShip(const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTurn, const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTurn_pass);
    double XY2Theta(const double &x, const double &y);
    double XY2Radius(const double &x, const double &y);
    void GetCZM(void);
    void InitZone(Zone &z, int num_sectors, int num_rings);
    void FlushPatchesInZone(Zone &patches, int num_sectors, int num_rings);
    void EstimatePlane(void);
    void ExtractInitialSeeds(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI>& p_sorted, pcl::PointCloud<pcl::PointXYZI> &init_seeds);
    void PC2CZM(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm);
    void ExtractPiecewiseground(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &dst, pcl::PointCloud<pcl::PointXYZI> &non_ground_dst);
    void EstimateGround(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, pcl::PointCloud<pcl::PointXYZI> &cloud_out, pcl::PointCloud<pcl::PointXYZI> &cloud_nonground);
    
    void ConvertPC(const pcl::PointCloud<pcl::PointXYZI> laserCloudIn);
    void Colorize(const std::vector<Point> points, pcl::PointCloud<pcl::PointXYZRGB> &color_laser);
    std::vector<int> CalculateCluster(const pcl::KdTreeFLANN<pcl::PointXYZI> kdtree, Point &point);
    int ExpandCluster(const pcl::KdTreeFLANN<pcl::PointXYZI> kdtree, Point &cur_point, int cluster_ID);
    void DBSCANCallback(const sensor_msgs::PointCloud2ConstPtr &input_lidar_msg);
};

DBSCAN::DBSCAN() : nh_ ("~"){
    lidar_sub_ = nh_.subscribe("/lidar_data",1, &DBSCAN::DBSCANCallback, this);
    colorize_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dbscan_cloud", 1);
}

void DBSCAN::RotateZ(double degree, pcl::PointCloud<pcl::PointXYZI>& laserCloudIn, pcl::PointCloud<pcl::PointXYZI>& laserCloudTurn){
    double rad = degree * M_PI/180;
    Eigen::Affine3f rotation_z = Eigen::Affine3f::Identity(); // 3x3변환행렬 I로 초기화
    rotation_z.rotate(Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(laserCloudIn, laserCloudTurn, rotation_z);
}

void DBSCAN::RemoveShip(const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTurn, const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTurn_pass){
    pcl::PointCloud<pcl::PointXYZI>::Ptr center(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_x.setInputCloud(laserCloudTurn);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-8.0, 4.3); // -8.0, 4.3 ,  -1.1, 3.0
    pass_x.filter(*center); // x 범위 안 포인트 추출
    pass_x.setFilterLimitsNegative(true); // true = 범위 밖
    pass_x.filter(*laserCloudTurn_pass); // x 범위 밖 포인트 추출
    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pass_y.setInputCloud(center);  // x 범위 안에서 y 범위 밖 추출
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-2.0, 2.0); // -2.0, 2.0  ,  -1.1, 1.1
    pass_y.setFilterLimitsNegative(true);
    pass_y.filter(*output);
    *laserCloudTurn_pass += *output; // 병합
}

double DBSCAN::XY2Theta(const double &x, const double &y){
    auto atan_value = atan2(y,x);
    return atan_value > 0 ? atan_value : atan_value + 2*M_PI;
}

double DBSCAN::XY2Radius(const double &x, const double &y){
    return sqrt(pow(x, 2) + pow(y, 2));
}

void DBSCAN::GetCZM(void){
    for(int iter = 0; iter < num_zones_; ++iter){
        Zone z;
        InitZone(z, num_sectors_each_zone_.at(iter), num_rings_each_zone_.at(iter));
        ConcetricZoneModel_.push_back(z);
    }
}

void DBSCAN::InitZone(Zone &z, int num_sectors, int num_rings){
    z.clear();
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(1000);
    Ring ring;
    for(int i=0; i<num_sectors; i++){
        ring.emplace_back(cloud);
    }
    for(int j=0; j<num_rings; j++){
        z.emplace_back(ring);
    }
}

void DBSCAN::FlushPatchesInZone(Zone &patches, int num_sectors, int num_rings){
    for (int i = 0; i < num_sectors; i++) {
        for (int j = 0; j < num_rings; j++) {
            if (!patches[j][i].points.empty()) patches[j][i].points.clear();
        }
    }
}

void DBSCAN::EstimatePlane(void){
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    normal_ = (svd.matrixU().col(2));
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    d_ = -(normal_.transpose()*seeds_mean)(0.0);
    th_dist_d_ = th_dist_ - d_;
}

void DBSCAN::ExtractInitialSeeds(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI>& p_sorted, pcl::PointCloud<pcl::PointXYZI> &init_seeds){
    init_seeds.points.clear();
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    int init_idx = 0;
    // Calculate the mean height value.
    for (int i          = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double   lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}

void DBSCAN::PC2CZM(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm){
    for (auto const &pt : src.points) {
        int    ring_idx, sector_idx;
        double r = XY2Radius(pt.x, pt.y);
        if ((r <= max_range_) && (r > min_range_)) {
            double theta = XY2Theta(pt.x, pt.y);
            if (r < min_range_z2_) { // In First rings
                ring_idx   = std::min(static_cast<int>(((r - min_range_) / ring_sizes_[0])), num_rings_each_zone_[0] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[0])), num_sectors_each_zone_[0] - 1);
                czm[0][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z3_) {
                ring_idx   = std::min(static_cast<int>(((r - min_range_z2_) / ring_sizes_[1])), num_rings_each_zone_[1] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[1])), num_sectors_each_zone_[1] - 1);
                czm[1][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z4_) {
                ring_idx   = std::min(static_cast<int>(((r - min_range_z3_) / ring_sizes_[2])), num_rings_each_zone_[2] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[2])), num_sectors_each_zone_[2] - 1);
                czm[2][ring_idx][sector_idx].points.emplace_back(pt);
            } else { // Far!
                ring_idx   = std::min(static_cast<int>(((r - min_range_z4_) / ring_sizes_[3])), num_rings_each_zone_[3] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[3])), num_sectors_each_zone_[3] - 1);
                czm[3][ring_idx][sector_idx].points.emplace_back(pt);
            }
        }
    }
}

void DBSCAN::ExtractPiecewiseground(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &dst, pcl::PointCloud<pcl::PointXYZI> &non_ground_dst){
    // 0. Initialization
    if (!g_ground_pc->empty()) g_ground_pc->clear();
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();

    // 1. set seeds!
    ExtractInitialSeeds(zone_idx, src, *g_ground_pc);

    // 2. Extract ground
    for (int i = 0; i < num_iter_; i++){
        EstimatePlane();
        g_ground_pc->clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(src.points.size(), 3);
        int j=0;
        for (auto       &p:src.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int        r      = 0; r < result.rows(); r++) {
            if (i < num_iter_ - 1) {
                if (result[r] < th_dist_d_) {
                    g_ground_pc->points.push_back(src[r]);
                }
            } else { // Final stage
                if (result[r] < th_dist_d_) {
                    dst.points.push_back(src[r]);
                } else {
                    if (i == num_iter_ - 1) {
                        non_ground_dst.push_back(src[r]);
                    }
                }
            }
        }
    }
}

void DBSCAN::EstimateGround(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, pcl::PointCloud<pcl::PointXYZI> &cloud_out, pcl::PointCloud<pcl::PointXYZI> &cloud_nonground){
    // Ground estimation for each patch
    GetCZM();
    for(int k=0; k<num_zones_; ++k){
        FlushPatchesInZone(ConcetricZoneModel_[k], num_sectors_each_zone_[k], num_rings_each_zone_[k]);
    }
    PC2CZM(cloud_in, ConcetricZoneModel_);

    cloud_out.clear();
    cloud_nonground.clear();

    int concentric_idx = 0;
    for(int k = 0; k < num_zones_; ++k){
        auto zone = ConcetricZoneModel_[k];
        for(uint16_t ring_idx = 0; ring_idx < num_rings_each_zone_[k]; ++ring_idx){
            for(uint16_t sector_idx = 0; sector_idx < num_sectors_each_zone_[k]; ++sector_idx){
                if(zone[ring_idx][sector_idx].points.size() > num_min_pts_){
                    sort(zone[ring_idx][sector_idx].points.begin(), zone[ring_idx][sector_idx].end(), point_compare);
                    ExtractPiecewiseground(k, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);

                    const double ground_z_vec = abs(normal_(2,0));

                    if(ground_z_vec < uprightness_thr_){ // All points are rejected
                        cloud_out += regionwise_ground_;
                        cloud_nonground += regionwise_nonground_;
                    }
                    else{ // satisfy uprightness
                        cloud_out += regionwise_ground_;
                        cloud_nonground += regionwise_nonground_;
                    }
                }
            }
            ++concentric_idx;
        }
    }
}

void DBSCAN::ConvertPC(const pcl::PointCloud<pcl::PointXYZI> laserCloudIn){
  for(int i = 0; i < laserCloudIn.points.size(); i++){
    if(!std::isnan(laserCloudIn.points[i].x) && !std::isnan(laserCloudIn.points[i].y) && !std::isnan(laserCloudIn.points[i].z)){
      Point temp;
      temp.p.x = laserCloudIn.points[i].x;
      temp.p.y = laserCloudIn.points[i].y;
      temp.p.z = laserCloudIn.points[i].z;
      temp.p.intensity = laserCloudIn.points[i].intensity;
      temp.clusterID = -1;
      Points_.push_back(temp);
    }
  }
}

void DBSCAN::Colorize(const std::vector<Point> points, pcl::PointCloud<pcl::PointXYZRGB> &color_laser){
  int N = points.size();
  color_laser.clear();
  pcl::PointXYZRGB temp;
  for(int i=0; i<N; ++i){
    if(points[i].clusterID > 0){
      const auto &pt = points[i].p;
      temp.x = pt.x;
      temp.y = pt.y;
      temp.z = pt.z;
      temp.r = color_arr_[points[i].clusterID - 1][0];
      temp.g = color_arr_[points[i].clusterID - 1][1];
      temp.b = color_arr_[points[i].clusterID - 1][2];
      color_laser.push_back(temp);
    }
  }
}

std::vector<int> DBSCAN::CalculateCluster(const pcl::KdTreeFLANN<pcl::PointXYZI> kdtree, Point &point){
  std::vector<int> indices;
  std::vector<float> sqr_dists;
  kdtree.radiusSearch(point.p, epsilon_, indices, sqr_dists);
  return indices;
}

int DBSCAN::ExpandCluster(const pcl::KdTreeFLANN<pcl::PointXYZI> kdtree, Point &cur_point, int cluster_ID){
  std::vector<int> cluster_seeds = CalculateCluster(kdtree, cur_point);
  if(cluster_seeds.size() < min_pts_){
    cur_point.clusterID = -2;
    // std::cout << "(" << cur_point.p.x << ", " << cur_point.p.y << ", " << cur_point.p.z << ", " << cluster_seeds.size() << ")";
    return -3; // -3 = fail
  }
  else{
    cur_point.clusterID = cluster_ID;
    int index = 0;
    int indexCorePoint = 0;
    std::vector<int>::iterator iter_seeds;
    for(iter_seeds = cluster_seeds.begin(); iter_seeds != cluster_seeds.end(); ++iter_seeds){
      if(Points_.at(*iter_seeds).p.x == cur_point.p.x && Points_.at(*iter_seeds).p.y == cur_point.p.y && Points_.at(*iter_seeds).p.z == cur_point.p.z){
        indexCorePoint = index;
      }
      index++;
    }
    cluster_seeds.erase(cluster_seeds.begin() + indexCorePoint);

    for(std::vector<int>::size_type i = 0, n = cluster_seeds.size(); i < n; ++i){
      std::vector<int> cluster_neighbors = CalculateCluster(kdtree, Points_.at(cluster_seeds[i]));
      if(cluster_neighbors.size() >= min_pts_){
        std::vector<int>::iterator iter_neighbors;
        for(iter_neighbors = cluster_neighbors.begin(); iter_neighbors != cluster_neighbors.end(); ++iter_neighbors){
          if(Points_.at(*iter_neighbors).clusterID == -1 || Points_.at(*iter_neighbors).clusterID == -2){
            cluster_seeds.push_back(*iter_neighbors);
            n = cluster_seeds.size();
          }
          Points_.at(*iter_neighbors).clusterID = cluster_ID;
        }
      }
    }
    // std::cout << "(" << cur_point.p.x << ", " << cur_point.p.y << ", " << cur_point.p.z << ", " << cluster_seeds.size() << ")";
    return 0; // 0 = success
  }
}


void DBSCAN::DBSCANCallback(const sensor_msgs::PointCloud2ConstPtr &input_lidar_msg){
    if(input_lidar_msg->data.size() > 0){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_lidar_msg, *cloud_in);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_turn(new pcl::PointCloud<pcl::PointXYZI>);
        RotateZ(180, *cloud_in, *cloud_turn);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZI>);
        RemoveShip(cloud_turn, cloud_pass);
        EstimateGround(*cloud_pass, *g_ground_pc, *g_not_ground_pc);
        /****************************************************************************************************/
        ConvertPC(*g_not_ground_pc);
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(g_not_ground_pc);
        int clusterID = 1;
        std::vector<Point>::iterator iter;
        for(iter = Points_.begin(); iter != Points_.end(); ++iter){
            if(iter -> clusterID == -1){
                if(ExpandCluster(kdtree, *iter, clusterID) != -3){
                    clusterID += 1;
                }
            }
        }
        std::cout << "num of cluster : " << clusterID - 1 << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
        Colorize(Points_, *pc_colored);
        sensor_msgs::PointCloud2 color_lidar_msg;
        pcl::toROSMsg(*pc_colored, color_lidar_msg);  
        color_lidar_msg.header.frame_id = input_lidar_msg->header.frame_id;
        colorize_lidar_pub_.publish(color_lidar_msg);
    }
    /******************************************************************************************/
    Points_.clear();
    /******************************************************************************************/
}

int main(int argc, char** argv){
    ros::init(argc, argv, "dbscan");
    DBSCAN node;
    ros::spin();
    return 0;
}