#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

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

int main(void){
    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);

    std::string filepath_1 = "/home/dj/Downloads/cloud_1.pcd";
    std::string filepath_2 = "/home/dj/Downloads/cloud_2.pcd"; 

    pcl::io::loadPCDFile<pcl::PointXYZ> (filepath_1, *cloud_1);
    pcl::io::loadPCDFile<pcl::PointXYZ> (filepath_2, *cloud_2);


    std::cout << "****Points size****" << std::endl;
    std::cout << "cloud_1 : " << cloud_1->points.size() << std::endl;
    std::cout << "cloud_2 : " << cloud_2->points.size() << std::endl;

    Eigen::Matrix4f coor_rt_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f coor_rt_2 = Eigen::Matrix4f::Identity();
    double angle = 90;
    coor_rt_1(0,0) = cos(angle * M_PI / 180);
    coor_rt_1(0,1) = -sin(angle * M_PI / 180);
    coor_rt_1(1,0) = sin(angle * M_PI / 180);
    coor_rt_1(1,1) = cos(angle * M_PI / 180);

    coor_rt_2(0,0) = cos(angle * M_PI / 180);
    coor_rt_2(0,1) = sin(angle * M_PI / 180);
    coor_rt_2(1,0) = -sin(angle * M_PI / 180);
    coor_rt_2(1,1) = cos(angle * M_PI / 180);

    Eigen::Matrix4f TR1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f TR2 = Eigen::Matrix4f::Identity();
    TR1 = coor_rt_1 * GetRotateZ(0) * GetRotateY(4.6) * GetRotateX(-3) * GetTranslation(0.8,0,0);
    TR2 = coor_rt_2 * GetRotateZ(0) * GetRotateY(8.1) * GetRotateX(4.1) * GetTranslation(0.8,0,0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr tr_cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
    tr_cloud_1->resize(cloud_1->points.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tr_cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
    tr_cloud_2->resize(cloud_2->points.size());
    TransformPC(cloud_1, tr_cloud_1, TR1);
    TransformPC(cloud_2, tr_cloud_2, TR2);
    std::cout << "tr_cloud_1 : " << tr_cloud_1->points.size() << std::endl;
    std::cout << "tr_cloud_2 : " << tr_cloud_2->points.size() << std::endl;
    writer.write("/home/dj/paper_ws/src/tracking_test/lidar_data/tr_cloud_1.pcd", *tr_cloud_1, false);
    writer.write("/home/dj/paper_ws/src/tracking_test/lidar_data/tr_cloud_2.pcd", *tr_cloud_2, false);
    /**********************************************************************************************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_combine(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto &p : *tr_cloud_1){
        cloud_combine->push_back(p);
    }
    for(auto &p : *tr_cloud_2){
        cloud_combine->push_back(p);
    }
    std::cout << "combine : " << cloud_combine->points.size() << std::endl;
    // writer.write("/home/dj/paper_ws/src/tracking_test/lidar_data/combine_test.pcd", *cloud_combine, false);
  /**********************************************************************************************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    float voxelsize = 0.2;
    voxel.setInputCloud(cloud_combine);
    voxel.setLeafSize(voxelsize, voxelsize, voxelsize);
    voxel.filter(*voxel_cloud);
    std::cout << "voxelize : " << voxel_cloud->points.size() << std::endl;
    // writer.write("/home/dj/paper_ws/src/tracking_test/lidar_data/voxel_test.pcd", *voxel_cloud, false);
    /**********************************************************************************************/

    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_map_pt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_map_out(new pcl::PointCloud<pcl::PointXYZ>);
    ref_map_out->resize(ref_map_pt->points.size());
    std::string map_path = "/home/dj/paper_ws/src/tracking_test/lidar_data/ref_map_1.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (map_path, *ref_map_pt);
    for(auto &point: *ref_map_pt){
        if(point.x >19){
            ref_map_out->push_back(point);
        }
    }
    writer.write("/home/dj/paper_ws/src/tracking_test/lidar_data/ref_map_1_out.pcd", *ref_map_out, false);

    return 0;
}