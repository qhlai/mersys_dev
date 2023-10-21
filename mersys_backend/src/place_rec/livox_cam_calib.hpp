#pragma once
#include <iostream>
#include <optional>
// #include <pcl/point_types.h>
#include "typedefs_base.hpp"


// #include <ad_localization_msgs/NavStateInfo.h>
// #include "scancontext/Scancontext.h"
// #include "mapmanager.hpp"
#include "map_rgb.hpp"



// https://github.com/hku-mars/livox_camera_calib
namespace mersys {

class Camera
{
public:
  float fx_, fy_, cx_, cy_, k1_, k2_, p1_, p2_, k3_, k4_, s_;
  int width_, height_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat init_ext_;
  Eigen::Matrix3d ext_R; // 初始旋转矩阵
  Eigen::Vector3d ext_t; // 初始平移向量

  cv::Mat rgb_img_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rgb_edge_cloud_;

  void update_Rt(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
  {
    ext_R << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
    ext_t << t(0), t(1), t(2);
  }
};

class Calibration {
public:
    using TransformType                 = TypeDefs::TransformType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Vector6Type                   = TypeDefs::Vector6Type;
    using QuaternionType                = TypeDefs::QuaternionType;

    using idpair                     = TypeDefs::idpair;

    using PointType                     = TypeDefs::PointType;
    using PointCloud                    = TypeDefs::PointCloud;
    using PointCloudPtr                    = TypeDefs::PointCloudPtr;
    using PointCloudMap = TypeDefs::PointCloudMap;
    using PointCloudEX  = TypeDefs::PointCloudEX; 
    using PointCloudEXPtr  = TypeDefs::PointCloudEXPtr; 
    using PointCloudEXList  = TypeDefs::PointCloudEXList; 
    using Image                         = TypeDefs::Image;
    using ImageEX                       = TypeDefs::ImageEX;
    using ImageEXPtr                    = TypeDefs::ImageEXPtr;
    using ImagePtr                      = TypeDefs::ImagePtr;
    using MapPtr                        = TypeDefs::MapPtr;
    using MapManagerPtr                 = TypeDefs::MapManagerPtr;
    enum ProjectionType { DEPTH, INTENSITY, BOTH };
    enum Direction { UP, DOWN, LEFT, RIGHT };
#if 0
  ros::NodeHandle _nh;

  // ROS
  ros::Publisher pub_plane = _nh.advertise<sensor_msgs::PointCloud2>("/voxel_plane", 100);
  ros::Publisher pub_edge = _nh.advertise<sensor_msgs::PointCloud2>("/lidar_edge", 100);
  ros::Publisher pub_color_cloud = _nh.advertise<sensor_msgs::PointCloud2>("/color_cloud", 100);
  ros::Publisher pub_residual = _nh.advertise<sensor_msgs::PointCloud2>("/residual", 1000);
  ros::Publisher pub_direct = _nh.advertise<visualization_msgs::MarkerArray>("/direct", 1000);
#endif

public:
    // using PCSurfFrame                        = std::shared_ptr<std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*>>;
    // using PCSurfFramesVector                 = std::vector<PCSurfFrame,Eigen::aligned_allocator<PCSurfFrame>>;
    // std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_frame;
    // using LidarEdgeMap    = std::map<idpair,PointCloudPtr,std::less<idpair>,Eigen::aligned_allocator<std::pair<const idpair,PointCloudPtr>>>;
    PointCloudMap lidar_edge_cloud_map_;
    std::map<idpair,std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>,std::less<idpair>,Eigen::aligned_allocator<std::pair<const idpair,std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>>> image_edge_cloud_map_;
    // enum ProjectionType { DEPTH, INTENSITY, BOTH };
public:
    Calibration();
    virtual auto add_lidar(PointCloudEXPtr pc)        ->void;
    virtual auto add_img(ImageEXPtr img,bool if_undistort)        ->void;


};




}