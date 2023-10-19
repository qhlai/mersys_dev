#pragma once
#include <iostream>
#include <optional>
// #include <pcl/point_types.h>
#include "typedefs_base.hpp"
#include "config_backend.hpp"
#include "calib/common.h"
#include "calib/mypcl.hpp"
#include "calib/ba.hpp"
#include "calib/tools.hpp"
// #include <ad_localization_msgs/NavStateInfo.h>
// #include "scancontext/Scancontext.h"
// #include "mapmanager.hpp"
#include "map_rgb.hpp"



// https://github.com/hku-mars/livox_camera_calib
namespace mersys {
class PointCloud_Calib{
    public:
        std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_edge_cloud_; // 存储平面相交得到的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_; // 存储原始点云
}
class Calibration {
public:
    using TransformType                 = TypeDefs::TransformType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Vector6Type                   = TypeDefs::Vector6Type;
    using QuaternionType                = TypeDefs::QuaternionType;

    using PointType                     = TypeDefs::PointType;
    using PointCloud                    = TypeDefs::PointCloud;
    using PointCloudPtr                    = TypeDefs::PointCloudPtr;
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
    using PCSurfFrame                        = std::shared_ptr<std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*>>;
    using PCSurfFramesVector                 = std::vector<PCSurfFrame,Eigen::aligned_allocator<PCSurfFrame>>;
    // std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_frame;

    
    // enum ProjectionType { DEPTH, INTENSITY, BOTH };
public:
    auto Calibration::Calibration() -> void;
    auto add_lidar(PointCloudEXPtr pc)        ->void;
    auto add_img(ImageEXPtr img,bool if_undistort)        ->void;


};

}