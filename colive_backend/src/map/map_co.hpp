
#pragma once

#include <set>
// C++
#include <mutex>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

// #include <boost/serialization/vector.hpp>
// #include <boost/serialization/export.hpp>


// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>



// #include <eigen3/Eigen/Core>

// #include "tools/tools_eigen.hpp"
#include "typedefs_base.hpp"
#include "map_rgb.hpp"
#include "config_backend.hpp"
namespace colive {

// struct Global_map;


struct MapInstance {
    using TransformType                 = TypeDefs::TransformType;
    using MapPtr                        = TypeDefs::MapPtr;
    using MapInstancePtr                = std::shared_ptr<MapInstance>;

    MapInstance()                                                                       = delete;
    MapInstance(int id);
    MapInstance(MapInstancePtr map_target, MapInstancePtr map_tofuse, TransformType T_wtofuse_wmatch);
    MapInstance(MapPtr external_map);

    MapPtr                      map;
    int                         usage_cnt                                               = 0;
    std::set<int>               check_nums;
    bool                        block_checkout                                          = false;
};

class Map: public std::enable_shared_from_this<Map>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;
    using MapPtr                        = TypeDefs::MapPtr;
    using TransformType                 = TypeDefs::TransformType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using PointCloud                 = TypeDefs::PointCloud;
    using PointCloudPtr                 = TypeDefs::PointCloudPtr;
    using PointCloudEX               = TypeDefs::PointCloudEX;
    using PointCloudEXPtr               = TypeDefs::PointCloudEXPtr;
    using ImageEXPtr          = TypeDefs::ImageEXPtr;
    using ImagePtr          = TypeDefs::ImagePtr;
    using KeyframeMap                   = TypeDefs::KeyframeMap;
    using LandmarkMap                   = TypeDefs::LandmarkMap;
    using PointCloudMap                 = TypeDefs::PointCloudMap;
    using PointCloudEXMap               = TypeDefs::PointCloudEXMap;
    using ImageMap                 = TypeDefs::ImageMap;
    using ImageEXMap               = TypeDefs::ImageEXMap;

    using LoopVector                    = TypeDefs::LoopVector;

    using ThreadPtr                  = TypeDefs::ThreadPtr;
    // using KeyframeVector                = TypeDefs::KeyframeVector;
    // using LandmarkVector                = TypeDefs::LandmarkVector;
    // using LandmarkSet                   = TypeDefs::LandmarkSet;


    // pcl::PointCloud<pcl::PointXYZ>::Ptr map_raw_pts;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_rbg_pts; // or as landmark
    Global_map m_map_rgb_pts; 
    //
    
    // std::pair<int,int> map_;
    std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;//适用于需要实现先进先出（FIFO）的队列操作，不需要直接访问队列的其他位置。

    double last_timestamp_lidar = -1;

    // std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;//适用于需要在队列两端进行频繁插入和删除操作的场景，同时需要保持随机访问的能力。
    // std::deque<sensor_msgs::PointCloud2::ConstPtr> image_buffer;
    // std::deque<sensor_msgs::PointCloud2::ConstPtr> odom_buffer;

    uint8_t map_type=0; // 0: unknow, 1: camera, 2: laser, 3: Cam + laser
public:

    Map()=delete;
    Map(size_t id);
    Map(MapPtr map_target, MapPtr map_tofuse, TransformType T_wtofuse_wmatch);
    virtual auto GetFamilyPc(size_t client_id)->TransformType;
    virtual auto GetFamilyImg(size_t client_id)->TransformType;
    virtual auto GetPointCloudEX(idpair idp)        ->PointCloudEXPtr;
    virtual auto GetPointCloudEXs()            ->PointCloudEXMap;
    virtual auto GetImageEX(idpair idp)        ->ImageEXPtr;
    virtual auto GetImageEXs()            ->ImageEXMap;
    virtual auto Display()->void;

    // virtual auto AddPointCloud(PointCloudEXPtr pc)->void;
    virtual auto AddImage(ImageEXPtr img, bool suppress_output=false)->void ;
    virtual auto AddPointCloud(PointCloudEXPtr pc, bool suppress_output=false)->void;
    virtual auto AddPointCloud_large(PointCloudEXPtr pc, bool suppress_output=false)->void;
    virtual auto Add2RGBMap_service()->void;
    virtual auto Add2RGBMap(PointCloudEXPtr pc)->void;
    // virtual auto RenderRGBMap_service()->void;

    virtual auto AddLoopConstraint(LoopConstraint lc)                                   ->void;
    virtual auto GetLoopConstraints()                                                   ->LoopVector;

    // void feat_points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg_in);
    // void image_callback(const sensor_msgs::ImageConstPtr &msg);
    // void image_comp_callback(const sensor_msgs::CompressedImageConstPtr &msg);
    // new(); //
    // feature_vectror
    // odom;
public:
    // Identifier
    size_t                      id_map_      = std::numeric_limits<size_t>::max();
    std::set<size_t>            associated_clients_; // set of clients
    TransformType               T_lm_w_= TransformType::Identity(); // 单机地图与全局地图的估计位姿关系 传感器相对于世界坐标系的位置和方向/

    bool have_real_pos=false;  // 是否与世界坐标系建立了联系

    int m_number_of_new_visited_voxel = 0;
protected:

    // Data
    // vector<PointCloudXYZI::Ptr>
    KeyframeMap                 keyframes_;
    LandmarkMap                 landmarks_;
    PointCloudEXMap             pointclouds_;// 收集到的所用lidar frame
    // // 几帧合一的大型点云
    // PointCloudEXPtr             p_pc_large_tmp =nullptr;
    size_t                      m_pc_large_frame=0;
    PointCloudEXMap             pointclouds_large_; // id 
    ImageEXMap                  images_;

    KeyframeMap                 keyframes_erased_;

    std::queue<PointCloudEXPtr> pcs_should_be_added_to_rgb_map;
    std::queue<PointCloudEXPtr> imgs_should_be_added_to_rgb_map;
    ThreadPtr                   thread_rgb_map_;
    // std::thread                  thread_rgb_map_;
    // Loop Correction
    LoopVector                  loop_constraints_;

    size_t                      max_id_img_                                              = 0;
    size_t                      max_id_lm_                                              = 0;
    size_t                      max_id_pc_                                              = 0;
    // Sync
    std::mutex                  mtx_map_;
    std::mutex                  mtx_update_;
};



}