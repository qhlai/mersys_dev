#pragma once

#include <set>
// C++
#include <mutex>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

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

namespace colive {


class Map: public std::enable_shared_from_this<Map>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using MapPtr                        = TypeDefs::MapPtr;
    using TransformType                 = TypeDefs::TransformType;


    pcl::PointCloud<pcl::PointXYZ>::Ptr map_raw_pts;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_rbg_pts; // or as landmark
    
    //
    // std::pair<int,int> map_;
    std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;//适用于需要实现先进先出（FIFO）的队列操作，不需要直接访问队列的其他位置。

    double last_timestamp_lidar = -1;
    std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;//适用于需要在队列两端进行频繁插入和删除操作的场景，同时需要保持随机访问的能力。

    std::deque<sensor_msgs::PointCloud2::ConstPtr> image_buffer;

    std::deque<sensor_msgs::PointCloud2::ConstPtr> odom_buffer;

    uint8_t map_type=0; // 0: unknow, 1: camera, 2: laser, 3: Cam + laser
public:

    Map()=delete;
    Map(size_t id);
    Map(MapPtr map_target, MapPtr map_tofuse, TransformType T_wtarget_wtofuse);
    // void feat_points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg_in);
    // void image_callback(const sensor_msgs::ImageConstPtr &msg);
    // void image_comp_callback(const sensor_msgs::CompressedImageConstPtr &msg);
    // new(); //
    // feature_vectror
    // odom;
public:
    // Identifier
    size_t                      id_map_      = std::numeric_limits<size_t>::max();
    std::set<size_t>            associated_clients_;
protected:

    std::mutex mutex_image_callback;
    std::mutex mutex_lidar_callback;
    std::mutex mutex_odom_callback;
    // Sync
    std::mutex                  mtx_map_;
    std::mutex                  mtx_update_;
};

struct MapInstance {
    using TransformType                 = TypeDefs::TransformType;
    using MapPtr                        = TypeDefs::MapPtr;
    using MapInstancePtr                = std::shared_ptr<MapInstance>;

    MapInstance()                                                                       = delete;
    MapInstance(int id);
    MapInstance(MapInstancePtr map_target, MapInstancePtr map_tofuse, TransformType T_wmatch_wtofuse);
    MapInstance(MapPtr external_map);

    MapPtr                      map;
    int                         usage_cnt                                               = 0;
    std::set<int>               check_nums;
    bool                        block_checkout                                          = false;
};

class MapManager{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using idpair                        = TypeDefs::idpair;
    using MapPtr                        = TypeDefs::MapPtr;
    using MapInstancePtr                = TypeDefs::MapInstancePtr;
    using MapContainer                  = std::map<int,MapInstancePtr>;

protected:


};

}