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

class Map_V {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using TransformType                 = TypeDefs::TransformType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;

    using KeyframeMap                   = TypeDefs::KeyframeMap;
    using LandmarkMap                   = TypeDefs::LandmarkMap;
    using KeyframeVector                = TypeDefs::KeyframeVector;
    using LandmarkVector                = TypeDefs::LandmarkVector;
    using LandmarkSet                   = TypeDefs::LandmarkSet;

public:
    Map_V(size_t id);

    // Getter
    virtual auto GetKeyframe(idpair idp, bool expect_null = false)                      ->KeyframePtr;
    virtual auto GetKeyframe(size_t kf_id, size_t client_id,
                             bool expect_null = false)                                  ->KeyframePtr {
        return GetKeyframe(std::make_pair(kf_id,client_id), expect_null);
    }
    virtual auto GetKeyframes()                                                         ->KeyframeMap;
    virtual auto GetKeyframesVec()                                                      ->KeyframeVector;
    virtual auto GetKeyframesErased()                                                   ->KeyframeMap;
    virtual auto GetLandmark(idpair idp)                                                ->LandmarkPtr;
    virtual auto GetLandmark(size_t lm_id, size_t client_id)                            ->LandmarkPtr {
        return GetLandmark(std::make_pair(lm_id,client_id));
    }
    virtual auto GetLandmarks()                                                         ->LandmarkMap;
    virtual auto GetLandmarksVec()                                                      ->LandmarkVector;

    virtual auto GetMaxKfId()                                                           ->size_t;
    virtual auto GetMaxLmId()                                                           ->size_t;

    // Add / Erase data
    virtual auto AddKeyframe(KeyframePtr kf)                                            ->void      = 0;
    virtual auto AddLandmark(LandmarkPtr lm)                                            ->void      = 0;
    virtual auto EraseKeyframe(KeyframePtr kf, bool mtx_lock = true)                    ->bool      = 0;
    virtual auto EraseLandmark(LandmarkPtr lm, bool mtx_lock = true)                    ->bool      = 0;

    // Clear Map
    virtual auto Clear()                                                                ->void;

    // Sync
    virtual auto LockMapUpdate()                                                        ->void {
        mtx_update_.lock();
    }
    virtual auto UnLockMapUpdate()                                                      ->void {
        mtx_update_.unlock();
    }

    // Identifier
    size_t                      id_map_                                                 = std::numeric_limits<size_t>::max();
    std::set<size_t>            associated_clients_;

protected:
    // Data
    KeyframeMap                 keyframes_;
    LandmarkMap                 landmarks_;

    KeyframeMap                 keyframes_erased_;

    size_t                      max_id_kf_                                              = 0;
    size_t                      max_id_lm_                                              = 0;

    // Sync
    std::mutex                  mtx_map_;
    std::mutex                  mtx_update_;
};

// class Map_L {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     using precision_t                   = TypeDefs::precision_t;
//     using idpair                        = TypeDefs::idpair;


// public:
//     MapBase(size_t id);

//     // Getter
//     virtual auto GetKeyframe(idpair idp, bool expect_null = false)                      ->KeyframePtr;
//     virtual auto GetKeyframe(size_t kf_id, size_t client_id,
//                              bool expect_null = false)                                  ->KeyframePtr {
//         return GetKeyframe(std::make_pair(kf_id,client_id), expect_null);
//     }
//     virtual auto GetKeyframes()                                                         ->KeyframeMap;
//     virtual auto GetKeyframesVec()                                                      ->KeyframeVector;
//     virtual auto GetKeyframesErased()                                                   ->KeyframeMap;
//     virtual auto GetLandmark(idpair idp)                                                ->LandmarkPtr;
//     virtual auto GetLandmark(size_t lm_id, size_t client_id)                            ->LandmarkPtr {
//         return GetLandmark(std::make_pair(lm_id,client_id));
//     }
//     virtual auto GetLandmarks()                                                         ->LandmarkMap;
//     virtual auto GetLandmarksVec()                                                      ->LandmarkVector;

//     virtual auto GetMaxKfId()                                                           ->size_t;
//     virtual auto GetMaxLmId()                                                           ->size_t;

//     // Add / Erase data
//     virtual auto AddKeyframe(KeyframePtr kf)                                            ->void      = 0;
//     virtual auto AddLandmark(LandmarkPtr lm)                                            ->void      = 0;
//     virtual auto EraseKeyframe(KeyframePtr kf, bool mtx_lock = true)                    ->bool      = 0;
//     virtual auto EraseLandmark(LandmarkPtr lm, bool mtx_lock = true)                    ->bool      = 0;

//     // Clear Map
//     virtual auto Clear()                                                                ->void;

//     // Sync
//     virtual auto LockMapUpdate()                                                        ->void {
//         mtx_update_.lock();
//     }
//     virtual auto UnLockMapUpdate()                                                      ->void {
//         mtx_update_.unlock();
//     }

//     // Identifier
//     size_t                      id_map_                                                 = std::numeric_limits<size_t>::max();
//     std::set<size_t>            associated_clients_;

// protected:
//     // Data
//     KeyframeMap                 keyframes_;
//     LandmarkMap                 landmarks_;

//     KeyframeMap                 keyframes_erased_;

//     size_t                      max_id_kf_                                              = 0;
//     size_t                      max_id_lm_                                              = 0;

//     // Sync
//     std::mutex                  mtx_map_;
//     std::mutex                  mtx_update_;
// };
class Map: public std::enable_shared_from_this<Map>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;

    using MapPtr                        = TypeDefs::MapPtr;
    using TransformType                 = TypeDefs::TransformType;

    // using KeyframePtr                   = TypeDefs::KeyframePtr;
    // using LandmarkPtr                   = TypeDefs::LandmarkPtr;

    // using KeyframeMap                   = TypeDefs::KeyframeMap;
    // using LandmarkMap                   = TypeDefs::LandmarkMap;
    // using KeyframeVector                = TypeDefs::KeyframeVector;
    // using LandmarkVector                = TypeDefs::LandmarkVector;
    // using LandmarkSet                   = TypeDefs::LandmarkSet;

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