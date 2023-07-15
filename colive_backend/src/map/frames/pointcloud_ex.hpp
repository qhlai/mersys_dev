#pragma once
#include "typedefs_base.hpp"
#include "frame_base.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <iostream>   // std::cout  
#include <string>     // std::string, std::to_string


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>

namespace colive {

class MsgPointCloud;



class PointCloud_ex : public FrameBase,  public std::enable_shared_from_this<PointCloud_ex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;
    using MapPtr                        = TypeDefs::MapPtr;
    using PointType                     = TypeDefs::PointType;
    using VoxelGrid                     = TypeDefs::VoxelGrid;
    using PointCloud                    = TypeDefs::PointCloud;
    using PointCloudEXPtr                    = TypeDefs::PointCloudEXPtr;

    using Vector3Type                   = TypeDefs::Vector3Type;
    using QuaternionType                = TypeDefs::QuaternionType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;
    struct pc_less{
        auto operator() (const PointCloudEXPtr a, const PointCloudEXPtr b) const                ->bool;
    };
public:

    bool sent_once_ = false;
    // VoxelGrid  vox_cloud;
    PointCloud pts_cloud;
    PointCloud pts_cloud_d;
    // Position
    Vector3Type             pos_ref;
    Vector3Type             pos_w;
    QuaternionType          quan_;
public:
    // TransformType           T_s_lm_ = TransformType::Identity(); // 当前帧与本地地图的迁移关系
    // TransformType           T_lm_w_ = TransformType::Identity(); // 
    // bool have_real_pose=false;
    MapPtr                   map_;
    // TransformType           T_w_s_ = TransformType::Identity(); // 当前帧与全局地图的迁移关系

    // Pointclou
    // Identifier
    

    PointCloud_ex()=default;
    PointCloud_ex(MsgPointCloud msg, MapPtr map);
    // virtual ~PointCloud_ex() {};
    // PointCloud_ex(PointCloud msg, MapPtr map);
    auto SetPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)->void;
    auto SetPointCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc)->void;

    virtual auto pointcloud_convert(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out)->void;
    virtual auto pointcloud_convert(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_out)->void;
    auto pointcloud_transform(TransformType tf)->void;
    // GetPoseTws
    
    auto ConvertToMsg(MsgPointCloud &msg, Vector3Type &pos_w, bool is_update, size_t cliend_id)->void;
    auto convert_to_tf()->TransformType;
    protected:
    std::mutex                   mtx_pose_;
    std::mutex                   mtx_in_;

};


}