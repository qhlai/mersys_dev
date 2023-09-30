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


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace colive {

class MsgPointCloud;

// class PointCloud_ex_base : public FrameBase{
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     using precision_t                   = TypeDefs::precision_t;
//     using idpair                        = TypeDefs::idpair;
//     using MapPtr                        = TypeDefs::MapPtr;
//     using PointType                     = TypeDefs::PointType;
//     using VoxelGrid                     = TypeDefs::VoxelGrid;
//     using PointCloud                    = TypeDefs::PointCloud;
//     using PointCloudEX                    = TypeDefs::PointCloudEX;
//     using PointCloudEXPtr                    = TypeDefs::PointCloudEXPtr;

//     using Vector3Type                   = TypeDefs::Vector3Type;
//     using QuaternionType                = TypeDefs::QuaternionType;
//     using Matrix3Type                   = TypeDefs::Matrix3Type;
//     using TransformType                 = TypeDefs::TransformType;
//     // VoxelGrid  vox_cloud;
//     PointCloud pts_cloud;
//     PointCloud pts_cloud_d;  // 降采样后的点云
//     // Position
//     // Vector3Type             pos_ref;
//     // Vector3Type             pos_w;
//     // QuaternionType          quan_;
// public:
//     // TransformType           T_s_lm_ = TransformType::Identity(); // 当前帧与本地地图的迁移关系
//     // TransformType           T_lm_w_ = TransformType::Identity(); // 
//     // bool have_real_pose=false;
//     MapPtr                   map_;
//     // TransformType           T_w_s_ = TransformType::Identity(); // 当前帧与全局地图的迁移关系

//     // Pointclou
//     // Identifier
// };

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
    using PointCloudEX                    = TypeDefs::PointCloudEX;
    using PointCloudEXPtr                    = TypeDefs::PointCloudEXPtr;

    using Vector3Type                   = TypeDefs::Vector3Type;
    using QuaternionType                = TypeDefs::QuaternionType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;
    struct pc_less{
        auto operator() (const PointCloudEXPtr a, const PointCloudEXPtr b) const                ->bool;
    };
    static auto CompStamp(PointCloudEXPtr kf1, PointCloudEXPtr kf2)                             ->bool;     //greater - newest stamp at beginning of container
public:

    bool sent_once_ = false;

    PointCloud pts_cloud;
    PointCloud pts_cloud_d;  // 降采样后的点云
    // Position
    // Vector3Type             pos_ref;
    // Vector3Type             pos_w;
    // QuaternionType          quan_;
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

    PointCloud_ex(const PointCloud_ex& other) {
        // TODO: Implem
        // std::lock_guard<std::mutex> lock(mtx_pose_);
        // std::lock_guard<std::mutex> lock(mtx_in_);
        // auto timePoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

        // std::unique_lock<std::mutex> lock1(mtx_pose_, timePoint);
        // std::unique_lock<std::mutex> lock2(mtx_in_, timePoint);
        // std::unique_lock<std::mutex> lock3(other.mtx_pose_, timePoint);
        // std::unique_lock<std::mutex> lock4(other.mtx_in_, timePoint);
        // uint32_t cnt = 0 ;
        // while (true) {
        //     // lock1.try_lock();
        //     // lock2.try_lock();
        //     // lock3.try_lock(); 
        //     // lock4.try_lock();
        //     if(lock1.owns_lock() && lock2.owns_lock() && lock3.owns_lock() && lock4.owns_lock()){
        //         id_= other.id_;
        //         timestamp_=other.timestamp_;    
        //         pts_cloud=other.pts_cloud;
        //         SetPoseTsw(other.T_s_w_);  

        //         break;

        //     }else{
        //         cnt++;
        //         usleep(10);
        //     }
        //     if (cnt == 30)
        //     {
        //         break;
        //     }
            
        // }
        // is dangerous
        id_= other.id_;
        timestamp_=other.timestamp_;    
        pts_cloud=other.pts_cloud;
        SetPoseTsw(other.T_s_w_);  
        // pts_cloud
        
        // 复制其他成员变量
    }
    PointCloud_ex& operator=(const PointCloud_ex& other) {
            if (this != &other) {
                // std::unique_lock<std::mutex> lock1(mtx_pose_, mtx_in_);
                // std::unique_lock<std::mutex> lock2(other.mtx_pose_, other.mtx_in_);
                id_= other.id_;
                timestamp_=other.timestamp_;    
                pts_cloud=other.pts_cloud;
                SetPoseTsw(other.T_s_w_);  
                // 复制其他成员变量
                // TODO:
            }
            return *this;
        }
    // virtual ~PointCloud_ex() {};
    // PointCloud_ex(PointCloud msg, MapPtr map);
    auto SetPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)->void;
    auto SetPointCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc)->void;

       

    virtual auto pointcloud_convert(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out)->void;
    virtual auto pointcloud_convert(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_out)->void;
    auto pointcloud_transform(TransformType tf)->void;
    // GetPoseTws
    
    auto ConvertToMsg(MsgPointCloud &msg,  bool is_update, size_t cliend_id)->void;
    auto convert_to_tf(Vector3Type pos_w, QuaternionType quan_)->TransformType;
    auto get_transformed_pc()->PointCloud;

    virtual auto add_and_merge_pointcloudex(PointCloudEXPtr pc)->void; 

    auto save_to_pcd( std::string dir_name, std::string _file_name = std::string( "/rgb_pt" ) , int save_pts_with_views = 3)->void;
    virtual auto save_and_display_pointcloud( std::string dir_name = std::string( "~/ros/temp/" ), std::string file_name = std::string( "/rgb_pt" ) ,  int save_pts_with_views = 3)->void;
    protected:
    std::mutex                   mtx_pose_;
    std::mutex                   mtx_in_;


};


}