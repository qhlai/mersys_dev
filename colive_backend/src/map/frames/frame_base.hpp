#pragma once

// C++
#include <memory>
#include <vector>
#include <set>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>

#include "typedefs_base.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
// #include "msgs/msg_pointcloud.hpp"

namespace colive {


class FrameBase: public std::enable_shared_from_this<FrameBase>
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
    // struct pc_less{
    //     auto operator() (const PointCloudEXPtr a, const PointCloudEXPtr b) const                ->bool;
    // };

public:
    idpair                      id_;
    double                      timestamp_;
    bool   is_loop_                = false;
public:
    virtual auto GetClientID()            ->size_t;
    virtual auto GetFrameID()            ->size_t;
    virtual auto GetFrameClientID()       ->idpair;
    virtual auto GetTimeStamp()            ->double;

    virtual auto SetErase()            ->void;
    virtual auto SetNotErase()         ->void;

    virtual auto GetPoseTcw()              ->TransformType;
    virtual auto GetPoseTwc()              ->TransformType;

    virtual auto GetPoseTws()              ->TransformType;
    virtual auto GetPoseTsw()              ->TransformType;

    virtual auto GetPoseTwg()              ->TransformType;
    virtual auto GetPoseTgw()              ->TransformType;
    virtual auto GetPoseTsg()              ->TransformType;
    virtual auto GetPoseTgs()              ->TransformType;

    virtual auto SetPoseTsw(TransformType Tsw, bool lock_mtx=true)    ->void;
    virtual auto SetPoseTws(TransformType Tsw, bool lock_mtx=true)    ->void;
    virtual auto SetPoseTwg(TransformType Twg, bool lock_mtx=true)    ->void;
    virtual auto convert2T( nav_msgs::Odometry &odometry)->TransformType ;
    virtual auto convert2T(double x, double y,double z,double qx,double qy,double qz,double qw)->TransformType;
    // virtual auto SetPoseTw_gw(TransformType Tws, bool lock_mtx=true)    ->void;

protected:
    friend class Map;
    // virtual auto SetInvalid()   ->bool;     // This function should only be called by the map
    // bool      pose_optimized_          = false;    // Indicates that this LM was part of an optimization process (important for landmark culling)
    // bool      vel_bias_optimized_                                     = false;



    bool  not_erase_            =false;

    // Infrastructure
    bool   invalid_               = false;
    bool   m_lock_Tws_               = false;

protected:
    // SE3 Pose, Bias, Velocity
    TransformType   T_s_c_   = TransformType::Identity();    // Tranformation IMU-Cam
    // TransformType   T_c_s_   = TransformType::Identity();    // Tranformation IMU-Cam

 // 当前帧与本地地图的迁移关系
    TransformType   T_w_s_      = TransformType::Identity();
    TransformType   T_s_w_       = TransformType::Identity();
    
    TransformType    T_c_w_      = TransformType::Identity();
    TransformType    T_w_c_     = TransformType::Identity();

    // local world with golbal world
    // 当前帧与全局地图的迁移关系
    TransformType    T_w_g_      = TransformType::Identity();
    TransformType    T_g_w_     = TransformType::Identity();
    // bool have_real_pose=false;
    // TransformType    T_w_s_vio_   = TransformType::Identity();
    // TransformType    T_s_w_vio_     = TransformType::Identity();
protected:
    // Mutexes
    std::mutex                  mtx_connections_;
    std::mutex                  mtx_features_;
    std::mutex                  mtx_pose_;
    std::mutex                  mtx_invalid_;
};


}