#pragma once

// C++
#include <memory>
#include <vector>
#include <set>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include "typedefs_base.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
// #include "msgs/msg_pointcloud.hpp"

namespace colive {


class Frame_base: public std::enable_shared_from_this<Device_info>
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
    // Vector3Type             pos_;
    // QuaternionType          q_;
    // TransformType           T_w_s_ = TransformType::Identity(); // 当前帧与本地地图的迁移关系
    // auto convert_to_tf()->TransformType;
    // virtual auto GetPoseTws()          ->TransformType;
    // virtual auto SetPoseTws()          ->TransformType;

};


}