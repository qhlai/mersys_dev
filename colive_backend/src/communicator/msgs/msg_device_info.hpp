#pragma once
#include "typedefs_base.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
// #include "msgs/msg_pointcloud.hpp"

namespace colive {


class MsgDevice_info: public Device_info, public std::enable_shared_from_this<MsgDevice_info>
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

   

};


}