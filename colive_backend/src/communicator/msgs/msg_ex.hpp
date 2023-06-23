#pragma once
#include "typedefs_base.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
// #include "msgs/msg_pointcloud.hpp"

namespace colive {


class MsgEX: public std::enable_shared_from_this<MsgEX>
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

    

    // bool sent_once_ = false;
    // VoxelGrid  vox_cloud;
    // PointCloud pts_cloud;
    // // Position
    // Vector3Type             pos_ref;
    // Vector3Type             pos_w;
    // QuaternionType          quan_;
    // // Vector3Type             pos_self;
    // TransformType           odom;
// 
    // Pointclou
    // Identifier
    
    idpair                      id_;
    double                      timestamp_;
    // PointCloud_ex()=default;
    // PointCloud_ex(MsgPointCloud msg, MapPtr map);
    // // PointCloud_ex(PointCloud msg, MapPtr map);
    // auto ConvertToMsg(MsgPointCloud &msg, Vector3Type &pos_w, bool is_update, size_t cliend_id)->void;


};


}