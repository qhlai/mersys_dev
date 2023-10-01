#pragma once
#include "typedefs_base.hpp"
#include "msg_define.hpp"
#include "instruction.hpp"
#include "tools_serialization_cereal.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
// #include "msgs/msg_pointcloud.hpp"

namespace mersys {


class MsgInstruction: public std::enable_shared_from_this<MsgInstruction>
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
    using MsgTypeVector                 = TypeDefs::MsgTypeVector;
    // struct pc_less{
    //     auto operator() (const PointCloudEXPtr a, const PointCloudEXPtr b) const                ->bool;
    // };
public:

    MsgInstruction();
    MsgInstruction(bool filesave);
    MsgInstruction(MsgTypeVector msgtype);

    // Interfaces
    auto SetMsgType(int msg_size)                                       ->void;
    auto SetMsgType(MsgTypeVector msgtype)                              ->void;

    // Infrastructure
    // Infrastructure
    MsgTypeVector           msg_type                    = std::vector<uint32_t>(5);     // size, is_update, ID of Keyframe, ???,;
    bool                    is_update_msg               = false;
    bool                    save_to_file                = false;     


    double                  timestamp_ = 0.0;
    idpair                  id_;
    idpair                  m_comm_direction;  // src -> dst
    bool if_from_server     = false;
    // bool    src_backend;
    TransformType       T_correction;



    template<class Archive>
    auto save(Archive &archive) const ->void {
        if(save_to_file) {
            archive(
                id_,
                timestamp_,
                m_comm_direction,
                if_from_server,
                T_correction
                    );
        } else if(is_update_msg){
            archive(
                id_,
                timestamp_,
                m_comm_direction,
                if_from_server,
                T_correction
                    );
        } else {
            archive(
                id_,
                timestamp_,
                m_comm_direction,
                if_from_server,
                T_correction
                    );
        }
    }

    template<class Archive>
    auto load(Archive &archive)->void {
        if(save_to_file) {
            archive(
                id_,
                timestamp_,
                m_comm_direction,
                if_from_server,
                T_correction
                    );
        } else if(msg_type[1] == true){
            archive(
                id_,
                timestamp_,
                m_comm_direction,
                if_from_server,
                T_correction
                    );
        } else {
            archive(
                id_,
                timestamp_,
                m_comm_direction,
                if_from_server,
                T_correction
                    );
        }
    }

};


}