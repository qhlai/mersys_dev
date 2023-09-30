#pragma once
#include "typedefs_base.hpp"
// #include "pointcloud_ex.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
#include "msgs/msg_instruction.hpp"

namespace colive {

#define INSTRUCTIONS_DRIFT_NONE 0x00
#define INSTRUCTIONS_DRIFT_CORRECTION 0x01

struct Instruction {
    using TransformType             = TypeDefs::TransformType;
    using KeyframePtr               = std::shared_ptr<Keyframe>;
    using PointCloudEX               = TypeDefs::PointCloudEX;
    using PointCloudEXPtr               = TypeDefs::PointCloudEXPtr;
    using Matrix6Type               = TypeDefs::Matrix6Type;
    using idpair               = TypeDefs::idpair;
    double                  timestamp_ = 0.0;
    idpair                  id_;
    idpair                  m_comm_direction;  // src -> dst
    bool if_from_server     = false;
    // bool    src_backend;
    TransformType       T_correction;

    Instruction()=default;
    Instruction(MsgInstruction msg);
    auto ConvertToMsg(MsgInstruction &msg,  bool is_update, size_t cliend_id)->void;

    // Matrix6Type         cov_mat;
};



}