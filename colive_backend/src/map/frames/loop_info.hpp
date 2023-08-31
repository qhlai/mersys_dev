#pragma once
#include "typedefs_base.hpp"
#include "pointcloud_ex.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
// #include "msgs/msg_pointcloud.hpp"

namespace colive {


struct LoopConstraint {
    using TransformType             = TypeDefs::TransformType;
    using KeyframePtr               = std::shared_ptr<Keyframe>;
    using PointCloudEX               = TypeDefs::PointCloudEX;
    using PointCloudEXPtr               = TypeDefs::PointCloudEXPtr;
    using Matrix6Type               = TypeDefs::Matrix6Type;
    using idpair               = TypeDefs::idpair;
    LoopConstraint(PointCloudEXPtr k1, PointCloudEXPtr k2, TransformType T_12,
                   Matrix6Type covm = Matrix6Type::Identity())
        : pc1(k1), pc2(k2), T_s1_s2(T_12), cov_mat(covm) {
            if (pc1->GetClientID() == pc2->GetClientID()){
                is_same_client=true;
            }else{
                is_same_client=false;
            }
            // pose1.first = k1->GetFrameID();
            // pose1.second = k1->GetClientID();
            pose1 = pc1->GetFrameClientID();

            pose2 = pc2->GetFrameClientID();


        }

    // LoopConstraint(PointCloudEXPtr k1, PointCloudEXPtr k2, TransformType T_12,
    //                Matrix6Type covm = Matrix6Type::Identity())
    //     : pc1(k1), pc2(k2), T_s1_s2(T_12), cov_mat(covm) {}
    uint8_t type = 0;    // 0: pc+pc, 1: pc+camera, 2: camera+camera 
    PointCloudEXPtr         pc1;
    PointCloudEXPtr         pc2;

    idpair                  pose1;
    idpair                  pose2;

    bool                    is_same_client = false;
    // PointCloudEXPtr         pc1;
    // PointCloudEXPtr         pc2;

    TransformType       T_s1_s2;
    Matrix6Type         cov_mat;
};



}