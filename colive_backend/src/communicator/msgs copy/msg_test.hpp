#pragma once

#include <eigen3/Eigen/Core>

// COVINS
#include "typedefs_base.hpp"

//SERIALIZATION
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/concepts/pair_associative_container.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/access.hpp>
#include <cstdint>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

namespace colive {

// struct MsgPointCloud {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     using precision_t                   = TypeDefs::precision_t;
//     using idpair                        = TypeDefs::idpair;

//     using Vector3Type                   = TypeDefs::Vector3Type;
//     using Matrix3Type                   = TypeDefs::Matrix3Type;
//     using TransformType                 = TypeDefs::TransformType;

//     using MsgTypeVector                 = TypeDefs::MsgTypeVector;

//     struct compare_less{bool operator() (const MsgLandmark &a, const MsgLandmark &b) const;};

// public:

//     MsgPointCloud();
//     MsgPointCloud(bool filesave);
//     MsgPointCloud(MsgTypeVector msgtype);

//     // Interfaces
//     auto SetMsgType(int msg_size)                                       ->void;
//     auto SetMsgType(MsgTypeVector msgtype)                              ->void;

//     // Infrastructure
//     MsgTypeVector           msg_type                                    = std::vector<uint32_t>(5);     // size, is_update, ID of Keyframe, ???,;
//     bool                    is_update_msg                               = false;
//     bool                    save_to_file                                = false;                        // indicates that this LM will be saved to a file, not send over network

//     // Identifier
//     idpair                  id;

//     // Position
//     Vector3Type             pos_ref;
//     Vector3Type             pos_w;
    
//     double                        downSample;
//     double                    leafsize_xyz[3];
//     pcl::VoxelGrid<PointType>      pts_cloud;



// protected:

//     // friend class cereal::access;                                                                                                // Serialization

//     // template<class Archive>
//     // auto save(Archive &archive) const ->void {
//     //     if(save_to_file) {
//     //         archive(id,
//     //                 pos_w,
//     //                 observations,id_reference
//     //                 );
//     //     } else if(is_update_msg){
//     //         archive(id,
//     //                 pos_ref,id_reference,
//     //                 is_update_msg);
//     //     } else {
//     //         archive(id,
//     //                 pos_ref,
//     //                 observations,id_reference,
//     //                 is_update_msg);
//     //     }
//     // }

//     // template<class Archive>
//     // auto load(Archive &archive)->void {
//     //     if(save_to_file) {
//     //         archive(id,
//     //                 pos_w,
//     //                 observations,id_reference
//     //                 );
//     //     } else if(msg_type[1] == true){
//     //         archive(id,
//     //                 pos_ref,id_reference,
//     //                 is_update_msg);
//     //     } else {
//     //         archive(id,
//     //                 pos_ref,
//     //                 observations,id_reference,
//     //                 is_update_msg);
//     //     }
//     // }
// };

} //end ns
