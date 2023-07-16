#pragma once

#include <eigen3/Eigen/Core>

// COVINS
#include "typedefs_base.hpp"

//SERIALIZATION
// #include <cereal/cereal.hpp>
// #include <cereal/types/memory.hpp>
// #include <cereal/types/utility.hpp>
// #include <cereal/types/vector.hpp>
// #include <cereal/types/polymorphic.hpp>
// #include <cereal/types/concepts/pair_associative_container.hpp>
// #include <cereal/types/base_class.hpp>
// #include <cereal/archives/binary.hpp>
// #include <cereal/archives/binary.hpp>
// #include <cereal/access.hpp>
#include <cstdint>

#include "tools_serialization_cereal.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include "pointcloud_ex.hpp"

namespace colive {


class MsgPointCloud {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;

    using MsgTypeVector                 = TypeDefs::MsgTypeVector;
    using PointType                     = TypeDefs::PointType;
    using VoxelGrid                     = TypeDefs::VoxelGrid;
    using PointCloud                    = TypeDefs::PointCloud;
    using QuaternionType                = TypeDefs::QuaternionType;

    struct compare_less{bool operator() (const MsgPointCloud &a, const MsgPointCloud &b) const;};

public:

    MsgPointCloud();
    MsgPointCloud(bool filesave);
    MsgPointCloud(MsgTypeVector msgtype);

    // Interfaces
    auto SetMsgType(int msg_size)                                       ->void;
    auto SetMsgType(MsgTypeVector msgtype)                              ->void;

    // Infrastructure
    MsgTypeVector           msg_type                                    = std::vector<uint32_t>(5);     // size, is_update, ID of Keyframe, ???,;
    bool                    is_update_msg                               = false;
    bool                    save_to_file                                = false;                        // indicates that this LM will be saved to a file, not send over network

    // Identifier
    double                  timestamp_;
    idpair                  id_;

    // Position
    // Vector3Type             pos_ref;
    // Vector3Type             pos_w;
    TransformType           T_w_s_ = TransformType::Identity(); 
    TransformType           T_s_w_ = TransformType::Identity();
    PointCloud              pts_cloud;
    // QuaternionType          quan_;


protected:

    friend class cereal::access;                                                                                                // Serialization

    template<class Archive>
    auto save(Archive &archive) const ->void {
        if(save_to_file) {
            archive(id_,
                    // pos_w,
                    // quan_,
                    T_s_w_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        } else if(is_update_msg){
            archive(id_,
                    // pos_w,
                    // quan_,
                    T_s_w_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        } else {
            archive(id_,
                    // pos_w,
                    // quan_,
                    T_s_w_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        }
    }

    template<class Archive>
    auto load(Archive &archive)->void {
        if(save_to_file) {
             archive(id_,
                    // pos_w,
                    // quan_,
                    T_s_w_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        } else if(msg_type[1] == true){
            archive(id_,
                    // pos_w,
                    // quan_,
                    T_s_w_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        } else {
            archive(id_,
                    // pos_w,
                    // quan_,
                    T_s_w_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        }
    }
};

} //end ns
// x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
//       normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
//       curvature = p.curvature;
//       intensity = p.intensity;
// namespace cereal {
//     // Save function for pcl::PointCloud type


//     template<class Archive>
//     inline
//     void save(Archive& ar, const Eigen::Isometry3d& T) {
//         // Save the size of the point cloud
//         // size_t size = pointCloud.size();
//         // ar(size);
//         // Eigen::Matrix4d T_4x4 = Eigen::Matrix4d::Identity();
//         // T_4x4.block<3, 3>(0, 0) = T.rotation();
//         // T_4x4.block<3, 1>(0, 3) = T.translation();
//         // ar(T_4x4);
//         ar(T.matrix());

//     }
//      // Load function for pcl::PointCloud type
//     template<class Archive>
//     inline
//     void load(Archive& ar, Eigen::Isometry3d& T) {

//         Eigen::Matrix4d T_4x4 = Eigen::Matrix4d::Identity();
//         // T=Eigen::Isometry3d::Identity();

//         ar(T_4x4);
//         Eigen::Isometry3d temp(T_4x4); 
//         T=temp;
//         // T.rotate(T_4x4.block<3, 3>(0, 0));
//         // T.translate(T_4x4.block<3, 1>(0, 3));

//     }

//     template<class Archive>
//     inline
//     void save(Archive& ar, const pcl::PointCloud<pcl::PointXYZI>& pointCloud) {
//         // Save the size of the point cloud
//         size_t size = pointCloud.size();
//         ar(size);

//         // Save each point in the point cloud
//         for (const auto& point : pointCloud) {
//             ar(point.x);
//             ar(point.y);
//             ar(point.z);
//             for (size_t i = 0; i <3;i++) {
//                 ar(point.data[i]);
//             }

//             ar(point.intensity);

//         }
//     }
//      // Load function for pcl::PointCloud type
//     template<class Archive>
//     inline
//     void load(Archive& ar, pcl::PointCloud<pcl::PointXYZI>& pointCloud) {
//         // Load the size of the point cloud
//         size_t size;
//         ar(size);

//         // Resize the point cloud to the loaded size
//         pointCloud.resize(size);

//         // Load each point in the point cloud
//         for (auto& point : pointCloud) {
//             ar(point.x);
//             ar(point.y);
//             ar(point.z);
//             for (size_t i = 0; i <3;i++) {
//                 ar(point.data[i]);
//             }
            
//             ar(point.intensity);
//         }
//     }
//     template<class Archive>
//     inline
//     void save(Archive& ar, const pcl::PointCloud<pcl::PointXYZINormal>& pointCloud) {
//         // Save the size of the point cloud
//         size_t size = pointCloud.size();
//         ar(size);

//         // Save each point in the point cloud
//         for (const auto& point : pointCloud) {
//             ar(point.x);
//             ar(point.y);
//             ar(point.z);
//             for (size_t i = 0; i <3;i++) {
//                 ar(point.data[i]);
//             }
            
//             ar(point.normal_x);
//             ar(point.normal_y);
//             ar(point.normal_z);
//             for (size_t i = 0; i <3;i++) {
//                 ar(point.data_n[i]);
//             }
            
//             ar(point.curvature);
//             ar(point.intensity);

//         }
//     }
//      // Load function for pcl::PointCloud type
//     template<class Archive>
//     inline
//     void load(Archive& ar, pcl::PointCloud<pcl::PointXYZINormal>& pointCloud) {
//         // Load the size of the point cloud
//         size_t size;
//         ar(size);

//         // Resize the point cloud to the loaded size
//         pointCloud.resize(size);

//         // Load each point in the point cloud
//         for (auto& point : pointCloud) {
//             ar(point.x);
//             ar(point.y);
//             ar(point.z);
//             for (size_t i = 0; i <3;i++) {
//                 ar(point.data[i]);
//             }
            
//             ar(point.normal_x);
//             ar(point.normal_y);
//             ar(point.normal_z);
//             for (size_t i = 0; i <3;i++) {
//                 ar(point.data_n[i]);
//             }
            
//             ar(point.curvature);
//             ar(point.intensity);
//         }
//     }

//     template<class Archive>
//     inline
//     void save(Archive& ar, const colive::TypeDefs::QuaternionType& q) {
//         ar(q.x());
//         ar(q.y());
//         ar(q.z());
//         ar(q.w());
       
//     }
//      // Load function for pcl::PointCloud type
//     template<class Archive>
//     inline
//     void load(Archive& ar, colive::TypeDefs::QuaternionType& q) {
//        colive::TypeDefs::precision_t  x, y, z, w;
//         ar(x);
//         ar(y);
//         ar(z);
//         ar(w);
//         q = colive::TypeDefs::QuaternionType(w, x, y, z);
//     }
// } 



