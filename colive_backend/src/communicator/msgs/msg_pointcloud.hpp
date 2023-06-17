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
class MsgPointCloud{
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
    double                  timestamp;
    idpair                  id;

    // Position
    Vector3Type             pos_ref;
    Vector3Type             pos_w;
    
    double                  downSample;
    double                  leafsize_xyz[3];

    VoxelGrid               vox_cloud;
    PointCloud              pts_cloud;
    QuaternionType          quan_;


protected:

    friend class cereal::access;                                                                                                // Serialization

    template<class Archive>
    auto save(Archive &archive) const ->void {
        if(save_to_file) {
            archive(id,
                    pos_w,
                    quan_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        } else if(is_update_msg){
            archive(id,
                    pos_w,
                    quan_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        } else {
            archive(id,
                    pos_w,
                    quan_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        }
    }

    template<class Archive>
    auto load(Archive &archive)->void {
        if(save_to_file) {
             archive(id,
                    pos_w,
                    quan_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        } else if(msg_type[1] == true){
            archive(id,
                    pos_w,
                    quan_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        } else {
            archive(id,
                    pos_w,
                    quan_,
                    pts_cloud,
                    // observations,id_reference
                    is_update_msg);
        }
    }
};

} //end ns

namespace cereal {
    // Save function for pcl::PointCloud type
    template<class Archive>
    inline
    void save(Archive& ar, const colive::TypeDefs::PointCloud& pointCloud) {
        // Save the size of the point cloud
        size_t size = pointCloud.size();
        ar(size);

        // Save each point in the point cloud
        for (const auto& point : pointCloud) {
            ar(point.x);
            ar(point.y);
            ar(point.z);
            // Additional serialization for other point cloud properties
            // ar(point.property1);
            // ar(point.property2);
            // ...
        }
    }
     // Load function for pcl::PointCloud type
    template<class Archive>
    inline
    void load(Archive& ar, colive::TypeDefs::PointCloud& pointCloud) {
        // Load the size of the point cloud
        size_t size;
        ar(size);

        // Resize the point cloud to the loaded size
        pointCloud.resize(size);

        // Load each point in the point cloud
        for (auto& point : pointCloud) {
            ar(point.x);
            ar(point.y);
            ar(point.z);
            // Additional deserialization for other point cloud properties
            // ar(point.property1);
            // ar(point.property2);
            // ...
        }
    }
    template<class Archive>
    inline
    void save(Archive& ar, const colive::TypeDefs::QuaternionType& q) {
        ar(q.x());
        ar(q.y());
        ar(q.z());
        ar(q.w());
       
    }
     // Load function for pcl::PointCloud type
    template<class Archive>
    inline
    void load(Archive& ar, colive::TypeDefs::QuaternionType& q) {
       colive::TypeDefs::precision_t  x, y, z, w;
        ar(x);
        ar(y);
        ar(z);
        ar(w);
        q = colive::TypeDefs::QuaternionType(w, x, y, z);
    }
} 



