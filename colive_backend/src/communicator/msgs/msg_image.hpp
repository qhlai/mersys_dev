#pragma once

#include <eigen3/Eigen/Core>

// COVINS
#include "typedefs_base.hpp"
#include "msg_define.hpp"
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


#include "image_ex.hpp"

#include "tools_serialization_cereal.hpp"

namespace colive {

// : public Image_ex
// : public Image_ex_base
class MsgImage {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;

    using MsgTypeVector                 = TypeDefs::MsgTypeVector;
    using Image                         = TypeDefs::Image;
    using ImageEX                       = TypeDefs::ImageEX;
    using ImageEXPtr                    = TypeDefs::ImageEXPtr;
    using ImagePtr                      = TypeDefs::ImagePtr;

    struct compare_less{bool operator() (const MsgPointCloud &a, const MsgPointCloud &b) const;};

public:

    MsgImage();
    MsgImage(bool filesave);
    MsgImage(MsgTypeVector msgtype);

    // Interfaces
    auto SetMsgType(int msg_size)                                       ->void;
    auto SetMsgType(MsgTypeVector msgtype)                              ->void;

    // Infrastructure
    MsgTypeVector           msg_type                    = std::vector<uint32_t>(5);     // size, is_update, ID of Keyframe, ???,;
    bool                    is_update_msg               = false;
    bool                    save_to_file                = false;                        // indicates that this LM will be saved to a file, not send over network

    // Identifier
    double                  timestamp_;
    idpair                  id_;

    // Position

    TransformType           T_w_s_ = TransformType::Identity(); 
    TransformType           T_s_w_ = TransformType::Identity();
    Image                   img_;
    Eigen::Matrix3d m_cam_K; //intrinsic

protected:

    friend class cereal::access;                                                                                                // Serialization

    template<class Archive>
    auto save(Archive &archive) const ->void {
        if(save_to_file) {
            archive(id_,
                    timestamp_,
                    T_s_w_,
                    img_,
                    m_cam_K,
                    is_update_msg);
        } else if(is_update_msg){
            archive(id_,
                    timestamp_,
                    T_s_w_,
                    img_,
                    m_cam_K,
                    // observations,id_reference
                    is_update_msg);
        } else {
            archive(id_,
                    timestamp_,
                    T_s_w_,
                    img_,
                    m_cam_K,
                    // observations,id_reference
                    is_update_msg);
        }
    }

    template<class Archive>
    auto load(Archive &archive)->void {
        if(save_to_file) {
             archive(id_,
                    timestamp_,
                    T_s_w_,
                    img_,
                    m_cam_K,
                    // observations,id_reference
                    is_update_msg);
        } else if(msg_type[1] == true){
            archive(id_,
                    timestamp_, 
                    T_s_w_,
                    img_,
                    m_cam_K,
                    // observations,id_reference
                    is_update_msg);
        } else {
            archive(id_,
                    timestamp_,
                    T_s_w_,
                    img_,
                    m_cam_K,
                    // observations,id_reference
                    is_update_msg);
        }
    }
};


} //end ns

namespace cereal {
    // Save function for Eigen::Isometry3ds type


    // template<class Archive>
    // inline
    // void save(Archive& ar, const Eigen::Isometry3d& T) {
    //     ar(T.matrix());

    // }
    //  // Load function for Eigen::Isometry3d type
    // template<class Archive>
    // inline
    // void load(Archive& ar, Eigen::Isometry3d& T) {

    //     Eigen::Matrix4d T_4x4 = Eigen::Matrix4d::Identity();
    //     // T=Eigen::Isometry3d::Identity();

    //     ar(T_4x4);
    //     Eigen::Isometry3d temp(T_4x4); 
    //     T=temp;
    //     // T.rotate(T_4x4.block<3, 3>(0, 0));
    //     // T.translate(T_4x4.block<3, 1>(0, 3));

    // }

    //save and load function for cv::Mat type
    // template<class Archive>
    // inline
    // void save(Archive& ar, const cv::Mat& mat) {
    //     int rows, cols, type;
    //     bool continuous;

    //     rows = mat.rows;
    //     cols = mat.cols;
    //     type = mat.type();
    //     continuous = mat.isContinuous();

    //     ar & rows & cols & type & continuous;

    //     if (continuous) {
    //         const int data_size = rows * cols * static_cast<int>(mat.elemSize());
    //         auto mat_data = cereal::binary_data(mat.ptr(), data_size);
    //         ar & mat_data;
    //     }
    //     else {
    //         const int row_size = cols * static_cast<int>(mat.elemSize());
    //         for (int i = 0; i < rows; i++) {
    //             auto row_data = cereal::binary_data(mat.ptr(i), row_size);
    //             ar & row_data;
    //         }
    //     }
    // }

    // template<class Archive>
    // void load(Archive& ar, cv::Mat& mat) {
    //     int rows, cols, type;
    //     bool continuous;

    //     ar & rows & cols & type & continuous;

    //     if (continuous) {
    //         mat.create(rows, cols, type);
    //         const int data_size = rows * cols * static_cast<int>(mat.elemSize());
    //         auto mat_data = cereal::binary_data(mat.ptr(), data_size);
    //         ar & mat_data;
    //     }
    //     else {
    //         mat.create(rows, cols, type);
    //         const int row_size = cols * static_cast<int>(mat.elemSize());
    //         for (int i = 0; i < rows; i++) {
    //             auto row_data = cereal::binary_data(mat.ptr(i), row_size);
    //             ar & row_data;
    //         }
    //     }
    // }
    
} 



