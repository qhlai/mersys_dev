#pragma once
#include "typedefs_base.hpp"
#include "frame_base.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <iostream>   // std::cout  
#include <string>     // std::string, std::to_string

namespace colive {

class MsgImage;



class Image_ex : public FrameBase,  public std::enable_shared_from_this<Image_ex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;
    using MapPtr                        = TypeDefs::MapPtr;
    using PointType                     = TypeDefs::PointType;
    using VoxelGrid                     = TypeDefs::VoxelGrid;
    using ImageEX                    = TypeDefs::ImageEX;
    using ImageEXPtr                    = TypeDefs::ImageEXPtr;
    using ImagePtr                    = TypeDefs::ImagePtr;


    using Vector3Type                   = TypeDefs::Vector3Type;
    using QuaternionType                = TypeDefs::QuaternionType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;
    struct img_less{
        auto operator() (const ImageEXPtr a, const ImageEXPtr b) const                ->bool;
    };
public:

    cv::Mat m_img;
    cv::Mat m_raw_img;
    cv::Mat m_img_gray;
    bool sent_once_ = false;

    

    Image_ex()=default;
    Image_ex(MsgImage msg);
//     // virtual ~PointCloud_ex() {};
//     // PointCloud_ex(PointCloud msg, MapPtr map);
    auto SetImage(ImagePtr img)->void;
//     virtual auto pointcloud_convert(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out)->void;
//     virtual auto pointcloud_convert(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_out)->void;
//     auto pointcloud_transform(TransformType tf)->void;
//     // GetPoseTws
    
    auto ConvertToMsg(MsgImage &msg, bool is_update, size_t cliend_id)->void;
//     auto convert_to_tf()->TransformType;
    protected:
    std::mutex                   mtx_pose_;
    std::mutex                   mtx_in_;

};


}