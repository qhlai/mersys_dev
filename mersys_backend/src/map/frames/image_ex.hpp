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

namespace mersys {

class MsgImage;

// class Image_ex_base: public FrameBase{
// public:
//     cv::Mat img_;
//     float intrinsic[4]={0}; // fx fy cx cy
// };

class Image_ex : public FrameBase,  public std::enable_shared_from_this<Image_ex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;
    using MapPtr                        = TypeDefs::MapPtr;
    using PointType                     = TypeDefs::PointType;
    using VoxelGrid                     = TypeDefs::VoxelGrid;
    using Image                    = TypeDefs::Image;
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

    // cv::Mat img_;

    // float intrinsic[4]={0}; // fx fy cx cy
    cv::Mat img_;

    bool m_if_have_set_intrinsic = false;
    Eigen::Matrix3d m_cam_K;
    double fx, fy, cx, cy;
    TypeDefs::Vector2Type m_gama_para;

    // float intrinsic[4]={0}; // fx fy cx cy
    float m_fov_margin = 0.005; // 图像无效边缘

    cv::Mat m_raw_img;
    cv::Mat m_img_gray;
    bool sent_once_ = false;

    
public:
    Image_ex()=default;
    Image_ex(MsgImage msg);
//     // virtual ~PointCloud_ex() {};
//     // PointCloud_ex(PointCloud msg, MapPtr map);
    auto SetImage(ImagePtr img)->void;
    auto SetImage(Image &img)->void;
//     virtual auto pointcloud_convert(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out)->void;
//     virtual auto pointcloud_convert(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_out)->void;
//     auto pointcloud_transform(TransformType tf)->void;
//     // GetPoseTws
    
    auto ConvertToMsg(MsgImage &msg, bool is_update, size_t cliend_id)->void;
public:

    auto set_intrinsic(Eigen::Matrix3d & camera_K)->void;
    auto project_3d_to_2d(const Vector3Type & in_pt, double &u, double &v, const double &scale=1.0)->bool;
    auto if_2d_points_available(const double &u, const double &v, const double &scale = 1.0, float fov_mar = -1.0)->bool;
    auto get_rgb(double &u, double v, int layer = 0, Vector3Type *rgb_dx = nullptr, Vector3Type *rgb_dy = nullptr)->Vector3Type;
    auto get_rgb( const double & u,  const double & v, int & r, int & g, int & b  )->bool;
    auto get_grey_color(double & u ,double & v, int layer= 0 )->double ;
    auto image_equalize(cv::Mat &img, int amp = 10.0)->void;
    auto image_equalize()->void;

    // auto project_3d_point_in_this_img(const pcl::PointXYZI & in_pt, double &u, double &v,   pcl::PointXYZRGB * rgb_pt = nullptr, double intrinsic_scale = 1.0)->bool;
    auto project_3d_point_in_this_img(const Vector3Type & in_pt, double &u, double &v, pcl::PointXYZRGB *rgb_pt = nullptr, double intrinsic_scale = 1.0)->bool;

    auto save_to_png( std::string dir_name, std::string _file_name = std::string( "/img" ))->void;

//     auto convert_to_tf()->TransformType;
    protected:
    std::mutex                   mtx_pose_;
    std::mutex                   mtx_in_;

};


}