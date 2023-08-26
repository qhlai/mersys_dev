#pragma once
#include <iostream>
#include <optional>
// #include <pcl/point_types.h>
#include "typedefs_base.hpp"
#include "config_backend.hpp"
#include "livox_cam/common.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <ad_localization_msgs/NavStateInfo.h>
// #include "scancontext/Scancontext.h"
#include "mapmanager.hpp"
#include "map_rgb.hpp"



// https://github.com/hku-mars/livox_camera_calib
namespace colive {
class Calibration {
public:
    using TransformType                 = TypeDefs::TransformType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Vector6Type                   = TypeDefs::Vector6Type;
    using QuaternionType                = TypeDefs::QuaternionType;

    using PointType                     = TypeDefs::PointType;
    using PointCloud                    = TypeDefs::PointCloud;
    using PointCloudPtr                    = TypeDefs::PointCloudPtr;
    using PointCloudEX  = TypeDefs::PointCloudEX; 
    using PointCloudEXPtr  = TypeDefs::PointCloudEXPtr; 
    using PointCloudEXList  = TypeDefs::PointCloudEXList; 
    using Image                         = TypeDefs::Image;
    using ImageEX                       = TypeDefs::ImageEX;
    using ImageEXPtr                    = TypeDefs::ImageEXPtr;
    using ImagePtr                      = TypeDefs::ImagePtr;
    using MapPtr                        = TypeDefs::MapPtr;
    using MapManagerPtr                 = TypeDefs::MapManagerPtr;
    enum ProjectionType { DEPTH, INTENSITY, BOTH };
    enum Direction { UP, DOWN, LEFT, RIGHT };

    // enum ProjectionType { DEPTH, INTENSITY, BOTH };
public:
    auto Calib(ImageEXPtr img_pose_, int pc)        ->void;
    auto roughCalib(std::vector<Calibration> &calibs, Vector6Type &calib_params, double search_resolution, int max_iter)   ->void;
    void edgeDetector(
    const int &canny_threshold, const int &edge_threshold,
    const cv::Mat &src_img, cv::Mat &edge_img,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_cloud);
    void LiDAREdgeExtraction(
    const std::unordered_map<VOXEL_LOC, Voxel *> &voxel_map,
    const float ransac_dis_thre, const int plane_size_threshold,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d);
    auto buildVPnp(
    Vector6Type &extrinsic_params, int dis_threshold,
    bool show_residual,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cam_edge_cloud_2d,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d,
    std::vector<VPnPData> &pnp_list)   ->void;

    auto buildPnp(
    Vector6Type &extrinsic_params, int dis_threshold,
    bool show_residual,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cam_edge_cloud_2d,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d,
    std::vector<PnPData> &pnp_list)   ->void;

    cv::Mat
    getConnectImg(const int dis_threshold,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &rgb_edge_cloud,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &depth_edge_cloud);
    bool checkFov(const cv::Point2d &p);
    void calcDirection(const std::vector<Eigen::Vector2d> &points,
                     Eigen::Vector2d &direction);
    cv::Mat getProjectionImg(const Vector6d &extrinsic_params);
    void projection(const Vector6d &extrinsic_params,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud,
                  const ProjectionType projection_type, const bool is_fill_img,
                  cv::Mat &projection_img);
    cv::Mat fillImg(const cv::Mat &input_img, const Direction first_direct,
                  const Direction second_direct);
    void calcLine(const std::vector<Plane> &plane_list, const double voxel_size,
                const Eigen::Vector3d origin,
                std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_cloud_list);
    MapPtr                       map_rgb_;


    ros::NodeHandle nh_;
    ros::Publisher rgb_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 1);
    ros::Publisher init_rgb_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("init_rgb_cloud", 1);
    ros::Publisher planner_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("planner_cloud", 1);
    ros::Publisher line_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("line_cloud", 1);
    ros::Publisher image_pub_ =
        nh_.advertise<sensor_msgs::Image>("camera_image", 1);
    
    std::string lidar_topic_name_ = "";
    std::string image_topic_name_ = "";

    int rgb_edge_minLen_ = 200;
    int rgb_canny_threshold_ = 20;
    int min_depth_ = 2.5;
    int max_depth_ = 50;
    int plane_max_size_ = 5;
    float detect_line_threshold_ = 0.02;
    int line_number_ = 0;
    int color_intensity_threshold_ = 5;
    // 相机内参
    float fx_, fy_, cx_, cy_, k1_, k2_, p1_, p2_, k3_, s_;
    int width_, height_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat init_extrinsic_;

    int is_use_custom_msg_;
    float voxel_size_ = 1.0;
    float down_sample_size_ = 0.02;
    float ransac_dis_threshold_ = 0.02;
    float plane_size_threshold_ = 60;
    float theta_min_;
    float theta_max_;
    float direction_theta_min_;
    float direction_theta_max_;
    float min_line_dis_threshold_ = 0.03;
    float max_line_dis_threshold_ = 0.06;

    cv::Mat rgb_image_;
    cv::Mat image_;
    cv::Mat grey_image_;
    // 裁剪后的灰度图像
    cv::Mat cut_grey_image_;

    // 初始旋转矩阵
    Eigen::Matrix3d init_rotation_matrix_;
    // 初始平移向量
    Eigen::Vector3d init_translation_vector_;

    // 存储从pcd/bag处获取的原始点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_lidar_cloud_;

    // 存储平面交接点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_line_cloud_;
    std::vector<int> plane_line_number_;
    // 存储RGB图像边缘点的2D点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr rgb_egde_cloud_;
    // 存储LiDAR Depth/Intensity图像边缘点的2D点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_edge_cloud_;
};

}