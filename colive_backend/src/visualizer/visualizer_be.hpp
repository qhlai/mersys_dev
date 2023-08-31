#pragma once


// C++
#include <mutex>
#include <eigen3/Eigen/Core>


#include "typedefs_base.hpp"

// Thirdparty
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>



#include "config_comm.hpp"
#include "config_backend.hpp"

#include "map_co.hpp"

#include <pcl/visualization/pcl_visualizer.h>

namespace colive{


class Visualizer:public std::enable_shared_from_this<Visualizer>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using TransformType                 = TypeDefs::TransformType;
    using TransformVector               = TypeDefs::TransformVector;
    using PoseMap                       = TypeDefs::PoseMap;
    using PoseMap_single                = TypeDefs::PoseMap_single;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using PointCloudEXPtr                   = TypeDefs::PointCloudEXPtr;
    using MapPtr                        = TypeDefs::MapPtr;
    using PointCloudPtr               = TypeDefs::PointCloudPtr;
    using MapManagerPtr                 = TypeDefs::MapManagerPtr;
    
    using KeyframeMap                   = TypeDefs::KeyframeMap;
    using LandmarkMap                   = TypeDefs::LandmarkMap;
    using PointCloud               = TypeDefs::PointCloud;
    using PointCloudEX               = TypeDefs::PointCloudEX;
    using PointCloudEXMap               = TypeDefs::PointCloudEXMap;
    using KeyframeVector                = TypeDefs::KeyframeVector;
    using LandmarkVector                = TypeDefs::LandmarkVector;
    using KeyframePairVector            = TypeDefs::KeyframePairVector;
    using LoopVector                    = TypeDefs::LoopVector;
    using RGBMap               = TypeDefs::RGBMap;
    using RGBMapPtr               = TypeDefs::RGBMapPtr;

    // using Global_map               = TypeDefs::Global_map;
    // using TransformType                 = TypeDefs::TransformType;
    // using PointCloudSetById               = std::set<PointCloudEXPtr,PointCloudEX::pc_less,Eigen::aligned_allocator<PointCloudEXPtr>>;
    // using LoopVector                    = TypeDefs::LoopVector;

    using PointCloudEXSetById               = std::set<PointCloudEXPtr,PointCloudEX::pc_less,Eigen::aligned_allocator<PointCloudEXPtr>>;
    // using PoseSetById               = std::set<TransformType,TransformType::less,Eigen::aligned_allocator<TransformType>>;

    struct VisBundle
    {
        uint32_t                frame_num_image;
        uint32_t                frame_num_pointcloud;

        KeyframeMap             keyframes;
        KeyframeMap             keyframes_erased;
        LandmarkMap             landmarks;
        PointCloudEXMap         pointCloud;
        PoseMap                 poseMap;
        MapPtr                  map;
        TransformType           vis_T;
        uint32_t                location_bias;

        RGBMapPtr              map_rgb_pts;
        size_t                  id_map;
        std::set<size_t>        associated_clients;
        LoopVector              loops;
        std::set<idpair>        most_recent_kfs;
        std::string             frame;
    };
    public:
    Visualizer(std::string topic_prefix= std::string());
    Visualizer(std::string topic_prefix, MapManagerPtr mapmanager);

    virtual auto Run()                    ->void;
    
    // Interfaces
    virtual auto DrawMap(MapPtr map)                                                    ->void;
    virtual auto PubPointCloud()                                                       ->void;
    virtual auto PubPointCloud_service()                                                       ->void;
    virtual auto PubPointCloud_service_bak()                                                       ->void;
    virtual auto PubTrajectories()        ->void; 
    virtual auto PubOdometries()        ->void;
    virtual auto PubLoopEdges()         ->void;
    // Draw Loaded Map
    auto DrawMapBitByBit(MapPtr map, std::string frame)                                 ->void;
    auto getRgbFromGray(double gray_value, int colormap_type)->Vector3Type;

    // Reset
    virtual auto RequestReset()                                                         ->void;
    
    // Auxiliary Functions
    static auto CreatePoint3D(Eigen::Vector3d &p3D, size_t client_id)                   ->pcl::PointXYZRGB;
    static auto MakeColorMsg(float fR,float fG, float fB)                               ->std_msgs::ColorRGBA;
    
    void print_dash_board();
    
protected:

    virtual auto CheckVisData()                                                         ->bool;
    virtual auto ResetIfRequested()                                                     ->void;

    // Infrastructure
    ros::NodeHandle             nh_                   = ros::NodeHandle();
    ros::Publisher              pub_marker_;
    ros::Publisher              pub_cloud_;
    ros::Publisher              pub_odom_;
    std::vector< std::shared_ptr <ros::Publisher> > pub_odom_vec_;
    std::vector< std::shared_ptr <ros::Publisher> > m_pub_rgb_render_pointcloud_ptr_vec;

    // Define the color map   intensity to rgb
    // pcl::visualization::PCLVisualizer::Ptr pcl_visualizer;
    // double min_intensity = 0.0;  // Minimum intensity value
    // double max_intensity = 255.0;  // Maximum intensity value

    std::string                 topic_prefix_                                           = std::string();

    MapManagerPtr mapmanager_;
    double g_last_stamped_mem_mb = 0;
    uint32_t g_lidar_frame_num = 0;
    uint32_t g_camera_frame_num = 0;
    uint32_t g_pointcloud_pts_num =0;
    uint32_t g_loop_self_num = 0;
    uint32_t g_loop_co_num = 0;

    // for pub pointcloud service
    int last_publish_map_idx = -3e8;
    int sleep_time_aft_pub = 10;  
    int number_of_pts_per_topic = 5000;

    // // Data
    std::map<size_t,VisBundle>  vis_data_;
    VisBundle                   curr_bundle_;

    // Reset
    bool                        reset_                                                  = false;

    // Sync
    std::mutex                  mtx_draw_;
    std::mutex                  mtx_reset_;


};



}