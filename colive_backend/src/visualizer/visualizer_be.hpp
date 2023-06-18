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

#include "pointcloud_ex.hpp"
#include "map_co.hpp"
namespace colive{


class Visualizer:public std::enable_shared_from_this<Visualizer>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using TransformType                 = TypeDefs::TransformType;


    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using PointCloudEXPtr                   = TypeDefs::PointCloudEXPtr;
    using MapPtr                        = TypeDefs::MapPtr;
    using PointCloudPtr               = TypeDefs::PointCloudPtr;
    
    using KeyframeMap                   = TypeDefs::KeyframeMap;
    using LandmarkMap                   = TypeDefs::LandmarkMap;
    using PointCloud               = TypeDefs::PointCloud;
    using PointCloudEX               = TypeDefs::PointCloudEX;
    using PointCloudEXMap               = TypeDefs::PointCloudEXMap;
    using KeyframeVector                = TypeDefs::KeyframeVector;
    using LandmarkVector                = TypeDefs::LandmarkVector;
    using KeyframePairVector            = TypeDefs::KeyframePairVector;
    // using LoopVector                    = TypeDefs::LoopVector;

    using PointCloudEXSetById               = std::set<PointCloudEXPtr,PointCloudEX::pc_less,Eigen::aligned_allocator<PointCloudEXPtr>>;

    struct VisBundle
    {
        KeyframeMap             keyframes;
        KeyframeMap             keyframes_erased;
        LandmarkMap             landmarks;
        PointCloudEXMap pointCloud;
        size_t                  id_map;
        std::set<size_t>        associated_clients;
        // LoopVector              loops;
        std::set<idpair>        most_recent_kfs;
        std::string             frame;
    };
    public:
    Visualizer();
    Visualizer(std::string topic_prefix= std::string());

    virtual auto Run()                    ->void;
    
    // Interfaces
    virtual auto DrawMap(MapPtr map)                                                    ->void;
    virtual auto PubPointCloud()                                                        ->void;
    // Draw Loaded Map
    auto DrawMapBitByBit(MapPtr map, std::string frame)                                 ->void;


    // Reset
    virtual auto RequestReset()                                                         ->void;
    
    // Auxiliary Functions
    static auto CreatePoint3D(Eigen::Vector3d &p3D, size_t client_id)                   ->pcl::PointXYZRGB;
    static auto MakeColorMsg(float fR,float fG, float fB)                               ->std_msgs::ColorRGBA;

    
protected:

    virtual auto CheckVisData()                                                         ->bool;
    virtual auto ResetIfRequested()                                                     ->void;

    // Infrastructure
    ros::NodeHandle             nh_                   = ros::NodeHandle();
    ros::Publisher              pub_marker_;
    ros::Publisher              pub_cloud_;
    std::string                 topic_prefix_                                           = std::string();

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