#pragma once
#include <iostream>
#include <optional>
// #include <pcl/point_types.h>
#include "typedefs_base.hpp"
#include "config_backend.hpp"


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
// #include <ad_localization_msgs/NavStateInfo.h>
// #include "scancontext/Scancontext.h"
#include "mapmanager.hpp"
#include "livox_cam_calib.hpp"




// https://github.com/hku-mars/livox_camera_calib
namespace mersys {

class Keyframe;
class Landmark;
class Map;
class MapManager;

class Keyframe;
class Landmark;
class Map;
class MapManager;


// class Calibration{

// };


class PlaceRecognition {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using TransformType                 = TypeDefs::TransformType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Vector6Type                   = TypeDefs::Vector6Type;
    using QuaternionType                = TypeDefs::QuaternionType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using LandmarkPtr                   = TypeDefs::LandmarkPtr;
    using MapPtr                        = TypeDefs::MapPtr;
    using MapManagerPtr                 = TypeDefs::MapManagerPtr;
    // using CalibrationPtr                = TypeDefs::CalibrationPtr;

using PoseMap                       = TypeDefs::PoseMap;

    using PointType                     = TypeDefs::PointType;
    using PointCloud                    = TypeDefs::PointCloud;
    using PointCloudPtr                    = TypeDefs::PointCloudPtr;
    using PointCloudEX  = TypeDefs::PointCloudEX; 
    using PointCloudEXPtr  = TypeDefs::PointCloudEXPtr; 
    using PointCloudEXList  = TypeDefs::PointCloudEXList; 
    // using PointCloudEXBuffer  = TypeDefs::PointCloudEXBuffer;    

    using Image                         = TypeDefs::Image;
    using ImageEX                       = TypeDefs::ImageEX;
    using ImageEXPtr                    = TypeDefs::ImageEXPtr;
    using ImagePtr                      = TypeDefs::ImagePtr;
    using ImageEXList                    = std::list<ImageEXPtr,Eigen::aligned_allocator<ImageEXPtr>>;
public:
    PlaceRecognition(MapManagerPtr man, bool perform_pgo = true);
    // virtual ~Placerec(){}
    virtual auto Run()                                                                  ->void;
    virtual auto InsertKeyframe(PointCloudEXPtr pc)                                         ->void;
    virtual auto InsertLargeKeyframe(PointCloudEXPtr pc)                                         ->void;
    virtual auto InsertKeyframe1(ImageEXPtr img)                                         ->void;
    // virtual auto CheckBufferExt()                                                       ->bool {
    //     return CheckBuffer();}
    // Synchronization
    auto SetFinish()                                                                    ->void  {
        std::unique_lock<std::mutex> lock(mtx_finish_); finish_ = true;}
    auto ShallFinish()                                                                  ->bool  {
        std::unique_lock<std::mutex> lock(mtx_finish_); return finish_;}
    virtual auto IsFinished()                                                           ->bool  {
        std::unique_lock<std::mutex> lock(mtx_finish_); return is_finished_;}

    // PGO
    virtual auto process_lcd()                                                          ->void;
    virtual auto performSCLoopClosure()                                                 ->void;
    virtual auto process_icp()                                                 ->void;
    virtual auto doICPVirtualRelative(int _loop_pc_idx, int _curr_pc_idx, TransformType &corrected_tf)        ->int;

protected:
    virtual auto CheckBuffer_pc()                                                          ->bool;
    virtual auto CheckBuffer_img()                                                          ->bool;
    virtual auto CheckBuffer_pcl()                                                          ->bool;
    virtual auto DetectLoop()                                                           ->bool;
    virtual auto DetectLoop_C()                                                           ->bool;
    virtual auto ComputeSE3()                                                           ->bool;
    virtual auto CorrectLoop()                                                          ->bool;
    virtual auto ConnectLoop(PointCloudEXPtr pc_query, PointCloudEXPtr pc_match, TransformType T_squery_smatch, PoseMap &corrected_poses, MapPtr map)                              ->void;

    // // Infrastructure
    MapManagerPtr                  mapmanager_;
    // SCManager                      scManager;
    std::queue<std::pair<int, int> >                        scLoopICPBuf;
    // std::queue<std::pair<int, int> >                        
    bool                           perform_pgo_             = true;
    // VocabularyPtr               voc_;

    // map<size_t,size_t>          last_loops_;
    // pcl::VoxelGrid<PointType> downSizeFilterScancontext;

    // // Data
    // KeyframeBufferType          buffer_kfs_in_;
    PointCloudEXList               buffer_pcs_in_;
    PointCloudEXList               buffer_pcs_large_in_;
    ImageEXList                    buffer_imgs_in_;

    PointCloudEXPtr                   pc_query_;
    PointCloudEXPtr                   pc_match_;
    ImageEXPtr                        img_query_;
    ImageEXPtr                        img_match_;
    PointCloud::Ptr                   sc_pcs_d;
    PointCloud::Ptr                   pts_cloud;
    // std::vector< size_t >             last_loop_frame_id;
    map<idpair,size_t>                last_loops_;
    // std::vector< std::shared_ptr <ros::Publisher> >
    
    precision_t                     mnCovisibilityConsistencyTh; // 一致性阈值

    Vector6Type                     keyframePoses;
    
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::noiseModel::Base::shared_ptr robustLoopNoise;

    // TransformType                   correctionLidarFrame;
    // vecConsistentGroup          mvConsistentGroups;
    // KeyframeVector              mvpEnoughConsistentCandidates;
    // KeyframeVector              mvpCurrentConnectedKFs;
    // LandmarkVector              mvpCurrentMatchedPoints;
    // LandmarkVector              mvpLoopMapPoints;
    // TransformType               mTcw;
    // TransformType               mTsw;
    // TypeDefs::Matrix6Type       mcov_mat;
    double                      mrelative_yaw = -1.0;
    
    int                         rgb_edge_minLen_ = 200;
    int                         rgb_canny_threshold_ = 20;
    int                         min_depth_ = 2.5;
    int                         max_depth_ = 50;
    int                         plane_max_size_ = 5;
    float                       detect_line_threshold_ = 0.1;
    int                         line_number_ = 0;
    int                         color_intensity_threshold_ = 5;

    // Sync
    std::mutex                  mtx_in_;
    std::mutex                  mtx_finish_;
    std::mutex                  mtx_mBuf_;
    std::mutex                  mtx_Posegraph_;


    bool                        finish_                                                 = false;
    bool is_finished_ = false;
};

}