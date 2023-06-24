#pragma once
#include "typedefs_base.hpp"
#include "config_backend.hpp"
#include "scancontext/Scancontext.h"

namespace colive {

class Keyframe;
class Landmark;
class Map;
class MapManager;

class Keyframe;
class Landmark;
class Map;
class MapManager;

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

using PoseMap                       = TypeDefs::PoseMap;

    using PointType                     = TypeDefs::PointType;
    using PointCloud                    = TypeDefs::PointCloud;
    using PointCloudPtr                    = TypeDefs::PointCloudPtr;
    using PointCloudEX  = TypeDefs::PointCloudEX; 
    using PointCloudEXPtr  = TypeDefs::PointCloudEXPtr; 
    using PointCloudEXList  = TypeDefs::PointCloudEXList; 
    // using PointCloudEXBuffer  = TypeDefs::PointCloudEXBuffer;    

public:
    PlaceRecognition(MapManagerPtr man, bool perform_pgo = true);
    // virtual ~Placerec(){}
    virtual auto Run()                                                                  ->void      = 0;
    virtual auto InsertKeyframe(PointCloudEXPtr pc)                                         ->void      = 0;
    virtual auto CheckBufferExt()                                                       ->bool      = 0;

    // Synchronization
    virtual auto SetFinish()                                                            ->void      = 0;
    virtual auto ShallFinish()                                                          ->bool      = 0;
    virtual auto IsFinished()                                                           ->bool      = 0;

    // PGO
    virtual auto process_lcd()                                                          ->void      = 0;
    virtual auto performSCLoopClosure()                                                 ->void      = 0;

protected:
    virtual auto CheckBuffer()                                                          ->bool;
    virtual auto DetectLoop()                                                           ->bool;
    virtual auto ComputeSE3()                                                           ->bool;
    virtual auto CorrectLoop()                                                          ->bool;
    virtual auto ConnectLoop(PointCloudEXPtr pc_query, PointCloudEXPtr pc_match, TransformType T_smatch_squery, PoseMap &corrected_poses, MapPtr map)                              ->void;

    // // Infrastructure
    MapManagerPtr                  mapmanager_;
    SCManager                      scManager;
    std::queue<std::pair<int, int> >                        scLoopICPBuf;
    bool                           perform_pgo_                                            = true;
    // VocabularyPtr               voc_;

    // map<size_t,size_t>          last_loops_;

    // // Data
    // KeyframeBufferType          buffer_kfs_in_;
    PointCloudEXList               buffer_pcs_in_;

    PointCloudPtr                   pc_query_;
    PointCloudPtr                   pc_match_;
    
    precision_t                     mnCovisibilityConsistencyTh; // 一致性阈值

    Vector6Type                     keyframePoses;
    // vecConsistentGroup          mvConsistentGroups;
    // KeyframeVector              mvpEnoughConsistentCandidates;
    // KeyframeVector              mvpCurrentConnectedKFs;
    // LandmarkVector              mvpCurrentMatchedPoints;
    // LandmarkVector              mvpLoopMapPoints;
    // TransformType               mTcw;
    // TransformType               mTsw;
    // TypeDefs::Matrix6Type       mcov_mat;
    double                      mrelative_yaw = -1.0;
    
    // Sync
    std::mutex                  mtx_in_;
    std::mutex                  mtx_finish_;
    std::mutex                  mtx_mBuf_;

    bool                        finish_                                                 = false;
    bool is_finished_ = false;
};

}