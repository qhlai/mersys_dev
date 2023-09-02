#pragma once

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>
// #include "pointcloud_rgbd.hpp"
#include "map_co.hpp"
#include "typedefs_base.hpp"
#include "scancontext/Scancontext.h"


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

namespace colive
{


class Map;
class MapInstance;

struct MergeInformation {
    using TransformType                 = TypeDefs::TransformType;
    using PointCloudEXPtr                 = TypeDefs::PointCloudEXPtr;
    // using KeyframePtr                   = TypeDefs::KeyframePtr;

    // KeyframePtr                 kf_query;
    // KeyframePtr                 kf_match;
    PointCloudEXPtr             pc_query;
    PointCloudEXPtr             pc_match;

    TransformType               T_squery_smatch          = TransformType::Identity();
    TypeDefs::Matrix6Type       cov_mat                  = TypeDefs::Matrix6Type::Identity();
}; 

class MapManager{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using idpair                        = TypeDefs::idpair;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using MapPtr                        = TypeDefs::MapPtr;
    using MapInstancePtr                = std::shared_ptr<MapInstance>;
    using PointType                     = TypeDefs::PointType;
    using PointCloud                    = TypeDefs::PointCloud;
    using PointCloudPtr                    = TypeDefs::PointCloudPtr;
    using PointCloudEX  = TypeDefs::PointCloudEX; 
    using PointCloudEXPtr  = TypeDefs::PointCloudEXPtr; 
    using PointCloudEXList  = TypeDefs::PointCloudEXList; 
    using VoxelGrid  = TypeDefs::VoxelGrid; 

    using MapContainer                  = std::map<int,MapInstancePtr>;
    using MergeBuffer                   = std::list<MergeInformation, Eigen::aligned_allocator<MergeInformation>>;

    using idpairVector           = TypeDefs::idpairVector;
    using PointCloudEXVector= TypeDefs::PointCloudEXVector;
    using PointCloudVector   = TypeDefs::PointCloudVector;
    
    using MapTransform   = TypeDefs::MapTransform;
    
    // using DatabasePtr                   = std::shared_ptr<KeyframeDatabaseBase>;
    // using VocabularyPtr                 = CovinsVocabulary::VocabularyPtr;
public:

    MapManager();

    // Main
    auto Run()->void;
    // Interfaces
    auto CheckoutMap(int map_id, int& check_num)                                        ->MapPtr;
    auto CheckoutMapExclusive(int map_id, int& check_num)                               ->MapPtr;
    auto CheckoutMapExclusiveOrWait(int map_id, int& check_num)                         ->MapPtr;
    auto CheckoutMapOrWait(int map_id, int& check_num)                                  ->MapPtr;
    auto SetCheckoutBlock(int map_id, bool val)                                         ->bool;    
    auto InitializeMap(int map_id)                                                      ->void;
    auto ReturnMap(int map_id, int& check_num)                                          ->void;

    auto RegisterMap(MapPtr external_map)                                               ->bool;

    auto RegisterMerge(MergeInformation merge_data)                                     ->void;

    auto Display()->void;
    auto lcd()->void;

    auto init_gtsam()->void;
    // auto GetVoc()                                                                       ->VocabularyPtr {   // will never change - no need to be guarded by mutex
    //     return voc_;
    // }

    // auto AddToDatabase(KeyframePtr kf)                                                  ->void;
    // auto GetDatabase()                                                                  ->DatabasePtr;
    // auto EraseFromDatabase(KeyframePtr kf)                                              ->void;

    auto AddToDatabase(PointCloudEXPtr pc)                                                  ->void;

    // auto GetDatabase()                                                                  ->DatabasePtr;
    // auto EraseFromDatabase(KeyframePtr kf)                                              ->void;
    SCManager scManager;
    PointCloudEXVector cl_pcs;
    // PointCloudVector cl_pcs_d;
    PointCloudEXPtr pc;
    PointCloud::Ptr          pcl_pc_d;
    PointCloud::Ptr          pcl_pc;
    // std::pair<idpair, TransformType> map_tf;
    MapTransform             maps_tf_;


public:
    gtsam::NonlinearFactorGraph maps_gtSAMgraph;
    gtsam::noiseModel::Base::shared_ptr maps_robustLoopNoise;


    bool maps_gtSAMgraphMade = false;
    bool maps_gtSAMgraphEnable = false;
    gtsam::Values maps_initialEstimate;
    gtsam::ISAM2 *isam2;
    gtsam::Values isamCurrentEstimate;

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise;
// noiseModel::Diagonal::shared_ptr RTKNoise;
// noiseModel::Base::shared_ptr robustLoopNoise;
// noiseModel::Base::shared_ptr robustGPSNoise;

protected:

    auto CheckMergeBuffer()            ->bool;
    auto PerformMerge()                ->void;
    auto RecordMerge()->void ;

    auto process_isam_maps()->void ;
    // auto Display_()->void;
    // Data
    MapContainer                maps_;
    MergeBuffer                 buffer_merge_;
    VoxelGrid downSizeFilterScancontext;
    VoxelGrid downSizeFilterICP;



    // pcl::VoxelGrid<PointType> downSizeFilterScancontext;



    // idpairVector sc_ids;

    

    // pcex
    // pcl:pc a= pcex->pc down
    // sc(a)
    // sc_pcs.push(pc)
    // pcl:pc_v.push(a)   //keyframeLaserClouds


    // icp  pcl:pc_v[currid]  pcl:pc_v[oldid]




    // id=pcexv[sc.id].id

    // DatabasePtr                 database_;
    // VocabularyPtr               voc_;

    // Sync
    std::mutex                  mtx_access_;
    std::mutex                  mtx_database_;
    std::mutex                  mtx_pgo_maps_;
};

}