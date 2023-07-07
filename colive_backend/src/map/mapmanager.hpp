#pragma once

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>
// #include "pointcloud_rgbd.hpp"
#include "map_co.hpp"
#include "typedefs_base.hpp"
#include "scancontext/Scancontext.h"

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

    TransformType               T_smatch_squery          = TransformType::Identity();
    TypeDefs::Matrix6Type       cov_mat                  = TypeDefs::Matrix6Type::Identity();
}; 

class MapManager{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using idpair                        = TypeDefs::idpair;
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

    using MapContainer                  = std::map<int,MapInstancePtr>;
    using MergeBuffer                   = std::list<MergeInformation, Eigen::aligned_allocator<MergeInformation>>;

    using idpairVector           = TypeDefs::idpairVector;
    using PointCloudEXVector= TypeDefs::PointCloudEXVector;
    using PointCloudVector   = TypeDefs::PointCloudVector;
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

protected:

    auto CheckMergeBuffer()            ->bool;
    auto PerformMerge()                ->void;
    // auto Display_()->void;
    // Data
    MapContainer                maps_;
    MergeBuffer                 buffer_merge_;
    pcl::VoxelGrid<PointType> downSizeFilterScancontext;
    pcl::VoxelGrid<PointType> downSizeFilterICP;



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
};

}