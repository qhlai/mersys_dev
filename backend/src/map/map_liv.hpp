#pragma once

#include "backend/map/map_base.hpp"
// #include <set>
// // C++
// #include <mutex>
// #include <eigen3/Eigen/Core>

// #include <boost/serialization/vector.hpp>
// #include <boost/serialization/export.hpp>

#include "pointcloud_rgbd.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <kd_tree/ikd_Tree.h>
#include <common_lib.h>
#include <pcl/filters/voxel_grid.h>

class MapLIV : public MapBase, public std::enable_shared_from_this<MapLIV> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using MapLIVPtr                        = TypeDefs::MapLIVPtr;
    
    Global_map m_map_rgb_pts;
    //surf feature in map
    PointCloudXYZINormal::Ptr featsFromMap;     
    PointCloudXYZINormal::Ptr cube_points_add;  
    //all points
    PointCloudXYZINormal::Ptr laserCloudFullRes2; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullResColor;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    #ifdef USE_ikdtree
    KD_TREE ikdtree;
    #endif

public:
    MapLIV()= delete;
    MapLIV(size_t id);
    // MapBase()-> delete();
    // MapBase(size_t id);


protected:
    // Sync
    std::mutex                  mtx_map_;
    std::mutex                  mtx_update_;
};
