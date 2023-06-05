#pragma once

#include <set>
// C++
#include <mutex>
#include <eigen3/Eigen/Core>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>

// #include "pointcloud_rgbd.hpp"
// #include <pcl/kdtree/kdtree_flann.h>
// #include <kd_tree/ikd_Tree.h>

#include "backend/base/typedefs_base.hpp"

class MapBase{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = double;
public:
    MapBase()= delete;
    MapBase(size_t id);

    // Clear Map
    // virtual auto Clear()                                                                ->void;

    // Sync
    virtual auto LockMapUpdate()                                                        ->void {
        mtx_update_.lock();
    }
    virtual auto UnLockMapUpdate()                                                      ->void {
        mtx_update_.unlock();
    }
    // Identifier
    size_t                      id_map_                                                 = std::numeric_limits<size_t>::max();
    std::set<size_t>            associated_clients_;
protected:
    // Sync
    std::mutex                  mtx_map_;
    std::mutex                  mtx_update_;
};