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
#include "map_rgb.hpp"



// https://github.com/hku-mars/livox_camera_calib
namespace colive {
class Calibration {
public:
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
public:
    virtual auto Calib(ImageEXPtr img_pose_, int pc)        ->void;


    MapPtr                       map_rgb_;
}

}