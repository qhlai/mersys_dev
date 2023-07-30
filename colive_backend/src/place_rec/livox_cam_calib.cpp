#include "place_rec.hpp"
// #include "scancontext/Scancontext.h"

// C++
#include <iostream>
#include <mutex>
#include <eigen3/Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "pointcloud_ex.hpp"
#include "image_ex.hpp"

namespace colive
{

    virtual auto Calibration::Calib(ImageEXPtr img_unposed, int pc) -> void
    {
        //


        // pc with img_unposed calib   -> T

        // return img_posed
        // img_unposed=img_posed
        // img_unposed.SetPoseTwg(T)
        // rgb_map_.render_with_a_image(img_pose, 1)
    }

}
