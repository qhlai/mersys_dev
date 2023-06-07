#include "typedefs_base.hpp"
#include "../msgs/msg_pointcloud.hpp"

namespace colive {

class Pointcloud_ex
{
public:

    using PointType                     = TypeDefs::PointType;
    using VoxelGrid                     = TypeDefs::VoxelGrid;
    using PointCloud                    = TypeDefs::PointCloud;

    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;

public:

    bool sent_once_ = false;
    VoxelGrid  vox_cloud;
    PointCloud pts_cloud;
    Vector3Type             pos_w;

    // Pointclou

    auto ConvertToMsg(colive::MsgPointcloud &msg, Vector3Type &pos_w, bool is_update, size_t cliend_id)->void;


};


}