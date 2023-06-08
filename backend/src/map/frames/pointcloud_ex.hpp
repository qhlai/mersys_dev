#include "typedefs_base.hpp"
// #include "../communicator/msgs/msg_pointcloud.hpp"
#include "msgs/msg_pointcloud.hpp"

namespace colive {


class PointCloud_ex: public std::enable_shared_from_this<Map>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;
    using MapPtr                        = TypeDefs::MapPtr;
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
    // Position
    Vector3Type             pos_ref;
    Vector3Type             pos_w;

    // Pointclou
    // Identifier
    idpair                      id_;
    double                      timestamp_;
    PointCloud_ex()=default;
    PointCloud_ex(MsgPointCloud msg, MapPtr map);
    auto ConvertToMsg(MsgPointCloud &msg, Vector3Type &pos_w, bool is_update, size_t cliend_id)->void;


};


}