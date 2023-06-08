
#include "pointcloud_ex.hpp"
namespace colive {


PointCloud_ex::PointCloud_ex(MsgPointCloud msg, MapPtr map){
    pts_cloud=msg.pts_cloud;
    pos_w = msg.pos_w;
}

auto PointCloud_ex::ConvertToMsg(colive::MsgPointCloud &msg,Vector3Type &pos_w, bool is_update, size_t cliend_id)->void{


    // std::unique_lock<std::mutex> lock_conn(mMutexConnections);
    // std::unique_lock<std::mutex> lock_feat(mMutexFeatures);
    // std::unique_lock<std::mutex> lock_pose(mMutexPose);
    // msg.downSample
    msg.pts_cloud = pts_cloud;
    msg.pos_w = pos_w;

}

}