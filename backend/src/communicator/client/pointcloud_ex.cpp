
#include "pointcloud_ex.hpp"
namespace colive {

auto Pointcloud_ex::ConvertToMsg(colive::MsgPointcloud &msg,Vector3Type &pos_w, bool is_update, size_t cliend_id)->void{


    // std::unique_lock<std::mutex> lock_conn(mMutexConnections);
    // std::unique_lock<std::mutex> lock_feat(mMutexFeatures);
    // std::unique_lock<std::mutex> lock_pose(mMutexPose);
    // msg.downSample
    msg.pts_cloud = pts_cloud;
    msg.pos_w = pos_w;

}

}