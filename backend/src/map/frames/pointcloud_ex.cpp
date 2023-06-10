
#include "pointcloud_ex.hpp"
namespace colive {


PointCloud_ex::PointCloud_ex(MsgPointCloud msg, MapPtr map){
    pts_cloud=msg.pts_cloud;
    pos_w = msg.pos_w;
    id_= msg.id;
}

auto PointCloud_ex::ConvertToMsg(colive::MsgPointCloud &msg,Vector3Type &pos_w_2, bool is_update, size_t cliend_id)->void{


    // std::unique_lock<std::mutex> lock_conn(mMutexConnections);
    // std::unique_lock<std::mutex> lock_feat(mMutexFeatures);
    // std::unique_lock<std::mutex> lock_pose(mMutexPose);
    // msg.downSample
    msg.id = id_;   // mnid clientid  
    msg.timestamp = 0;//std::chrono::system_clock::now();
    msg.pts_cloud = pts_cloud;
    msg.pos_w = pos_w;

}
auto PointCloud_ex::pc_less::operator ()(const PointCloudEXPtr a, const PointCloudEXPtr b) const ->bool
{
    if(a->id_.second < b->id_.second)
        return true;
    else if(a->id_.second > b->id_.second)
        return false;
    else {
        return a->id_.first < b->id_.first;
    }
}

}