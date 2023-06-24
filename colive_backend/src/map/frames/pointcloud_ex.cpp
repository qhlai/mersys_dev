
#include "pointcloud_ex.hpp"
#include "msgs/msg_pointcloud.hpp"
namespace colive {


PointCloud_ex::PointCloud_ex(MsgPointCloud msg, MapPtr map){
    id_= msg.id_;
    pos_w = msg.pos_w;
    quan_ = msg.quan_;
    pts_cloud=msg.pts_cloud;
    timestamp_=msg.timestamp_;
    
}
// auto PointCloud_ex::GetPoseTws()->TransformType {
//     std::unique_lock<std::mutex> lock(mtx_pose_);
//     return T_w_s_;
// }
auto PointCloud_ex::ConvertToMsg(colive::MsgPointCloud &msg,Vector3Type &pos_w_2, bool is_update, size_t cliend_id)->void{


    // std::unique_lock<std::mutex> lock_conn(mMutexConnections);
    // std::unique_lock<std::mutex> lock_feat(mMutexFeatures);
    // std::unique_lock<std::mutex> lock_pose(mMutexPose);
    // msg.downSample
    msg.id_ = id_;   // mnid clientid  
    msg.timestamp_ = timestamp_;//std::chrono::system_clock::now();
    
    msg.pos_w = pos_w;
    msg.quan_ = quan_;
    msg.pts_cloud = pts_cloud;
    
    // msg.pts_cloud=

}

// void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
// {
//     V3D p_body(pi->x, pi->y, pi->z);
//     V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

//     po->x = p_global(0);
//     po->y = p_global(1);
//     po->z = p_global(2);
//     po->intensity = pi->intensity;
// }
// PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
// int size = laserCloudFullRes->points.size();
// PointCloudXYZI::Ptr laserCloudWorld( \
//                 new PointCloudXYZI(size, 1));

// for (int i = 0; i < size; i++)
// {
//     RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
//                         &laserCloudWorld->points[i]);
// }
        // int size = pc->pts_cloud.points.size();
        // TransformType T_w_s_befcorrection = pc->GetPoseTws();
        // TransformType T_w_s_corrected = T_wtarget_wtofuse * T_w_s_befcorrection;
        // kf->SetPoseTws(T_w_s_corrected);
        // kf->velocity_ = T_wtarget_wtofuse.block<3,3>(0,0) * kf->velocity_;
auto PointCloud_ex::GetPoseTws()->TransformType {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    return T_w_s_;
}

auto PointCloud_ex::SetPoseTws()->TransformType {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    return T_w_s_;
}
// TODO：
// 尽量不要把所有的点都改位置，就改pointcloudex 的 一个转移矩阵就行了
auto PointCloud_ex::pointcloud_transform(TransformType tf)->void{
    uint32_t size = pts_cloud.points.size();
    for (int i = 0; i < size; i++){
        // PointType pi=pts_cloud.points[i];
        Vector3Type p_body(pts_cloud.points[i].x, pts_cloud.points[i].y, pts_cloud.points[i].z);


        // Vector3Type p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
        // TODO: 
        // Vector3Type p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

        
//         po->x = p_global(0);
// //     po->y = p_global(1);
// //     po->z = p_global(2);
        // pts_cloud.points[i] 
    }
    // PointCloudXYZI::Ptr laserCloudWorld( \
    //             new PointCloudXYZI(size, 1));

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