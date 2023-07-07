
#include "pointcloud_ex.hpp"
#include "msgs/msg_pointcloud.hpp"
namespace colive {


PointCloud_ex::PointCloud_ex(MsgPointCloud msg, MapPtr map){
    id_= msg.id_;
    pos_w = msg.pos_w;
    quan_ = msg.quan_;
    pts_cloud=msg.pts_cloud;
    timestamp_=msg.timestamp_;
    T_lm_s_=msg.T_lm_s_;
    
    map_=map;
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
    msg.T_lm_s_=T_lm_s_;
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
    return T_lm_s_;
}

auto PointCloud_ex::SetPoseTws()->TransformType {
    std::unique_lock<std::mutex> lock(mtx_pose_);
    return T_lm_s_;
}

auto PointCloud_ex::SetPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)->void {
    std::unique_lock<std::mutex> lock(mtx_in_);

    pts_cloud=*pc;

}
auto PointCloud_ex::SetPointCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc)->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZI>);
    pointcloud_convert(pc,pc_out);    
    // pointcloud_convert(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out);
    pts_cloud=*pc_out;
}

auto PointCloud_ex::pointcloud_convert(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out)->void{

    for (size_t i = 0; i < pc_in->size(); ++i)
    {
    pcl::PointXYZI point;
    point.x = pc_in->points[i].x;
    point.y = pc_in->points[i].y;
    point.z = pc_in->points[i].z;
    point.intensity = pc_in->points[i].intensity;
    pc_out->push_back(point);
    }
}
auto PointCloud_ex::pointcloud_convert(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_out)->void{

    for (size_t i = 0; i < pc_in->size(); ++i)
    {
    pcl::PointXYZINormal point;
    point.x = pc_in->points[i].x;
    point.y = pc_in->points[i].y;
    point.z = pc_in->points[i].z;
    point.intensity = pc_in->points[i].intensity;
    pc_out->push_back(point);
    }
}
// TODO：
// 尽量不要把所有的点都改位置，就改pointcloudex 的 一个转移矩阵就行了
auto PointCloud_ex::pointcloud_transform(TransformType T)->void{
    uint32_t size = pts_cloud.points.size();
    PointCloud pc;
    pc.resize(size);
    for (int i = 0; i < size; i++){
        // PointType pi=pts_cloud.points[i];
        Vector3Type p_old(pts_cloud.points[i].x, pts_cloud.points[i].y, pts_cloud.points[i].z);

        Vector3Type p_new(0,0,0);
        pc.points[i].x=  T(0, 0) * p_old[0] + T(0, 1) * p_old[0] + T(0, 2) * p_old[0] + T(0, 3);
        pc.points[i].y = T(1, 0) * p_old[1] + T(1, 1) * p_old[1] + T(1, 2) * p_old[1] + T(1, 3);
        pc.points[i].z = T(2, 0) * p_old[2] + T(2, 1) * p_old[2] + T(2, 2) * p_old[2] + T(2, 3);

        pc.points[i].intensity =pts_cloud.points[i].intensity;

    }
    pts_cloud=pc;
    // PointCloudXYZI::Ptr laserCloudWorld( \
    //             new PointCloudXYZI(size, 1));

}
auto PointCloud_ex::convert_to_tf()->TransformType{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(pos_w);
    T.rotate(quan_);
    // T_w_s_.block<3, 3>(0, 0)=T.rotation();
    // T_w_s_.block<3, 1>(0, 3) = T.translation();
    return T;
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