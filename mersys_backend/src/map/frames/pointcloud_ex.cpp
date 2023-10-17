
#include "pointcloud_ex.hpp"
#include "msgs/msg_pointcloud.hpp"
#include "os_compatible.hpp"
#include <dbg.h>
namespace mersys {


PointCloud_ex::PointCloud_ex(MsgPointCloud msg, MapPtr map){
    id_= msg.id_;
    timestamp_=msg.timestamp_; 
    dbg("msg.timestamp_", msg.timestamp_, false);   
    // std::cout << "msg.timestamp_" <<msg.timestamp_ << std::endl;
    // pos_w = msg.pos_w;
    // quan_ = msg.quan_;
    pts_cloud=msg.pts_cloud;

    // T_s_w_=msg.T_s_w_;
    SetPoseTsw(msg.T_s_w_);
    
    map_=map;
}
// auto PointCloud_ex::GetPoseTws()->TransformType {
//     std::unique_lock<std::mutex> lock(mtx_pose_);
//     return T_w_s_;
// }
auto PointCloud_ex::ConvertToMsg(mersys::MsgPointCloud &msg, bool is_update, size_t cliend_id)->void{


    // std::unique_lock<std::mutex> lock_conn(mMutexConnections);
    // std::unique_lock<std::mutex> lock_feat(mMutexFeatures);
    // std::unique_lock<std::mutex> lock_pose(mMutexPose);
    // msg.downSample
    msg.id_ = id_;   // mnid clientid  
    msg.timestamp_ = timestamp_;//std::chrono::system_clock::now();
    
    // msg.pos_w = pos_w;
    // msg.quan_ = quan_;
    msg.pts_cloud = pts_cloud;
    msg.T_s_w_=GetPoseTsw();
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
auto PointCloud_ex::convert_to_tf(Vector3Type pos_w, QuaternionType quan_)->TransformType{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(pos_w);
    T.rotate(quan_);
    // T_w_s_.block<3, 3>(0, 0)=T.rotation();
    // T_w_s_.block<3, 1>(0, 3) = T.translation();
    return T;
}
auto PointCloud_ex::get_transformed_pc()->PointCloud{
    PointCloud pt_cloud_tf;
    pcl::transformPointCloud(pts_cloud,pt_cloud_tf, GetPoseTsg().matrix());
    return pt_cloud_tf;
}

auto PointCloud_ex::add_and_merge_pointcloudex(PointCloudEXPtr pc)->void{
    // pts_cloud+=pc->get_transformed_pc();
    // 这里可能会有未知内存报错
    std::unique_lock<std::mutex> lock(mtx_in_);
    if(!pc){
        std::cout << COUTWARN <<"pc is nullptr"<< std::endl;
        return;
    }
    PointCloud cloud_acc;
    pcl::transformPointCloud(pc->pts_cloud, cloud_acc, (pc->GetPoseTsw()*GetPoseTsw().inverse()).matrix());
    // std::cout << COUTDEBUG <<"cloud_acc.size()"<<cloud_acc.size()<< std::endl;
    // if(cloud_acc.size()<1000){
    //     return;
    // }
    pts_cloud+=cloud_acc;
}
    

auto PointCloud_ex::pc_less::operator ()(const PointCloudEXPtr a, const PointCloudEXPtr b) const ->bool
{
    if(a->GetClientID() < b->GetClientID())
        return true;
    else if(a->GetClientID() > b->GetClientID())
        return false;
    else {
        return a->GetFrameID() < b->GetFrameID();
    }
}
auto PointCloud_ex::save_to_pcd( std::string dir_name, std::string _file_name , int save_pts_with_views)->void{
    Common_tools::create_dir(dir_name);
    std::string file_name = std::string(dir_name).append(_file_name).append(".pcd");
    // 更快 ,但人工不可读
    pcl::io::savePCDFileBinary(std::string(file_name), pts_cloud);
    std::cout << COUTDEBUG << " save to "<< file_name<< std::endl;
    // pcl::io::savePCDFileASCII(file_name, pts_cloud);
}
auto PointCloud_ex::CompStamp(PointCloudEXPtr kf1, PointCloudEXPtr kf2)->bool {
    return  kf1->timestamp_ > kf2->timestamp_;
}
auto PointCloud_ex::save_and_display_pointcloud( std::string dir_name, std::string file_name ,  int save_pts_with_views)->void{


    save_to_pcd(dir_name, file_name, save_pts_with_views);
    scope_color(ANSI_COLOR_WHITE_BOLD);
    cout << "========================================================" << endl;
    cout << "Open pcl_viewer to display point cloud, close the viewer's window to continue mapping process ^_^" << endl;
    cout << "========================================================" << endl;
    system(std::string("pcl_viewer ").append(dir_name).append(file_name).c_str());

}
}