#include "map_co.hpp"




#include "pointcloud_ex.hpp"
#include "image_ex.hpp"

// #include "map_rgb.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

namespace colive {


MapInstance::MapInstance(int id) {
    map.reset(new Map(id));
}
MapInstance::MapInstance(MapInstancePtr map_target, MapInstancePtr map_tofuse, TransformType T_wtofuse_wmatch)
    : usage_cnt(-1)
{
    map.reset(new Map(map_target->map,map_tofuse->map,T_wtofuse_wmatch));
}

MapInstance::MapInstance(MapPtr external_map) {
    map = external_map;
}


Map::Map(size_t id_)
: id_map_(id_)
{
    if(id_map_ > 1000) {
        // std::cout << COUTFATAL << "Map initialized with ID " << id_map_ << std::endl;
        exit(-1);
    }
    associated_clients_.insert(id_);
    // T_ws <<  1,0,0,0,
    //                 0,1,0,0,
    //                 0,0,1,0,
    //                 0,0,0,1;
}
// 把map_tofuse 转换到 map_target位置 并生成一个新的map
Map::Map(MapPtr map_target, MapPtr map_tofuse, TransformType T_wtofuse_wtarget)
    // : Map_V(map_target->id_map_)
{
    
    // // data map_target
    std::set<size_t> associated_clients_target = map_target->associated_clients_;
    PointCloudEXMap pointcloudex_target = map_target->GetPointCloudEXs();
    ImageEXMap image_target = map_target->GetImageEXs();
    LoopVector loops_target = map_target->GetLoopConstraints();    
    // KeyframeMap keyframes_target = map_target->GetKeyframes();
    // LandmarkMap landmarks_target = map_target->GetLandmarks();
    // size_t tmax_id_kf_target = map_target->GetMaxKfId();
    // size_t max_id_lm_target = map_target->GetMaxLmId();


    // // data map_tofuse
    std::set<size_t> associated_clients_tofuse = map_tofuse->associated_clients_;
    PointCloudEXMap pointcloudex_tofuse = map_tofuse->GetPointCloudEXs();
    ImageEXMap image_tofuse = map_tofuse->GetImageEXs();
    LoopVector loops_tofuse = map_tofuse->GetLoopConstraints();    
    // KeyframeMap keyframes_tofuse = map_tofuse->GetKeyframes();
    // LandmarkMap landmarks_tofuse = map_tofuse->GetLandmarks();
    // size_t tmax_id_kf_tofuse = map_tofuse->GetMaxKfId();
    // size_t max_id_lm_tofuse = map_tofuse->GetMaxLmId();


    // //fill new map
    associated_clients_.insert(associated_clients_target.begin(),associated_clients_target.end());
    associated_clients_.insert(associated_clients_tofuse.begin(),associated_clients_tofuse.end());

    pointclouds_.insert(pointcloudex_target.begin(),pointcloudex_target.end());
    pointclouds_.insert(pointcloudex_tofuse.begin(),pointcloudex_tofuse.end());


    images_.insert(image_target.begin(),image_target.end());
    images_.insert(image_tofuse.begin(),image_tofuse.end());

    // keyframes_.insert(keyframes_target.begin(),keyframes_target.end());
    // keyframes_.insert(keyframes_tofuse.begin(),keyframes_tofuse.end());
    // landmarks_.insert(landmarks_target.begin(),landmarks_target.end());
    // landmarks_.insert(landmarks_tofuse.begin(),landmarks_tofuse.end());
    // max_id_kf_ = std::max(tmax_id_kf_target,tmax_id_kf_tofuse);
    // max_id_lm_ = std::max(max_id_lm_target,max_id_lm_tofuse);
    loop_constraints_.insert(loop_constraints_.end(),loops_target.begin(),loops_target.end());
    loop_constraints_.insert(loop_constraints_.end(),loops_tofuse.begin(),loops_tofuse.end());

    // // Transform poses of map_tofuse

    // PointCloudEXPtr target = pointcloudex_target.begin()->second;
    // TransformType T_lm_s_befcorrection = pc->GetPoseTwg();

        
    for(PointCloudEXMap::iterator mit =pointcloudex_tofuse.begin();mit != pointcloudex_tofuse.end();++mit) {
        PointCloudEXPtr pc = mit->second;
        TransformType T_lm_s_befcorrection = pc->GetPoseTwg();
        TransformType T_lm_s_corrected = T_lm_s_befcorrection*T_wtofuse_wtarget; // tofuse 里的帧在target下的坐标
        pc->SetPoseTwg(T_lm_s_corrected);
        // pc->pointcloud_transform(T_wtarget_wtofuse);
    }

    // rebuild the rgb map
    for(PointCloudEXMap::iterator mit =pointclouds_.begin();mit != pointclouds_.end();++mit){
        PointCloudEXPtr pc = mit->second;
        // m_map_rgb_pts.append_points_to_global_map(pc)
        Add2RGBMap(pc);
    }
    // Matrix3Type R_wmatch_wtofuse = T_wtarget_wtofuse.block<3,3>(0,0);
    // Vector3Type t_wmatch_wtofuse = T_wtarget_wtofuse.block<3,1>(0,3);
    // for(KeyframeMap::iterator mit = keyframes_tofuse.begin();mit != keyframes_tofuse.end();++mit) {
    //     KeyframePtr kf = mit->second;
    //     TransformType T_w_s_befcorrection = kf->GetPoseTws();
    //     TransformType T_w_s_corrected = T_wtarget_wtofuse * T_w_s_befcorrection;
    //     kf->SetPoseTws(T_w_s_corrected);
    //     kf->velocity_ = T_wtarget_wtofuse.block<3,3>(0,0) * kf->velocity_;
    // }
    // Matrix3Type R_wmatch_wtofuse = T_wtarget_wtofuse.block<3,3>(0,0);
    // Vector3Type t_wmatch_wtofuse = T_wtarget_wtofuse.block<3,1>(0,3);
    // for(LandmarkMap::iterator mit = landmarks_tofuse.begin();mit != landmarks_tofuse.end();++mit) {
    //     LandmarkPtr lm = mit->second;
    //     Vector3Type pos_w_befcorrection = lm->GetWorldPos();
    //     Vector3Type pos_w_corrected = R_wmatch_wtofuse * pos_w_befcorrection + t_wmatch_wtofuse;
    //     lm->SetWorldPos(pos_w_corrected);
    // }
}
// auto Map::AddPointCloud(PointCloudEXPtr pc)->void {
//     this->AddPointCloud(pc,false);
// }
auto Map::AddPointCloud(PointCloudEXPtr pc, bool suppress_output)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    pointclouds_[pc->id_] = pc;
    max_id_pc_ = std::max(max_id_pc_,pc->GetFrameID());

    Add2RGBMap(pc);

    if(!suppress_output && !(pointclouds_.size() % 50)) {
        // std::cout << "Map " << this->id_map_  << " : " << keyframes_.size() << " KFs | " << landmarks_.size() << " LMs" << std::endl;
        // this->WriteKFsToFile();
        // this->WriteKFsToFileAllAg();
    }
}
// auto Map::AddImage(ImageEXPtr img)->void {
//     this->AddImage(img,false);
// }
auto Map::AddImage(ImageEXPtr img, bool suppress_output)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    images_[img->id_] = img;
    max_id_img_ = std::max(max_id_img_,img->GetFrameID());

    // Add2RGBMap(pc);

    if(!suppress_output && !(pointclouds_.size() % 50)) {
        // std::cout << "Map " << this->id_map_  << " : " << keyframes_.size() << " KFs | " << landmarks_.size() << " LMs" << std::endl;
        // this->WriteKFsToFile();
        // this->WriteKFsToFileAllAg();
    }
}
auto Map::Add2RGBMap(PointCloudEXPtr pc)->void {
    int m_append_global_map_point_step = 1;
    bool m_if_record_mvs =true;//if record_offline_map
    if ( m_if_record_mvs )
    {
        std::vector< std::shared_ptr< RGB_pts > > pts_last_hitted;
        pts_last_hitted.reserve( 1e6 );

        m_number_of_new_visited_voxel = m_map_rgb_pts.append_points_to_global_map( pc, &pts_last_hitted,m_append_global_map_point_step);

        m_map_rgb_pts.m_mutex_pts_last_visited->lock();
        m_map_rgb_pts.m_pts_last_hitted = pts_last_hitted;
        m_map_rgb_pts.m_mutex_pts_last_visited->unlock();

    }else{
        m_number_of_new_visited_voxel = m_map_rgb_pts.append_points_to_global_map( pc, nullptr,m_append_global_map_point_step);
    }

}
auto Map::AddLoopConstraint(LoopConstraint lc)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    // std::cout << COUTDEBUG << "frame: " << lc.pc1->GetClientID() << lc.pc2->GetClientID()<<std::endl;
    loop_constraints_.push_back(lc);
    lc.pc1->is_loop_ = true;
    lc.pc2->is_loop_ = true;
}
auto Map::GetLoopConstraints()->LoopVector {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return loop_constraints_;
}
// auto Map::GetPointCloudEX()->PointCloudEXMap {
//     std::unique_lock<std::mutex> lock(mtx_map_);
//     return pointclouds_;
// }
auto Map::GetFamilyPc(size_t client_id)->TransformType  {
    std::unique_lock<std::mutex> lock(mtx_map_);
    idpair idp;
    idp.first=0;
    idp.second=client_id;
    PointCloudEXMap::iterator mit =  pointclouds_.find(idp);
    if(mit != pointclouds_.end()) return mit->second->GetPoseTwg();
    else {
        return  TransformType::Identity() ;
    }
}
auto Map::GetPointCloudEX(idpair idp)->PointCloudEXPtr {
    std::unique_lock<std::mutex> lock(mtx_map_);
    PointCloudEXMap::iterator mit =  pointclouds_.find(idp);
    if(mit != pointclouds_.end()) return mit->second;
    else {
        return nullptr;
    }
}
auto Map::GetPointCloudEXs()->PointCloudEXMap {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return pointclouds_;
}

auto Map::GetFamilyImg(size_t client_id)->TransformType  {
    std::unique_lock<std::mutex> lock(mtx_map_);
    idpair idp;
    idp.first=0;
    idp.second=client_id;
    ImageEXMap::iterator mit =  images_.find(idp);
    if(mit != images_.end()) return mit->second->GetPoseTwg();
    else {
        return  TransformType::Identity() ;
    }
}
auto Map::GetImageEX(idpair idp)->ImageEXPtr {
    std::unique_lock<std::mutex> lock(mtx_map_);
    ImageEXMap::iterator mit =  images_.find(idp);
    if(mit != images_.end()) return mit->second;
    else {
        return nullptr;
    }
}
auto Map::GetImageEXs()->ImageEXMap {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return images_;
}
auto Map::Display()->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    // keyframes_.size();
    // landmarks_
    std::cout<<"pointcloud frames num:"<<pointclouds_.size()<<std::endl;
}


}