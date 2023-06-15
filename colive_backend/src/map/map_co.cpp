#include "map_co.hpp"

namespace colive {


MapInstance::MapInstance(int id) {
    map.reset(new Map(id));
}
MapInstance::MapInstance(MapInstancePtr map_target, MapInstancePtr map_tofuse, TransformType T_wmatch_wtofuse)
    : usage_cnt(-1)
{
    map.reset(new Map(map_target->map,map_tofuse->map,T_wmatch_wtofuse));
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
}

Map::Map(MapPtr map_target, MapPtr map_tofuse, TransformType T_wtarget_wtofuse)
    // : Map_V(map_target->id_map_)
{
    // // data map_target
    // std::set<size_t> associated_clients_target = map_target->associated_clients_;
    // KeyframeMap keyframes_target = map_target->GetKeyframes();
    // LandmarkMap landmarks_target = map_target->GetLandmarks();
    // size_t tmax_id_kf_target = map_target->GetMaxKfId();
    // size_t max_id_lm_target = map_target->GetMaxLmId();
    // LoopVector loops_target = map_target->GetLoopConstraints();

    // // data map_tofuse
    // std::set<size_t> associated_clients_tofuse = map_tofuse->associated_clients_;
    // KeyframeMap keyframes_tofuse = map_tofuse->GetKeyframes();
    // LandmarkMap landmarks_tofuse = map_tofuse->GetLandmarks();
    // size_t tmax_id_kf_tofuse = map_tofuse->GetMaxKfId();
    // size_t max_id_lm_tofuse = map_tofuse->GetMaxLmId();
    // LoopVector loops_tofuse = map_tofuse->GetLoopConstraints();

    // //fill new map
    // associated_clients_.insert(associated_clients_target.begin(),associated_clients_target.end());
    // associated_clients_.insert(associated_clients_tofuse.begin(),associated_clients_tofuse.end());
    // keyframes_.insert(keyframes_target.begin(),keyframes_target.end());
    // keyframes_.insert(keyframes_tofuse.begin(),keyframes_tofuse.end());
    // landmarks_.insert(landmarks_target.begin(),landmarks_target.end());
    // landmarks_.insert(landmarks_tofuse.begin(),landmarks_tofuse.end());
    // max_id_kf_ = std::max(tmax_id_kf_target,tmax_id_kf_tofuse);
    // max_id_lm_ = std::max(max_id_lm_target,max_id_lm_tofuse);
    // loop_constraints_.insert(loop_constraints_.end(),loops_target.begin(),loops_target.end());
    // loop_constraints_.insert(loop_constraints_.end(),loops_tofuse.begin(),loops_tofuse.end());

    // // Transform poses of map_tofuse
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
auto Map::AddPointCloud(PointCloudEXPtr pc)->void {
    this->AddPointCloud(pc,false);
}
auto Map::AddPointCloud(PointCloudEXPtr pc, bool suppress_output)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    pointclouds_[pc->id_] = pc;
    max_id_pc_ = std::max(max_id_pc_,pc->id_.first);
    if(!suppress_output && !(pointclouds_.size() % 50)) {
        // std::cout << "Map " << this->id_map_  << " : " << keyframes_.size() << " KFs | " << landmarks_.size() << " LMs" << std::endl;
        // this->WriteKFsToFile();
        // this->WriteKFsToFileAllAg();
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
auto Map::Display()->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    // keyframes_.size();
    // landmarks_
    std::cout<<"pointcloud frames num:"<<pointclouds_.size()<<std::endl;
}


}