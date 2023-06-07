#include "map_co.hpp"

namespace colive {

Map_V::Map_V(size_t id)
    : id_map_(id)
{
    if(id_map_ > 1000) {
        std::cout << COUTFATAL << "Map initialized with ID " << id_map_ << std::endl;
        exit(-1);
    }
}

auto Map_V::Clear()->void {
    keyframes_.clear();
    landmarks_.clear();
    max_id_kf_ = 0;
    max_id_lm_ = 0;
}

auto Map_V::GetKeyframe(idpair idp, bool expect_null)->KeyframePtr {
    std::unique_lock<std::mutex> lock(mtx_map_);
    KeyframeMap::iterator mit = keyframes_.find(idp);
    if(mit != keyframes_.end()) return mit->second;
    else {
        return nullptr;
    }
}

auto Map_V::GetKeyframes()->KeyframeMap {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return keyframes_;
}

auto Map_V::GetKeyframesErased()->KeyframeMap {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return keyframes_erased_;
}

auto Map_V::GetKeyframesVec()->KeyframeVector {
    std::unique_lock<std::mutex> lock(mtx_map_);
    KeyframeVector kfs;
    for(KeyframeMap::iterator mit = keyframes_.begin();mit!=keyframes_.end();++mit)
        kfs.push_back(mit->second);
    return kfs;
}

auto Map_V::GetLandmark(idpair idp)->LandmarkPtr {
    std::unique_lock<std::mutex> lock(mtx_map_);
    LandmarkMap::iterator mit = landmarks_.find(idp);
    if(mit != landmarks_.end()) return mit->second;
    else {
        return nullptr;
    }
}

auto Map_V::GetLandmarks()->LandmarkMap {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return landmarks_;
}

auto Map_V::GetLandmarksVec()->LandmarkVector {
    std::unique_lock<std::mutex> lock(mtx_map_);
    LandmarkVector lms;
    for(LandmarkMap::iterator mit = landmarks_.begin();mit!=landmarks_.end();++mit)
        lms.push_back(mit->second);
    return lms;
}

auto Map_V::GetMaxKfId()->size_t {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return max_id_kf_;
}

auto Map_V::GetMaxLmId()->size_t {
    std::unique_lock<std::mutex> lock(mtx_map_);
    return max_id_lm_;
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


}