#include "mapmanager.hpp"
namespace colive
{

MapManager::MapManager()
{
    std::cout << "new MapManager" << std::endl;

    
    // // Other types of place recognition system could be integrated and activated using the placerec::type parameter
    // if(covins_params::placerec::type == "COVINS" || covins_params::placerec::type == "COVINS_G") {
    //     database_.reset(new KeyframeDatabase(voc_));
    // } else {
    //     std::cout << COUTFATAL << "Place Recognition System Type \"" << covins_params::placerec::type << "\" not valid" << std::endl;
    //     exit(-1);
    // }

    // if(!voc_) {
    //     std::cout << COUTFATAL << "invalid vocabulary ptr" << std::endl;
    //     exit(-1);
    // }

    pcl_pc.reset(new PointCloud);
    pcl_pc_d.reset(new PointCloud);
    float filter_size = 0.4; 
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);
    scManager.setSCdistThres(0.2);   //scDistThres
    scManager.setMaximumRadius(80);//scMaximumRadius
}

auto MapManager::Run()->void {
    while(1) {
        if(this->CheckMergeBuffer()) {
            this->PerformMerge();
        }

        usleep(5000);
    }
}

auto MapManager::CheckoutMap(int map_id, int &check_num)->MapPtr {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit == maps_.end()){
        std::cout << COUTERROR << "no existing map with Map-ID " << map_id << std::endl;
        return nullptr;
    }

    MapInstancePtr map = mit->second;

    if(map->usage_cnt == -1){
        return nullptr;
    } else if(map->block_checkout) {
        return nullptr;
    } else {
        int check = rand();
        map->usage_cnt++;
        map->check_nums.insert(check);
        check_num = check;

        return map->map;
    }
    return nullptr;
}

auto MapManager::CheckoutMapExclusive(int map_id, int &check_num)->MapPtr {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit == maps_.end()){
        std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
        return nullptr;
    }

    MapInstancePtr map = mit->second;

    if(map->usage_cnt != 0){
        return nullptr;
    } else {
        int check = rand();
        map->usage_cnt = -1;
        map->check_nums.insert(check);
        map->block_checkout = false;
        check_num = check;

        return map->map;
    }
}

auto MapManager::CheckoutMapExclusiveOrWait(int map_id, int &check_num)->MapPtr {

    {
        MapContainer::iterator mit = maps_.find(map_id);
        if(mit == maps_.end()){
            std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
            return nullptr;
        }
    }

    MapPtr map = this->CheckoutMapExclusive(map_id,check_num);
    if(!map) {
        while(!this->SetCheckoutBlock(map_id,true))
            usleep(100);
    }

    while(!map){
        map = this->CheckoutMapExclusive(map_id,check_num);
        usleep(100);
    }

    return map;
}

auto MapManager::CheckoutMapOrWait(int map_id, int &check_num)->MapPtr {

    {
        MapContainer::iterator mit = maps_.find(map_id);
        if(mit == maps_.end()){
            std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
            return nullptr;
        }
    }

    MapPtr map = this->CheckoutMap(map_id,check_num);
    while(!map){
        map = this->CheckoutMap(map_id,check_num);
        usleep(100);
    }
    return map;
}
auto MapManager::SetCheckoutBlock(int map_id, bool val)->bool {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit == maps_.end()){
        std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
        exit(-1);
    }

    MapInstancePtr map = mit->second;

    if(val && map->block_checkout) {
        return false;
    } else {
        map->block_checkout = val;
        return true;
    }
}

auto MapManager::InitializeMap(int map_id)->void {
    std::unique_lock<std::mutex> lock(mtx_access_);
    MapContainer::iterator mit = maps_.find(map_id);
    if(mit != maps_.end()){
        std::cout << COUTFATAL << "Existing map with Map-ID " << map_id << std::endl;
        exit(-1);
    }
    std::cout << "create a new map with Map-ID " << map_id << std::endl;
    MapInstancePtr map(new MapInstance(map_id));
    maps_[map_id] = map;

    // this->Display();
}
auto MapManager::ReturnMap(int map_id, int &check_num)->void {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit == maps_.end()){
        std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
        exit(-1);
    }

    MapInstancePtr map = mit->second;

    if(!map->check_nums.count(check_num)){
        std::cout << COUTFATAL << "check_num error" << std::endl;
        exit(-1);
    }

    if(map->usage_cnt == -1)
        map->usage_cnt = 0;
    else
        map->usage_cnt--;

    map->check_nums.erase(check_num);
}

auto MapManager::RegisterMap(MapPtr external_map)->bool {

    // if(enter_kfs_in_database) {
    //     auto keyframes = external_map->GetKeyframesVec();
    //     for(const auto& kf : keyframes) {
    //         this->AddToDatabase(kf);
    //     }
    // } else {
    //     // skip this because we do a PR test
    //     std::cout << COUTNOTICE << "!!! KFs not entered in database !!!" << std::endl;
    // }

    MapInstancePtr map_inst(new MapInstance(external_map));
    for(std::set<size_t>::iterator sit = map_inst->map->associated_clients_.begin();sit != map_inst->map->associated_clients_.end();++sit) {
        std::cout << "----> Add maps_[" << *sit << "]" << std::endl;
        maps_[*sit] = map_inst;
    }
    return true;
}

auto MapManager::RegisterMerge(MergeInformation merge_data)->void {
    std::unique_lock<std::mutex> lock(mtx_access_);
    buffer_merge_.push_back(merge_data);
}


auto MapManager::CheckMergeBuffer()->bool {
    std::unique_lock<std::mutex> lock(mtx_access_);
    return !(buffer_merge_.empty());
}
auto MapManager::PerformMerge()->void {

    std::cout << "+++ Perform Merge +++" << std::endl;

    // KeyframePtr kf_query;
    // KeyframePtr kf_match;
    // TransformType T_smatch_squery;
    // TypeDefs::Matrix6Type cov_mat;

    std::cout << "--> Fetch merge data" << std::endl;
    {
        // std::unique_lock<std::mutex> lock(mtx_access_);
        // MergeInformation merge = buffer_merge_.front();
        // buffer_merge_.pop_front();
        // kf_query = merge.kf_query;
        // kf_match = merge.kf_match;
        // T_smatch_squery = merge.T_smatch_squery;
        // cov_mat = merge.cov_mat;
    }
    std::cout << "--> Process merge data" << std::endl;

    // int check_query, check_match;

    // this->CheckoutMapExclusiveOrWait(kf_query->id_.second,check_query);
    // MapInstancePtr map_query = maps_[kf_query->id_.second];

    // this->CheckoutMapExclusiveOrWait(kf_match->id_.second,check_match);
    // MapInstancePtr map_match = maps_[kf_match->id_.second];


    std::cout << "----> Merge maps" << std::endl;

//     TransformType T_w_squery = kf_query->GetPoseTws();
//     TransformType T_w_smatch = kf_match->GetPoseTws();
//     TransformType T_wmatch_wquery = T_w_smatch * T_smatch_squery * T_w_squery.inverse();

//     MapInstancePtr map_merged(new MapInstance(map_match,map_query,T_wmatch_wquery));

//     LoopConstraint lc(kf_match,kf_query,T_smatch_squery, cov_mat);
//     map_merged->map->AddLoopConstraint(lc);

// //    std::cout << "Map Match: Agents|KFs|LMs :" << map_match->map->associated_clients_.size() << "|" << map_match->map->GetKeyframes().size() << "|" << map_match->map->GetLandmarks().size() << std::endl;
// //    std::cout << "Map Query: Agents|KFs|LMs :" << map_query->map->associated_clients_.size() << "|" << map_query->map->GetKeyframes().size() << "|" << map_query->map->GetLandmarks().size() << std::endl;
//     std::cout << "Merged Map: Agents|KFs|LMs: " << map_merged->map->associated_clients_.size() << "|" << map_merged->map->GetKeyframes().size() << "|" << map_merged->map->GetLandmarks().size() << std::endl;

//     for(std::set<size_t>::iterator sit = map_merged->map->associated_clients_.begin();sit != map_merged->map->associated_clients_.end();++sit) {
//         maps_[*sit] = map_merged;
//     }

//     map_merged->usage_cnt = 0;

    std::cout << "\033[1;32m+++ MAPS MERGED +++\033[0m" << std::endl;
}
auto MapManager::Display()->void {
    std::unique_lock<std::mutex> lock(mtx_access_);

    // while(true) {//temp should be deleted
    for(auto iter = maps_.begin(); iter != maps_.end(); iter++)  {
            std::cout<<"Mapmanager: map id:"<<iter->first<<"  "<<std::endl;;
            auto map = iter->second->map;
            // std::cout<<"poitcloud frame num:"<<map->pointclouds_.size()<<"  "<<std::endl;
            map->Display();
        }
    // sleep(5);
    // }
   
}
// auto MapManager::Display()->void {
//     this->Display();
// }

auto MapManager::lcd()->void {


}

// TODO: should be deleted
void pointcloud_convert1(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out){

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
auto MapManager::AddToDatabase(PointCloudEXPtr pc)    ->void{
    *pcl_pc  = pc->pts_cloud;
    downSizeFilterScancontext.setInputCloud(pcl_pc);
    downSizeFilterScancontext.filter(*pcl_pc_d);

    // PointCloud::Ptr cloud_in(new PointCloud(pc->pts_cloud));
#if 0
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    pointcloud_convert1(pcl_pc_d,cloud_out);
    scManager.makeAndSaveScancontextAndKeys(*cloud_out);
#endif    
#if 1
    scManager.makeAndSaveScancontextAndKeys(*pcl_pc_d);
    // std::cout<<"add query1"<<std::endl;
#endif 
    // std::cout<<"add query2"<<std::endl;
    pc->pts_cloud_d = *pcl_pc_d;
    cl_pcs.push_back(pc);

    // scManager.makeAndSaveScancontextAndKeys(*pcl_pc_d);
    // cl_pcs_d.push_back(PointCloud::Ptr in(new *pcl_pc_d));
    // *pts_cloud = pc->pts_cloud;
    // downSizeFilterScancontext.setInputCloud(pts_cloud);
    // downSizeFilterScancontext.filter(*sc_pcs_d);

}

}