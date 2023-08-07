#include "mapmanager.hpp"


// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/common/transforms.h>


#include "pointcloud_ex.hpp"
#include "image_ex.hpp"

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
    // p_pc_large_tmps.reserve(MAX_CLIENT_NUM);
    float filter_size = 0.4; 
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);
    scManager.setSCdistThres(0.1);   //scDistThres
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
// 获取共享map
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
        // std::cout << COUTDEBUG << "CheckoutMap usage_cnt"<<map->usage_cnt<<std::endl;
        return nullptr;
    } else {
        int check = rand();
        map->usage_cnt++;
        // std::cout << COUTDEBUG << "map->usage_cnt++:"<<map->usage_cnt<<std::endl;
        map->check_nums.insert(check);
        check_num = check;

        return map->map;
    }
    return nullptr;
}
// 获取专享map
auto MapManager::CheckoutMapExclusive(int map_id, int &check_num)->MapPtr {
    std::unique_lock<std::mutex> lock(mtx_access_);

    MapContainer::iterator mit = maps_.find(map_id);
    if(mit == maps_.end()){
        std::cout << COUTFATAL << "no existing map with Map-ID " << map_id << std::endl;
        return nullptr;
    }

    MapInstancePtr map = mit->second;

    if(map->usage_cnt != 0 ){
    // if(map->usage_cnt != 0 && map->usage_cnt > 0 && map->check_nums.size() != 0){
        // std::cout << COUTDEBUG << "map->usage_cnt"<<map->usage_cnt<<std::endl;
        return nullptr;
    } else {
        if(map->check_nums.size() != 0){
            static uint32_t check_cout=0;
            if(check_cout%100==0){
                std::cout << COUTFATAL << "have error here, force repair"<<std::endl;
            }
            check_cout++;
            return nullptr;
        }
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
    uint32_t wait_cout = 0 ;
    MapPtr map = this->CheckoutMapExclusive(map_id,check_num);
    if(!map) {
        while(!this->SetCheckoutBlock(map_id,true)){
            usleep(100);
            wait_cout++;
            if(wait_cout>3*1000000/100 && wait_cout % 1000 == 0){
                std::cout << COUTWARN << "CheckoutMapExclusive SetCheckoutBlock spend too much time."  << std::endl;
            }
        }
            
    }
    wait_cout=0;
    while(!map){
        map = this->CheckoutMapExclusive(map_id,check_num);
        // std::cout << COUTFATAL << "blocked " << map_id << std::endl;
        usleep(100);
        wait_cout++;
        if(wait_cout>3*1000000/100 && wait_cout % 1000 == 0){
            std::cout << COUTWARN << "CheckoutMapExclusive spend too much time."  << std::endl;
        }
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
    uint32_t wait_cout = 0 ;
    MapPtr map = this->CheckoutMap(map_id,check_num);
    while(!map){
        map = this->CheckoutMap(map_id,check_num);
        usleep(100);
        wait_cout++;
        if(wait_cout>3*1000000/100 && wait_cout % 1000 == 0){
            std::cout << COUTWARN << "CheckoutMap spend too much time."  << std::endl;
        }
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
        std::cout << COUTFATAL <<"check_num :"<< map_id<<check_num<< "check_num have";
        for (const auto& check_num_ : map->check_nums) {
            std::cout << check_num_ << " ";
        }
        std::cout << std::endl;
        exit(-1);
    }

    if(map->usage_cnt == -1)
        map->usage_cnt = 0;
    else
        map->usage_cnt--;
                // std::cout << COUTDEBUG << "map->usage_cnt--:"<<map->usage_cnt<<std::endl;
    // std::cout << COUTFATAL <<"return"<<map->usage_cnt<<std::endl;
    map->check_nums.erase(check_num);
    if(map->check_nums.size() != map->usage_cnt && map->usage_cnt != -1){
        std::cout << COUTFATAL <<  "returnmap error"<<map->check_nums.size() <<" "<<map->usage_cnt<<std::endl;
    }
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

    PointCloudEXPtr pc_query;
    PointCloudEXPtr pc_match;
    // KeyframePtr kf_query;
    // KeyframePtr kf_match;
    TransformType T_squery_smatch;
    // TypeDefs::Matrix6Type cov_mat;

    std::cout << "--> Fetch merge data" << std::endl;
    {
        std::unique_lock<std::mutex> lock(mtx_access_);
        MergeInformation merge = buffer_merge_.front();
        buffer_merge_.pop_front();
        pc_query = merge.pc_query;
        pc_match = merge.pc_match;
        T_squery_smatch = merge.T_squery_smatch;
        // cov_mat = merge.cov_mat;
    }
    std::cout << "--> Process merge data" << std::endl;

    int check_query, check_match;

    // MapInstancePtr map_query = MapInstancePtr this->CheckoutMapExclusiveOrWait(pc_query->GetClientID(),check_query);
    //
    if(pc_match->GetClientID() == pc_query->GetClientID()){
        std::cout << COUTFATAL << std::endl;
        return;
    }
 
    this->CheckoutMapExclusiveOrWait(pc_match->GetClientID(),check_match);
    MapInstancePtr map_match = maps_[pc_match->GetClientID()];

    // std::cout << COUTDEBUG << "CheckoutMapExclusiveOrWait "<< pc_query->GetClientID()<<check_query<< std::endl;
    if(pc_match->map_->associated_clients_.count(pc_query->GetClientID())){
        std::cout << COUTERROR << "map have merged" <<std::endl;
        // std::cout << COUTFATAL <<"map_query->map->associated_clients_ :"<< map_id<<check_num<< "check_num have";
        // for (const auto& client : map_query->map->associated_clients_) {
        //     std::cout << client << " ";
        // }
        // std::cout << std::endl;
        return;
    }
   this->CheckoutMapExclusiveOrWait(pc_query->GetClientID(),check_query);
    MapInstancePtr map_query = maps_[pc_query->GetClientID()];



    // std::cout << COUTDEBUG << "CheckoutMapExclusiveOrWait "<< pc_match->GetClientID()<<check_match<< std::endl;

    // MapInstancePtr map_query = maps_[pc_query->id_.second];
    // MapInstancePtr map_match = maps_[pc_match->id_.second];
    // this->CheckoutMapExclusiveOrWait(kf_match->GetClientID(),check_match);
    // MapInstancePtr map_match = maps_[kf_match->id_.second];


    std::cout << "----> Merge maps" << std::endl;

    // T_qw_qs=T_qw_qlm*T_qlm_qs
    // T_qw_qs=T_mw_mlm*T_mlm_ms
    // // T_qw_ms=T_qw_qs*T_qs_ms
    // T_mw_qs=T_mw_ms*T_ms_qs
    // T_qw_qlm=T_mw_qs*T_qs_qlm   ==qTG

    // T_squery_smatch = pc_query.T_body_s_*pc_match.T_body_s_.inverse();  
    // TransformType T_s_wquery = pc_query.T_lm_w_*pc_query.T_s_lm_;  // TG
    // TransformType T_s_wmatch = pc_match.T_lm_w_*pc_match.T_s_lm_;
    // TransformType T_wmatch_wquery = T_w_smatch * T_squery_smatch * T_w_squery.inverse();
#if 1
    // auto pc_query_w_g = pc_query->GetPoseTws() * T_squery_smatch * pc_match->GetPoseTsg();
    // pc_query->SetPoseTwg(pc_query_w_g);
    // TransformType T_wtofuse_wmatch = T_squery_smatch;
    TransformType T_wquery_wmatch = pc_query->GetPoseTws() *  T_squery_smatch * pc_match->GetPoseTsw();

    TransformType T_wquery_gmatch =  T_wquery_wmatch * pc_match->GetPoseTwg();

    TransformType T_wquery_wquery_new = T_wquery_gmatch * pc_query->GetPoseTwg().inverse();


    // TransformType T_gtofuse_gmatch = T_gtofuse_wtofuse * T_wtofuse_gmatch;

    // for(PointCloudEXMap::iterator mit =pointcloudex_tofuse.begin();mit != pointcloudex_tofuse.end();++mit) {
    //     PointCloudEXPtr pc_tofuse = mit->second;
    //     TransformType T_gtofuse_gmatch = pc_tofuse->GetPoseTgw() * T_wtofuse_gmatch;
        
    // }
    // TransformType T_gtofuse_gmatch = pc_query->GetPoseTgs() *  T_squery_smatch * pc_match->GetPoseTsg();

#endif


    MapInstancePtr map_merged(new MapInstance(map_match,map_query,T_wquery_wquery_new));

//     LoopConstraint lc(kf_match,kf_query,T_squery_smatch, cov_mat);
//     map_merged->map->AddLoopConstraint(lc);

//    std::cout << "Map Match: Agents|KFs|LMs :" << map_match->map->associated_clients_.size() << "|" << map_match->map->GetKeyframes().size() << "|" << map_match->map->GetLandmarks().size() << std::endl;
//    std::cout << "Map Query: Agents|KFs|LMs :" << map_query->map->associated_clients_.size() << "|" << map_query->map->GetKeyframes().size() << "|" << map_query->map->GetLandmarks().size() << std::endl;
    std::cout <<COUTNOTICE<< "Merged Map: Agents|PCs|Pts: " << map_merged->map->associated_clients_.size() << "|" << map_merged->map->GetPointCloudEXs().size() << "|" << 
    map_merged->map->m_map_rgb_pts->m_rgb_pts_vec.size()<< "|" << 
    std::endl;

    
    for(std::set<size_t>::iterator sit = map_merged->map->associated_clients_.begin();sit != map_merged->map->associated_clients_.end();++sit) {
        std::cout  << COUTDEBUG <<"remap client:"<< *sit <<  std::endl;
        maps_[*sit] = map_merged;
    }

    map_merged->usage_cnt = 0; // 激活与释放map，让其他线程可调用

    // map->check_nums
    // ReturnMap(pc_query->GetClientID(),check_query);
    // ReturnMap(pc_match->GetClientID(),check_match);
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

    // livox camera "calibration"

    
    // pc_large

    // *pc_large=*pc
    // pc_large->id_=pc->id_;
    // pc_large->timestamp_=pc->timestamp_;
    // pc_large->pts_cloud=pc->pts_cloud;
    // pc_large->SetPoseTws(pc->GetPoseTws());
    // scManager.makeAndSaveScancontextAndKeys(*pcl_pc_d);
    // cl_pcs_d.push_back(PointCloud::Ptr in(new *pcl_pc_d));
    // *pts_cloud = pc->pts_cloud;
    // downSizeFilterScancontext.setInputCloud(pts_cloud);
    // downSizeFilterScancontext.filter(*sc_pcs_d);

}

}