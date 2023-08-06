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

    m_thread_pool_ptr->commit_task(&Map::Add2RGBMap_service,this);

    m_map_rgb_pts.reset(new Global_map());
    // thread_rgb_map_.reset(new std::thread(&Map::Add2RGBMap_service,this));
    // thread_rgb_map_->detach();
    // thread_rgb_map_=std::thread(Map::Add2RGBMap_service);
    // T_ws <<  1,0,0,0,
    //                 0,1,0,0,
    //                 0,0,1,0,
    //                 0,0,0,1;
}
// 把map_tofuse 转换到 map_target位置 并生成一个新的map
Map::Map(MapPtr map_target, MapPtr map_tofuse, TransformType T_wtofuse_wmatch)
    : id_map_(map_target->id_map_)
{
    // id_map_=map_target.id_map_;
    if(id_map_ > 1000) {
        std::cout << COUTFATAL << "Map initialized with ID " << id_map_ << std::endl;
        exit(-1);
    }

    m_thread_pool_ptr->commit_task(&Map::Add2RGBMap_service,this);

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

    
    TransformType T_wquery_wquery_new = T_wtofuse_wmatch;
    TransformType T_test = TransformType::Identity();

    // T_test.translate(Vector3Type(0,0,8));
    // T_test.tranlate();
    for(PointCloudEXMap::iterator mit =pointcloudex_tofuse.begin();mit != pointcloudex_tofuse.end();++mit) {
        PointCloudEXPtr pc = mit->second;
        // TransformType T = T_wtofuse_wmatch;
        pc->SetPoseTwg( T_test);

    // TransformType T_wtarget_gtarget  = map_target->GetFamilyPc(map_target);
    // TransformType T_wtofuse_gtarget  = T_wtofuse_wtarget * T_wtarget_gtarget;

        // TransformType T_wtofuse_gtofuse = pc->GetPoseTwg();
        // TransformType T_wtarget_gtofuse_corrected =T_wtofuse_gtofuse*T_wtofuse_wmatch; // tofuse 里的帧在target下的坐标
        // pc->SetPoseTwg(T_wtarget_gtofuse_corrected);// TODO： 这里应该算错了吧

        // pc->pointcloud_transform(T_wtarget_wtofuse);
    }

    // rebuild the rgb map
    // m_thread_pool_ptr->commit_task( &PlaceRecognition::Run,placerec_ );
    auto rgbpts_target = map_target->m_map_rgb_pts->m_rgb_pts_vec.size();
    auto rgbpts_tofuse = map_tofuse->m_map_rgb_pts->m_rgb_pts_vec.size();

    map_target->m_map_rgb_pts->merge(map_tofuse->m_map_rgb_pts, T_test);
    // m_map_rgb_pts.reset(map_target->m_map_rgb_pts);
    m_map_rgb_pts=map_target->m_map_rgb_pts;
    
    auto rgbpts_merged = map_target->m_map_rgb_pts->m_rgb_pts_vec.size();

    std::cout << COUTNOTICE <<"merge info: " 
    << "|" << "target:"<<map_target->id_map_ <<"."<<rgbpts_target
    << "|" << "tofuse:"<<map_tofuse->id_map_<<"."<<rgbpts_tofuse
    << "|" << "final:"<<rgbpts_merged 
    << "|" << "final:"<<m_map_rgb_pts->m_rgb_pts_vec.size() 
    << "|" << "T:"<<std::endl
    // <<T_wtofuse_wtarget.matrix()
    << std::endl;

    // pc->save_to_pcd( std::string(colive_params::sys::output_path).append("/frames/pcd/").append(std::to_string(pc->GetClientID())).append("/"), std::to_string(pc->GetTimeStamp()) , 0);

    m_map_rgb_pts->save_to_pcd( std::string(colive_params::sys::output_path).append("/frames/pcd/"),std::string("merged"));
    // for(PointCloudEXMap::iterator mit =pointclouds_.begin();mit != pointclouds_.end();++mit){
    //     PointCloudEXPtr pc = mit->second;
    //     // m_map_rgb_pts.append_points_to_global_map(pc)
    //     // Add2RGBMap(pc);
    //     pcs_should_be_added_to_rgb_map.push(pc);
    // }

    // thread_rgb_map_.reset(new std::thread(&Map::Add2RGBMap_service,this));
    // thread_rgb_map_->detach();

}
// auto Map::AddPointCloud(PointCloudEXPtr pc)->void {
//     this->AddPointCloud(pc,false);
// }
auto Map::AddPointCloud(PointCloudEXPtr pc, bool suppress_output)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    pointclouds_[pc->id_] = pc;
    max_id_pc_ = std::max(max_id_pc_,pc->GetFrameID());

    // size_t client = pc->GetClientID();


// 在另外一个线程里加rgb_map
    pcs_should_be_added_to_rgb_map.push(pc);

    // Add2RGBMap(pc);
    if(!suppress_output && !(pointclouds_.size() % 50)) {
        // std::cout << "Map " << this->id_map_  << " : " << keyframes_.size() << " KFs | " << landmarks_.size() << " LMs" << std::endl;
        // this->WriteKFsToFile();
        // this->WriteKFsToFileAllAg();
    }
}
auto Map::AddPointCloud_large(PointCloudEXPtr pc, bool suppress_output)->void {
    if(pc==nullptr){
        return;
    }

    std::unique_lock<std::mutex> lock(mtx_map_);
    // pointclouds_[pc->id_] = pc;
    // max_id_pc_ = std::max(max_id_pc_,pc->GetFrameID());

    // size_t client = pc->GetClientID();

    // 建立大型点云
    pointclouds_large_[pc->id_] = pc;

    // Add2RGBMap(pc);
    if(!suppress_output && !(pointclouds_.size() % 50)) {
        // std::cout << "Map " << this->id_map_  << " : " << keyframes_.size() << " KFs | " << landmarks_.size() << " LMs" << std::endl;
        // this->WriteKFsToFile();
        // this->WriteKFsToFileAllAg();
    }
}
auto Map::AddImage(ImageEXPtr img, bool suppress_output)->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    images_[img->id_] = img;
    max_id_img_ = std::max(max_id_img_,img->GetFrameID());
// 在另外一个线程里加rgb_map
    // pcs_should_be_added_to_rgb_map.push(pc);

    // Add2RGBMap(pc);
    if(!suppress_output && !(images_.size() % 50)) {
        // std::cout << "Map " << this->id_map_  << " : " << keyframes_.size() << " KFs | " << landmarks_.size() << " LMs" << std::endl;
        // this->WriteKFsToFile();
        // this->WriteKFsToFileAllAg();
    }
}

// auto Map::RenderRGBMap_service->void{
    // while(1){
    //     // std::cout<< COUTDEBUG << "Add2RGBMap_servic " << std::endl;
    //     if(imgs_posed_should_be_added_to_rgb_map.size()>20){
    //         std::cout << COUTNOTICE << "RGBMap load too high" << std::endl;
    //     }
    //     if (imgs_posed_should_be_added_to_rgb_map.size() > 0) 
    //     {
    //         // q.front()
            
    //         m_map_rgb_pts.render_with_a_image(imgs_posed_should_be_added_to_rgb_map.front());
    //         // std::cout<< COUTDEBUG << "Added " << std::endl;
    //         imgs_posed_should_be_added_to_rgb_map.pop();
    //         /* code */
    //     }
        
    //     if (imgs_should_be_added_to_rgb_map.size() > 0) 
    //     {
    //         /* code */
    //     }
    //     std::this_thread::sleep_for(std::chrono::microseconds(10));        
    // }


// }

auto Map::Add2RGBMap_service()->void {
    // std::vector<idpair> = std::make_unique<
    // size_t pc_index = 0;
    // size_t img_index = 0;
// std::cout<< COUTDEBUG << "Add2RGBMap_servic " << std::endl;
    while(1){
        // std::cout<< COUTDEBUG << "Add2RGBMap_servic " << std::endl;
        if(pcs_should_be_added_to_rgb_map.size()>20){
            std::cout << COUTNOTICE << "RGBMap load too high" << std::endl;
        }
        if (pcs_should_be_added_to_rgb_map.size() > 0) 
        {
            // q.front()
            // std::cout<< COUTNOTICE << "consume a pc " << std::endl;
            Add2RGBMap(pcs_should_be_added_to_rgb_map.front());
            // std::cout<< COUTDEBUG << "Added " << std::endl;
            pcs_should_be_added_to_rgb_map.pop();
            /* code */
        }
        
        if (imgs_should_be_added_to_rgb_map.size() > 0) 
        {
            /* code */
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));        
    }

}

auto Map::Add2RGBMap(PointCloudEXPtr pc)->void {
    int m_append_global_map_point_step = 2; // 间隔一个点加入global map
    bool m_if_record_mvs =false;//if record_offline_map
    if ( m_if_record_mvs )
    {
        std::vector< std::shared_ptr< RGB_pts > > pts_last_hitted;
        pts_last_hitted.reserve( 1e6 );

        m_number_of_new_visited_voxel = m_map_rgb_pts->append_points_to_global_map( pc, &pts_last_hitted,m_append_global_map_point_step);

        m_map_rgb_pts->m_mutex_pts_last_visited->lock();
        m_map_rgb_pts->m_pts_last_hitted = pts_last_hitted;
        m_map_rgb_pts->m_mutex_pts_last_visited->unlock();

    }else{
        m_number_of_new_visited_voxel = m_map_rgb_pts->append_points_to_global_map( pc, nullptr,m_append_global_map_point_step);
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
auto Map::GetPoseMap()         ->PoseMap {
    // 
    // auto pcs = GetPointCloudEXs();
    std::unique_lock<std::mutex> lock(mtx_map_);
    // PoseMap
    for(std::map<idpair,PointCloudEXPtr>::iterator mit = pointclouds_.begin();mit!=pointclouds_.end();++mit){
        m_pose_map[mit->first] = mit->second->GetPoseTsg();
    }

    return m_pose_map;
}

auto Map::Display()->void {
    std::unique_lock<std::mutex> lock(mtx_map_);
    // keyframes_.size();
    // landmarks_
    std::cout<<"pointcloud frames num:"<<pointclouds_.size()<<std::endl;
}


}