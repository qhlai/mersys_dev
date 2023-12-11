#include "place_rec.hpp"
// #include "scancontext/Scancontext.h"

// C++
#include <iostream>
#include <mutex>
#include <eigen3/Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


#include "pointcloud_ex.hpp"
#include "image_ex.hpp"
// #include "livox_cam_calib.hpp"

namespace mersys {
    
PlaceRecognition::PlaceRecognition(MapManagerPtr man, bool perform_pgo)
    : mapmanager_(man),
      perform_pgo_(perform_pgo),
    //   voc_(mapmanager_->GetVoc()),
      mnCovisibilityConsistencyTh(mersys_params::placerec::cov_consistency_thres)
{
    double loopNoiseScore = 0.001; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    // last_loop_frame_id.reserve(MAX_CLIENT_NUM);
}
auto PlaceRecognition::process_lcd()->void {
    float loopClosureFrequency = 1.0; // can change 
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok())
    {
        rate.sleep();
        performSCLoopClosure();
        // performRSLoopClosure(); // TODO
    }
}

auto PlaceRecognition::performSCLoopClosure()->void {
}

auto PlaceRecognition::doICPVirtualRelative(int _loop_pc_idx, int _curr_pc_idx, TransformType &corrected_tf)->int
{
    
    auto loop_pc = mapmanager_->cl_pcs[_loop_pc_idx];
    auto curr_pc = mapmanager_->cl_pcs[_curr_pc_idx];
    // parse pointclouds
    int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr currKeyframeCloud(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr currKeyframeCloudViz(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr icpKeyframeCloudViz(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr targetKeyframeCloudViz(new pcl::PointCloud<PointType>());
    // loopFindNearKeyframesCloud(currKeyframeCloud, currKeyframeCloudViz, _curr_kf_idx, 0); // use same root of loop kf idx 
    // loopFindNearKeyframesCloud(targetKeyframeCloud, targetKeyframeCloudViz, _loop_kf_idx, historyKeyframeSearchNum); 
    *currKeyframeCloud = curr_pc->pts_cloud_d;
    *targetKeyframeCloud = loop_pc->pts_cloud_d;
    int currKeyframeid = curr_pc->id_.first;
    int targetKeyframeid = loop_pc->id_.first;

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // // cal the the initial position transform
    TransformType T_init = TransformType::Identity();
    // TransformType T_relative;
    TransformType T_loop = loop_pc->GetPoseTsw();
    TransformType T_curr = curr_pc->GetPoseTsw();
    T_init=T_loop.inverse()*T_curr;
    // T_relative = T_loop.inverseTimes(T_curr);
    // tf::transformTFToEigen (T_relative, T_init);

    // // Align pointclouds
    icp.setInputSource(currKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result, T_init.matrix().cast<float>());

    float loopFitnessScoreThreshold = 0.3; // user parameter but fixed low value is safe. 
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << COUTDEBUG << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return -1;// return std::nullopt;
    } else {
        std::cout << COUTDEBUG << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    corrected_tf = correctionLidarFrame.cast<double>().matrix();
    return 0;
} // doICPVirtualRelative

auto PlaceRecognition::process_icp()->void
{
    while(1)
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 10 ) {
                std::cout << COUTWARN <<"Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)"<<std::endl;
            }

            mtx_mBuf_.lock(); 
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mtx_mBuf_.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            
            auto loop_pc = mapmanager_->cl_pcs[prev_node_idx];
            auto curr_pc = mapmanager_->cl_pcs[curr_node_idx];



            // 回环限制
            // 发生回环的两个设备 // curr, loop
            idpair client_idpair;

            client_idpair.first = curr_pc->GetClientID();
            client_idpair.second = loop_pc->GetClientID(); 
            // if(last_loops_.count(client_idpair)) {
            
            //     if((curr_pc->GetFrameID() - last_loops_[client_idpair]) < mersys_params::placerec::consecutive_loop_dist) {
            //         std::cout << COUTNOTICE << "loop too offten, ignored" << std::endl;
            //         curr_pc->SetErase();
            //         return;
            //         // return;
            //     }
            // }
            int check_num_map;
            MapPtr map_query = mapmanager_->CheckoutMapExclusiveOrWait(curr_pc->GetClientID(),check_num_map);
            
            // std::cout << COUTDEBUG << "CheckoutMapExclusiveOrWait "<< curr_pc->GetClientID()<<check_num_map<< std::endl;
            
            // 检查是否已经有回环关系
            bool existing_match = false;
            auto lcs = map_query->GetLoopConstraints();
            for(int i = 0; i < lcs.size() && !existing_match;i++) {
                auto lc=lcs[i];
                if(lc.type==0){
                    if(lc.pc1 == loop_pc && lc.pc2 == curr_pc) existing_match = true;
                    if(lc.pc2 == loop_pc && lc.pc1 == curr_pc) existing_match = true;
                }

                if(!existing_match) continue; // 如果不重复，继续检查下一个
                std::cout << "!!! Loop Constraint already exisiting -- skip !!!" << std::endl;
                existing_match=true;
                curr_pc->SetErase();
                pc_match_->SetErase();
                mapmanager_->ReturnMap(curr_pc->GetClientID(),check_num_map);// 这个不能删

                // continue;
            }
            // 若已经地图融合过了，则退出此次循环
            if(existing_match){
                continue;
            }
            // LoopConstraint lc(kf_match,kf_query,T_smatch_squery);
            // loop_constraints_.push_back(lc);
            TransformType T_curr_loop;// 当前转到过去回环的  T_curr_loop
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx, T_curr_loop);
        
            if(relative_pose_optional==0) {
                // 记录历史回环帧
                std::cout << "\033[1;32m+++ PLACE RECOGNITION FOUND +++\033[0m" << std::endl;
                std::cout<<COUTDEBUG<< "T:"<<std::endl<<T_curr_loop.matrix()<<std::endl;
                // 回环限制更新
                idpair client_idpair;
                client_idpair.first = curr_pc->GetClientID();
                client_idpair.second = loop_pc->GetClientID();
                last_loops_[client_idpair] = curr_pc->GetFrameID();

                if(map_query->GetPointCloudEX(loop_pc->id_)){
                    std::cout << COUTNOTICE<< "+++ Single Close Loop FOUND +++" << std::endl;
                    bool perform_pgo_ =false;

                    // std::cout << "do icp success0.2" << std::endl;
                    if(perform_pgo_){
                        Eigen::Vector3d position = T_curr_loop.translation();
                        Eigen::Vector3d euler = T_curr_loop.rotation().eulerAngles(0, 1, 2);

                        gtsam::Pose3 relative_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]), gtsam::Point3(position[0], position[1], position[2]));
                        // std::cout << "do icp success1" << relative_pose << std::endl;
                        // std::cout << "do icp success2" << T_curr_loop.matrix() << std::endl;
                        mtx_Posegraph_.lock();
                        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                        mtx_Posegraph_.unlock();
                    } else {
                        std::cout << COUTNOTICE << "!!! PGO deativated !!!" << std::endl;
                    }
                }else{
                        std::cout << COUTNOTICE<< "+++ FUSION FOUND +++" << std::endl;
                        // std::cout << "\033[1;32m+++ FUSION FOUND +++\033[0m" << std::endl;
                        MergeInformation merge;
                        merge.pc_query = curr_pc;
                        merge.pc_match = loop_pc;
                        merge.T_squery_smatch = T_curr_loop;
                        // merge.cov_mat = mcov_mat;
                        mapmanager_->RegisterMerge(merge);
                }
                // 记录回环边
                LoopConstraint lc(curr_pc,loop_pc, T_curr_loop);
                map_query->AddLoopConstraint(lc); // have fix crash here
            }
            // std::cout << COUTDEBUG << "ReturnMap "<< curr_pc->GetClientID()<<check_num_map<< std::endl;
            mapmanager_->ReturnMap(curr_pc->GetClientID(),check_num_map);
        
        }

        // wait (must required for running the while loop)
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }
} // process_icp

auto PlaceRecognition::CheckBuffer_pc()->bool {
    std::unique_lock<std::mutex> lock(mtx_in_);
    return (!buffer_pcs_in_.empty());
}
auto PlaceRecognition::CheckBuffer_img()->bool {
    std::unique_lock<std::mutex> lock(mtx_in_);
    return (!buffer_imgs_in_.empty());
}
auto PlaceRecognition::CheckBuffer_pcl()->bool {
    std::unique_lock<std::mutex> lock(mtx_in_);
    return (!buffer_pcs_large_in_.empty());
}
auto PlaceRecognition::ComputeSE3() -> bool {

    // const size_t nInitialCandidates = mvpEnoughConsistentCandidates.size();

    return true;
}
auto PlaceRecognition::ConnectLoop(PointCloudEXPtr pc_query, PointCloudEXPtr pc_match, TransformType T_squery_smatch, PoseMap &corrected_poses, MapPtr map)->void {


}
auto PlaceRecognition::CorrectLoop()->bool {

    bool if_detect = false;

    // while ( !scLoopICPBuf.empty() )
    // {
    //     // std::cout << "\033[1;32m+++ PLACE RECOGNITION FOUND +++\033[0m" << std::endl;

    //     if( scLoopICPBuf.size() > 30 ) {
    //         std::cout << COUTWARN <<"Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)"<<std::endl;
    //     }

    //     mtx_mBuf_.lock(); 
    //     std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
    //     scLoopICPBuf.pop();
    //     mtx_mBuf_.unlock(); 

    //     const int prev_node_idx = loop_idx_pair.first;
    //     const int curr_node_idx = loop_idx_pair.second;

    //     auto loop_pc = mapmanager_->cl_pcs[prev_node_idx];
    //     auto curr_pc = mapmanager_->cl_pcs[curr_node_idx];


    //     int check_num_map;
    //     MapPtr map_query = mapmanager_->CheckoutMapExclusiveOrWait(curr_pc->GetClientID(),check_num_map);
        
    //     // 检查是否已经有回环关系
    //     for(auto lc : map_query->GetLoopConstraints()) {
    //         bool existing_match = false;
    //         if(lc.type==0){
    //             if(lc.pc1 == loop_pc && lc.pc2 == curr_pc) existing_match = true;
    //             if(lc.pc2 == loop_pc && lc.pc1 == curr_pc) existing_match = true;
    //         }

    //         if(!existing_match) continue;
    //         std::cout << "!!! Loop Constraint already exisiting -- skip !!!" << std::endl;
    //         curr_pc->SetErase();
    //         pc_match_->SetErase();
    //         mapmanager_->ReturnMap(curr_pc->GetClientID(),check_num_map);
    //         return false;
    //     }
    //     // LoopConstraint lc(kf_match,kf_query,T_smatch_squery);
    //     // loop_constraints_.push_back(lc);
    //     TransformType T_curr_loop;// 当前转到过去回环的  T_curr_loop
    //     auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx, T_curr_loop);
    //     if(relative_pose_optional==0) {
    //         // 记录历史回环帧
    //         std::cout << "\033[1;32m+++ PLACE RECOGNITION FOUND +++\033[0m" << std::endl;

    //         last_loops_[curr_pc->GetClientID()] = curr_pc->GetFrameID();

    //         if_detect = true;
    //         // std::cout << COUTDEBUG << "CheckoutMapExclusiveOrWait OK " << std::endl;
    //         // T_squery_smatch = tf;
    //         // std::cout << "do icp success0" << std::endl;
    //         // auto loop_pc = mapmanager_->cl_pcs[prev_node_idx];
    //         // auto curr_pc = mapmanager_->cl_pcs[curr_node_idx];

    //         // std::cout << "do icp success0.1:"<<loop_pc->id_.first<<"sdada:"<<curr_pc->id_.first << std::endl;
    //         // curr_pc->map_.GetPointCloudEX(loop_pc->id_)
    //         if(map_query->GetPointCloudEX(loop_pc->id_)){
    //             bool perform_pgo_ =true;
    //             LoopConstraint lc(curr_pc,loop_pc, T_curr_loop);
    //             map_query->AddLoopConstraint(lc); // have fix crash here
    //             // std::cout << "do icp success0.2" << std::endl;
    //             if(perform_pgo_){
    //                 Eigen::Vector3d position = T_curr_loop.translation();
    //                 Eigen::Vector3d euler = T_curr_loop.rotation().eulerAngles(0, 1, 2);

    //                 gtsam::Pose3 relative_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]), gtsam::Point3(position[0], position[1], position[2]));
    //                 // std::cout << "do icp success1" << relative_pose << std::endl;
    //                 // std::cout << "do icp success2" << T_curr_loop.matrix() << std::endl;
    //                 mtx_Posegraph_.lock();
    //                 gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
    //                 mtx_Posegraph_.unlock();
    //             } else {
    //                 std::cout << COUTNOTICE << "!!! PGO deativated !!!" << std::endl;
    //             }
    //         }else{
    //                 std::cout << COUTNOTICE<< "+++ FUSION FOUND +++" << std::endl;
    //                 // std::cout << "\033[1;32m+++ FUSION FOUND +++\033[0m" << std::endl;
    //                 MergeInformation merge;
    //                 merge.pc_query = curr_pc;
    //                 merge.pc_match = loop_pc;
    //                 merge.T_squery_smatch = T_curr_loop;
    //                 // merge.cov_mat = mcov_mat;
    //                 mapmanager_->RegisterMerge(merge);
    //         }

    //     }else{
    //         mapmanager_->ReturnMap(curr_pc->id_.second,check_num_map);
    //         continue;
    //     }
    //     mapmanager_->ReturnMap(curr_pc->id_.second,check_num_map);
    // }
    
    return if_detect;
}

auto PlaceRecognition::DetectLoop()->bool {

    // performSCLoopClosure();
    {
        std::unique_lock<std::mutex> lock(mtx_in_);
        pc_query_ = buffer_pcs_in_.front();
        buffer_pcs_in_.pop_front();
        // pc_query_->SetNotErase();
    }
    mapmanager_->AddToDatabase(pc_query_);

    // if( int(mapmanager_->cl_pcs.size()) < mapmanager_->scManager.NUM_EXCLUDE_RECENT) // do not try too early 
    //     return false;
    if(pc_query_->GetFrameID() < mersys_params::placerec::start_after_kf || int(mapmanager_->cl_pcs.size()) < mapmanager_->scManager.NUM_EXCLUDE_RECENT) {
        pc_query_->SetErase();
        std::cout<< COUTNOTICE <<"eraly loop"<<std::endl;
        return false;
    }
   std::pair<int, float> detectResult;
    {
        std::unique_lock<std::mutex> lock_database(mapmanager_->mtx_database_);
        detectResult = mapmanager_->scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    }
    int SCclosestHistoryFrameID = detectResult.first;
    // std::cout << "try detectLoopClosure:" << SCclosestHistoryFrameID<<std::endl;
    if( SCclosestHistoryFrameID != -1 ) {
        
        std::cout << "detectLoopClosureID: " << SCclosestHistoryFrameID<<std::endl;
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = mapmanager_->cl_pcs.size() - 1; // because cpp starts 0 and ends n-1



        auto loop_pc = mapmanager_->cl_pcs[prev_node_idx];
        auto curr_pc = mapmanager_->cl_pcs[curr_node_idx];
        // 回环限制
        // 发生回环的两个设备 // curr, loop
        idpair client_idpair;

        client_idpair.first = curr_pc->GetClientID();
        client_idpair.second = loop_pc->GetClientID();     
        // if(curr_pc->GetFrameID()  <= loop_pc->GetFrameID()){
        //     client_idpair.first = curr_pc->GetFrameID();
        //     client_idpair.second = loop_pc->GetFrameID();
        // }else{
        //     client_idpair.first = loop_pc->GetFrameID();
        //     client_idpair.second = curr_pc->GetFrameID();
        // }
        std::cout << COUTDEBUG << "debug loop:"<<curr_pc->GetClientID() <<":"<<curr_pc->GetFrameID()<< ", lastloop:" << client_idpair<<":" << last_loops_[client_idpair]<< std::endl;

        // if(last_loops_.count(client_idpair)) {
            
        //     if((curr_pc->GetFrameID() - last_loops_[client_idpair]) < mersys_params::placerec::consecutive_loop_dist) {
        //         std::cout << COUTNOTICE << "loop too offten, ignored" << std::endl;
        //         curr_pc->SetErase();
        //         return false;
        //         // return;
        //     }
        // }

        std::cout << COUTNOTICE << "SC Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        std::unique_lock<std::mutex> lock(mtx_mBuf_);
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mtx_mBuf_.unlock();
        return true;
    }

    return false;
}
auto PlaceRecognition::DetectLoop_C()->bool {

    performSCLoopClosure();
    {
        std::unique_lock<std::mutex> lock(mtx_in_);
        img_query_ = buffer_imgs_in_.front();
        rgb_edge_cloud_ = calibration_.add_img(img_query_, true);
        buffer_imgs_in_.pop_front();

        pc_query_ = buffer_pcs_large_in_.front();
        lidar_edge_cloud_ =calibration_.add_lidar(pc_query_);
        buffer_pcs_large_in_.pop_front();
        
        //first init 
        camera_.update_Rt(Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero());
        calibration_.roughCalib(camera_, lidar_edge_cloud_, rgb_edge_cloud_, DEG2RAD(0.1), 30);
        // img_query_->SetNotErase();
    }
    
    Eigen::Vector3d euler_angle = camera_.ext_R.eulerAngles(2, 1, 0);
    Eigen::Vector3d transation = camera_.ext_t;
    Vector6d calib_params;
    calib_params << euler_angle(0), euler_angle(1), euler_angle(2), transation(0), transation(1), transation(2);
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    R = camera_.ext_R;
    T = camera_.ext_t;
    // // outfile.open(result_file, ofstream::app);
    // // for(int i = 0; i < 3; i++)
    // //     outfile << R(i, 0) << "," << R(i, 1) << "," << R(i, 2) << "," << T[i] << "\n";
    // // outfile << 0 << "," << 0 << "," << 0 << "," << 1 << "\n";
    // // outfile.close();

     /* visualize the colorized point cloud */
    calib_params << euler_angle(0), euler_angle(1), euler_angle(2),
                    transation(0), transation(1), transation(2);
    calibration_.colorCloud(calib_params, 1, camera_, img_query_->img_, pc_query_);

    // auto calib = mapmanager_->calibration.Calibration()
    
    // mapmanager_->AddToDatabase(pc_query_);

    // // if( int(mapmanager_->cl_pcs.size()) < mapmanager_->scManager.NUM_EXCLUDE_RECENT) // do not try too early 
    // //     return false;
    // if(img_query_->GetFrameID() < mersys_params::placerec::start_after_kf || int(mapmanager_->cl_pcs.size()) < mapmanager_->scManager.NUM_EXCLUDE_RECENT) {
    //     pc_query_->SetErase();
    //     std::cout<< COUTNOTICE <<"eraly loop"<<std::endl;
    //     return false;
    // }

    // auto detectResult = mapmanager_->scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    // int SCclosestHistoryFrameID = detectResult.first;
    // // std::cout << "try detectLoopClosure:" << SCclosestHistoryFrameID<<std::endl;
    // if( SCclosestHistoryFrameID != -1 ) {
        
    //     std::cout << "detectLoopClosureID: " << SCclosestHistoryFrameID<<std::endl;
    //     const int prev_node_idx = SCclosestHistoryFrameID;
    //     const int curr_node_idx = mapmanager_->cl_pcs.size() - 1; // because cpp starts 0 and ends n-1



    //     auto loop_pc = mapmanager_->cl_pcs[prev_node_idx];
    //     auto curr_pc = mapmanager_->cl_pcs[curr_node_idx];
    //     // 回环限制
    //     // 发生回环的两个设备 // curr, loop
    //     idpair client_idpair;

    //     client_idpair.first = curr_pc->GetClientID();
    //     client_idpair.second = loop_pc->GetClientID();     
    //     // if(curr_pc->GetFrameID()  <= loop_pc->GetFrameID()){
    //     //     client_idpair.first = curr_pc->GetFrameID();
    //     //     client_idpair.second = loop_pc->GetFrameID();
    //     // }else{
    //     //     client_idpair.first = loop_pc->GetFrameID();
    //     //     client_idpair.second = curr_pc->GetFrameID();
    //     // }
    //     std::cout << COUTDEBUG << "debug loop:"<<curr_pc->GetClientID() <<":"<<curr_pc->GetFrameID()<< ", lastloop:" << client_idpair<<":" << last_loops_[client_idpair]<< std::endl;

    //     if(last_loops_.count(client_idpair)) {
            
    //         if((curr_pc->GetFrameID() - last_loops_[client_idpair]) < mersys_params::placerec::consecutive_loop_dist) {
    //             std::cout << COUTNOTICE << "loop too offten, ignored" << std::endl;
    //             curr_pc->SetErase();
    //             return false;
    //             // return;
    //         }
    //     }

    //     std::cout << COUTNOTICE << "SC Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

    //     std::unique_lock<std::mutex> lock(mtx_mBuf_);
    //     scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
    //     // addding actual 6D constraints in the other thread, icp_calculation.
    //     mtx_mBuf_.unlock();
    //     return true;
    // }

    // return false;
}
auto PlaceRecognition::InsertKeyframe(PointCloudEXPtr pc)->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    buffer_pcs_in_.push_back(pc);   
}
auto PlaceRecognition::InsertLargeKeyframe(PointCloudEXPtr pc_large)->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    buffer_pcs_large_in_.push_back(pc_large);   
}
auto PlaceRecognition::InsertKeyframe1(ImageEXPtr img)->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    buffer_imgs_in_.push_back(img);   
}

auto PlaceRecognition::Run()->void {

    int num_runs = 0;
    int num_detected = 0;

    m_thread_pool_ptr->commit_task(&PlaceRecognition::process_icp,this);
    while(1){
        // std::cout<<"add query"<<std::endl;
        if (CheckBuffer_pc()) {
            // if (CheckBuffer_img()) {
            // num_runs++;
            // bool detected = DetectLoop_C(); 
            // }
            num_runs++;
            bool detected = DetectLoop(); 
        }

        if (CheckBuffer_img()) {
            if (CheckBuffer_pcl()){
                num_runs++;
                bool detected = DetectLoop_C(); 
            }
            
        }

        if(this->ShallFinish()){
            std::cout << "PlaceRec " << ": close" << std::endl;
            break;
        }
        // std::chrono::milliseconds dura(1000);
        // std::this_thread::sleep_for(dura);
        std::this_thread::sleep_for(std::chrono::microseconds(1)); 
        // usleep(1000);
    }

    std::unique_lock<std::mutex> lock(mtx_finish_);
    // is_finished_ = true;
}

}
