#include "place_rec.hpp"
// #include "scancontext/Scancontext.h"

// C++
#include <iostream>
#include <mutex>
#include <eigen3/Eigen/Core>


namespace colive {
    
PlaceRecognition::PlaceRecognition(MapManagerPtr man, bool perform_pgo)
    : mapmanager_(man),
      perform_pgo_(perform_pgo),
    //   voc_(mapmanager_->GetVoc()),
      mnCovisibilityConsistencyTh(colive_params::placerec::cov_consistency_thres)
{
    double loopNoiseScore = 0.001; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );
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
    if( int(mapmanager_->cl_pcs.size()) < mapmanager_->scManager.NUM_EXCLUDE_RECENT) // do not try too early 
        return;

    auto detectResult = mapmanager_->scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    int SCclosestHistoryFrameID = detectResult.first;
    // std::cout << "try detectLoopClosure:" << SCclosestHistoryFrameID<<std::endl;
    if( SCclosestHistoryFrameID != -1 ) {
        std::cout << "detectLoopClosureID: " << SCclosestHistoryFrameID<<std::endl;
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = mapmanager_->cl_pcs.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        std::unique_lock<std::mutex> lock(mtx_mBuf_);
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mtx_mBuf_.unlock();
    }
}

auto PlaceRecognition::doICPVirtualRelative(int _loop_pc_idx, int _curr_pc_idx, TransformType &corrected_tf)->int
{
    // Check that the loopGap is correct
    // Pose6D pose_loop;
    // Pose6D pose_curr;
    // pose_loop = mapmanager_->cl_pcs[_loop_kf_idx].odom;//    keyframePosesUpdated[_loop_kf_idx];
    // pose_curr = mapmanager_->cl_pcs[_curr_kf_idx].odom;//    keyframePosesUpdated[_curr_kf_idx];
    // // double loopGapThreshold = 5.0;
    // double dx = pose_loop.x - pose_curr.x, dy = pose_loop.y - pose_curr.y;
    // double loopGap = std::sqrt(dx * dx + dy * dy);
    // if (loopGap > loopGapThreshold) {
    //   std::cout << "loopGap is too big " << std::endl;
    //   std::cout << "loopGap = " << loopGap << std::endl;
    //   std::cout << "pose_curr = " << pose_curr.x << " " << pose_curr.y << " " << pose_curr.z << std::endl;
    //   std::cout << "pose_loop = " << pose_loop.x << " " << pose_loop.y << " " << pose_loop.z << std::endl;
    //   return std::nullopt;
    // }
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
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return -1;// return std::nullopt;
    } else {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    corrected_tf = correctionLidarFrame.cast<double>().matrix();
    // pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    // corrected_pos[0]=x;
    // corrected_pos[1]=y;
    // corrected_pos[2]=z;
    // corrected_quan.coeffs() <<roll, pitch, yaw;
    // // corrected_quan[]=(roll, pitch, yaw);
    // corrected_tf = corrected_pos * corrected_quan;
    // gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
    return 0;
} // doICPVirtualRelative

auto PlaceRecognition::process_icp()->void
{
    // while(1)
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                ROS_WARN("Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mtx_mBuf_.lock(); 
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mtx_mBuf_.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            TransformType tf;
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx, tf);
            if(relative_pose_optional==0) {
                Eigen::Vector3d position = tf.translation();
                Eigen::Vector3d euler = tf.rotation().eulerAngles(0, 1, 2);

                // gtsam::Pose3 relative_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]), gtsam::Point3(position[0], position[1], position[2]));
                // mtx_Posegraph_.lock();
                // gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                // std::cout << "do icp success" << relative_pose << std::endl;
                // runISAM2opt();
                // mtx_Posegraph_.unlock();
            }else{
                continue;
            }
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

auto PlaceRecognition::CheckBuffer()->bool {
    std::unique_lock<std::mutex> lock(mtx_in_);
    return (!buffer_pcs_in_.empty());
}
auto PlaceRecognition::ComputeSE3() -> bool {

    // const size_t nInitialCandidates = mvpEnoughConsistentCandidates.size();

    return true;
}
auto PlaceRecognition::ConnectLoop(PointCloudEXPtr pc_query, PointCloudEXPtr pc_match, TransformType T_squery_smatch, PoseMap &corrected_poses, MapPtr map)->void {

    // TransformType T_w_squery = kf_query->GetPoseTws();
    // TransformType T_w_smatch = kf_match->GetPoseTws();
    // TransformType T_w_sqcorr = T_w_smatch*T_squery_smatch;

    // for(auto kfi : mvpCurrentConnectedKFs) {
    //     if(kfi == kf_query) continue;
    //     TransformType T_wq_si = kfi->GetPoseTws();
    //     TransformType T_sq_si = T_w_squery.inverse() * T_wq_si;
    //     TransformType T_w_sicorr = T_w_sqcorr * T_sq_si;
    //     corrected_poses[kfi->id_] = T_w_sicorr;
    // }

    // corrected_poses[kf_query->id_] = T_w_sqcorr;
}
auto PlaceRecognition::CorrectLoop()->bool {


    while ( !scLoopICPBuf.empty() )
    {
        std::cout << "\033[1;32m+++ PLACE RECOGNITION FOUND +++\033[0m" << std::endl;
        if( scLoopICPBuf.size() > 30 ) {
            ROS_WARN("Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
        }

        mtx_mBuf_.lock(); 
        std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
        scLoopICPBuf.pop();
        mtx_mBuf_.unlock(); 

        const int prev_node_idx = loop_idx_pair.first;
        const int curr_node_idx = loop_idx_pair.second;
        
        // for(auto lc : map_query->GetLoopConstraints()) {
        //     bool existing_match = false;
        //     if(lc.kf1 == kf_match_ && lc.kf2 == kf_query_) existing_match = true;
        //     if(lc.kf2 == kf_match_ && lc.kf1 == kf_query_) existing_match = true;
        //     if(!existing_match) continue;
        //     std::cout << "!!! Loop Constraint already exisiting -- skip !!!" << std::endl;
        //     kf_query_->SetErase();
        //     kf_match_->SetErase();
        //     mapmanager_->ReturnMap(kf_query_->id_.second,check_num_map);
        //     return false;
        // }
        TransformType T_curr_loop;// 当前转到过去回环的  T_curr_loop
        auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx, T_curr_loop);

        if(relative_pose_optional==0) {
            int check_num_map;
            MapPtr map_query = mapmanager_->CheckoutMapExclusiveOrWait(pc_query_->id_.second,check_num_map);

        // std::cout << COUTDEBUG << "CheckoutMapExclusiveOrWait OK " << std::endl;
            // T_squery_smatch = tf;
            // std::cout << "do icp success0" << std::endl;
            auto loop_pc = mapmanager_->cl_pcs[prev_node_idx];
            auto curr_pc = mapmanager_->cl_pcs[curr_node_idx];

            // std::cout << "do icp success0.1:"<<loop_pc->id_.first<<"sdada:"<<curr_pc->id_.first << std::endl;
            // curr_pc->map_.GetPointCloudEX(loop_pc->id_)
            if(map_query->GetPointCloudEX(loop_pc->id_)){
                bool perform_pgo_ =true;
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
            mapmanager_->ReturnMap(pc_query_->id_.second,check_num_map);
        }else{
            continue;
        }
        
    }
    // if( gtSAMgraphMade ) {
    //         mtxPosegraph.lock();
    //         runISAM2opt();
    //         cout << "running isam2 optimization ..." << endl;
    //         mtxPosegraph.unlock();
    //     }
    // last_loops_[kf_query_->id_.second] = kf_query_->id_.first;
    // TransformType T_squery_smatch = TransformType::Identity();
    
    // T_squery_smatch = kf_match_->GetPoseTsw() * mTsw.inverse();
    
    // int check_num_map;
    // MapPtr map_query = mapmanager_->CheckoutMapExclusiveOrWait(kf_query_->id_.second,check_num_map);

    // for(auto lc : map_query->GetLoopConstraints()) {
    //     bool existing_match = false;
    //     if(lc.kf1 == kf_match_ && lc.kf2 == kf_query_) existing_match = true;
    //     if(lc.kf2 == kf_match_ && lc.kf1 == kf_query_) existing_match = true;
    //     if(!existing_match) continue;
    //     std::cout << "!!! Loop Constraint already exisiting -- skip !!!" << std::endl;
    //     kf_query_->SetErase();
    //     kf_match_->SetErase();
    //     mapmanager_->ReturnMap(kf_query_->id_.second,check_num_map);
    //     return false;
    // }

    // // Ensure current keyframe is updated
    // map_query->UpdateCovisibilityConnections(kf_query_->id_);

    // mvpCurrentConnectedKFs.clear();
    // mvpCurrentConnectedKFs.push_back(kf_query_);

    // PoseMap corrected_poses;
    // this->ConnectLoop(kf_query_,kf_match_,T_squery_smatch,corrected_poses,map_query);



    // mapmanager_->ReturnMap(kf_query_->id_.second,check_num_map);

    return true;
}

auto PlaceRecognition::DetectLoop()->bool {

    // if(pc_query_->id_.first < colive_params::placerec::start_after_kf) {
    //     // pc_query_->SetErase();
    //     std::cout<<"------------eraly loop ----------------"<<std::endl;
    //     return false;
    // }
    performSCLoopClosure();
    // if(pc_query_->id_.first < covins_params::placerec::start_after_kf) {
    //     pc_query_->SetErase();
    //     return false;
    // }

    // if(last_loops_.count(kf_query_->id_.second)) {
    //     if((pc_query_->id_.first - last_loops_[pc_query_->id_.second]) < covins_params::placerec::consecutive_loop_dist) {
    //         pc_query_->SetErase();
    //         return false;
    //     }
    // }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    // const auto vpConnectedKeyFrames =
    //     pc_query_->GetConnectedNeighborKeyframes();
        
    // const DBoW2::BowVector &CurrentBowVec = kf_query_->bow_vec_;
    // float minScore = 1;
    // // std::cout << "Number of neighbors: " << vpConnectedKeyFrames.size() << std::endl;
    // for (size_t i = 0; i < vpConnectedKeyFrames.size(); i++) {
    //         KeyframePtr pKF = vpConnectedKeyFrames[i];
    //         if(pKF->IsInvalid()) continue;

    //         const DBoW2::BowVector &BowVec = pKF->bow_vec_;
    //         float score = voc_->score(CurrentBowVec, BowVec);
    //     if(score<minScore) {
    //         minScore = score;
    //     }
    // }

    // // Query the database imposing the minimum score
    // auto database = mapmanager_->GetDatabase();
    // KeyframeVector vpCandidateKFs = database->DetectCandidates(kf_query_, minScore*0.7);

    // // If there are no loop candidates, just add new keyframe and return false
    // if (vpCandidateKFs.empty()) {
    //     mvConsistentGroups.clear(); //Danger: Why deleting the found consistent groups in this case?
    //     kf_query_->SetErase();
    //     return false;
    // }

    // // For each loop candidate check consistency with previous loop candidates
    // // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // // A group is consistent with a previous group if they share at least a keyframe
    // // We must detect a consistent loop in several consecutive keyframes to accept it
    // mvpEnoughConsistentCandidates.clear();

    // vecConsistentGroup vCurrentConsistentGroups; //pair <set<KF*>,int> --> int counts consistent groups found for this group
    // vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    // //mvConsistentGroups stores the last found consistent groups.

    // for (size_t i = 0; i < vpCandidateKFs.size(); ++i) {
    //     KeyframePtr pCandidateKF = vpCandidateKFs[i];

    //     auto candidate_connections = pCandidateKF->GetConnectedNeighborKeyframes();
    //     KeyframeSet spCandidateGroup(candidate_connections.begin(),candidate_connections.end());
    //     spCandidateGroup.insert(pCandidateKF);
    //     //group with candidate and connected KFs

    //     bool bEnoughConsistent = false;
    //     bool bConsistentForSomeGroup = false;
    //     for(size_t iG = 0; iG < mvConsistentGroups.size(); ++iG) {
    //         auto sPreviousGroup = mvConsistentGroups[iG].first;

    //         bool bConsistent = false;
    //         for(KeyframeSet::iterator sit = spCandidateGroup.begin(); sit != spCandidateGroup.end(); ++sit) {
    //             if (sPreviousGroup.count(*sit)) {
    //                 //KF found that is contained in candidate's group and comparison group
    //                 bConsistent=true;
    //                 bConsistentForSomeGroup=true;
    //                 break;
    //             }
    //         }
    //         if (bConsistent) {
    //             int nPreviousConsistency = mvConsistentGroups[iG].second;
    //             int nCurrentConsistency = nPreviousConsistency + 1;
    //             if(!vbConsistentGroup[iG]) {
    //                 ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
    //                 vCurrentConsistentGroups.push_back(cg);
    //                 vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
    //             }
    //             if(nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent) {
    //                 mvpEnoughConsistentCandidates.push_back(pCandidateKF);
    //                 bEnoughConsistent=true; //this avoid to insert the same candidate more than once
    //             }
    //         }
    //     }

    //     // If the group is not consistent with any previous group insert with consistency counter set to zero
    //     if (!bConsistentForSomeGroup) {
    //         ConsistentGroup cg = make_pair(spCandidateGroup,0); //For "ConsistentGroup" the "int" is initialized with 0
    //         vCurrentConsistentGroups.push_back(cg);
    //     }
    // }

    // // Update Covisibility Consistent Groups
    // mvConsistentGroups = vCurrentConsistentGroups;

    // if (mvpEnoughConsistentCandidates.empty()) {
    //     kf_query_->SetErase();
    //     return false;
    // } else {
    //     return true;
    // }
    // kf_query_->SetErase();
    return true;
}
auto PlaceRecognition::InsertKeyframe(PointCloudEXPtr pc)->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    buffer_pcs_in_.push_back(pc);   
}

auto PlaceRecognition::Run()->void {

    int num_runs = 0;
    int num_detected = 0;

    while(1){
        if (CheckBuffer()) {
            num_runs++;
            {
                std::unique_lock<std::mutex> lock(mtx_in_);
                pc_query_ = buffer_pcs_in_.front();
                buffer_pcs_in_.pop_front();
                // pc_query_->SetNotErase();
            }

            // scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);
            mapmanager_->AddToDatabase(pc_query_);
            bool detected = DetectLoop();
            // std::cout<<"add query"<<std::endl;
            if(detected) {
                num_detected++;
                this->CorrectLoop();    
            }
            
            
        }

        if(this->ShallFinish()){
            std::cout << "PlaceRec " << ": close" << std::endl;
            break;
        }

        usleep(1000);
    }

    std::unique_lock<std::mutex> lock(mtx_finish_);
    // is_finished_ = true;
}

}
