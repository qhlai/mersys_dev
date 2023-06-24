#include "place_rec.hpp"
#include "scancontext/Scancontext.h"

// C++
#include <iostream>
#include <mutex>
#include <eigen3/Eigen/Core>


namespace colive {
    

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
    if( int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT) // do not try too early 
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) { 
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        std::unique_lock<std::mutex> lock(mtx_mBuf_);
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mtx_mBuf_.unlock();
    }
}

PlaceRecognition::PlaceRecognition(MapManagerPtr man, bool perform_pgo)
    : mapmanager_(man),
      perform_pgo_(perform_pgo),
    //   voc_(mapmanager_->GetVoc()),
      mnCovisibilityConsistencyTh(colive_params::placerec::cov_consistency_thres)
{
  //...
}
auto PlaceRecognition::CheckBuffer()->bool {
    std::unique_lock<std::mutex> lock(mtx_in_);
    return (!buffer_pcs_in_.empty());
}
auto PlaceRecognition::ComputeSE3() -> bool {

    // const size_t nInitialCandidates = mvpEnoughConsistentCandidates.size();

    return true;
}
auto PlaceRecognition::ConnectLoop(PointCloudEXPtr pc_query, PointCloudEXPtr pc_match, TransformType T_smatch_squery, PoseMap &corrected_poses, MapPtr map)->void {

    // TransformType T_w_squery = kf_query->GetPoseTws();
    // TransformType T_w_smatch = kf_match->GetPoseTws();
    // TransformType T_w_sqcorr = T_w_smatch*T_smatch_squery;

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
    std::cout << "\033[1;32m+++ PLACE RECOGNITION FOUND +++\033[0m" << std::endl;

    // last_loops_[kf_query_->id_.second] = kf_query_->id_.first;
    // TransformType T_smatch_squery = TransformType::Identity();
    
    // T_smatch_squery = kf_match_->GetPoseTsw() * mTsw.inverse();
    
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
    // this->ConnectLoop(kf_query_,kf_match_,T_smatch_squery,corrected_poses,map_query);

    // if (map_query->GetKeyframe(kf_match_->id_)) {

    //     LoopConstraint lc(kf_match_, kf_query_, T_smatch_squery,
    //                       mcov_mat);
    //     map_query->AddLoopConstraint(lc);
      
    //     if(perform_pgo_)
    //     {
    //         KeyframeVector current_connections_query = kf_query_->GetConnectedKeyframesByWeight(0);
            
    //         for(auto kfi : current_connections_query) {
    //             map_query->UpdateCovisibilityConnections(kfi->id_);
    //         }

    //         Optimization::PoseGraphOptimization(map_query, corrected_poses);
    //         map_query->WriteKFsToFileAllAg();
    //     } else {
    //         std::cout << COUTNOTICE << "!!! PGO deativated !!!" << std::endl;
    //     }
    // } else {
    //     std::cout << "\033[1;32m+++ FUSION FOUND +++\033[0m" << std::endl;
    //     MergeInformation merge;
    //     merge.kf_query = kf_query_;
    //     merge.kf_match = kf_match_;
    //     merge.T_smatch_squery = T_smatch_squery;
    //     merge.cov_mat = mcov_mat;
    //     mapmanager_->RegisterMerge(merge);
    // }

    // mapmanager_->ReturnMap(kf_query_->id_.second,check_num_map);

    return true;
}

auto PlaceRecognition::DetectLoop()->bool {
    // {
    //     std::unique_lock<std::mutex> lock(mtx_in_);
    //     kf_query_ = buffer_pcs_in_.front();
    //     buffer_pcs_in_.pop_front();
    //     pc_query_->SetNotErase();
    // }

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
    return false;
}
auto PlaceRecognition::InsertKeyframe(PointCloudEXPtr pc)->void {
    // std::unique_lock<std::mutex> lock(mtx_in_);
    // buffer_pcs_in_.push_back(pc);
}

auto PlaceRecognition::Run()->void {

    int num_runs = 0;
    int num_detected = 0;

    // while(1){
    //     if (CheckBuffer()) {
    //         num_runs++;
    //         bool detected = DetectLoop();
        //     if(detected) {
        //         num_detected++;
        //         bool found_se3 = ComputeSE3();
        //         if(found_se3) {
        //             this->CorrectLoop();
        //         }
        //     }
            // mapmanager_->AddToDatabase(kf_query_);
        // }

        // if(this->ShallFinish()){
        //     std::cout << "PlaceRec " << ": close" << std::endl;
        //     break;
        // }

        // usleep(1000);
    // }

    std::unique_lock<std::mutex> lock(mtx_finish_);
    // is_finished_ = true;
}

}
