/**
* This file is part of mersys.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/mersys>
*
* mersys is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* mersys is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with mersys. If not, see <http://www.gnu.org/licenses/>.
*/

#include <config_backend.hpp>

namespace mersys_params {

auto ShowParamsBackend()->void
{
    std::cout << "++++++++++ System ++++++++++" << std::endl;
    std::cout << "threads_server: " << mersys_params::sys::threads_server << std::endl;
    std::cout << "covis_thres: " << mersys_params::sys::covis_thres << std::endl;
    std::cout << "map_path0: " << mersys_params::sys::map_path0 << std::endl;
    std::cout << "map_path1: " << mersys_params::sys::map_path1 << std::endl;
    std::cout << "map_path2: " << mersys_params::sys::map_path2 << std::endl;
    std::cout << "map_path3: " << mersys_params::sys::map_path3 << std::endl;
    std::cout << "map_path4: " << mersys_params::sys::map_path4 << std::endl;
    std::cout << "map_path5: " << mersys_params::sys::map_path5 << std::endl;
    std::cout << "map_path6: " << mersys_params::sys::map_path6 << std::endl;
    std::cout << "map_path7: " << mersys_params::sys::map_path7 << std::endl;
    std::cout << "map_path8: " << mersys_params::sys::map_path8 << std::endl;
    std::cout << "map_path9: " << mersys_params::sys::map_path9 << std::endl;
    std::cout << "map_path10: " << mersys_params::sys::map_path10 << std::endl;
    std::cout << "map_path11: " << mersys_params::sys::map_path11 << std::endl;
    std::cout << "--------------------------" << std::endl;
    std::cout << "ORB Voc dir: " << mersys_params::sys::voc_orb_dir << std::endl;
    std::cout << "--------------------------" << std::endl;
    std::cout << "output_dir: " << mersys_params::sys::output_dir << std::endl;
    std::cout << "trajectory_format: " << mersys_params::sys::trajectory_format << std::endl;
    std::cout << std::endl;
    std::cout << "++++++++++ Feature Extraction ++++++++++" << std::endl;
    std::cout << "feature type: " << mersys_params::features::type << std::endl;
    std::cout << "desc_length: " << mersys_params::features::desc_length << std::endl;
    std::cout << "num_octaves: " << mersys_params::features::num_octaves << std::endl;
    std::cout << "scale_factor: " << mersys_params::features::scale_factor << std::endl;
    std::cout << "++++++++++ Feature Matching ++++++++++" << std::endl;
    std::cout << "desc_matching_th_low: " << mersys_params::matcher::desc_matching_th_low << std::endl;
    std::cout << "desc_matching_th_high: " << mersys_params::matcher::desc_matching_th_high << std::endl;
    std::cout << "search_radius_SE3: " << mersys_params::matcher::search_radius_SE3 << std::endl;
    std::cout << "search_radius_proj: " << mersys_params::matcher::search_radius_proj << std::endl;
    std::cout << "search_radius_fuse: " << mersys_params::matcher::search_radius_fuse << std::endl;
    std::cout << "image_matching_threshold (mersys_G): " << mersys_params::features::img_match_thres << std::endl;
    std::cout << "ratio_threshold (mersys_G): " << mersys_params::features::ratio_thres << std::endl;
    std::cout << "++++++++++ Mapping ++++++++++" << std::endl;
    std::cout << "activate_lm_culling: " << (int)mersys_params::mapping::activate_lm_culling << std::endl;
    std::cout << "kf_culling_th_red: " << mersys_params::mapping::kf_culling_th_red << std::endl;
    std::cout << "kf_culling_max_time_dist: " << mersys_params::mapping::kf_culling_max_time_dist << std::endl;
    std::cout << "++++++++++ Place Rec ++++++++++" << std::endl;
    std::cout << "active: " << (int)mersys_params::placerec::active << std::endl;
    if(mersys_params::placerec::active) {
        std::cout << "type: " << "\033[1;32m" << mersys_params::placerec::type << "\033[0m" << std::endl;
        std::cout << "start_after_kf: " << mersys_params::placerec::start_after_kf << std::endl;
        std::cout << "consecutive_loop_dist: " << mersys_params::placerec::consecutive_loop_dist << std::endl;
        std::cout << "min_loop_dist: " << mersys_params::placerec::min_loop_dist << std::endl;
        std::cout << "cov_consistency_thres: " << mersys_params::placerec::cov_consistency_thres << std::endl;
        std::cout << "matches_thres: " << mersys_params::placerec::matches_thres << std::endl;
        std::cout << "inliers_thres: " << mersys_params::placerec::inliers_thres << std::endl;
        std::cout << "total_matches_thres: " << mersys_params::placerec::total_matches_thres << std::endl;
        std::cout << "matches_thres_merge: " << mersys_params::placerec::matches_thres_merge << std::endl;
        std::cout << "inter_map_matches_only: " << (int)mersys_params::placerec::inter_map_matches_only << std::endl;
        std::cout << "exclude_kfs_with_id_less_than: " << mersys_params::placerec::exclude_kfs_with_id_less_than << std::endl;
        std::cout << "--- RANSAC ---" << std::endl;
        std::cout << "min_inliers: " << mersys_params::placerec::ransac::min_inliers << std::endl;
        std::cout << "probability: " << mersys_params::placerec::ransac::probability << std::endl;
        std::cout << "max_iterations: " << mersys_params::placerec::ransac::max_iterations << std::endl;
        std::cout << "class_threshold: " << mersys_params::placerec::ransac::class_threshold << std::endl;
        std::cout << "-- mersys_G Parameters --" << std::endl;
        std::cout << "--- Non-Central Rel Pose ---" << std::endl;
        std::cout << "non_central_rel_pose_rep_err: " << mersys_params::placerec::nc_rel_pose::rp_error << std::endl;
        std::cout << "non_central_rel_pose_rep_err_cov: " << mersys_params::placerec::nc_rel_pose::rp_error_cov << std::endl;
        std::cout << "non_central_rel_pose_min_inliers: " << mersys_params::placerec::nc_rel_pose::min_inliers << std::endl;
        std::cout << "non_central_rel_pose_max_iters: " << mersys_params::placerec::nc_rel_pose::max_iters << std::endl;
        std::cout << "non_central_rel_pose_cov_thres: " << mersys_params::placerec::nc_rel_pose::cov_thres << std::endl;
        std::cout << "non_central_rel_pose_cov_iters: " << mersys_params::placerec::nc_rel_pose::cov_iters << std::endl;
        std::cout << "non_central_rel_pose_cov_max_iters: " << mersys_params::placerec::nc_rel_pose::cov_max_iters << std::endl;
        std::cout << "--- Rel Pose ---" << std::endl;
        std::cout << "rel_pose_err_thres: " << mersys_params::placerec::rel_pose::error_thres << std::endl;
        std::cout << "rel_pose_min_inliers: " << mersys_params::placerec::rel_pose::min_inliers << std::endl;
        std::cout << "rel_pose_max_iters: " << mersys_params::placerec::rel_pose::max_iters << std::endl;
        std::cout << "rel_pose_min_img_matches: " << mersys_params::placerec::rel_pose::min_img_matches << std::endl;
        std::cout << "--- Loop Thresholds ---" << std::endl;
        std::cout << "max yaw (deg): " << mersys_params::placerec::max_yaw << std::endl;
        std::cout << "max trans (m): " << mersys_params::placerec::max_trans << std::endl;
        
    } else {
        std::cout << "\033[1;33m Attention: Place Recognition Deactivated \033[0m " << std::endl;
    }
    std::cout << "++++++++++ Opt ++++++++++" << std::endl;
    std::cout << "gba_iteration_limit: " << mersys_params::opt::gba_iteration_limit << std::endl;
    std::cout << "th_outlier_align: " << mersys_params::opt::th_outlier_align << std::endl;
    std::cout << "th_gba_outlier_global: " << mersys_params::opt::th_gba_outlier_global << std::endl;
    std::cout << "pgo_iteration_limit: " << mersys_params::opt::pgo_iteration_limit << std::endl;
    std::cout << "gba_use_map_loop_constraints: " << (int)mersys_params::opt::gba_use_map_loop_constraints << std::endl;
    std::cout << "perform_pgo: " << (int)mersys_params::opt::perform_pgo << std::endl;
    std::cout << "pgo_use_neighbor_kfs: " << (int)mersys_params::opt::use_nbr_kfs << std::endl;
    std::cout << "pgo_use_robust_loss: " << (int)mersys_params::opt::use_robust_loss << std::endl;
    std::cout << "pgo_fix_kfs_after_gba: " << (int)mersys_params::opt::pgo_fix_kfs_after_gba << std::endl;
    std::cout << "pgo_fix_poses_loaded_maps: " << (int)mersys_params::opt::pgo_fix_poses_loaded_maps << std::endl;
    std::cout << "gba_fix_poses_loaded_maps: " << (int)mersys_params::opt::gba_fix_poses_loaded_maps << std::endl;
    std::cout << "++++++++++ Vis ++++++++++" << std::endl;
    std::cout << "active: " << (int)mersys_params::vis::active << std::endl;
    std::cout << "showcovgraph: " << (int)mersys_params::vis::showcovgraph << std::endl;
    std::cout << "showlandmarks: " << (int)mersys_params::vis::showlandmarks << std::endl;
    std::cout << "showtraj: " << (int)mersys_params::vis::showtraj << std::endl;
    std::cout << "showkeyframes: " << (int)mersys_params::vis::showkeyframes << std::endl;
    std::cout << "covgraph_minweight: " << mersys_params::vis::covgraph_minweight << std::endl;
    std::cout << "covgraph_shared_edges_only: " << (int)mersys_params::vis::covgraph_shared_edges_only << std::endl;
    std::cout << "scalefactor: " << mersys_params::vis::scalefactor << std::endl;
    std::cout << "trajmarkersize: " << mersys_params::vis::trajmarkersize << std::endl;
    std::cout << "covmarkersize: " << mersys_params::vis::covmarkersize << std::endl;
    std::cout << "loopmarkersize: " << mersys_params::vis::loopmarkersize << std::endl;
    std::cout << "camsize: " << mersys_params::vis::camsize << std::endl;
    std::cout << "camlinesize: " << mersys_params::vis::camlinesize << std::endl;
    std::cout << std::endl;
    std::cout << "++++++++++ Colors [R|G|B] ++++++++++" << std::endl;
    for(int idx=0;idx<12;++idx)
        std::cout << idx << ": " << mersys_params::colors::col_vec[idx].mfR << "|" << mersys_params::colors::col_vec[idx].mfG << "|" << mersys_params::colors::col_vec[idx].mfB << std::endl;
    std::cout << "Covis: " << mersys_params::colors::color_cov.mfR << "|" << mersys_params::colors::color_cov.mfG << "|" << mersys_params::colors::color_cov.mfB << std::endl;
    std::cout << std::endl;
}

} //end ns
