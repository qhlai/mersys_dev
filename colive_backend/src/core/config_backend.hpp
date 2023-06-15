/**
* This file is part of colive.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/colive>
*
* colive is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* colive is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with colive. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

// C++
#include <iostream>
#include <opencv2/opencv.hpp>

// colive
#include <typedefs_base.hpp>

namespace colive_params {

using precision_t = colive::TypeDefs::precision_t;

const std::string s0 (__FILE__);
const std::size_t p0 = s0.find("colive/colive_backend/src/core");
const std::string s1 (s0.substr(0,p0));
const std::string s2 ("colive/colive_backend/config/config_backend.yaml");
const std::string s3 = s1 + s2;
const std::string conf (s3);

const std::string out0 (__FILE__);
const std::size_t iout0 = out0.find("src/core");
const std::string out1 (out0.substr(0,iout0));
const std::string out2 ("output/");
const std::string out3 = out1 + out2;
const std::string outpath (out3);

struct VisColorRGB {
public:
    VisColorRGB(float fR,float fG, float fB)
        : mfR(fR),mfG(fG),mfB(fB),
          mu8R((u_int8_t)(fR*255)),mu8G((u_int8_t)(fG*255)),mu8B((u_int8_t)(fB*255))
        {}

    const float mfR,mfG,mfB;
    const u_int8_t mu8R,mu8G,mu8B;
};

inline std::vector<VisColorRGB> LoadColAsVec(std::string path)
{
    const VisColorRGB mc0                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR0"),read_parm::GetValFromYaml<float>(path,"vis.colorG0"),read_parm::GetValFromYaml<float>(path,"vis.colorB0"));
    const VisColorRGB mc1                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR1"),read_parm::GetValFromYaml<float>(path,"vis.colorG1"),read_parm::GetValFromYaml<float>(path,"vis.colorB1"));
    const VisColorRGB mc2                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR2"),read_parm::GetValFromYaml<float>(path,"vis.colorG2"),read_parm::GetValFromYaml<float>(path,"vis.colorB2"));
    const VisColorRGB mc3                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR3"),read_parm::GetValFromYaml<float>(path,"vis.colorG3"),read_parm::GetValFromYaml<float>(path,"vis.colorB3"));
    const VisColorRGB mc4                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR4"),read_parm::GetValFromYaml<float>(path,"vis.colorG4"),read_parm::GetValFromYaml<float>(path,"vis.colorB4"));
    const VisColorRGB mc5                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR5"),read_parm::GetValFromYaml<float>(path,"vis.colorG5"),read_parm::GetValFromYaml<float>(path,"vis.colorB5"));
    const VisColorRGB mc6                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR6"),read_parm::GetValFromYaml<float>(path,"vis.colorG6"),read_parm::GetValFromYaml<float>(path,"vis.colorB6"));
    const VisColorRGB mc7                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR7"),read_parm::GetValFromYaml<float>(path,"vis.colorG7"),read_parm::GetValFromYaml<float>(path,"vis.colorB7"));
    const VisColorRGB mc8                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR8"),read_parm::GetValFromYaml<float>(path,"vis.colorG8"),read_parm::GetValFromYaml<float>(path,"vis.colorB8"));
    const VisColorRGB mc9                               = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR9"),read_parm::GetValFromYaml<float>(path,"vis.colorG9"),read_parm::GetValFromYaml<float>(path,"vis.colorB9"));
    const VisColorRGB mc10                              = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR10"),read_parm::GetValFromYaml<float>(path,"vis.colorG10"),read_parm::GetValFromYaml<float>(path,"vis.colorB10"));
    const VisColorRGB mc11                              = VisColorRGB(read_parm::GetValFromYaml<float>(path,"vis.colorR11"),read_parm::GetValFromYaml<float>(path,"vis.colorG11"),read_parm::GetValFromYaml<float>(path,"vis.colorB11"));
    std::vector<VisColorRGB> colors;
    colors.push_back(mc0);
    colors.push_back(mc1);
    colors.push_back(mc2);
    colors.push_back(mc3);
    colors.push_back(mc4);
    colors.push_back(mc5);
    colors.push_back(mc6);
    colors.push_back(mc7);
    colors.push_back(mc8);
    colors.push_back(mc9);
    colors.push_back(mc10);
    colors.push_back(mc11);
    return colors;
}

namespace sys {
    const int threads_server                            = read_parm::GetValFromYaml<int>(conf,"sys.threads_server");
    const int covis_thres                               = read_parm::GetValFromYaml<int>(conf,"sys.covis_thres");
    const std::string map_path0                         = read_parm::GetStringFromYaml(conf,"sys.map_path0");
    const std::string map_path1                         = read_parm::GetStringFromYaml(conf,"sys.map_path1");
    const std::string map_path2                         = read_parm::GetStringFromYaml(conf,"sys.map_path2");
    const std::string map_path3                         = read_parm::GetStringFromYaml(conf,"sys.map_path3");
    const std::string map_path4                         = read_parm::GetStringFromYaml(conf,"sys.map_path4");
    const std::string map_path5                         = read_parm::GetStringFromYaml(conf,"sys.map_path5");
    const std::string map_path6                         = read_parm::GetStringFromYaml(conf,"sys.map_path6");
    const std::string map_path7                         = read_parm::GetStringFromYaml(conf,"sys.map_path7");
    const std::string map_path8                         = read_parm::GetStringFromYaml(conf,"sys.map_path8");
    const std::string map_path9                         = read_parm::GetStringFromYaml(conf,"sys.map_path9");
    const std::string map_path10                        = read_parm::GetStringFromYaml(conf,"sys.map_path10");
    const std::string map_path11                        = read_parm::GetStringFromYaml(conf,"sys.map_path11");
    //--------------------------
    const std::string voc_orb_dir                       = s1 + "config/ORBvoc.txt";
    //--------------------------
    const std::string output_dir                        = outpath;
    const std::string trajectory_format                 = read_parm::GetStringFromYaml(conf,"sys.trajectory_format");
}

namespace features {
    const std::string type                              = read_parm::GetStringFromYaml(conf,"feat.type");
    const int desc_length                               = read_parm::GetValFromYaml<int>(conf,"feat.desc_length");
    const int num_octaves                               = read_parm::GetValFromYaml<int>(conf,"feat.num_octaves");
    const precision_t scale_factor                      = read_parm::GetValFromYaml<precision_t>(conf,"feat.scale_factor");
    const float img_match_thres                         = read_parm::GetValFromYaml<float>(conf,"extractor.img_match_thres");
    const float ratio_thres                             = read_parm::GetValFromYaml<float>(conf,"extractor.ratio_thres");
}

namespace matcher {
    const int desc_matching_th_low                      = read_parm::GetValFromYaml<int>(conf,"matcher.desc_matching_th_low");
    const int desc_matching_th_high                     = read_parm::GetValFromYaml<int>(conf,"matcher.desc_matching_th_high");
    const precision_t search_radius_SE3                 = read_parm::GetValFromYaml<precision_t>(conf,"matcher.search_radius_SE3");
    const precision_t search_radius_proj                = read_parm::GetValFromYaml<precision_t>(conf,"matcher.search_radius_proj");
    const precision_t search_radius_fuse                = read_parm::GetValFromYaml<precision_t>(conf,"matcher.search_radius_fuse");
}

namespace mapping {
    const bool activate_lm_culling                      = read_parm::GetValFromYaml<bool>(conf,"mapping.activate_lm_culling");
    const precision_t kf_culling_th_red                 = read_parm::GetValFromYaml<precision_t>(conf,"mapping.kf_culling_th_red");
    const precision_t kf_culling_max_time_dist          = read_parm::GetValFromYaml<precision_t>(conf,"mapping.kf_culling_max_time_dist");
}

namespace placerec {
    const bool active                                   = read_parm::GetValFromYaml<bool>(conf,"placerec.active");
    const std::string type                              = read_parm::GetStringFromYaml(conf,"placerec.type");
    const size_t start_after_kf                         = read_parm::GetValFromYaml<int>(conf,"placerec.start_after_kf");
    const size_t consecutive_loop_dist                  = read_parm::GetValFromYaml<int>(conf,"placerec.consecutive_loop_dist");
    const int min_loop_dist                             = read_parm::GetValFromYaml<int>(conf,"placerec.min_loop_dist");
    const int cov_consistency_thres                     = read_parm::GetValFromYaml<int>(conf,"placerec.cov_consistency_thres");
    const int matches_thres                             = read_parm::GetValFromYaml<int>(conf,"placerec.matches_thres");
    const int matches_thres_merge                       = read_parm::GetValFromYaml<int>(conf,"placerec.matches_thres_merge");
    const int inliers_thres                             = read_parm::GetValFromYaml<int>(conf,"placerec.inliers_thres");
    const int total_matches_thres                       = read_parm::GetValFromYaml<int>(conf,"placerec.total_matches_thres");

    const bool inter_map_matches_only                   = read_parm::GetValFromYaml<bool>(conf,"placerec.inter_map_matches_only");
    const int exclude_kfs_with_id_less_than             = read_parm::GetValFromYaml<size_t>(conf,"placerec.exclude_kfs_with_id_less_than");

    namespace ransac {
        const int min_inliers               = read_parm::GetValFromYaml<int>(conf,"placerec.ransac.min_inliers");
        const precision_t probability       = read_parm::GetValFromYaml<precision_t>(conf,"placerec.ransac.probability");
        const int max_iterations            = read_parm::GetValFromYaml<int>(conf,"placerec.ransac.max_iterations");
        const int class_threshold           = read_parm::GetValFromYaml<int>(conf,"placerec.ransac.class_threshold");
    } // namespace ransac

    namespace nc_rel_pose {
        const int cov_iters                 = read_parm::GetValFromYaml<int>(conf,"placerec.nc_rel_pose.cov_iters");
        const int cov_max_iters             = read_parm::GetValFromYaml<int>(conf,"placerec.nc_rel_pose.cov_max_iters");
        const int max_iters                 = read_parm::GetValFromYaml<int>(conf,"placerec.nc_rel_pose.max_iters");
        const int min_inliers               = read_parm::GetValFromYaml<int>(conf,"placerec.nc_rel_pose.min_inliers");
        const float cov_thres               = read_parm::GetValFromYaml<float>(conf,"placerec.nc_rel_pose.cov_thres");
        const float rp_error                = read_parm::GetValFromYaml<float>(conf,"placerec.nc_rel_pose.rp_error");
        const float rp_error_cov            = read_parm::GetValFromYaml<float>(conf,"placerec.nc_rel_pose.rp_error_cov");
    } //namespace nc_rel_pose

    namespace rel_pose {
        const int max_iters                 = read_parm::GetValFromYaml<int>(conf,"placerec.rel_pose.max_iters");
        const int min_inliers               = read_parm::GetValFromYaml<int>(conf,"placerec.rel_pose.min_inliers");
        const float error_thres             = read_parm::GetValFromYaml<float>(conf,"placerec.rel_pose.error_thres");
        const int min_img_matches           = read_parm::GetValFromYaml<int>(conf,"placerec.rel_pose.min_img_matches");
    } //namespace nc_rel_pose

    const float max_yaw                     = read_parm::GetValFromYaml<float>(conf,"placerec.max_yaw");
    const float max_trans                   = read_parm::GetValFromYaml<float>(conf,"placerec.max_trans");
}

namespace opt {
    const int gba_iteration_limit                       = read_parm::GetValFromYaml<int>(conf,"opt.gba_iteration_limit");
    const precision_t th_outlier_align                  = read_parm::GetValFromYaml<precision_t>(conf,"opt.th_outlier_align");
    const precision_t th_gba_outlier_global             = read_parm::GetValFromYaml<precision_t>(conf,"opt.th_gba_outlier_global");
    const int pgo_iteration_limit                       = read_parm::GetValFromYaml<int>(conf,"opt.pgo_iteration_limit");

    const bool perform_pgo                              = read_parm::GetValFromYaml<bool>(conf,"opt.perform_pgo");
    const bool use_nbr_kfs                              = read_parm::GetValFromYaml<bool>(conf,"opt.use_nbr_kfs");
    const bool use_robust_loss                          = read_parm::GetValFromYaml<bool>(conf,"opt.use_robust_loss");
    const bool pgo_fix_kfs_after_gba                    = read_parm::GetValFromYaml<bool>(conf,"opt.pgo_fix_kfs_after_gba");
    const bool pgo_fix_poses_loaded_maps                = read_parm::GetValFromYaml<bool>(conf,"opt.pgo_fix_poses_loaded_maps");
    const bool gba_fix_poses_loaded_maps                = read_parm::GetValFromYaml<bool>(conf,"opt.gba_fix_poses_loaded_maps");

    const bool pgo_use_cov_edges                        = read_parm::GetValFromYaml<bool>(conf,"opt.pgo_use_cov_edges");
    const int pgo_min_edge_weight                       = read_parm::GetValFromYaml<int>(conf,"opt.pgo_min_edge_weight");
    const bool pgo_use_map_loop_constraints             = read_parm::GetValFromYaml<bool>(conf,"opt.pgo_use_map_loop_constraints");
    const bool pgo_use_loop_edges                       = read_parm::GetValFromYaml<bool>(conf,"opt.pgo_use_loop_edges");

    const bool gba_use_map_loop_constraints             = read_parm::GetValFromYaml<bool>(conf,"opt.gba_use_map_loop_constraints");

    // For Weighting Loops and KFs in PGO
    const float wt_kf_r             = read_parm::GetValFromYaml<float>(conf,"opt.wt_kf_R");
    const float wt_kf_t             = read_parm::GetValFromYaml<float>(conf,"opt.wt_kf_T");
    const float wt_kf_n1           = read_parm::GetValFromYaml<float>(conf,"opt.wt_kf_n1");
    const float wt_kf_n23            = read_parm::GetValFromYaml<float>(conf,"opt.wt_kf_n23");
    const float wt_kf_n45            = read_parm::GetValFromYaml<float>(conf,"opt.wt_kf_n45");
}

namespace vis {
    const bool active                                   = read_parm::GetValFromYaml<bool>(conf,"vis.active");
    const bool showcovgraph                             = read_parm::GetValFromYaml<bool>(conf,"vis.showcovgraph");
    const bool showlandmarks                            = read_parm::GetValFromYaml<bool>(conf,"vis.showlandmarks");
    const bool showtraj                                 = read_parm::GetValFromYaml<bool>(conf,"vis.showtraj");
    const bool showkeyframes                            = read_parm::GetValFromYaml<bool>(conf,"vis.showkeyframes"); //-1=no KFs;0=frusta;1=spheres
    const int covgraph_minweight                        = read_parm::GetValFromYaml<int>(conf,"vis.covgraph_minweight");
    const bool covgraph_shared_edges_only               = read_parm::GetValFromYaml<bool>(conf,"vis.covgraph_shared_edges_only"); // show only cov edges between trajectories from different agents

    const precision_t scalefactor                       = read_parm::GetValFromYaml<precision_t>(conf,"vis.scalefactor");
    const precision_t trajmarkersize                    = read_parm::GetValFromYaml<precision_t>(conf,"vis.trajmarkersize");
    const precision_t covmarkersize                     = read_parm::GetValFromYaml<precision_t>(conf,"vis.covmarkersize");
    const precision_t loopmarkersize                    = read_parm::GetValFromYaml<precision_t>(conf,"vis.loopmarkersize");
    const precision_t camsize                           = read_parm::GetValFromYaml<precision_t>(conf,"vis.camsize");
    const precision_t camlinesize                       = read_parm::GetValFromYaml<precision_t>(conf,"vis.camlinesize");
}

namespace colors {
    const std::vector<VisColorRGB> col_vec              = LoadColAsVec(conf);
    const VisColorRGB color_cov                         = VisColorRGB(read_parm::GetValFromYaml<colive::TypeDefs::precision_t>(conf,"vis.colorRcov"),read_parm::GetValFromYaml<colive::TypeDefs::precision_t>(conf,"vis.colorGcov"),read_parm::GetValFromYaml<colive::TypeDefs::precision_t>(conf,"vis.colorBcov"));
}

void ShowParamsBackend();

} //end ns
