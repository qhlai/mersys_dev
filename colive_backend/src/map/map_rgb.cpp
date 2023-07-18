#include "map_rgb.hpp"

#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include <opencv2/opencv.hpp>
// #include "common_tools.h"
namespace colive {

cv::RNG g_rng = cv::RNG(0);
// std::atomic<long> g_pts_index(0);

void RGB_pts::set_pos(const TypeDefs::Vector3Type &pos)
{
    m_pos[0] = pos(0);
    m_pos[1] = pos(1);
    m_pos[2] = pos(2);
}

TypeDefs::Vector3Type RGB_pts::get_pos()
{
    return TypeDefs::Vector3Type(m_pos[0], m_pos[1], m_pos[2]);
}

TypeDefs::Matrix3Type RGB_pts::get_rgb_cov()
{
    TypeDefs::Matrix3Type cov_mat = TypeDefs::Matrix3Type::Zero();
    for (int i = 0; i < 3; i++)
    {
        cov_mat(i, i) = m_cov_rgb[i];
    }
    return cov_mat;
}

TypeDefs::Vector3Type RGB_pts::get_rgb()
{
    return TypeDefs::Vector3Type(m_rgb[0], m_rgb[1], m_rgb[2]);
}

pcl::PointXYZI RGB_pts::get_pt()
{
    pcl::PointXYZI pt;
    pt.x = m_pos[0];
    pt.y = m_pos[1];
    pt.z = m_pos[2];
    return pt;
}

void RGB_pts::update_gray(const double gray, const double obs_dis)
{
    if (m_obs_dis != 0 && (obs_dis > m_obs_dis * 1.2))
    {
        return;
    }
    m_gray = (m_gray * m_N_gray + gray) / (m_N_gray + 1);
    if (m_obs_dis == 0 || (obs_dis < m_obs_dis))
    {
        m_obs_dis = obs_dis;
        // m_gray = gray;
    }
    m_N_gray++;
    // TODO: cov update
};

const double image_obs_cov = 15;
const double process_noise_sigma = 0.1;

int RGB_pts::update_rgb(const TypeDefs::Vector3Type &rgb, const double obs_dis, const TypeDefs::Vector3Type obs_sigma, const double obs_time)
{
    if (m_obs_dis != 0 && (obs_dis > m_obs_dis * 1.2))
    {
        return 0;
    }

    if( m_N_rgb == 0)
    {
        // For first time of observation.
        m_last_obs_time = obs_time;
        m_obs_dis = obs_dis;
        for (int i = 0; i < 3; i++)
        {
            m_rgb[i] = rgb[i];
            m_cov_rgb[i] = obs_sigma(i) ;
        }
        m_N_rgb = 1;
        return 0;
    }
    // State estimation for robotics, section 2.2.6, page 37-38
    for(int i = 0 ; i < 3; i++)
    {
        m_cov_rgb[i] = (m_cov_rgb[i] + process_noise_sigma * (obs_time - m_last_obs_time)); // Add process noise
        double old_sigma = m_cov_rgb[i];
        m_cov_rgb[i] = sqrt( 1.0 / (1.0 / m_cov_rgb[i] / m_cov_rgb[i] + 1.0 / obs_sigma(i) / obs_sigma(i)) );
        m_rgb[i] = m_cov_rgb[i] * m_cov_rgb[i] * ( m_rgb[i] / old_sigma / old_sigma + rgb(i) / obs_sigma(i) / obs_sigma(i) );
    }

    if (obs_dis < m_obs_dis)
    {
        m_obs_dis = obs_dis;
    }
    m_last_obs_time = obs_time;
    m_N_rgb++;
    return 1;
}

void RGB_pts::Intensity2Rgb( int colormap_type){
    // Define the color map
    cv::Mat colormap;
    cv::applyColorMap(cv::Mat(1, 1, CV_8U, intensity), colormap, colormap_type);

    // Split the RGB channels
    // Vector3Type  rgb_value;
    cv::Vec3b* ptr = colormap.ptr<cv::Vec3b>();
    bgr_intensity[0] = ptr[0][0];  // blue channel
    bgr_intensity[1] = ptr[0][1];  // green channel
    bgr_intensity[2] = ptr[0][2];  // red channel

    // return rgb_value;
}

void Global_map::clear()
{
    m_rgb_pts_vec.clear();
}

void Global_map::set_minmum_dis(double minimum_dis)
{
    m_hashmap_3d_pts.clear();
    m_minimum_pts_size = minimum_dis;
}
Global_map::Global_map( int if_start_service )
{
    m_mutex_pts_vec = std::make_shared< std::mutex >();
    m_mutex_img_pose_for_projection = std::make_shared< std::mutex >();
    m_mutex_recent_added_list = std::make_shared< std::mutex >();
    m_mutex_rgb_pts_in_recent_hitted_boxes = std::make_shared< std::mutex >();
    m_mutex_m_box_recent_hitted = std::make_shared< std::mutex >();
    m_mutex_pts_last_visited = std::make_shared< std::mutex >();


    m_voxels_recent_visited.reserve(MAX_CLIENT_NUM);
    // m_last_visited_time.reserve(MAX_CLIENT_NUM);

    // // Allocate memory for pointclouds
    if ( Common_tools::get_total_phy_RAM_size_in_GB() < 16 )
    {
        std::cout << COUTNOTICE << "less memory occupy" << std::endl;
        m_rgb_pts_vec.reserve( 1e8 );
    }
    else
    {
        m_rgb_pts_vec.reserve( 1e9 );
    }
    
    // m_rgb_pts_in_recent_visited_voxels.reserve( 1e6 );
    if ( if_start_service )
    {
        // m_thread_service = std::make_shared< std::thread >( &Global_map::service_refresh_pts_for_projection, this );
    }
}
Global_map::~Global_map(){};

void Global_map::service_refresh_pts_for_projection()
{
    // eigen_q last_pose_q = eigen_q::Identity();
    // Common_tools::Timer                timer;
    // std::shared_ptr< Image_frame > img_for_projection = std::make_shared< Image_frame >();
    // while (1)
    // {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     m_mutex_img_pose_for_projection->lock();
         
    //     *img_for_projection = m_img_for_projection;
    //     m_mutex_img_pose_for_projection->unlock();
    //     if (img_for_projection->m_img_cols == 0 || img_for_projection->m_img_rows == 0)
    //     {
    //         continue;
    //     }

    //     if (img_for_projection->m_frame_idx == m_updated_frame_index)
    //     {
    //         continue;
    //     }
    //     timer.tic(" ");
    //     std::shared_ptr<std::vector<std::shared_ptr<RGB_pts>>> pts_rgb_vec_for_projection = std::make_shared<std::vector<std::shared_ptr<RGB_pts>>>();
    //     if (m_if_get_all_pts_in_boxes_using_mp)
    //     {
    //         std::vector<std::shared_ptr<RGB_pts>>  pts_in_recent_hitted_boxes;
    //         pts_in_recent_hitted_boxes.reserve(1e6);
    //         std::unordered_set< std::shared_ptr< RGB_Voxel> > boxes_recent_hitted;
    //         m_mutex_m_box_recent_hitted->lock();
    //         boxes_recent_hitted = m_voxels_recent_visited;
    //         m_mutex_m_box_recent_hitted->unlock();

    //         // get_all_pts_in_boxes(boxes_recent_hitted, pts_in_recent_hitted_boxes);
    //         m_mutex_rgb_pts_in_recent_hitted_boxes->lock();
    //         // m_rgb_pts_in_recent_visited_voxels = pts_in_recent_hitted_boxes;
    //         m_mutex_rgb_pts_in_recent_hitted_boxes->unlock();
    //     }
    //     selection_points_for_projection(img_for_projection, pts_rgb_vec_for_projection.get(), nullptr, 10.0, 1);
    //     m_mutex_pts_vec->lock();
    //     m_pts_rgb_vec_for_projection = pts_rgb_vec_for_projection;
    //     m_updated_frame_index = img_for_projection->m_frame_idx;
    //     // cout << ANSI_COLOR_MAGENTA_BOLD << "Refresh pts_for_projection size = " << m_pts_rgb_vec_for_projection->size()
    //     //      << " | " << m_rgb_pts_vec.size()
    //     //      << ", cost time = " << timer.toc() << ANSI_COLOR_RESET << endl;
    //     m_mutex_pts_vec->unlock();
    //     last_pose_q = img_for_projection->m_pose_w2c_q;
    // }
}
void Global_map::render_points_for_projection( TypeDefs::ImageEXPtr img_ptr )
{
    m_mutex_pts_vec->lock();
    if (m_pts_rgb_vec_for_projection != nullptr)
    {
        // render_pts_in_voxels(img_ptr, *m_pts_rgb_vec_for_projection);
        // render_pts_in_voxels(img_ptr, m_rgb_pts_vec);
    }
    m_last_updated_frame_idx = img_ptr->id_;
    m_mutex_pts_vec->unlock();
}
void Global_map::update_pose_for_projection(TypeDefs::ImageEXPtr img, double fov_margin)
{
    m_mutex_img_pose_for_projection->lock();
    // m_img_for_projection.set_intrinsic(img->m_cam_K);
    // m_img_for_projection.m_img_cols = img->m_img_cols;
    // m_img_for_projection.m_img_rows = img->m_img_rows;
    // m_img_for_projection.m_fov_margin = fov_margin;
    // m_img_for_projection.m_frame_idx = img->m_frame_idx;
    // m_img_for_projection.m_pose_w2c_q = img->m_pose_w2c_q;
    // m_img_for_projection.m_pose_w2c_t = img->m_pose_w2c_t;
    // m_img_for_projection.m_img_gray = img->m_img_gray; // clone?
    // m_img_for_projection.m_img = img->m_img;           // clone?
    // m_img_for_projection.refresh_pose_for_projection();
    m_mutex_img_pose_for_projection->unlock();
}
bool Global_map::is_busy()
{
    return m_in_appending_pts;
}
void Global_map::set_busy()
{
    m_in_appending_pts = true;
}
void Global_map::unset_busy()
{
    m_in_appending_pts = false;
}
// 这里是单机，要改成多机
// template <typename T>
int Global_map::append_points_to_global_map(TypeDefs::PointCloudEXPtr pc_in,  std::vector<std::shared_ptr<RGB_pts>> *pts_added_vec, int step)
{
    set_busy();
    uint32_t client_id=pc_in->GetClientID();
    double added_time=pc_in->GetTimeStamp();
    // Common_tools::Timer tim;
    // tim.tic();
    int acc = 0;
    int rej = 0;
    if (pts_added_vec != nullptr)
    {
        pts_added_vec->clear();
    }
    std::unordered_set< std::shared_ptr< RGB_Voxel > > voxels_recent_visited;
    if (m_recent_visited_voxel_activated_time == 0)
    {
        voxels_recent_visited.clear();
    }
    else
    {
        // 从 m_voxels_recent_visited 中移除最近未访问的体素。
        m_mutex_m_box_recent_hitted->lock();
        voxels_recent_visited = m_voxels_recent_visited[client_id];
        m_mutex_m_box_recent_hitted->unlock();
        for( Voxel_set_iterator it = voxels_recent_visited.begin(); it != voxels_recent_visited.end();  )
        {
            if ( added_time - ( *it )->m_last_visited_time > m_recent_visited_voxel_activated_time ) // 将该体素从集合中移除
            {
                it = voxels_recent_visited.erase( it );
                continue;
            }
            it++;
        }
        cout << "Restored voxel number = " << voxels_recent_visited.size() << endl;
    }
    // pc_in->pts_cloud
    // 要转移到真实位置
    TypeDefs::PointCloud pc = pc_in->get_transformed_pc();

    int number_of_voxels_before_add = voxels_recent_visited.size();
    int pt_size = pc.points.size();
    // // step = 4;
    for (int pt_idx = 0; pt_idx < pt_size; pt_idx += step)
    {
        int add = 1;
        int grid_x = std::round(pc.points[pt_idx].x / m_minimum_pts_size);
        int grid_y = std::round(pc.points[pt_idx].y / m_minimum_pts_size);
        int grid_z = std::round(pc.points[pt_idx].z / m_minimum_pts_size);
        int box_x =  std::round(pc.points[pt_idx].x / m_voxel_resolution);
        int box_y =  std::round(pc.points[pt_idx].y / m_voxel_resolution);
        int box_z =  std::round(pc.points[pt_idx].z / m_voxel_resolution);
        if (m_hashmap_3d_pts.if_exist(grid_x, grid_y, grid_z))// 这里写错了吗？
        {
            add = 0;
            if (pts_added_vec != nullptr)
            {
                pts_added_vec->push_back(m_hashmap_3d_pts.m_map_3d_hash_map[grid_x][grid_y][grid_z]);
            }
        }
        RGB_voxel_ptr box_ptr;
        if(!m_hashmap_voxels.if_exist(box_x, box_y, box_z))
        {
            std::shared_ptr<RGB_Voxel> box_rgb = std::make_shared<RGB_Voxel>();
            m_hashmap_voxels.insert( box_x, box_y, box_z, box_rgb );
            box_ptr = box_rgb;
        }
        else
        {
            box_ptr = m_hashmap_voxels.m_map_3d_hash_map[box_x][box_y][box_z];
        }
        voxels_recent_visited.insert( box_ptr );
        box_ptr->m_last_visited_time = added_time;
        if (add == 0)
        {
            rej++;
            continue;
        }
        acc++;
        std::shared_ptr<RGB_pts> pt_rgb = std::make_shared<RGB_pts>();

        pt_rgb->set_pos(TypeDefs::Vector3Type(
            pc.points[pt_idx].x, 
            pc.points[pt_idx].y, 
            pc.points[pt_idx].z));

        pt_rgb->m_pt_index = m_rgb_pts_vec.size();
// for intensity
#if POINTCLOUD_HAVE_INTENSITY
        pt_rgb->intensity=pc.points[pt_idx].intensity;
        // float intensity = static_cast<float>(map_rgb_pts.m_rgb_pts_vec[ i ]->intensity);
        // TypeDefs::Vector3Type bgr = getRgbFromIntensity(pt_rgb->intensity, cv::COLORMAP_RAINBOW);
#if DISPLAY_POINTCLOUD_INTENSITY
        pt_rgb->Intensity2Rgb();
        // colored_point.r = static_cast<uint8_t>(r * 255);
        // colored_point.g = static_cast<uint8_t>(g * 255);
        // colored_point.b = static_cast<uint8_t>(b * 255);
#endif
        // pc_rgb.points[ pub_idx_size ].r = bgr[2];
        // pc_rgb.points[ pub_idx_size ].g = bgr[1];
        // pc_rgb.points[ pub_idx_size ].b = bgr[0];
        // std::cout << COUTDEBUG << "intensity:"<<intensity<< std::endl;
#endif
        m_rgb_pts_vec.push_back(pt_rgb);
        m_hashmap_3d_pts.insert(grid_x, grid_y, grid_z, pt_rgb);
        box_ptr->add_pt(pt_rgb);
        if (pts_added_vec != nullptr)
        {
            pts_added_vec->push_back(pt_rgb);
        }
    }
    unset_busy();
    m_mutex_m_box_recent_hitted->lock();
    m_voxels_recent_visited[pc_in->GetClientID()] = voxels_recent_visited ;
    m_mutex_m_box_recent_hitted->unlock();
    // std::cout << COUTDEBUG <<"m_rgb_pts_vec size:"<<m_rgb_pts_vec.size()<<", pts_added_vec size:"<< pts_added_vec->size()<<std::endl;
    return (m_voxels_recent_visited[client_id].size() -  number_of_voxels_before_add);
    // return 0;
}
void Global_map::selection_points_for_projection(TypeDefs::ImageEXPtr image_pose, std::vector<std::shared_ptr<RGB_pts>> *pc_out_vec,
                                                            std::vector<cv::Point2f> *pc_2d_out_vec, double minimum_dis,
                                                            int skip_step,
                                                            int use_all_pts)
{
    // Common_tools::Timer tim;
    // tim.tic();
    size_t client_id = image_pose->GetClientID();
    if (pc_out_vec != nullptr)
    {
        pc_out_vec->clear();
    }
    if (pc_2d_out_vec != nullptr)
    {
        pc_2d_out_vec->clear();
    }
    Hash_map_2d<int, int> mask_index;
    Hash_map_2d<int, float> mask_depth;

    std::map<int, cv::Point2f> map_idx_draw_center;
    std::map<int, cv::Point2f> map_idx_draw_center_raw_pose;

    int u, v;
    double u_f, v_f;
    // cv::namedWindow("Mask", cv::WINDOW_FREERATIO);
    int acc = 0;
    int blk_rej = 0;
    // int pts_size = m_rgb_pts_vec.size();
    std::vector<std::shared_ptr<RGB_pts>> pts_for_projection;
    m_mutex_m_box_recent_hitted->lock();
    std::unordered_set< std::shared_ptr< RGB_Voxel > > boxes_recent_hitted = m_voxels_recent_visited[client_id];
    m_mutex_m_box_recent_hitted->unlock();
    // 找到预备投影点
    if ( (!use_all_pts) && boxes_recent_hitted.size()) // 最近访问点
    {
        m_mutex_rgb_pts_in_recent_hitted_boxes->lock();
        
        for(Voxel_set_iterator it = boxes_recent_hitted.begin(); it != boxes_recent_hitted.end(); it++)
        {
            // pts_for_projection.push_back( (*it)->m_pts_in_grid.back() );
            if ( ( *it )->m_pts_in_grid.size() )
            {
                 pts_for_projection.push_back( (*it)->m_pts_in_grid.back() );
                // pts_for_projection.push_back( ( *it )->m_pts_in_grid[ 0 ] );
                // pts_for_projection.push_back( ( *it )->m_pts_in_grid[ ( *it )->m_pts_in_grid.size()-1 ] );
            }
        }

        m_mutex_rgb_pts_in_recent_hitted_boxes->unlock();
    }
    else// 全部点
    {
        pts_for_projection = m_rgb_pts_vec;
    }


    int pts_size = pts_for_projection.size();
    for (int pt_idx = 0; pt_idx < pts_size; pt_idx += skip_step)
    {
        // 映射点与相机点距离
        TypeDefs::Vector3Type pt = pts_for_projection[pt_idx]->get_pos();
        double depth = (pt - image_pose->GetPoseTwc().translation()).norm();
        if (depth > m_maximum_depth_for_projection)
        {
            continue;
        }
        if (depth < m_minimum_depth_for_projection)
        {
            continue;
        }
        // bool res = image_pose->project_3d_point_in_this_img(pt, u_f, v_f, nullptr, 1.0);
        // if (res == false)
        // {
        //     continue;
        // }
        // u = std::round(u_f / minimum_dis) * minimum_dis; // Why can not work
        // v = std::round(v_f / minimum_dis) * minimum_dis;
        // if ((!mask_depth.if_exist(u, v)) || mask_depth.m_map_2d_hash_map[u][v] > depth)
        // {
        //     acc++;
        //     if (mask_index.if_exist(u, v))
        //     {
        //         // erase old point
        //         int old_idx = mask_index.m_map_2d_hash_map[u][v];
        //         blk_rej++;
        //         map_idx_draw_center.erase(map_idx_draw_center.find(old_idx));
        //         map_idx_draw_center_raw_pose.erase(map_idx_draw_center_raw_pose.find(old_idx));
        //     }
        //     mask_index.m_map_2d_hash_map[u][v] = (int)pt_idx;
        //     mask_depth.m_map_2d_hash_map[u][v] = (float)depth;
        //     map_idx_draw_center[pt_idx] = cv::Point2f(v, u);
        //     map_idx_draw_center_raw_pose[pt_idx] = cv::Point2f(u_f, v_f);
        // }
    }

    // if (pc_out_vec != nullptr)
    // {
    //     for (auto it = map_idx_draw_center.begin(); it != map_idx_draw_center.end(); it++)
    //     {
    //         // pc_out_vec->push_back(m_rgb_pts_vec[it->first]);
    //         pc_out_vec->push_back(pts_for_projection[it->first]);
    //     }
    // }

    // if (pc_2d_out_vec != nullptr)
    // {
    //     for (auto it = map_idx_draw_center.begin(); it != map_idx_draw_center.end(); it++)
    //     {
    //         pc_2d_out_vec->push_back(map_idx_draw_center_raw_pose[it->first]);
    //     }
    // }

}



}