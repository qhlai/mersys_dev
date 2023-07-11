#include "map_rgb.hpp"
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
    // m_mutex_pts_vec = std::make_shared< std::mutex >();
    // m_mutex_img_pose_for_projection = std::make_shared< std::mutex >();
    // m_mutex_recent_added_list = std::make_shared< std::mutex >();
    // m_mutex_rgb_pts_in_recent_hitted_boxes = std::make_shared< std::mutex >();
    // m_mutex_m_box_recent_hitted = std::make_shared< std::mutex >();
    // m_mutex_pts_last_visited = std::make_shared< std::mutex >();
    // // Allocate memory for pointclouds
    // if ( Common_tools::get_total_phy_RAM_size_in_GB() < 12 )
    // {
    //     scope_color( ANSI_COLOR_RED_BOLD );
    //     std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
    //     cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
    //     cout << "I have detected your physical memory smaller than 12GB (currently: " << Common_tools::get_total_phy_RAM_size_in_GB()
    //          << "GB). I recommend you to add more physical memory for improving the overall performance of R3LIVE." << endl;
    //     cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
    //     std::this_thread::sleep_for( std::chrono::seconds( 5 ) );
    //     m_rgb_pts_vec.reserve( 1e8 );
    // }
    // else
    // {
    //     m_rgb_pts_vec.reserve( 1e9 );
    // }
    // // m_rgb_pts_in_recent_visited_voxels.reserve( 1e6 );
    // if ( if_start_service )
    // {
    //     m_thread_service = std::make_shared< std::thread >( &Global_map::service_refresh_pts_for_projection, this );
    // }
}
// Global_map::~Global_map(){};

}