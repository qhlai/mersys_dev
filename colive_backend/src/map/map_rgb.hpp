
#pragma once

#include <set>
// C++
#include <atomic>
#include <unordered_set>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>


// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/io/pcd_io.h>

#include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <atomic>
// #include <unordered_set>

// #include <eigen3/Eigen/Core>

// #include "tools/tools_eigen.hpp"
#include "typedefs_base.hpp"
#include "pointcloud_ex.hpp"
#include "image_ex.hpp"
// #include "image_ex.hpp"

#include "tools_kd_hash.hpp"
// #include "tools_serialization.hpp"



namespace colive {

extern cv::RNG g_rng;

// extern std::atomic< long > g_pts_index;
class RGB_pts
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#if 0
    std::atomic<double> m_pos[3];
    std::atomic<double> m_rgb[3];
    std::atomic<double> m_cov_rgb[3];
    std::atomic<double> m_gray;
    std::atomic<double> m_cov_gray;
    std::atomic<int> m_N_gray;
    std::atomic<int> m_N_rgb;
#else
    double m_pos[ 3 ] = { 0 };
    float m_rgb[ 3 ] = { 0 };
    float intensity = 0;
    uint8_t bgr_intensity[3] = {0};
    double m_cov_rgb[ 3 ] = { 0 };
    double m_gray = 0;
    double m_cov_gray = 0;
    int    m_N_gray = 0;
    int    m_N_rgb = 0;
    int    m_pt_index = 0;
#endif
    TypeDefs::Vector2Type      m_img_vel;
    TypeDefs::Vector2Type      m_img_pt_in_last_frame;
    TypeDefs::Vector2Type      m_img_pt_in_current_frame;
    int        m_is_out_lier_count = 0;
    cv::Scalar m_dbg_color;
    double     m_obs_dis = 0;
    double     m_last_obs_time = 0;
    void       clear()
    {
        m_rgb[ 0 ] = 0;
        m_rgb[ 1 ] = 0;
        m_rgb[ 2 ] = 0;
        m_gray = 0;
        m_cov_gray = 0;
        m_N_gray = 0;
        m_N_rgb = 0;
        m_obs_dis = 0;
        m_last_obs_time = 0;
        int r = g_rng.uniform( 0, 256 );
        int g = g_rng.uniform( 0, 256 );
        int b = g_rng.uniform( 0, 256 );
        m_dbg_color = cv::Scalar( r, g, b );
        // m_rgb = TypeDefs::Vector3Type(255, 255, 255);
    };

    RGB_pts()
    {
        // m_pt_index = g_pts_index++;
        clear();
    };
    ~RGB_pts(){};

    void set_pos( const TypeDefs::Vector3Type &pos );
    TypeDefs::Vector3Type          get_pos();
    TypeDefs::Vector3Type          get_rgb();
    TypeDefs::Matrix3Type        get_rgb_cov();
    pcl::PointXYZI get_pt();
    void update_gray( const double gray, double obs_dis = 1.0 );
    int update_rgb( const TypeDefs::Vector3Type &rgb, const double obs_dis, const TypeDefs::Vector3Type obs_sigma, const double obs_time );
    void Intensity2Rgb( int colormap_type=cv::COLORMAP_RAINBOW);
  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive &ar, const unsigned int version )
    {
        ar &m_pos;
        ar &m_rgb;
        ar &m_pt_index;
        ar &m_cov_rgb;
        ar &m_gray;
        ar &m_N_rgb;
        ar &m_N_gray;
    }
};
using RGB_pt_ptr = std::shared_ptr< RGB_pts >;


class RGB_Voxel
{
  public:
    std::vector< RGB_pt_ptr > m_pts_in_grid;
    double                   m_last_visited_time =0;
    RGB_Voxel() = default;
    ~RGB_Voxel() = default;
    void add_pt( RGB_pt_ptr &rgb_pts ) { m_pts_in_grid.push_back( rgb_pts ); }
};

using RGB_voxel_ptr = std::shared_ptr< RGB_Voxel >;
using Voxel_set_iterator = std::unordered_set< std::shared_ptr< RGB_Voxel > >::iterator;

struct Global_map
{
    int                                                          m_map_major_version = 0;
    int                                                          m_map_minor_version = 1;
    int                                                          m_if_get_all_pts_in_boxes_using_mp = 1;
    std::vector< RGB_pt_ptr >                    m_rgb_pts_vec;
    // std::vector< RGB_pt_ptr >                    m_rgb_pts_in_recent_visited_voxels;
    std::shared_ptr< std::vector< RGB_pt_ptr> >                  m_pts_rgb_vec_for_projection = nullptr;
    std::shared_ptr< std::mutex >                                m_mutex_pts_vec;
    std::shared_ptr< std::mutex >                                m_mutex_recent_added_list;
    std::shared_ptr< std::mutex >                                m_mutex_img_pose_for_projection;
    std::shared_ptr< std::mutex >                                m_mutex_rgb_pts_in_recent_hitted_boxes;
    std::shared_ptr< std::mutex >                                m_mutex_m_box_recent_hitted;
    std::shared_ptr< std::mutex >                                m_mutex_pts_last_visited;
    TypeDefs::ImageEXPtr                                         m_img_for_projection;
    double                                                       m_recent_visited_voxel_activated_time = 0.0;
    bool                                                         m_in_appending_pts = 0;
    int                                                          m_updated_frame_index = 0;
    std::shared_ptr< std::thread >                               m_thread_service;
    int                                                          m_if_reload_init_voxel_and_hashed_pts = true;

    Hash_map_3d< long, RGB_pt_ptr >   m_hashmap_3d_pts;
    Hash_map_3d< long, std::shared_ptr< RGB_Voxel > > m_hashmap_voxels;
    std::vector<std::unordered_set< std::shared_ptr< RGB_Voxel > >> m_voxels_recent_visited;
    std::vector< std::shared_ptr< RGB_pts > >          m_pts_last_hitted;
    double                                   m_minimum_pts_size = 0.05; // 5cm minimum distance.
    double                                   m_voxel_resolution = 0.1;
    double                                   m_maximum_depth_for_projection = 200;
    double                                   m_minimum_depth_for_projection = 3;
    TypeDefs::idpair                         m_last_updated_frame_idx;
    void                                     clear();
    void set_minmum_dis( double minimum_dis );

    Global_map( int if_start_service = 1 );
    // Global_map(Global_map &map_target, Global_map &map_tofuse, TypeDefs::TransformType &T_wtofuse_wtarget,int if_start_service = 1);
    ~Global_map();

    void service_refresh_pts_for_projection();
    void render_points_for_projection( TypeDefs::ImageEXPtr img_ptr );
    void update_pose_for_projection( TypeDefs::ImageEXPtr img, double fov_margin = 0.0001 );
    bool is_busy();
    void set_busy();
    void wait_free();
    void unset_busy();
    // template < typename T >
    int append_points_to_global_map( TypeDefs::PointCloudEXPtr pc_in,  std::vector< RGB_pt_ptr > *pts_added_vec = nullptr, int step = 1 );

    // TypeDefs::Vector3Type getRgbFromIntensity(double gray_value, int colormap_type);
    void render_with_a_image( TypeDefs::ImageEXPtr img_ptr, int if_select = 1 );
    void selection_points_for_projection(TypeDefs::ImageEXPtr image_pose, std::vector< std::shared_ptr< RGB_pts > > *pc_out_vec = nullptr,
                                          std::vector< cv::Point2f > *pc_2d_out_vec = nullptr, double minimum_dis = 5, int skip_step = 1,int use_all_pts = 0 );
    // void save_to_pcd( std::string dir_name, std::string file_name = std::string( "/rgb_pt" ) , int save_pts_with_views = 3);
    // void save_and_display_pointcloud( std::string dir_name = std::string( "/home/ziv/temp/" ), std::string file_name = std::string( "/rgb_pt" ) ,  int save_pts_with_views = 3);
    void render_pts_in_voxels( TypeDefs::ImageEXPtr img_ptr, std::vector< std::shared_ptr< RGB_pts > > &voxels_for_render, double obs_time = 0 );
    
    void merge(TypeDefs::RGBMapPtr &map_tofuse, TypeDefs::TransformType &T_wtofuse_wtarget,  int step=2);
    

  private:
    // friend class boost::serialization::access;
    // template < typename Archive >
    // void serialize( Archive &ar, const unsigned int version )
    // {
    //     boost::serialization::split_free( ar, *this, version );
    // }
};

void render_pts_in_voxels_mp( TypeDefs::ImageEXPtr  img_ptr, std::unordered_set< RGB_voxel_ptr > *voxels_for_render, const double &obs_time = 0 );

// template < typename Archive >
// inline void load( Archive &ar, Global_map &global_map, const unsigned int /*version*/ )
// {
//     Common_tools::Timer tim;
//     tim.tic();
//     int vector_size;
//     vector_size = global_map.m_rgb_pts_vec.size();
//     ar >> global_map.m_map_major_version;
//     ar >> global_map.m_map_minor_version;
//     ar >> global_map.m_minimum_pts_size;
//     ar >> global_map.m_voxel_resolution;
//     ar >> vector_size;
//     int grid_x, grid_y, grid_z, box_x, box_y, box_z;
//     scope_color( ANSI_COLOR_YELLOW_BOLD );
//     for ( int i = 0; i < vector_size; i++ )
//     {
//         // printf_line;
//         std::shared_ptr< RGB_pts > rgb_pt = std::make_shared< RGB_pts >();
//         ar >> *rgb_pt;
//         CV_Assert( rgb_pt->m_pt_index == global_map.m_rgb_pts_vec.size() );
//         global_map.m_rgb_pts_vec.push_back( rgb_pt );
//         grid_x = std::round( rgb_pt->m_pos[ 0 ] / global_map.m_minimum_pts_size );
//         grid_y = std::round( rgb_pt->m_pos[ 1 ] / global_map.m_minimum_pts_size );
//         grid_z = std::round( rgb_pt->m_pos[ 2 ] / global_map.m_minimum_pts_size );
//         box_x = std::round( rgb_pt->m_pos[ 0 ] / global_map.m_voxel_resolution );
//         box_y = std::round( rgb_pt->m_pos[ 1 ] / global_map.m_voxel_resolution );
//         box_z = std::round( rgb_pt->m_pos[ 2 ] / global_map.m_voxel_resolution );

//         if ( global_map.m_if_reload_init_voxel_and_hashed_pts )
//         {
//             // if init voxel and hashmap_3d_pts, comment this to save loading time if necessary.
//             global_map.m_hashmap_3d_pts.insert( grid_x, grid_y, grid_z, rgb_pt );
//             if ( !global_map.m_hashmap_voxels.if_exist( box_x, box_y, box_z ) )
//             {
//                 std::shared_ptr< RGB_Voxel > box_rgb = std::make_shared< RGB_Voxel >();
//                 global_map.m_hashmap_voxels.insert( box_x, box_y, box_z, box_rgb );
//             }
//             global_map.m_hashmap_voxels.m_map_3d_hash_map[ box_x ][ box_y ][ box_z ]->add_pt( rgb_pt );
//         }
//         if ( ( i % 10000 == 0 ) || ( i == vector_size - 1 ) )
//         {
//             cout << ANSI_DELETE_CURRENT_LINE << "Loading global map: " << ( i * 100.0 / (vector_size-1) ) << " %";
//             ANSI_SCREEN_FLUSH;
//         }
//     }
//     cout << endl;
//     cout << "Load offine global map cost: " << tim.toc() << " ms" << ANSI_COLOR_RESET << endl;
// }


}