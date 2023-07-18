
#include "image_ex.hpp"
#include "msgs/msg_image.hpp"
namespace colive {

auto Image_ex::img_less::operator ()(const ImageEXPtr a, const ImageEXPtr b) const ->bool
{
    if(a->GetClientID() < b->GetClientID())
        return true;
    else if(a->GetClientID() > b->GetClientID())
        return false;
    else {
        return a->GetFrameID() < b->GetFrameID();
    }
}
Image_ex::Image_ex(MsgImage msg){
    id_= msg.id_;
    // pts_cloud=msg.pts_cloud;
    timestamp_=msg.timestamp_;
    // T_s_w_=msg.T_s_w_;
    SetPoseTsw(msg.T_s_w_);
   img_=msg.img_;
   set_intrinsic(msg.m_cam_K);

#if (CV_MAJOR_VERSION >= 4)
    cv::cvtColor(img_, m_img_gray, cv::COLOR_RGB2GRAY);
#else
    cv::cvtColor(img_, m_img_gray, CV_RGB2GRAY);
#endif

    // m_cam_K=msg.m_cam_K;
//    memcpy(intrinsic,msg.intrinsic,sizeof(msg.intrinsic));
    // map_=map;
}
// // auto PointCloud_ex::GetPoseTws()->TransformType {
// //     std::unique_lock<std::mutex> lock(mtx_pose_);
// //     return T_w_s_;
// // }
auto Image_ex::ConvertToMsg(MsgImage &msg, bool is_update, size_t cliend_id)->void{


//     // std::unique_lock<std::mutex> lock_conn(mMutexConnections);
//     // std::unique_lock<std::mutex> lock_feat(mMutexFeatures);
//     // std::unique_lock<std::mutex> lock_pose(mMutexPose);
    // msg.downSample
    msg.id_ = id_;   // mnid clientid  
    msg.timestamp_ = timestamp_;//std::chrono::system_clock::now();
    msg.T_s_w_=GetPoseTsw();
    msg.img_ = img_;
    // memcpy(msg.intrinsic,intrinsic,sizeof(intrinsic));
    msg.m_cam_K=m_cam_K;
    // intrinsic
    // msg.pts_cloud=

}

// // void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
// // {
// //     V3D p_body(pi->x, pi->y, pi->z);
// //     V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

// //     po->x = p_global(0);
// //     po->y = p_global(1);
// //     po->z = p_global(2);
// //     po->intensity = pi->intensity;
// // }
// // PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
// // int size = laserCloudFullRes->points.size();
// // PointCloudXYZI::Ptr laserCloudWorld( \
// //                 new PointCloudXYZI(size, 1));

// // for (int i = 0; i < size; i++)
// // {
// //     RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
// //                         &laserCloudWorld->points[i]);
// // }
//         // int size = pc->pts_cloud.points.size();
//         // TransformType T_w_s_befcorrection = pc->GetPoseTws();
//         // TransformType T_w_s_corrected = T_wtarget_wtofuse * T_w_s_befcorrection;
//         // kf->SetPoseTws(T_w_s_corrected);
//         // kf->velocity_ = T_wtarget_wtofuse.block<3,3>(0,0) * kf->velocity_;

auto Image_ex::SetImage(ImagePtr img)->void {
    std::unique_lock<std::mutex> lock(mtx_in_);

    img_=*img;

}
auto Image_ex::SetImage(Image &img)->void {
    std::unique_lock<std::mutex> lock(mtx_in_);

    img_=img;

}

auto Image_ex::set_intrinsic(Eigen::Matrix3d & camera_K)->void{
    m_cam_K = camera_K;
    m_if_have_set_intrinsic = true;
    fx = camera_K(0, 0);
    fy = camera_K(1, 1);
    cx = camera_K(0, 2);
    cy = camera_K(1, 2);
    m_gama_para(0) = 1.0;
    m_gama_para(1) = 0.0;
}

auto Image_ex::project_3d_to_2d(const Vector3Type & in_pt, double &u, double &v, const double &scale)->bool {

    // should have set image
    if (m_if_have_set_intrinsic )
    {
        std::cout<< COUTWARN << "You have not set the intrinsic yet!!!" << std::endl;
        // while (1)
        // {} ;
        return true;
    }
        Vector3Type pt_cam;
        Vector3Type pt_w = in_pt;
        auto Twc = GetPoseTwc();

        pt_cam = (Twc.rotation() * pt_w + Twc.translation());
        if (pt_cam[2] < 0.001)
        {
            return false;
        }

        u = (pt_cam[0] * fx / pt_cam[2] + cx) * scale;
        v = (pt_cam[1] * fy / pt_cam[2] + cy) * scale;
        
        return true;
}
auto Image_ex::if_2d_points_available(const double &u, const double &v, const double &scale, float fov_mar)->bool
{
    float used_fov_margin = m_fov_margin;
    if (fov_mar > 0.0)
    {
        used_fov_margin = fov_mar;
    }
    if ((u / scale >= (used_fov_margin * img_.cols + 1)) && (std::ceil(u / scale) < ((1 - used_fov_margin) * img_.cols)) &&
        (v / scale >= (used_fov_margin * img_.rows + 1)) && (std::ceil(v / scale) < ((1 - used_fov_margin) * img_.rows)))
    {
        return true;
    }
    else
    {
        return false;
    }
}
template<typename T>
inline T getSubPixel(cv::Mat & mat, const double & row, const  double & col, double pyramid_layer = 0)
{
	int floor_row = floor(row);
	int floor_col = floor(col);
	double frac_row = row - floor_row;
	double frac_col = col - floor_col;
	int ceil_row = floor_row + 1;
	int ceil_col = floor_col + 1;
    if (pyramid_layer != 0)
    {
        int pos_bias = pow(2, pyramid_layer - 1);
        floor_row -= pos_bias;
        floor_col -= pos_bias;
        ceil_row += pos_bias;
        ceil_row += pos_bias;
    }
    return ((1.0 - frac_row) * (1.0 - frac_col) * (T)mat.ptr<T>(floor_row)[floor_col]) +
               (frac_row * (1.0 - frac_col) * (T)mat.ptr<T>(ceil_row)[floor_col]) +
               ((1.0 - frac_row) * frac_col * (T)mat.ptr<T>(floor_row)[ceil_col]) +
               (frac_row * frac_col * (T)mat.ptr<T>(ceil_row)[ceil_col]);
}
auto Image_ex::get_rgb(double &u, double v, int layer, Vector3Type *rgb_dx, Vector3Type *rgb_dy)->Vector3Type{
    cv::Vec3b rgb = getSubPixel< cv::Vec3b >( img_, v, u, layer );
    if ( rgb_dx != nullptr )
    {
        cv::Vec3b rgb_left = getSubPixel< cv::Vec3b >( img_, v, u - 1, layer );
        cv::Vec3b rgb_right = getSubPixel< cv::Vec3b >( img_, v, u + 1, layer );
        cv::Vec3b cv_rgb_dx = rgb_right - rgb_left;
        *rgb_dx = Vector3Type( cv_rgb_dx( 0 ), cv_rgb_dx( 1 ), cv_rgb_dx( 2 ) );
    }
    if ( rgb_dy != nullptr )
    {
        cv::Vec3b rgb_down = getSubPixel< cv::Vec3b >( img_, v - 1, u, layer );
        cv::Vec3b rgb_up = getSubPixel< cv::Vec3b >( img_, v + 1, u, layer );
        cv::Vec3b cv_rgb_dy = rgb_up - rgb_down;
        *rgb_dy = Vector3Type( cv_rgb_dy( 0 ), cv_rgb_dy( 1 ), cv_rgb_dy( 2 ) );
    }
    return Vector3Type( rgb( 0 ), rgb( 1 ), rgb( 2 ) );
}
auto Image_ex::get_rgb(const double &u, const double &v, int &r, int &g, int &b)->bool
{
    r = img_.at<cv::Vec3b>(v, u)[2];
    g = img_.at<cv::Vec3b>(v, u)[1];
    b = img_.at<cv::Vec3b>(v, u)[0];
    return true;
}
auto Image_ex::get_grey_color( double &u, double &v, int layer )->double 
{
    double val = 0;

    if ( layer == 0 )
    {
        double gray_val = getSubPixel< uchar >( img_, v, u );
        return gray_val;
    }
    else
    {
        // TODO
        while ( 1 )
        {
            cout << "To be process here" << __LINE__ << endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        };
    }

    return m_gama_para( 0 ) * val + m_gama_para( 1 );
}
auto Image_ex::image_equalize(cv::Mat &img, int amp)->void
{
    cv::Mat img_temp;
    cv::Size eqa_img_size = cv::Size(std::max(img.cols * 32.0 / 640, 4.0), std::max(img.cols * 32.0 / 640, 4.0));
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_img_size);
    // Equalize gray image.
    clahe->apply(img, img_temp);
    img = img_temp;
}
inline void image_equalize(cv::Mat &img, int amp)
{
    cv::Mat img_temp;
    cv::Size eqa_img_size = cv::Size(std::max(img.cols * 32.0 / 640, 4.0), std::max(img.cols * 32.0 / 640, 4.0));
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_img_size);
    // Equalize gray image.
    clahe->apply(img, img_temp);
    img = img_temp;
}

inline cv::Mat equalize_color_image_Ycrcb(cv::Mat &image)
{
    cv::Mat hist_equalized_image;
    cv::cvtColor(image, hist_equalized_image, cv::COLOR_BGR2YCrCb);

    //Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
    std::vector<cv::Mat> vec_channels;
    cv::split(hist_equalized_image, vec_channels);

    //Equalize the histogram of only the Y channel
    // cv::equalizeHist(vec_channels[0], vec_channels[0]);
    image_equalize( vec_channels[0], 1 );
    cv::merge(vec_channels, hist_equalized_image);
    cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);
    return hist_equalized_image;
}

auto Image_ex::image_equalize()->void
{
    image_equalize(m_img_gray, 3.0);
    // cv::imshow("before", m_img.clone());
    img_ = equalize_color_image_Ycrcb(img_);
    // cv::imshow("After", m_img.clone());
}

auto  Image_ex::project_3d_point_in_this_img(const Vector3Type & in_pt, double &u, double &v, pcl::PointXYZRGB *rgb_pt, double intrinsic_scale)->bool
{
    if (project_3d_to_2d(in_pt,  u, v, intrinsic_scale) == false)
    {
        return false;
    }
    if (if_2d_points_available(u, v, intrinsic_scale) == false)
    {
        // printf_line;
        return false;
    }
    if (rgb_pt != nullptr)
    {
        int r = 0;
        int g = 0;
        int b = 0;
        get_rgb(u, v, r, g, b);
        rgb_pt->x = in_pt[0];
        rgb_pt->y = in_pt[1];
        rgb_pt->z = in_pt[2];
        rgb_pt->r = r;
        rgb_pt->g = g;
        rgb_pt->b = b;
        rgb_pt->a = 255;
    }
    return true;
}


}
// auto PointCloud_ex::SetPointCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc)->void {
//     std::unique_lock<std::mutex> lock(mtx_in_);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZI>);
//     pointcloud_convert(pc,pc_out);    
//     // pointcloud_convert(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out);
//     pts_cloud=*pc_out;
// }

// auto PointCloud_ex::pointcloud_convert(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out)->void{

//     for (size_t i = 0; i < pc_in->size(); ++i)
//     {
//     pcl::PointXYZI point;
//     point.x = pc_in->points[i].x;
//     point.y = pc_in->points[i].y;
//     point.z = pc_in->points[i].z;
//     point.intensity = pc_in->points[i].intensity;
//     pc_out->push_back(point);
//     }
// }
// auto PointCloud_ex::pointcloud_convert(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_out)->void{

//     for (size_t i = 0; i < pc_in->size(); ++i)
//     {
//     pcl::PointXYZINormal point;
//     point.x = pc_in->points[i].x;
//     point.y = pc_in->points[i].y;
//     point.z = pc_in->points[i].z;
//     point.intensity = pc_in->points[i].intensity;
//     pc_out->push_back(point);
//     }
// }
// // TODO：
// // 尽量不要把所有的点都改位置，就改pointcloudex 的 一个转移矩阵就行了
// auto PointCloud_ex::pointcloud_transform(TransformType T)->void{
//     uint32_t size = pts_cloud.points.size();
//     PointCloud pc;
//     pc.resize(size);
//     for (int i = 0; i < size; i++){
//         // PointType pi=pts_cloud.points[i];
//         Vector3Type p_old(pts_cloud.points[i].x, pts_cloud.points[i].y, pts_cloud.points[i].z);

//         Vector3Type p_new(0,0,0);
//         pc.points[i].x=  T(0, 0) * p_old[0] + T(0, 1) * p_old[0] + T(0, 2) * p_old[0] + T(0, 3);
//         pc.points[i].y = T(1, 0) * p_old[1] + T(1, 1) * p_old[1] + T(1, 2) * p_old[1] + T(1, 3);
//         pc.points[i].z = T(2, 0) * p_old[2] + T(2, 1) * p_old[2] + T(2, 2) * p_old[2] + T(2, 3);

//         pc.points[i].intensity =pts_cloud.points[i].intensity;

//     }
//     pts_cloud=pc;
//     // PointCloudXYZI::Ptr laserCloudWorld( \
//     //             new PointCloudXYZI(size, 1));

// }
// auto PointCloud_ex::convert_to_tf()->TransformType{
//     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//     T.translate(pos_w);
//     T.rotate(quan_);
//     // T_w_s_.block<3, 3>(0, 0)=T.rotation();
//     // T_w_s_.block<3, 1>(0, 3) = T.translation();
//     return T;
// }
// auto PointCloud_ex::pc_less::operator ()(const PointCloudEXPtr a, const PointCloudEXPtr b) const ->bool
// {
//     if(a->id_.second < b->id_.second)
//         return true;
//     else if(a->id_.second > b->id_.second)
//         return false;
//     else {
//         return a->id_.first < b->id_.first;
//     }
// }

