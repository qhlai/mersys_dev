#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/fpfh.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <opencv2/opencv.hpp>
// #include <pcl/organized_point_cloud.h>
#include "include/lidar_camera_calib_3.hpp"
#include "ceres/ceres.h"
#include "include/common.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <opencv2/line_descriptor/descriptor.hpp>

// Data path
string image_file;
string pcd_file;
string result_file;

// Camera config
vector<double> camera_matrix;
vector<double> dist_coeffs;
double width;
double height;

// Calib config
bool use_rough_calib;
string calib_config_file;
// instrins matrix
Eigen::Matrix3d inner;
// Distortion coefficient
Eigen::Vector4d distor;
Eigen::Vector4d quaternion;
Eigen::Vector3d transation;


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;
  ros::Rate loop_rate(0.1);

  nh.param<string>("common/image_file", image_file, "");
  nh.param<string>("common/pcd_file", pcd_file, "");
  nh.param<string>("common/result_file", result_file, "");
  std::cout << "pcd_file path:" << pcd_file << std::endl;
  nh.param<vector<double>>("camera/camera_matrix", camera_matrix,
                           vector<double>());
  nh.param<vector<double>>("camera/dist_coeffs", dist_coeffs, vector<double>());
  nh.param<bool>("calib/use_rough_calib", use_rough_calib, false);
  nh.param<string>("calib/calib_config_file", calib_config_file, "");

  Calibration calibra(image_file, pcd_file, calib_config_file);
  calibra.fx_ = camera_matrix[0];
  calibra.cx_ = camera_matrix[2];
  calibra.fy_ = camera_matrix[4];
  calibra.cy_ = camera_matrix[5];
  calibra.k1_ = dist_coeffs[0];
  calibra.k2_ = dist_coeffs[1];
  calibra.p1_ = dist_coeffs[2];
  calibra.p2_ = dist_coeffs[3];
  calibra.k3_ = dist_coeffs[4];
  Eigen::Vector3d init_euler_angle =
      calibra.init_rotation_matrix_.eulerAngles(2, 1, 0);
  Eigen::Vector3d init_transation = calibra.init_translation_vector_;

  Vector6d calib_params;
  calib_params << init_euler_angle(0), init_euler_angle(1), init_euler_angle(2),
      init_transation(0), init_transation(1), init_transation(2);

  ROS_INFO_STREAM("Finish prepare!");
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  inner << calibra.fx_, 0.0, calibra.cx_, 0.0, calibra.fy_, calibra.cy_, 0.0,
      0.0, 1.0;
  distor << calibra.k1_, calibra.k2_, calibra.p1_, calibra.p2_;
  R = calibra.init_rotation_matrix_;
  T = calibra.init_translation_vector_;
  std::cout << "Initial rotation matrix:" << std::endl
            << calibra.init_rotation_matrix_ << std::endl;
  std::cout << "Initial translation:"
            << calibra.init_translation_vector_.transpose() << std::endl;
  // cv::Mat test_img = calibra.getProjectionImg(calib_params, false);
  cv::Mat depth_projection_img;

  
  calibra.projection_1(calib_params, calibra.raw_lidar_cloud_, Calibration::ProjectionType::DEPTH, true, depth_projection_img);
  
  // cv::imshow("After rough extrinsic", test_img);
  // cv::waitKey(0);

  cv::imshow("depth_projection_img", depth_projection_img);
  cv::waitKey(0);


  cv::Mat gray=depth_projection_img.clone();

  cv::Rect roi(150, 150, gray.cols-150, gray.rows-150);
  gray = gray(roi);  
  cv::Mat edges, dst;
  cv::Canny(gray, edges, 25, 200, 3);
  cv::imshow("edge", edges);
  cv::cvtColor(edges, dst, cv::COLOR_GRAY2BGR);

// void cv::HoughLinesP(
//     cv::InputArray image,        // 输入图像
//     cv::OutputArray lines,       // 输出线段集合
//     double rho,                  // rho 参数，表示极径的精度
//     double theta,                // theta 参数，表示极角的精度
//     int threshold,               // 投票阈值，用于确定线段
//     double minLineLength,        // 线段最小长度
//     double maxLineGap            // 线段最大间隙
// );
std::vector<cv::Vec4i> lines;
cv::HoughLinesP(edges, lines, 1, 2* CV_PI / 180, 100, 50, 30);
for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4i l = lines[i];
    cv::line(dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
}
imshow("HoughLinesP result", dst);
  //   // cv::Mat gray, binary;
  // // cv::cvtColor(test_img, gray, cv::COLOR_BGR2GRAY);
  // int gaussian_size = 5;
  // // cv::waitKey(0);
  // cv::GaussianBlur(gray, gray, cv::Size(gaussian_size, gaussian_size), 0,
  //                  0);
  // int length_threshold = 20;
  // float distance_threshold = 1.41421356f;
  // double canny_th1 = 20.0;//用于控制高阈值和低阈值之间的差异。通常情况下，建议将 threshold1 设置为整个灰度级范围的 1/3。
  // double canny_th2 = 200.0;//用于控制低阈值和噪声之间的差异。通常情况下，建议将 threshold2 设置为整个灰度级范围的 1/2。
  // int canny_aperture_size = 7;//可以设置为 3、5、7 等奇数值。较大的算子可以检测到更宽的边缘，但也更容易受到噪声的影响。
  // bool do_merge = false;

  // cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(length_threshold,
  // distance_threshold, canny_th1, canny_th2, canny_aperture_size,
  // do_merge);

  // std::vector<cv::Vec4f> lines_fld;
  //   fld->detect(gray, lines_fld);

  // cv::Mat line_image_fld(gray);
  // fld->drawSegments(line_image_fld, lines_fld);
  // imshow("FLD result", line_image_fld);

cv::waitKey(0);
  return 0;
}

// int main(int argc, char** argv)
// {
  
//   ros::init(argc, argv, "lidarCamCalib");
//   ros::NodeHandle nh;
//   ros::Rate loop_rate(0.1);

//   nh.param<string>("common/image_file", image_file, "");
//   nh.param<string>("common/pcd_file", pcd_file, "");
//   nh.param<string>("common/result_file", result_file, "");
//   std::cout << "pcd_file path:" << pcd_file << std::endl;
//   nh.param<vector<double>>("camera/camera_matrix", camera_matrix,
//                            vector<double>());
//   nh.param<vector<double>>("camera/dist_coeffs", dist_coeffs, vector<double>());
//   nh.param<bool>("calib/use_rough_calib", use_rough_calib, false);
//   nh.param<string>("calib/calib_config_file", calib_config_file, "");

//   Calibration calibra(image_file, pcd_file, calib_config_file);
//   calibra.fx_ = camera_matrix[0];
//   calibra.cx_ = camera_matrix[2];
//   calibra.fy_ = camera_matrix[4];
//   calibra.cy_ = camera_matrix[5];
//   calibra.k1_ = dist_coeffs[0];
//   calibra.k2_ = dist_coeffs[1];
//   calibra.p1_ = dist_coeffs[2];
//   calibra.p2_ = dist_coeffs[3];
//   calibra.k3_ = dist_coeffs[4];
//   Eigen::Vector3d init_euler_angle =
//       calibra.init_rotation_matrix_.eulerAngles(2, 1, 0);
//   Eigen::Vector3d init_transation = calibra.init_translation_vector_;

//   Vector6d calib_params;
//   calib_params << init_euler_angle(0), init_euler_angle(1), init_euler_angle(2),
//       init_transation(0), init_transation(1), init_transation(2);

//   ROS_INFO_STREAM("Finish prepare!");
//   Eigen::Matrix3d R;
//   Eigen::Vector3d T;
//   inner << calibra.fx_, 0.0, calibra.cx_, 0.0, calibra.fy_, calibra.cy_, 0.0,
//       0.0, 1.0;
//   distor << calibra.k1_, calibra.k2_, calibra.p1_, calibra.p2_;
//   R = calibra.init_rotation_matrix_;
//   T = calibra.init_translation_vector_;
//   std::cout << "Initial rotation matrix:" << std::endl
//             << calibra.init_rotation_matrix_ << std::endl;
//   std::cout << "Initial translation:"
//             << calibra.init_translation_vector_.transpose() << std::endl;
//   cv::Mat test_img = calibra.getProjectionImg(calib_params, false);
//   cv::imshow("After rough extrinsic", test_img);
//   cv::waitKey(0);

//   cv::Mat close_img;// = cv::Mat::zeros(height_, width_, CV_8UC3);
//   cv::morphologyEx(test_img,test_img, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));

//   // cv::imshow("After close", close_img);\


//     cv::Mat gray, binary;
//   cv::cvtColor(test_img, gray, cv::COLOR_BGR2GRAY);
//   int gaussian_size = 5;
//   // cv::waitKey(0);
//   cv::GaussianBlur(gray, gray, cv::Size(gaussian_size, gaussian_size), 0,
//                    0);
//  int length_threshold = 10;
//  float distance_threshold = 1.41421356f;
//  double canny_th1 = 20.0;
//  double canny_th2 = 150.0;
//  int canny_aperture_size = 3;
//  bool do_merge = false;

//  cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(length_threshold,
//  distance_threshold, canny_th1, canny_th2, canny_aperture_size,
//  do_merge);



//  std::vector<cv::Vec4f> lines_fld;
//   fld->detect(gray, lines_fld);
// //   for(int run_count = 0; run_count < 10; run_count++) {
// //  double freq = cv::getTickFrequency();
// //  lines_fld.clear();
// //  int64 start = cv::getTickCount();
// //  // Detect the lines with FLD

// //  fld->detect(gray, lines_fld);
// //  double duration_ms = double(cv::getTickCount() - start) * 1000 / freq;
// //  std::cout << "Elapsed time for FLD " << duration_ms << " ms." << std::endl;
// //  }
//  // Show found lines with FLD
//  cv::Mat line_image_fld(gray);
//  fld->drawSegments(line_image_fld, lines_fld);
//  imshow("FLD result", line_image_fld);
//   // 创建LSD
//   // cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
	
// 	// // 创建一个4维向量的vector，作为后续lsd的输出
//   //   std::vector<cv::Vec4f> lines_1;
//   //   // lsd检测，检测出的线特征以一个4维向量表示
//   //   // 向量中是线段起始端点和终点的坐标值
//   //   // [x1, y1, x2, y2], 1是起始点，2是终点
//   //   lsd->detect(test_img, lines_1);
//   //    // 显示线特征，把原图拷贝下，当然不拷贝也行
//   //   cv::Mat image_output = test_img.clone();
//   //   // 显示线特征用到lsd中的drawSegment函数
//   //   lsd->drawSegments(image_output, lines_1);

//   //   cv::imshow("1", image_output);
//   //   cv::waitKey(0);
 
//   // cv::Mat gray, binary;
//   // cv::cvtColor(test_img, gray, cv::COLOR_BGR2GRAY);
//   // cv::imshow("gray",gray);

//   // cv::Mat edge_image;
//   // // 存储PC图像边缘点的2D点云
//   // pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud;
//   // // calibra.edgeDetector(50, 200, gray, edge_image,pc_egde_cloud_);

//   // cv::Mat guass = cv::Mat::zeros(gray.cols, gray.rows, CV_8UC1);
//   // int gaussian_size = 5;
//   // // cv::waitKey(0);
//   // cv::GaussianBlur(gray, guass, cv::Size(gaussian_size, gaussian_size), 0,
//   //                  0);
//   // cv::Mat canny_result = cv::Mat::zeros(gray.rows, gray.cols, CV_8UC1);
//   // cv::Canny(guass, canny_result, 25, 25 * 3, 3,
//   //           true);
//   // // cv::waitKey(0);
//   // std::vector<std::vector<cv::Point>> contours;
//   // std::vector<cv::Vec4i> hierarchy;
//   // cv::findContours(canny_result, contours, hierarchy, cv::RETR_EXTERNAL,
//   //                  cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
//   // edge_image = cv::Mat::zeros(gray.rows, gray.cols, CV_8UC1);


//   // // cv::waitKey(0);

//   // // edge_cloud =
//   // //     pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//   // for (size_t i = 0; i < contours.size(); i++) {
//   //   if (contours[i].size() > 200) {
//   //     // cv::Mat debug_img = cv::Mat::zeros(height_, width_, CV_8UC1);
//   //     for (size_t j = 0; j < contours[i].size(); j++) {
//   //       pcl::PointXYZ p;
//   //       p.x = contours[i][j].x;
//   //       p.y = -contours[i][j].y;
//   //       p.z = 0;
//   //       edge_image.at<uchar>(-p.y, p.x) = 255;
//   //     }
//   //   }
//   // }
//   // // for (int x = 0; x < edge_image.cols; x++) {
//   // //   for (int y = 0; y < edge_image.rows; y++) {
//   // //     if (edge_image.at<uchar>(y, x) == 255) {
//   // //       pcl::PointXYZ p;
//   // //       p.x = x;
//   // //       p.y = -y;
//   // //       p.z = 0;
//   // //       edge_cloud->points.push_back(p);
//   // //     }
//   // //   }
//   // // }
//   // // edge_cloud->width = edge_cloud->points.size();
//   // // edge_cloud->height = 1;

//   // cv::imshow("edge_image",edge_image);

//   cv::waitKey(0);


//   return 0;
// }