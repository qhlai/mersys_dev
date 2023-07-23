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

int main(int argc, char** argv)
{
  // Load input point cloud with intensity information
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI>("/home/lqh/ros/r3live_ws/output/frames/pcd_large/0/1627807357.700470.pcd", *cloud);

  // Compute normals
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.05);
  ne.compute(*normals);

  // Compute principal curvatures
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZI, pcl::Normal, pcl::PrincipalCurvatures> pc;
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
  pc.setInputCloud(cloud);
  pc.setInputNormals(normals);
  pc.setSearchMethod(tree);
  pc.setRadiusSearch(0.05);
  pc.compute(*principal_curvatures);

  // Extract line features using curvature and fitting
  pcl::PointCloud<pcl::PointXYZI>::Ptr line_features(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < cloud->size(); i++)
  {
    if (std::abs(principal_curvatures->points[i].pc1) < 0.1 && std::abs(principal_curvatures->points[i].pc2) < 0.1)
    {
      pcl::PointXYZI point = cloud->points[i];
      line_features->push_back(point);
    }
  }

  // Fit lines to line features using RANSAC
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(line_features);
  seg.segment(*inliers, *coefficients);

  // Visualize results
  pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");



  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> input_cloud_intensity_handler (cloud, "intensity");
  viewer.addPointCloud<pcl::PointXYZI> (cloud, input_cloud_intensity_handler , "input_cloud");
  // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");

  // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> line_features_intensity_handler (line_features, "intensity");
  // viewer.addPointCloud<pcl::PointXYZI>(line_features, line_features_intensity_handler, "line_features");
  // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line_features");
// pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
pcl::PointXYZ pt1(coefficients->values[0],coefficients->values[1], coefficients->values[2]);
pcl::PointXYZ pt2(coefficients->values[3], coefficients->values[4], coefficients->values[5]);

// projection

//   pcl::ModelCoefficients::Ptr coefficients_pro (new pcl::ModelCoefficients ());
//   coefficients_pro->values.resize (4);
//   coefficients_pro->values[0] = 1.0;
//   coefficients_pro->values[1] = 0.0;
//   coefficients_pro->values[2] = 0.0;
//   coefficients_pro->values[3] = 0.0;
//   pcl::ProjectInliers<pcl::PointXYZI> proj;
//   proj.setModelType (pcl::SACMODEL_PLANE);
//   proj.setInputCloud (cloud);
//   proj.setModelCoefficients (coefficients_pro);


//   // Project point cloud onto plane
//  pcl::OrganizedPointCloud<pcl::PointXYZI>::Ptr organized_cloud (new pcl::OrganizedPointCloud<pcl::PointXYZI> ());
//   proj.filter (*cloud_projected);

  // // Create OpenCV image from projected point cloud
  // cv::Mat img (cloud_projected->height, cloud_projected->width, CV_8UC3);
  // for (int y = 0; y < img.rows; ++y)
  // {
  //   for (int x = 0; x < img.cols; ++x)
  //   {
  //     pcl::PointXYZI& pt = cloud_projected->at (x, y);
  //     img.at<cv::Vec3b> (y, x) = cv::Vec3b (pt.intensity * 255, pt.intensity * 255, pt.intensity * 255);
  //   }
  // }

  // // Save image to file
  // cv::imwrite ("/home/lqh/ros/r3live_ws/output/frames/projected_image.png", img);

// viewer.addLine(pt1, pt2, 0, 0, 255,"line");  // Add line to viewer with blue color
  viewer.spin();

  return 0;
}