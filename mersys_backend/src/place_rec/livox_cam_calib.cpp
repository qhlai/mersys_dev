// #include "place_rec.hpp"
#include "livox_cam_calib.hpp"
// #include "scancontext/Scancontext.h"

// C++
#include <iostream>
#include <mutex>
#include <eigen3/Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "config_backend.hpp"
#include "pointcloud_ex.hpp"
#include "image_ex.hpp"

#include "calib/common.h"
#include "calib/BA/mypcl.hpp"
#include "calib/BA/ba.hpp"
#include "calib/BA/tools.hpp"

namespace mersys
{

class PointCloud_Calib{
    public:
        std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_edge_cloud_; // 存储平面相交得到的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_; // 存储原始点云
};


  void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*>& feat_map,
                 pcl::PointCloud<pcl::PointXYZI>& pl_feat,
                 double voxel_size, float eigen_ratio)
  {
    float loc_xyz[3];
    printf("total point size %ld\n", pl_feat.points.size());
    for(pcl::PointXYZI& p_c: pl_feat.points)
    {
      Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);

      for(int j = 0; j < 3; j++)
      {
        loc_xyz[j] = pvec_orig[j] / voxel_size;
        if(loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
      }

      VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
      auto iter = feat_map.find(position);
      if(iter != feat_map.end())
      {
        iter->second->all_points.push_back(pvec_orig);
        iter->second->vec_orig.push_back(pvec_orig);        
        iter->second->sig_orig.push(pvec_orig);
      }
      else
      {
        OCTO_TREE_ROOT* ot = new OCTO_TREE_ROOT(eigen_ratio);
        ot->all_points.push_back(pvec_orig);
        ot->vec_orig.push_back(pvec_orig);
        ot->sig_orig.push(pvec_orig);
        ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
        ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
        ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
        ot->quater_length = voxel_size / 4.0;
        ot->layer = 0;
        feat_map[position] = ot;
      }
    }
  }
  void mergePlane(std::vector<Plane*>& origin_list, std::vector<Plane*>& merge_list)
  {
    for(size_t i = 0; i < origin_list.size(); i++)
      origin_list[i]->id = 0; // 初始化

    int current_id = 1; // 平面id
    for(auto iter = origin_list.end() - 1; iter != origin_list.begin(); iter--)
    {
      for(auto iter2 = origin_list.begin(); iter2 != iter; iter2++)
      {
        Eigen::Vector3d normal_diff = (*iter)->normal - (*iter2)->normal; // 发向量同向
        Eigen::Vector3d normal_add = (*iter)->normal + (*iter2)->normal; // 发向量反向
        double dis1 = fabs((*iter)->normal(0) * (*iter2)->center(0) +
                           (*iter)->normal(1) * (*iter2)->center(1) +
                           (*iter)->normal(2) * (*iter2)->center(2) + (*iter)->d);
        double dis2 = fabs((*iter2)->normal(0) * (*iter)->center(0) +
                           (*iter2)->normal(1) * (*iter)->center(1) +
                           (*iter2)->normal(2) * (*iter)->center(2) + (*iter2)->d);
        if(normal_diff.norm() < 0.2 || normal_add.norm() < 0.2) // 11.3度
          if(dis1 < 0.05 && dis2 < 0.05)
          {
            if((*iter)->id == 0 && (*iter2)->id == 0)
            {
              (*iter)->id = current_id;
              (*iter2)->id = current_id;
              current_id++;
            }
            else if((*iter)->id == 0 && (*iter2)->id != 0)
              (*iter)->id = (*iter2)->id;
            else if((*iter)->id != 0 && (*iter2)->id == 0)
              (*iter2)->id = (*iter)->id;
          }
      }
    }

    std::vector<int> merge_flag;
    for(size_t i = 0; i < origin_list.size(); i++)
    {
      auto it = std::find(merge_flag.begin(), merge_flag.end(), origin_list[i]->id);
      if(it != merge_flag.end()) continue; // 已经merge过的平面，直接跳过
      
      if(origin_list[i]->id == 0) // 没有merge的平面
      {
        if(origin_list[i]->points_size > 100)
          merge_list.push_back(origin_list[i]);
        continue;
      }

      Plane* merge_plane = new Plane;
      (*merge_plane) = (*origin_list[i]);
      for(size_t j = 0; j < origin_list.size(); j++)
      {
        if(i == j) continue;
        if(origin_list[i]->id != 0)
          if(origin_list[j]->id == origin_list[i]->id)
            for(auto pv: origin_list[j]->plane_points)
              merge_plane->plane_points.push_back(pv); // 跟当前平面id相同的都merge
      }

      merge_plane->covariance = Eigen::Matrix3d::Zero();
      merge_plane->center = Eigen::Vector3d::Zero();
      merge_plane->normal = Eigen::Vector3d::Zero();
      merge_plane->points_size = merge_plane->plane_points.size();
      merge_plane->radius = 0;
      for(auto pv: merge_plane->plane_points)
      {
        merge_plane->covariance += pv * pv.transpose();
        merge_plane->center += pv;
      }
      merge_plane->center = merge_plane->center / merge_plane->points_size;
      merge_plane->covariance = merge_plane->covariance / merge_plane->points_size -
                                merge_plane->center * merge_plane->center.transpose();
      Eigen::EigenSolver<Eigen::Matrix3d> es(merge_plane->covariance);
      Eigen::Matrix3cd evecs = es.eigenvectors();
      Eigen::Vector3cd evals = es.eigenvalues();
      Eigen::Vector3d evalsReal;
      evalsReal = evals.real();
      Eigen::Matrix3f::Index evalsMin, evalsMax;
      evalsReal.rowwise().sum().minCoeff(&evalsMin);
      evalsReal.rowwise().sum().maxCoeff(&evalsMax);
      merge_plane->id = origin_list[i]->id;
      merge_plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
      merge_plane->min_eigen_value = evalsReal(evalsMin);
      merge_plane->radius = sqrt(evalsReal(evalsMax));
      merge_plane->d = -(merge_plane->normal(0) * merge_plane->center(0) +
                        merge_plane->normal(1) * merge_plane->center(1) +
                        merge_plane->normal(2) * merge_plane->center(2));
      merge_plane->p_center.x = merge_plane->center(0);
      merge_plane->p_center.y = merge_plane->center(1);
      merge_plane->p_center.z = merge_plane->center(2);
      merge_plane->p_center.normal_x = merge_plane->normal(0);
      merge_plane->p_center.normal_y = merge_plane->normal(1);
      merge_plane->p_center.normal_z = merge_plane->normal(2);
      merge_plane->is_plane = true;
      merge_flag.push_back(merge_plane->id);
      merge_list.push_back(merge_plane);
    }
  }
  void projectLine(const Plane* plane1, const Plane* plane2, std::vector<Eigen::Vector3d>& line_point)
  {
    float theta = plane1->normal.dot(plane2->normal);
    if(!(theta > mersys_params::fusion::theta_max_ && theta < mersys_params::fusion::theta_min_)) return;

    Eigen::Vector3d c1 = plane1->center;
    Eigen::Vector3d c2 = plane2->center;
    Eigen::Vector3d n1 = plane1->normal;
    Eigen::Vector3d n2 = plane2->normal;

    Eigen::Matrix3d A;
    Eigen::Vector3d d = n1.cross(n2).normalized();
    A.row(0) = n1.transpose();
    A.row(1) = d.transpose();
    A.row(2) = n2.transpose();
    Eigen::Vector3d b(n1.dot(c1), d.dot(c1), n2.dot(c2));
    Eigen::Vector3d O = A.colPivHouseholderQr().solve(b);

    double c1_to_line = (c1 - O).norm();
    double c2_to_line = ((c2 - O) - (c2 - O).dot(d) * d).norm();

    if(c1_to_line/c2_to_line > 8 || c2_to_line/c1_to_line > 8) return;
    
    if(plane1->points_size < plane2->points_size)
      for(auto pt: plane1->plane_points)
      {
        Eigen::Vector3d p = (pt - O).dot(d) * d + O;
        line_point.push_back(p);
      }
    else
      for(auto pt: plane2->plane_points)
      {
        Eigen::Vector3d p = (pt - O).dot(d) * d + O;
        line_point.push_back(p);
      }
    
    return;
  }

  void estimate_edge(std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*>& surf_map,pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_edge_cloud_)
  {
    ros::Rate loop(500);
    lidar_edge_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
    {
      std::vector<Plane*> plane_list;
      std::vector<Plane*> merge_plane_list;
      iter->second->get_plane_list(plane_list);

      if(plane_list.size() > 1)
      {
        pcl::KdTreeFLANN<pcl::PointXYZI> kd_tree;
        pcl::PointCloud<pcl::PointXYZI> input_cloud;
        for(auto pv: iter->second->all_points)
        {
          pcl::PointXYZI p;
          p.x = pv(0); p.y = pv(1); p.z = pv(2);
          input_cloud.push_back(p);
        }
        kd_tree.setInputCloud(input_cloud.makeShared());
        mergePlane(plane_list, merge_plane_list);
        if(merge_plane_list.size() <= 1) continue;

        for(auto plane: merge_plane_list)
        {
          pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
          std::vector<unsigned int> colors;
          colors.push_back(static_cast<unsigned int>(rand() % 255));
          colors.push_back(static_cast<unsigned int>(rand() % 255));
          colors.push_back(static_cast<unsigned int>(rand() % 255));
          for(auto pv: plane->plane_points)
          {
            pcl::PointXYZRGB pi;
            pi.x = pv[0]; pi.y = pv[1]; pi.z = pv[2];
            pi.r = colors[0]; pi.g = colors[1]; pi.b = colors[2];
            color_cloud.points.push_back(pi);
          }
          // sensor_msgs::PointCloud2 dbg_msg;
          // pcl::toROSMsg(color_cloud, dbg_msg);
          // dbg_msg.header.frame_id = "camera_init";
          // pub_plane.publish(dbg_msg);
          loop.sleep();
        }

        for(size_t p1_index = 0; p1_index < merge_plane_list.size()-1; p1_index++)
          for(size_t p2_index = p1_index+1; p2_index < merge_plane_list.size(); p2_index++)
          {
            std::vector<Eigen::Vector3d> line_point;
            projectLine(merge_plane_list[p1_index], merge_plane_list[p2_index], line_point);
            
            if(line_point.size() == 0) break;

            pcl::PointCloud<pcl::PointXYZI> line_cloud, debug_cloud;

            for(size_t j = 0; j < line_point.size(); j++)
            {
              pcl::PointXYZI p;
              p.x = line_point[j][0]; p.y = line_point[j][1]; p.z = line_point[j][2];
              // debug_cloud.points.push_back(p);
              int K = 5;
              // 创建两个向量，分别存放近邻的索引值、近邻的中心距
              std::vector<int> pointIdxNKNSearch(K);
              std::vector<float> pointNKNSquaredDistance(K);
              if(kd_tree.nearestKSearch(p, K, pointIdxNKNSearch, pointNKNSquaredDistance) == K)
              {
                Eigen::Vector3d tmp(input_cloud.points[pointIdxNKNSearch[K-1]].x,
                                    input_cloud.points[pointIdxNKNSearch[K-1]].y,
                                    input_cloud.points[pointIdxNKNSearch[K-1]].z);
                // if(pointNKNSquaredDistance[K-1] < 0.01)
                if((tmp - line_point[j]).norm() < 0.05)
                {
                  line_cloud.points.push_back(p);
                  lidar_edge_cloud_->points.push_back(p);
                }
              }
            }
            // sensor_msgs::PointCloud2 dbg_msg;
            // pcl::toROSMsg(line_cloud, dbg_msg);
            // dbg_msg.header.frame_id = "camera_init";
            // pub_edge.publish(dbg_msg);
            // pcl::toROSMsg(debug_cloud, dbg_msg);
            // dbg_msg.header.frame_id = "camera_init";
            // pub_color_cloud.publish(dbg_msg);
            loop.sleep();
          }
      }
    }
  }
  void calcLine(const std::vector<SinglePlane>& plane_lists, const double voxel_size,
                const Eigen::Vector3d origin,
                std::vector<pcl::PointCloud<pcl::PointXYZI>>& edge_cloud_lists)
  {
    if(plane_lists.size() >= 2 && plane_lists.size() <= mersys_params::fusion::plane_max_size_)
    {
      pcl::PointCloud<pcl::PointXYZI> temp_line_cloud;
      for(size_t plane_idx1 = 0; plane_idx1 < plane_lists.size() - 1; plane_idx1++)
      {
        for(size_t plane_idx2 = plane_idx1 + 1; plane_idx2 < plane_lists.size(); plane_idx2++)
        {
          float a1 = plane_lists[plane_idx1].normal[0];
          float b1 = plane_lists[plane_idx1].normal[1];
          float c1 = plane_lists[plane_idx1].normal[2];
          float x1 = plane_lists[plane_idx1].p_center.x;
          float y1 = plane_lists[plane_idx1].p_center.y;
          float z1 = plane_lists[plane_idx1].p_center.z;
          float a2 = plane_lists[plane_idx2].normal[0];
          float b2 = plane_lists[plane_idx2].normal[1];
          float c2 = plane_lists[plane_idx2].normal[2];
          float x2 = plane_lists[plane_idx2].p_center.x;
          float y2 = plane_lists[plane_idx2].p_center.y;
          float z2 = plane_lists[plane_idx2].p_center.z;
          float theta = a1 * a2 + b1 * b2 + c1 * c2;
          float point_dis_threshold = 0.00;
          if(theta > mersys_params::fusion::theta_max_ && theta < mersys_params::fusion::theta_min_)
          {
            if(plane_lists[plane_idx1].cloud.size() > 0 ||
               plane_lists[plane_idx2].cloud.size() > 0)
            {
              float matrix[4][5];
              matrix[1][1] = a1; matrix[1][2] = b1; matrix[1][3] = c1;
              matrix[1][4] = a1 * x1 + b1 * y1 + c1 * z1;
              matrix[2][1] = a2; matrix[2][2] = b2; matrix[2][3] = c2;
              matrix[2][4] = a2 * x2 + b2 * y2 + c2 * z2;

              std::vector<Eigen::Vector3d> points;
              Eigen::Vector3d point;
              matrix[3][1] = 1; matrix[3][2] = 0; matrix[3][3] = 0;
              matrix[3][4] = origin[0];
              calc<float>(matrix, point);
              if(point[0] >= origin[0] - point_dis_threshold &&
                 point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                 point[1] >= origin[1] - point_dis_threshold &&
                 point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                 point[2] >= origin[2] - point_dis_threshold &&
                 point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

              matrix[3][1] = 0; matrix[3][2] = 1; matrix[3][3] = 0;
              matrix[3][4] = origin[1];
              calc<float>(matrix, point);
              if(point[0] >= origin[0] - point_dis_threshold &&
                 point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                 point[1] >= origin[1] - point_dis_threshold &&
                 point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                 point[2] >= origin[2] - point_dis_threshold &&
                 point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

              matrix[3][1] = 0; matrix[3][2] = 0; matrix[3][3] = 1;
              matrix[3][4] = origin[2];
              calc<float>(matrix, point);
              if(point[0] >= origin[0] - point_dis_threshold &&
                 point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                 point[1] >= origin[1] - point_dis_threshold &&
                 point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                 point[2] >= origin[2] - point_dis_threshold &&
                 point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

              matrix[3][1] = 1; matrix[3][2] = 0; matrix[3][3] = 0;
              matrix[3][4] = origin[0] + voxel_size;
              calc<float>(matrix, point);
              if(point[0] >= origin[0] - point_dis_threshold &&
                 point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                 point[1] >= origin[1] - point_dis_threshold &&
                 point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                 point[2] >= origin[2] - point_dis_threshold &&
                 point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

              matrix[3][1] = 0; matrix[3][2] = 1; matrix[3][3] = 0;
              matrix[3][4] = origin[1] + voxel_size;
              calc<float>(matrix, point);
              if(point[0] >= origin[0] - point_dis_threshold &&
                 point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                 point[1] >= origin[1] - point_dis_threshold &&
                 point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                 point[2] >= origin[2] - point_dis_threshold &&
                 point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

              matrix[3][1] = 0; matrix[3][2] = 0; matrix[3][3] = 1;
              matrix[3][4] = origin[2] + voxel_size;
              calc<float>(matrix, point);
              if(point[0] >= origin[0] - point_dis_threshold &&
                 point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                 point[1] >= origin[1] - point_dis_threshold &&
                 point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                 point[2] >= origin[2] - point_dis_threshold &&
                 point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

              if(points.size() == 2)
              {
                pcl::PointCloud<pcl::PointXYZI> edge_clouds;
                pcl::PointXYZ p1(points[0][0], points[0][1], points[0][2]);
                pcl::PointXYZ p2(points[1][0], points[1][1], points[1][2]);
                float length = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                                    pow(p1.z - p2.z, 2));
                // 指定近邻个数
                int K = 1;
                // 创建两个向量，分别存放近邻的索引值、近邻的中心距
                std::vector<int> pointIdxNKNSearch1(K);
                std::vector<float> pointNKNSquaredDistance1(K);
                std::vector<int> pointIdxNKNSearch2(K);
                std::vector<float> pointNKNSquaredDistance2(K);
                pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree1(
                  new pcl::search::KdTree<pcl::PointXYZI>());
                pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree2(
                  new pcl::search::KdTree<pcl::PointXYZI>());
                kdtree1->setInputCloud(plane_lists[plane_idx1].cloud.makeShared());
                kdtree2->setInputCloud(plane_lists[plane_idx2].cloud.makeShared());
                for(float inc = 0; inc <= length; inc += 0.01)
                {
                  pcl::PointXYZI p;
                  p.x = p1.x + (p2.x - p1.x) * inc / length;
                  p.y = p1.y + (p2.y - p1.y) * inc / length;
                  p.z = p1.z + (p2.z - p1.z) * inc / length;
                  p.intensity = 100;
                  if((kdtree1->nearestKSearch(p, K, pointIdxNKNSearch1,
                                              pointNKNSquaredDistance1) > 0) &&
                      (kdtree2->nearestKSearch(p, K, pointIdxNKNSearch2,
                                               pointNKNSquaredDistance2) > 0))
                  {
                    float dis1 =
                      pow(p.x - plane_lists[plane_idx1].cloud.points[pointIdxNKNSearch1[0]].x, 2) +
                      pow(p.y - plane_lists[plane_idx1].cloud.points[pointIdxNKNSearch1[0]].y, 2) +
                      pow(p.z - plane_lists[plane_idx1].cloud.points[pointIdxNKNSearch1[0]].z, 2);
                    float dis2 =
                      pow(p.x - plane_lists[plane_idx2].cloud.points[pointIdxNKNSearch2[0]].x, 2) +
                      pow(p.y - plane_lists[plane_idx2].cloud.points[pointIdxNKNSearch2[0]].y, 2) +
                      pow(p.z - plane_lists[plane_idx2].cloud.points[pointIdxNKNSearch2[0]].z, 2);
                    if((dis1 < mersys_params::fusion::min_line_dis_threshold_ * mersys_params::fusion::min_line_dis_threshold_ &&
                        dis2 < mersys_params::fusion::max_line_dis_threshold_ * mersys_params::fusion::max_line_dis_threshold_) ||
                        ((dis1 < mersys_params::fusion::max_line_dis_threshold_ * mersys_params::fusion::max_line_dis_threshold_ &&
                        dis2 < mersys_params::fusion::min_line_dis_threshold_ * mersys_params::fusion::min_line_dis_threshold_)))
                        edge_clouds.push_back(p);
                  }
                }
                if(edge_clouds.size() > 30) edge_cloud_lists.push_back(edge_clouds);
              }
            }
          }
        }
      }
    }
  }

  void initVoxel(pcl::PointCloud<pcl::PointXYZI>& lidar_cloud_, const float voxel_size, std::unordered_map<VOXEL_LOC, Voxel*>& voxel_map)
  {
    ROS_INFO_STREAM("Building Voxel");    
    for(size_t i = 0; i < lidar_cloud_.size(); i++)
    {
      const pcl::PointXYZI& p_t = lidar_cloud_.points[i];
      Eigen::Vector3d pt(p_t.x, p_t.y, p_t.z);
      pcl::PointXYZI p_c;
      p_c.x = pt(0); p_c.y = pt(1); p_c.z = pt(2);
      float loc_xyz[3];
      for(int j = 0; j < 3; j++)
      {
        loc_xyz[j] = p_c.data[j] / voxel_size;
        if(loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
      }
      VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
      auto iter = voxel_map.find(position);
      if(iter != voxel_map.end())
        voxel_map[position]->cloud->push_back(p_c);
      else
      {
        Voxel* voxel = new Voxel(voxel_size);
        voxel_map[position] = voxel;
        voxel_map[position]->voxel_origin[0] = position.x * voxel_size;
        voxel_map[position]->voxel_origin[1] = position.y * voxel_size;
        voxel_map[position]->voxel_origin[2] = position.z * voxel_size;
        voxel_map[position]->cloud->push_back(p_c);
      }
    }
    for(auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++)
      if(iter->second->cloud->size() > 20)
        downsample_voxel(*(iter->second->cloud), 0.03);
  }

  void LiDAREdgeExtraction(const std::unordered_map<VOXEL_LOC, Voxel*>& voxel_map,
                           const float ransac_dis_thre, const int plane_size_threshold,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_edge_cloud_)
  {
    ROS_INFO_STREAM("Extracting Lidar Edge");
    ros::Rate loop(5000);
    lidar_edge_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++)
    {
      if(iter->second->cloud->size() > 50)
      {
        std::vector<SinglePlane> plane_lists;
        // 创建一个体素滤波器
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*iter->second->cloud, *cloud_filter);
        //创建一个模型参数对象，用于记录结果
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // inliers表示误差能容忍的点，记录点云序号
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        //创建一个分割器
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        // Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
        seg.setOptimizeCoefficients(true);
        // Mandatory-设置目标几何形状
        seg.setModelType(pcl::SACMODEL_PLANE);
        //分割方法：随机采样法
        seg.setMethodType(pcl::SAC_RANSAC);
        //设置误差容忍范围，也就是阈值
        seg.setDistanceThreshold(ransac_dis_thre);
        pcl::PointCloud<pcl::PointXYZRGB> color_planner_cloud;
        int plane_index = 0;
        while(cloud_filter->points.size() > 10)
        {
          pcl::PointCloud<pcl::PointXYZI> planner_cloud;
          pcl::ExtractIndices<pcl::PointXYZI> extract;
          //输入点云
          seg.setInputCloud(cloud_filter);
          seg.setMaxIterations(500);
          //分割点云
          seg.segment(*inliers, *coefficients);
          if(inliers->indices.size() == 0)
          {
            ROS_INFO_STREAM("Could not estimate a planner model for the given dataset");
            break;
          }
          extract.setIndices(inliers);
          extract.setInputCloud(cloud_filter);
          extract.filter(planner_cloud);

          if(planner_cloud.size() > plane_size_threshold)
          {
            pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
            std::vector<unsigned int> colors;
            colors.push_back(static_cast<unsigned int>(rand() % 256));
            colors.push_back(static_cast<unsigned int>(rand() % 256));
            colors.push_back(static_cast<unsigned int>(rand() % 256));
            pcl::PointXYZ p_center(0, 0, 0);
            for(size_t i = 0; i < planner_cloud.points.size(); i++)
            {
              pcl::PointXYZRGB p;
              p.x = planner_cloud.points[i].x;
              p.y = planner_cloud.points[i].y;
              p.z = planner_cloud.points[i].z;
              p_center.x += p.x;
              p_center.y += p.y;
              p_center.z += p.z;
              p.r = colors[0]; p.g = colors[1]; p.b = colors[2];
              color_cloud.push_back(p);
              color_planner_cloud.push_back(p);
            }
            p_center.x = p_center.x / planner_cloud.size();
            p_center.y = p_center.y / planner_cloud.size();
            p_center.z = p_center.z / planner_cloud.size();
            SinglePlane single_plane;
            single_plane.cloud = planner_cloud;
            single_plane.p_center = p_center;
            single_plane.normal << coefficients->values[0],
              coefficients->values[1], coefficients->values[2];
            single_plane.index = plane_index;
            plane_lists.push_back(single_plane);
            plane_index++;
          }
          extract.setNegative(true);
          pcl::PointCloud<pcl::PointXYZI> cloud_f;
          extract.filter(cloud_f);
          *cloud_filter = cloud_f;
        }
        if(plane_lists.size() >= 1)
        {
          // sensor_msgs::PointCloud2 dbg_msg;
          // pcl::toROSMsg(color_planner_cloud, dbg_msg);
          // dbg_msg.header.frame_id = "camera_init";
          loop.sleep();
        }
        std::vector<pcl::PointCloud<pcl::PointXYZI>> edge_cloud_lists;
        calcLine(plane_lists, mersys_params::fusion::voxel_size_, iter->second->voxel_origin, edge_cloud_lists);
        if(edge_cloud_lists.size() > 0 && edge_cloud_lists.size() <= 5)
          for(size_t a = 0; a < edge_cloud_lists.size(); a++)
          {
            for(size_t i = 0; i < edge_cloud_lists[a].size(); i++)
            {
              pcl::PointXYZI p = edge_cloud_lists[a].points[i];
              lidar_edge_cloud_->points.push_back(p);
            }
            // sensor_msgs::PointCloud2 dbg_msg;
            // pcl::toROSMsg(edge_cloud_lists[a], dbg_msg);
            // dbg_msg.header.frame_id = "camera_init";
            // pub_plane.publish(dbg_msg);
            loop.sleep();
          }
      }
    }
  }

  void edgeDetector(const int& canny_threshold, const int& edge_threshold,
                    const cv::Mat& src_img, cv::Mat& edge_img,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_clouds)
  {
    int gaussian_size = 5;
    edge_clouds = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cv::GaussianBlur(src_img, src_img, cv::Size(gaussian_size, gaussian_size), 0, 0);
    cv::Mat canny_result = cv::Mat::zeros(src_img.rows, src_img.cols, CV_8UC1);
    cv::Canny(src_img, canny_result, canny_threshold, canny_threshold * 3, 3, true);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny_result, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    edge_img = cv::Mat::zeros(src_img.rows, src_img.cols, CV_8UC1);
    
    for(size_t i = 0; i < contours.size(); i++)
      if(contours[i].size() > edge_threshold)
      {
        cv::Mat debug_img = cv::Mat::zeros(src_img.rows, src_img.cols, CV_8UC1);
        for(size_t j = 0; j < contours[i].size(); j++)
        {
          pcl::PointXYZ p;
          p.x = contours[i][j].x;
          p.y = -contours[i][j].y;
          p.z = 0;
          edge_img.at<uchar>(-p.y, p.x) = 255;
        }
      }
    for(int x = 0; x < edge_img.cols; x++)
      for(int y = 0; y < edge_img.rows; y++)
        if(edge_img.at<uchar>(y, x) == 255)
        {
          pcl::PointXYZ p;
          p.x = x;
          p.y = -y;
          p.z = 0;
          edge_clouds->points.push_back(p);
        }
    edge_clouds->width = edge_clouds->points.size();
    edge_clouds->height = 1;
  }
    Calibration::Calibration()
    {

    }

    auto Calibration::add_lidar(PointCloudEXPtr pc) -> void
    {
        std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;
        ROS_INFO_STREAM("use_ada_voxel");
        bool use_ada_voxel =true;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_edge_cloud_;
        if(use_ada_voxel)
        {
          cut_voxel(surf_map, pc->pts_cloud, mersys_params::fusion::voxel_size_, mersys_params::fusion::eigen_ratio_);
          // std::cout << "surf_map.size"<< surf_map.size() << std::endl;
          for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
            iter->second->recut();
          // std::cout << "surf_map.size"<< surf_map.size() << std::endl;

          // #if 1
          //   pcl::PointCloud<pcl::PointXYZINormal> color_cloud;
          //   visualization_msgs::MarkerArray marker_array;

          //   for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
          //     iter->second->tras_display(color_cloud, marker_array);

          //   sensor_msgs::PointCloud2 dbg_msg;
          //   std::cout << "color_cloud.size"<< color_cloud.size() << std::endl;
          //   pcl::toROSMsg(color_cloud, dbg_msg);
          //   dbg_msg.header.frame_id = "camera_init";
          //   pub_residual.publish(dbg_msg);

          //   pub_direct.publish(marker_array);
          // #endif

          estimate_edge(surf_map,lidar_edge_cloud_);
        }
        else
        {
          // enable_ada_voxel = false;
          std::unordered_map<VOXEL_LOC, Voxel*> voxel_map;
          initVoxel(pc->pts_cloud, mersys_params::fusion::voxel_size_, voxel_map);
          // ROS_INFO_STREAM("Init voxel sucess!");
          LiDAREdgeExtraction(voxel_map, mersys_params::fusion::ransac_dis_threshold_, mersys_params::fusion::plane_size_threshold_, lidar_edge_cloud_);
        }
        std::cout << "lidar edge size:" << lidar_edge_cloud_->size() << std::endl;
        // TypeDefs::PointCloudPtr sharedCloudPtr = lidar_edge_cloud_;
        //TODO: 提高性能
        mersys::TypeDefs::PointCloudPtr sharedCloudPtr(new pcl::PointCloud<pcl::PointXYZI>(*lidar_edge_cloud_));
        lidar_edge_cloud_map_[pc->GetFrameClientID()]=sharedCloudPtr;
    }
    auto Calibration::add_img(ImageEXPtr img,bool if_undistort=true) -> void
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr rgb_edge_cloud_;
        cv::Mat gray_img, rgb_edge_img;
        cv::cvtColor(img->img_, gray_img, cv::COLOR_BGR2GRAY);
        edgeDetector(mersys_params::fusion::rgb_canny_threshold_, mersys_params::fusion::rgb_edge_minLen_, gray_img, rgb_edge_img, rgb_edge_cloud_);
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sharedCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(*rgb_edge_cloud_));
        image_edge_cloud_map_[img->GetFrameClientID()]=sharedCloudPtr;

        // return rgb_edge_cloud_
        // ROS_INFO_STREAM("Initialization complete2");
    }
   
}
