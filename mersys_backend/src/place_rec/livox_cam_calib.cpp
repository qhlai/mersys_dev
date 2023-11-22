// #include "place_rec.hpp"
#include "livox_cam_calib.hpp"
// #include "scancontext/Scancontext.h"

// C++
#include <iostream>
#include <mutex>
#include <eigen3/Eigen/Core>
#include "ceres/ceres.h"

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

Eigen::Matrix3d inner_;
Eigen::Matrix<double, 5, 1> distor_;

namespace mersys
{

class PointCloud_Calib{
    public:
        std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_edge_cloud_; // 存储平面相交得到的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_; // 存储原始点云
};

class vpnp_calib
{
public:
  vpnp_calib(VPnPData p) {pd = p;}
  template <typename T>
  bool operator()(const T *_q, const T *_t, T *residuals) const
  {
    Eigen::Matrix<T, 3, 3> innerT = inner_.cast<T>();
    Eigen::Quaternion<T> q_incre{_q[3], _q[0], _q[1], _q[2]};
    Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};
    Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
    Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
    #ifdef FISHEYE
    Eigen::Matrix<T, 4, 1> distorT = distor_.cast<T>();
    const T &fx = innerT.coeffRef(0, 0);
    const T &cx = innerT.coeffRef(0, 2);
    const T &fy = innerT.coeffRef(1, 1);
    const T &cy = innerT.coeffRef(1, 2);
    T a = p_c[0] / p_c[2];
    T b = p_c[1] / p_c[2];
    T r = sqrt(a * a + b * b);
    T theta = atan(r);
    T theta_d = theta *
      (T(1) + distorT[0] * pow(theta, T(2)) + distorT[1] * pow(theta, T(4)) +
        distorT[2] * pow(theta, T(6)) + distorT[3] * pow(theta, T(8)));

    T dx = (theta_d / r) * a;
    T dy = (theta_d / r) * b;
    T ud = fx * dx + cx;
    T vd = fy * dy + cy;
    residuals[0] = ud - T(pd.u);
    residuals[1] = vd - T(pd.v);
    #else
    Eigen::Matrix<T, 5, 1> distorT = distor_.cast<T>();
    Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;
    T uo = p_2[0] / p_2[2];
    T vo = p_2[1] / p_2[2];
    const T& fx = innerT.coeffRef(0, 0);
    const T& cx = innerT.coeffRef(0, 2);
    const T& fy = innerT.coeffRef(1, 1);
    const T& cy = innerT.coeffRef(1, 2);
    T xo = (uo - cx) / fx;
    T yo = (vo - cy) / fy;
    T r2 = xo * xo + yo * yo;
    T r4 = r2 * r2;
    T distortion = 1.0 + distorT[0] * r2 + distorT[1] * r4 + distorT[4] * r2 * r4;
    T xd = xo * distortion + (distorT[2] * xo * yo + distorT[2] * xo * yo) +
            distorT[3] * (r2 + xo * xo + xo * xo);
    T yd = yo * distortion + distorT[3] * xo * yo + distorT[3] * xo * yo +
            distorT[2] * (r2 + yo * yo + yo * yo);
    T ud = fx * xd + cx;
    T vd = fy * yd + cy;

    if(T(pd.direction(0)) == T(0.0) && T(pd.direction(1)) == T(0.0))
    {
      residuals[0] = ud - T(pd.u);
      residuals[1] = vd - T(pd.v);
    }
    else
    {
      residuals[0] = ud - T(pd.u);
      residuals[1] = vd - T(pd.v);
      Eigen::Matrix<T, 2, 2> I = Eigen::Matrix<float, 2, 2>::Identity().cast<T>();
      Eigen::Matrix<T, 2, 1> n = pd.direction.cast<T>();
      Eigen::Matrix<T, 1, 2> nt = pd.direction.transpose().cast<T>();
      Eigen::Matrix<T, 2, 2> V = n * nt;
      V = I - V;
      Eigen::Matrix<T, 2, 1> R = Eigen::Matrix<float, 2, 1>::Zero().cast<T>();
      R.coeffRef(0, 0) = residuals[0];
      R.coeffRef(1, 0) = residuals[1];
      R = V * R;
      residuals[0] = R.coeffRef(0, 0);
      residuals[1] = R.coeffRef(1, 0);
    }
    #endif
    return true;
  }
  static ceres::CostFunction *Create(VPnPData p)
  {
    return (new ceres::AutoDiffCostFunction<vpnp_calib, 2, 4, 3>(new vpnp_calib(p)));
  }

private:
  VPnPData pd;
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
  double cos_angle(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return (a.dot(b) / a.norm()) / b.norm();
}

void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
            uint8_t &b) {
  r = 255;
  g = 255;
  b = 255;

  if(v < vmin) {
    v = vmin;
  }

  if(v > vmax) {
    v = vmax;
  }

  double dr, dg, db;

  if(v < 0.1242) {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  } else if(v < 0.3747) {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  } else if(v < 0.6253) {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  } else if(v < 0.8758) {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  } else {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
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
    if(!(theta >mersys_params::fusion::direction_theta_max_ && theta < mersys_params::fusion::direction_theta_min_)) return;

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
          if(theta > mersys_params::fusion::direction_theta_max_ && theta < mersys_params::fusion::direction_theta_min_)
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
  void calcDirection(const std::vector<Eigen::Vector2d>& points, Eigen::Vector2d& direction)
  {
    Eigen::Vector2d mean_point(0, 0);
    for(size_t i = 0; i < points.size(); i++)
    {
      mean_point(0) += points[i](0);
      mean_point(1) += points[i](1);
    }
    mean_point(0) = mean_point(0) / points.size();
    mean_point(1) = mean_point(1) / points.size();
    Eigen::Matrix2d S;
    S << 0, 0, 0, 0;
    for(size_t i = 0; i < points.size(); i++)
    {
      Eigen::Matrix2d s = (points[i] - mean_point) * (points[i] - mean_point).transpose();
      S += s;
    }
    Eigen::EigenSolver<Eigen::Matrix<double, 2, 2>> es(S);
    Eigen::MatrixXcd evecs = es.eigenvectors();
    Eigen::MatrixXcd evals = es.eigenvalues();
    Eigen::MatrixXd evalsReal;
    evalsReal = evals.real();
    Eigen::MatrixXf::Index evalsMax;
    evalsReal.rowwise().sum().maxCoeff(&evalsMax); //得到最大特征值的位置
    direction << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax);
  }
  
  cv::Mat getConnectImg(const Camera& cam, const int dis_threshold,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &rgb_edge_cloud,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &depth_edge_cloud)
  {
    cv::Mat connect_img = cam.rgb_img_.clone();
    // cv::Mat connect_img = cv::Mat::zeros(cam.height_, cam.width_, CV_8UC3);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_cam(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud_cam = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    kdtree_cam->setInputCloud(rgb_edge_cloud);
    tree_cloud_cam = rgb_edge_cloud;
    for(size_t i = 0; i < depth_edge_cloud->points.size(); i++)
    {
      cv::Point2d p2(depth_edge_cloud->points[i].x, -depth_edge_cloud->points[i].y);
      if(p2.x > 0 && p2.x < cam.width_ && p2.y > 0 && p2.y < cam.height_)
      {
        pcl::PointXYZ p = depth_edge_cloud->points[i];
        search_cloud->points.push_back(p);
      }
    }

    int line_count = 0;
    // 指定近邻个数
    int K = 1;
    // 创建两个向量，分别存放近邻的索引值、近邻的中心距
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for(size_t i = 0; i < search_cloud->points.size(); i++)
    {
      pcl::PointXYZ searchPoint = search_cloud->points[i];
      if(kdtree_cam->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        for(int j = 0; j < K; j++)
        {
          float distance =
            sqrt(pow(searchPoint.x - tree_cloud_cam->points[pointIdxNKNSearch[j]].x, 2) +
                 pow(searchPoint.y - tree_cloud_cam->points[pointIdxNKNSearch[j]].y, 2));
          if(distance < dis_threshold)
          {
            cv::Scalar color = cv::Scalar(0, 255, 0);
            line_count++;
            if((line_count % 3) == 0)
            {
              cv::line(connect_img, cv::Point(search_cloud->points[i].x,
                       -search_cloud->points[i].y),
              cv::Point(tree_cloud_cam->points[pointIdxNKNSearch[j]].x,
                        -tree_cloud_cam->points[pointIdxNKNSearch[j]].y), color, 2);
            }
          }
        }
      }
    }
    for(size_t i = 0; i < rgb_edge_cloud->size(); i++)
    {
      cv::Point2f p2(rgb_edge_cloud->points[i].x, -rgb_edge_cloud->points[i].y);
      cv::circle(connect_img, p2, 1, cv::Scalar(0, 0, 255), -1); // bgr
    }
    for(size_t i = 0; i < search_cloud->size(); i++)
    {
      cv::Point2f p2(search_cloud->points[i].x, -search_cloud->points[i].y);
      cv::circle(connect_img, p2, 2, cv::Scalar(255, 0, 0), -1);
    }
    return connect_img;
  }

  

  void projection(const Vector6d& extrinsic_params, const Camera& cam,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud,
                  cv::Mat& projection_img)
  {
    std::vector<cv::Point3f> pts_3d;
    std::vector<float> intensity_list;
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
    for(size_t i = 0; i < lidar_cloud->size(); i++)
    {
      pcl::PointXYZI point_3d = lidar_cloud->points[i];
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
      intensity_list.emplace_back(lidar_cloud->points[i].intensity);
    }
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
      << cam.fx_, cam.s_, cam.cx_, 0.0, cam.fy_, cam.cy_, 0.0, 0.0, 1.0);
    cv::Mat distortion_coeff =
      (cv::Mat_<double>(1, 5) << cam.k1_, cam.k2_, cam.p1_, cam.p2_, cam.k3_);
    cv::Mat r_vec = (cv::Mat_<double>(3, 1)
      << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
         rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
         rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
    cv::Mat t_vec = (cv::Mat_<double>(3, 1)
      << extrinsic_params[3], extrinsic_params[4], extrinsic_params[5]);
    // project 3d-points into image view
    std::vector<cv::Point2f> pts_2d;
    #ifdef FISHEYE
    cv::fisheye::projectPoints(pts_3d, pts_2d, r_vec, t_vec, camera_.camera_matrix_, camera_.dist_coeffs_);
    #else
    cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff, pts_2d);
    #endif
    cv::Mat image_project = cv::Mat::zeros(cam.height_, cam.width_, CV_16UC1);
    cv::Mat rgb_image_project = cv::Mat::zeros(cam.height_, cam.width_, CV_8UC3);
    for(size_t i = 0; i < pts_2d.size(); ++i)
    {
      cv::Point2f point_2d = pts_2d[i];
      if(point_2d.x <= 0 || point_2d.x >= cam.width_ || point_2d.y <= 0 || point_2d.y >= cam.height_)
        continue;
      else
      {
        // test depth and intensity both
        float depth = sqrt(pow(pts_3d[i].x, 2) + pow(pts_3d[i].y, 2) + pow(pts_3d[i].z, 2));
        if(depth >= 40) depth = 40;
        float grey = depth / 40 * 65535;
        image_project.at<ushort>(point_2d.y, point_2d.x) = grey;
      }
    }
    cv::Mat grey_image_projection;
    cv::cvtColor(rgb_image_project, grey_image_projection, cv::COLOR_BGR2GRAY);

    image_project.convertTo(image_project, CV_8UC1, 1 / 256.0);
    projection_img = image_project.clone();
  }

  cv::Mat getProjectionImg(const Camera& camera_,pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud_, const Vector6d& extrinsic_params)
  {
    cv::Mat depth_projection_img;
    Camera cam = camera_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<PointType>);

    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_(rotation_vector3);
    Eigen::Vector3d t_(extrinsic_params[3], extrinsic_params[4], extrinsic_params[5]);
    int cnt = 0;
    lidar_cloud->points.resize(5e6);
    for(size_t i = 0; i < lidar_cloud_->points.size(); i++)
    {
      pcl::PointXYZI point_3d = lidar_cloud_->points[i];
      Eigen::Vector3d pt1(point_3d.x, point_3d.y, point_3d.z);
      Eigen::Vector3d pt2(0, 0, 1);
      if(cos_angle(q_ * pt1 + t_, pt2) > cos(DEG2RAD(mersys_params::fusion::cam_fov_/2.0)))
      {
        lidar_cloud->points[cnt].x = pt1(0);
        lidar_cloud->points[cnt].y = pt1(1);
        lidar_cloud->points[cnt].z = pt1(2);
        lidar_cloud->points[cnt].intensity = lidar_cloud_->points[i].intensity;
        cnt++;
      }
    }
    std::cout << "lidar cloud size:" << lidar_cloud->size() << std::endl;
    lidar_cloud->points.resize(cnt);
    // downsample_voxel(*lidar_cloud, 0.03);
    projection(extrinsic_params, cam, lidar_cloud, depth_projection_img);
    cv::Mat map_img = cv::Mat::zeros(cam.height_, cam.width_, CV_8UC3);
    for(int x = 0; x < map_img.cols; x++)
    {
      for(int y = 0; y < map_img.rows; y++)
      {
        uint8_t r, g, b;
        float norm = depth_projection_img.at<uchar>(y, x) / 256.0;
        mapJet(norm, 0, 1, r, g, b);
        map_img.at<cv::Vec3b>(y, x)[0] = b;
        map_img.at<cv::Vec3b>(y, x)[1] = g;
        map_img.at<cv::Vec3b>(y, x)[2] = r;
      }
    }
    // cv::resize(projection_img, projection_img, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    cv::imshow("rough calib map_img", map_img);
    cv::imshow("rough calib cam.rgb_img_", cam.rgb_img_);
    // cv::waitKey(10);

    cv::Mat merge_img = 0.8 * map_img + 0.8 * cam.rgb_img_;
    return merge_img;
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
    auto Calibration::roughCalib(Camera& camera_,pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_edge_cloud_, double search_resolution, int max_iter)  -> void
    {
      float match_dis = 25;
      Eigen::Vector3d fix_adjust_euler(0, 0, 0);
      // ROS_INFO_STREAM("roughCalib");
      for(int n = 0; n < 2; n++)
        for(int round = 0; round < 3; round++)
        {
          Eigen::Matrix3d rot = camera_.ext_R;
          Eigen::Vector3d transation = camera_.ext_t;
          float min_cost = 1000;
          for(int iter = 0; iter < max_iter; iter++)
          {
            Eigen::Vector3d adjust_euler = fix_adjust_euler;
            adjust_euler[round] = fix_adjust_euler[round] + pow(-1, iter) * int(iter / 2) * search_resolution;
            Eigen::Matrix3d adjust_rotation_matrix;
            adjust_rotation_matrix =
              Eigen::AngleAxisd(adjust_euler[0], Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(adjust_euler[1], Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(adjust_euler[2], Eigen::Vector3d::UnitX());
            Eigen::Matrix3d test_rot = rot * adjust_rotation_matrix;
            Eigen::Vector3d test_euler = test_rot.eulerAngles(2, 1, 0);
            Vector6d test_params;
            test_params << test_euler[0], test_euler[1], test_euler[2], transation[0], transation[1], transation[2];
            std::vector<VPnPData> pnp_list;
            // ROS_INFO_STREAM("roughCalib1");
            buildVPnp(camera_, test_params, match_dis,
                              false, camera_.rgb_edge_cloud_,
                              lidar_edge_cloud_, pnp_list);
            // ROS_INFO_STREAM("roughCalib2");
            int edge_size = lidar_edge_cloud_->size();
            int pnp_size = pnp_list.size();
            float cost = ((float)(edge_size - pnp_size) / (float)edge_size);
            #ifdef debug_mode
            std::cout << "n " << n << " round " << round  << " iter "
                      << iter << " cost:" << cost << std::endl;
            #endif
            if(cost < min_cost)
            {
              ROS_INFO_STREAM("cost " << cost << " edge size "
                                      << lidar_edge_cloud_->size()
                                      << " pnp_list " << pnp_list.size());
              min_cost = cost;
              Eigen::Matrix3d rot;
              rot = Eigen::AngleAxisd(test_params[0], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(test_params[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(test_params[2], Eigen::Vector3d::UnitX());
              camera_.update_Rt(rot, transation);
              buildVPnp(camera_, test_params, match_dis,
                                true, camera_.rgb_edge_cloud_,
                                lidar_edge_cloud_, pnp_list);
              Eigen::Matrix3d R;
              Eigen::Vector3d T;
              R = camera_.ext_R;
              T = camera_.ext_t;
              Eigen::Quaterniond q(R);
              double ext[7];
              ext[0] = q.x(); ext[1] = q.y(); ext[2] = q.z(); ext[3] = q.w();
              ext[4] = T[0]; ext[5] = T[1]; ext[6] = T[2];
              Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(ext);
              Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);

              ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
              ceres::Problem problem;
              problem.AddParameterBlock(ext, 4, q_parameterization);
              problem.AddParameterBlock(ext + 4, 3);
              for(auto val: pnp_list)
              {
                ceres::CostFunction* cost_function;
                if(only_calib_rotation_)
                {
                  // cost_function = vpnp_calib_rotation::Create(val, T);
                  // problem.AddResidualBlock(cost_function, NULL, ext);
                }
                else
                {
                  cost_function = vpnp_calib::Create(val);
                  problem.AddResidualBlock(cost_function, NULL, ext, ext + 4);
                }
              cv::Mat projection_img = getProjectionImg(camera_,lidar_edge_cloud_,test_params);
              // cv::resize(projection_img, projection_img, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
              cv::imshow("rough calib", projection_img);
              cv::waitKey(0);
              }
            }
          }
        }
    }
    auto Calibration::buildVPnp(const Camera& cam,
                 const Vector6d& extrinsic_params, const int dis_threshold,
                 const bool show_residual,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cam_edge_clouds_2d,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_edge_clouds_3d,
                 std::vector<VPnPData>& pnp_list)  -> void
  {
    pnp_list.clear();
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
      << cam.fx_, cam.s_, cam.cx_, 0.0, cam.fy_, cam.cy_, 0.0, 0.0, 1.0);
    cv::Mat distortion_coeff =
      (cv::Mat_<double>(1, 5) << cam.k1_, cam.k2_, cam.p1_, cam.p2_, cam.k3_);
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_(rotation_vector3);
    // ROS_INFO_STREAM("roughCalib1.1");
    std::vector<std::vector<std::vector<pcl::PointXYZI>>> img_pts_container;
    for(int y = 0; y < cam.height_; y++)
    {
      std::vector<std::vector<pcl::PointXYZI>> row_pts_container;
      for(int x = 0; x < cam.width_; x++)
      {
        std::vector<pcl::PointXYZI> col_pts_container;
        row_pts_container.push_back(col_pts_container);
      }
      img_pts_container.push_back(row_pts_container);
    }
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    cv::Mat r_vec = (cv::Mat_<double>(3, 1)
      << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
         rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
         rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
    Eigen::Vector3d t_(extrinsic_params[3], extrinsic_params[4], extrinsic_params[5]);
    cv::Mat t_vec = (cv::Mat_<double>(3, 1) << t_(0), t_(1), t_(2));
// ROS_INFO_STREAM("roughCalib1.2");
    for(size_t i = 0; i < lidar_edge_clouds_3d->size(); i++)
    {
        pcl::PointXYZI point_3d = lidar_edge_clouds_3d->points[i];
        Eigen::Vector3d pt1(point_3d.x, point_3d.y, point_3d.z);
        Eigen::Vector3d pt2(0, 0, 1);      
      #ifdef FISHEYE
        if(cos_angle(q_ * pt1 + t_, pt2) > cos(DEG2RAD(mersys_params::fusion::cam_fov_/2.0))) // fisheye cam FoV check
          pts_3d.emplace_back(cv::Point3f(pt1(0), pt1(1), pt1(2)));
      #else
        if(cos_angle(q_ * pt1 + t_, pt2) > 0.8) // FoV check
          pts_3d.emplace_back(cv::Point3f(pt1(0), pt1(1), pt1(2)));
      #endif
    }
    // ROS_INFO_STREAM("roughCalib1.3");
    // std::cout << "pts_3d.size():"<<std::endl<<pts_3d.size() << std::endl;
    // std::cout << "r_vec:"<<std::endl<<r_vec << std::endl;
    // std::cout << "t_vec:"<<std::endl<<t_vec << std::endl;    
    #ifdef FISHEYE
    cv::fisheye::projectPoints(pts_3d, pts_2d, r_vec, t_vec, camera_.camera_matrix_, camera_.dist_coeffs_);
    #else

    // std::cout << "camera_matrix:"<<std::endl<<camera_matrix << std::endl;
    // std::cout << "distortion_coeff:"<<std::endl<<distortion_coeff << std::endl;
    // std::cout << "pts_2d.size():"<<std::endl<<pts_2d.size() << std::endl;
    cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff, pts_2d);
    #endif
    // ROS_INFO_STREAM("roughCalib1.4");
    pcl::PointCloud<pcl::PointXYZ>::Ptr line_edge_cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> line_edge_cloud_2d_number;
    for(size_t i = 0; i < pts_2d.size(); i++)
    {
      pcl::PointXYZ p;
      p.x = pts_2d[i].x;
      p.y = -pts_2d[i].y;
      p.z = 0;
      pcl::PointXYZI pi_3d;
      pi_3d.x = pts_3d[i].x;
      pi_3d.y = pts_3d[i].y;
      pi_3d.z = pts_3d[i].z;
      pi_3d.intensity = 1;
      if(p.x > 0 && p.x < cam.width_ && pts_2d[i].y > 0 && pts_2d[i].y < cam.height_)
      {
        if(img_pts_container[pts_2d[i].y][pts_2d[i].x].size() == 0)
        {
          line_edge_cloud_2d->points.push_back(p);
          img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
        }
        else
          img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
      }
    }
    if(show_residual)
    {
      cv::Mat residual_img = getConnectImg(cam, dis_threshold, cam_edge_clouds_2d, line_edge_cloud_2d);
      cv::resize(residual_img, residual_img, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
      cv::imshow("residual", residual_img);
      cv::waitKey(10);
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_cam(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_lidar(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud_cam =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud_lidar =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    kdtree_cam->setInputCloud(cam_edge_clouds_2d);
    kdtree_lidar->setInputCloud(line_edge_cloud_2d);
    tree_cloud_cam = cam_edge_clouds_2d;
    tree_cloud_lidar = line_edge_cloud_2d;
    search_cloud = line_edge_cloud_2d;

    int K = 5; // 指定近邻个数
    // 创建两个向量，分别存放近邻的索引值、近邻的中心距
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::vector<int> pointIdxNKNSearchLidar(K);
    std::vector<float> pointNKNSquaredDistanceLidar(K);
    std::vector<cv::Point2d> lidar_2d_list;
    std::vector<cv::Point2d> img_2d_list;
    std::vector<Eigen::Vector2d> camera_direction_list;
    std::vector<Eigen::Vector2d> lidar_direction_list;
    std::vector<int> lidar_2d_number;
    for(size_t i = 0; i < search_cloud->points.size(); i++)
    {
      pcl::PointXYZ searchPoint = search_cloud->points[i];
      kdtree_lidar->nearestKSearch(searchPoint, K, pointIdxNKNSearchLidar,
                                    pointNKNSquaredDistanceLidar);
      if(kdtree_cam->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        bool dis_check = true;
        for(int j = 0; j < K; j++)
        {
          float distance =
            sqrt(pow(searchPoint.x - tree_cloud_cam->points[pointIdxNKNSearch[j]].x, 2) +
                  pow(searchPoint.y - tree_cloud_cam->points[pointIdxNKNSearch[j]].y, 2));
          if(distance > dis_threshold) dis_check = false;
        }
        if(dis_check)
        {
          cv::Point p_l_2d(search_cloud->points[i].x, -search_cloud->points[i].y);
          cv::Point p_c_2d(tree_cloud_cam->points[pointIdxNKNSearch[0]].x,
                            -tree_cloud_cam->points[pointIdxNKNSearch[0]].y);
          Eigen::Vector2d direction_cam(0, 0);
          std::vector<Eigen::Vector2d> points_cam;
          for(size_t i = 0; i < pointIdxNKNSearch.size(); i++)
          {
            Eigen::Vector2d p(tree_cloud_cam->points[pointIdxNKNSearch[i]].x,
                              -tree_cloud_cam->points[pointIdxNKNSearch[i]].y);
            points_cam.push_back(p);
          }
          calcDirection(points_cam, direction_cam);
          Eigen::Vector2d direction_lidar(0, 0);
          std::vector<Eigen::Vector2d> points_lidar;
          for(size_t i = 0; i < pointIdxNKNSearch.size(); i++)
          {
            Eigen::Vector2d p(tree_cloud_lidar->points[pointIdxNKNSearchLidar[i]].x,
                              -tree_cloud_lidar->points[pointIdxNKNSearchLidar[i]].y);
            points_lidar.push_back(p);
          }
          calcDirection(points_lidar, direction_lidar);
          if(p_l_2d.x > 0 && p_l_2d.x < cam.width_ && p_l_2d.y > 0 &&
              p_l_2d.y < cam.height_)
          {
            lidar_2d_list.push_back(p_l_2d);
            img_2d_list.push_back(p_c_2d);
            camera_direction_list.push_back(direction_cam);
            lidar_direction_list.push_back(direction_lidar);
          }
        }
      }
    }
    for(size_t i = 0; i < lidar_2d_list.size(); i++)
    {
      int y = lidar_2d_list[i].y;
      int x = lidar_2d_list[i].x;
      int pixel_points_size = img_pts_container[y][x].size();
      if(pixel_points_size > 0)
      {
        VPnPData pnp;
        pnp.x = 0; pnp.y = 0; pnp.z = 0;
        pnp.u = img_2d_list[i].x;
        pnp.v = img_2d_list[i].y;
        for(int j = 0; j < pixel_points_size; j++)
        {
          pnp.x += img_pts_container[y][x][j].x;
          pnp.y += img_pts_container[y][x][j].y;
          pnp.z += img_pts_container[y][x][j].z;
        }
        pnp.x = pnp.x / pixel_points_size;
        pnp.y = pnp.y / pixel_points_size;
        pnp.z = pnp.z / pixel_points_size;
        pnp.direction = camera_direction_list[i];
        pnp.direction_lidar = lidar_direction_list[i];
        pnp.number = 0;
        float theta = pnp.direction.dot(pnp.direction_lidar);
        if(theta > mersys_params::fusion::direction_theta_min_ || theta < mersys_params::fusion::direction_theta_max_)
          pnp_list.push_back(pnp);
      }
    }
  }
   
}
