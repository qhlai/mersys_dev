
#pragma once

#include "typedefs_base.hpp"

//SERIALIZATION
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/concepts/pair_associative_container.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/access.hpp>

// #include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cereal {

//save and load function for Eigen::Matrix type

    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline
    typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    save(Archive& ar, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix) {
        const std::int32_t rows = static_cast<std::int32_t>(matrix.rows());
        const std::int32_t cols = static_cast<std::int32_t>(matrix.cols());
        ar(rows);
        ar(cols);
        ar(binary_data(matrix.data(), rows * cols * sizeof(_Scalar)));
    }

    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline
    typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    load(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix) {
        std::int32_t rows;
        std::int32_t cols;
        ar(rows);
        ar(cols);

        matrix.resize(rows, cols);

        ar(binary_data(matrix.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
    }
    // Save function for pcl::PointCloud type


    template<class Archive>
    inline
    void save(Archive& ar, const Eigen::Isometry3d& T) {
        // Save the size of the point cloud
        // size_t size = pointCloud.size();
        // ar(size);
        // Eigen::Matrix4d T_4x4 = Eigen::Matrix4d::Identity();
        // T_4x4.block<3, 3>(0, 0) = T.rotation();
        // T_4x4.block<3, 1>(0, 3) = T.translation();
        // ar(T_4x4);
        ar(T.matrix());

    }
     // Load function for pcl::PointCloud type
    template<class Archive>
    inline
    void load(Archive& ar, Eigen::Isometry3d& T) {

        Eigen::Matrix4d T_4x4 = Eigen::Matrix4d::Identity();
        // T=Eigen::Isometry3d::Identity();

        ar(T_4x4);
        Eigen::Isometry3d temp(T_4x4); 
        T=temp;
        // T.rotate(T_4x4.block<3, 3>(0, 0));
        // T.translate(T_4x4.block<3, 1>(0, 3));

    }

    template<class Archive>
    inline
    void save(Archive& ar, const pcl::PointCloud<pcl::PointXYZI>& pointCloud) {
        // Save the size of the point cloud
        size_t size = pointCloud.size();
        ar(size);

        // Save each point in the point cloud
        for (const auto& point : pointCloud) {
            ar(point.x);
            ar(point.y);
            ar(point.z);
            for (size_t i = 0; i <3;i++) {
                ar(point.data[i]);
            }

            ar(point.intensity);

        }
    }
     // Load function for pcl::PointCloud type
    template<class Archive>
    inline
    void load(Archive& ar, pcl::PointCloud<pcl::PointXYZI>& pointCloud) {
        // Load the size of the point cloud
        size_t size;
        ar(size);

        // Resize the point cloud to the loaded size
        pointCloud.resize(size);

        // Load each point in the point cloud
        for (auto& point : pointCloud) {
            ar(point.x);
            ar(point.y);
            ar(point.z);
            for (size_t i = 0; i <3;i++) {
                ar(point.data[i]);
            }
            
            ar(point.intensity);
        }
    }
    template<class Archive>
    inline
    void save(Archive& ar, const pcl::PointCloud<pcl::PointXYZINormal>& pointCloud) {
        // Save the size of the point cloud
        size_t size = pointCloud.size();
        ar(size);

        // Save each point in the point cloud
        for (const auto& point : pointCloud) {
            ar(point.x);
            ar(point.y);
            ar(point.z);
            for (size_t i = 0; i <3;i++) {
                ar(point.data[i]);
            }
            
            ar(point.normal_x);
            ar(point.normal_y);
            ar(point.normal_z);
            for (size_t i = 0; i <3;i++) {
                ar(point.data_n[i]);
            }
            
            ar(point.curvature);
            ar(point.intensity);

        }
    }
     // Load function for pcl::PointCloud type
    template<class Archive>
    inline
    void load(Archive& ar, pcl::PointCloud<pcl::PointXYZINormal>& pointCloud) {
        // Load the size of the point cloud
        size_t size;
        ar(size);

        // Resize the point cloud to the loaded size
        pointCloud.resize(size);

        // Load each point in the point cloud
        for (auto& point : pointCloud) {
            ar(point.x);
            ar(point.y);
            ar(point.z);
            for (size_t i = 0; i <3;i++) {
                ar(point.data[i]);
            }
            
            ar(point.normal_x);
            ar(point.normal_y);
            ar(point.normal_z);
            for (size_t i = 0; i <3;i++) {
                ar(point.data_n[i]);
            }
            
            ar(point.curvature);
            ar(point.intensity);
        }
    }

    template<class Archive>
    inline
    void save(Archive& ar, const colive::TypeDefs::QuaternionType& q) {
        ar(q.x());
        ar(q.y());
        ar(q.z());
        ar(q.w());
       
    }
     // Load function for pcl::PointCloud type
    template<class Archive>
    inline
    void load(Archive& ar, colive::TypeDefs::QuaternionType& q) {
       colive::TypeDefs::precision_t  x, y, z, w;
        ar(x);
        ar(y);
        ar(z);
        ar(w);
        q = colive::TypeDefs::QuaternionType(w, x, y, z);
    }
    //    save and load function for cv::Mat type
    template<class Archive>
    inline
    void save(Archive& ar, const cv::Mat& mat) {
        int rows, cols, type;
        bool continuous;

        rows = mat.rows;
        cols = mat.cols;
        type = mat.type();
        continuous = mat.isContinuous();

        ar & rows & cols & type & continuous;

        if (continuous) {
            const int data_size = rows * cols * static_cast<int>(mat.elemSize());
            auto mat_data = cereal::binary_data(mat.ptr(), data_size);
            ar & mat_data;
        }
        else {
            const int row_size = cols * static_cast<int>(mat.elemSize());
            for (int i = 0; i < rows; i++) {
                auto row_data = cereal::binary_data(mat.ptr(i), row_size);
                ar & row_data;
            }
        }
    }

    template<class Archive>
    void load(Archive& ar, cv::Mat& mat) {
        int rows, cols, type;
        bool continuous;

        ar & rows & cols & type & continuous;

        if (continuous) {
            mat.create(rows, cols, type);
            const int data_size = rows * cols * static_cast<int>(mat.elemSize());
            auto mat_data = cereal::binary_data(mat.ptr(), data_size);
            ar & mat_data;
        }
        else {
            mat.create(rows, cols, type);
            const int row_size = cols * static_cast<int>(mat.elemSize());
            for (int i = 0; i < rows; i++) {
                auto row_data = cereal::binary_data(mat.ptr(i), row_size);
                ar & row_data;
            }
        }
    }
} 



