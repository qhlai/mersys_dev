
#pragma once
#include <Eigen/Geometry>

Eigen::Isometry3d AverageIsometry(const Eigen::Isometry3d& isometry1, const Eigen::Isometry3d& isometry2, double t_trans=0.5, double t_rot=0.5)
{
    Eigen::Vector3d translation1 = isometry1.translation();  // 提取isometry1的平移向量
    Eigen::Vector3d translation2 = isometry2.translation();  // 提取isometry2的平移向量

    Eigen::Quaterniond rotation1(isometry1.rotation());  // 提取isometry1的旋转部分，构造四元数
    Eigen::Quaterniond rotation2(isometry2.rotation());  // 提取isometry2的旋转部分，构造四元数

    // 插值参数t
    Eigen::Vector3d avgTranslation = (1-t_trans)*translation1 + t_trans * translation2;  // 计算平均平移向量

    // 插值参数t
    Eigen::Quaterniond avgRotation = rotation1.slerp(t_rot, rotation2);  // 进行四元数的Slerp插值，权重为0-1, 1代表rotation2比重最大

    Eigen::Isometry3d avgIsometry = Eigen::Isometry3d::Identity();  // 创建单位变换矩阵
    avgIsometry.translation() = avgTranslation;  // 设置平均平移向量
    avgIsometry.rotate(avgRotation);  // 设置平均旋转部分

    return avgIsometry;
}
