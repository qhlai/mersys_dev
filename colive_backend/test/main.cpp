#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <iostream>

int main()
{
    // 创建ISAM2对象
    gtsam::ISAM2 isam2;

    // 创建一个NonlinearFactorGraph对象，用于存储因子图
    gtsam::NonlinearFactorGraph graph;

    // 创建一个Values对象，用于存储变量的初始估计值
    gtsam::Values initialEstimate;

    // 定义变量的Key
   // gtsam::Key key1 = gtsam::Symbol('x', 1);
    //gtsam::Key key2 = gtsam::Symbol('x', 2);
    //gtsam::Key key3 = gtsam::Symbol('x', 3);


    int key1 = 1;
    int key2 = 2;
    int key3 = 3;
    
    // 创建一个先验因子，设置起始位置的初始估计
    gtsam::Pose3 priorPose1(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
    gtsam::PriorFactor<gtsam::Pose3> priorFactor1(key1, priorPose1, gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished()));
    graph.add(priorFactor1);
    initialEstimate.insert(key1, priorPose1);
        
    // 创建一个运动因子，设置从位置1到位置2的运动模型
    gtsam::Pose3 relativePose(gtsam::Rot3::RzRyRx(3.01, -3.11, 2.19), gtsam::Point3(-25.3, 2.41, 2.72));
    gtsam::BetweenFactor<gtsam::Pose3> motionFactor(key1, key2, relativePose, gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished()));
    graph.add(motionFactor);
    initialEstimate.insert(key2, gtsam::Pose3()); // 插入键"x2"的初始估计



    std::cout << "1" << std::endl;
    // 初始化ISAM2
    isam2.update(graph, initialEstimate);
    isam2.update();
    
    graph.resize(0);
    initialEstimate.clear();
    // 输出优化结果
    gtsam::Values optimizedEstimate = isam2.calculateEstimate();
    std::cout << "Optimized estimate for pose1: " << optimizedEstimate.at<gtsam::Pose3>(key1) << std::endl;
    std::cout << "Optimized estimate for pose2: " << optimizedEstimate.at<gtsam::Pose3>(key2) << std::endl;

    std::cout << "2 " << std::endl;

    relativePose = gtsam::Pose3(gtsam::Rot3::RzRyRx(3.02, -3.11, 2.18), gtsam::Point3(-25.2, -10.8, 2.45));
    gtsam::BetweenFactor<gtsam::Pose3> motionFactor3(key1, key2, relativePose, gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished()));
    std::cout << "add " << std::endl;
    graph.add(motionFactor3);
    // 初始化ISAM2
    isam2.update(graph, initialEstimate);
    isam2.update();
    
    graph.resize(0);
    initialEstimate.clear();
    // 输出优化结果
    optimizedEstimate = isam2.calculateEstimate();
    std::cout << "Optimized estimate for pose1: " << optimizedEstimate.at<gtsam::Pose3>(key1) << std::endl;
    std::cout << "Optimized estimate for pose2: " << optimizedEstimate.at<gtsam::Pose3>(key2) << std::endl;
    
    std::cout << "3" << std::endl;
    relativePose = gtsam::Pose3(gtsam::Rot3::RzRyRx(3.13, -3.12, 2.21), gtsam::Point3(7.13, 26.3, -0.735));
    gtsam::BetweenFactor<gtsam::Pose3> motionFactor4(key1, key2, relativePose, gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 1, 1, 1, 0.1).finished()));
    graph.add(motionFactor4);
    
    // 初始化ISAM2
    isam2.update(graph, initialEstimate);
    isam2.update();
    
    graph.resize(0);
    initialEstimate.clear();
    // 输出优化结果
    optimizedEstimate = isam2.calculateEstimate();
    std::cout << "Optimized estimate for pose1: " << optimizedEstimate.at<gtsam::Pose3>(key1) << std::endl;
    std::cout << "Optimized estimate for pose2: " << optimizedEstimate.at<gtsam::Pose3>(key2) << std::endl;
    
    return 0;
}
