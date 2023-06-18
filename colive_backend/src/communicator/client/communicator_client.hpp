/**
* This file is part of COVINS.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/covins>
*
* COVINS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* COVINS is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with COVINS. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

// C++
#include <unistd.h>
#include <mutex>
#include <thread>
#include <list>
#include <string>
#include <netinet/in.h>
#include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Geometry>

#include "communicator_base.hpp"
#include "pointcloud_ex.hpp"
// COVINS
#include "typedefs_base.hpp"
#include "config_comm.hpp"
#include "map_co.hpp"

#include "msgs/msg_landmark.hpp"
#include "msgs/msg_keyframe.hpp"

#include "msgs/msg_pointcloud.hpp"
#include "msgs/msg_odometry.hpp"
// #include "msgs/msg_landmark.hpp"
#define ContainerSize 10

namespace colive {


class KeyFrame;
class PointCloud;
class a{
    public:
    int b;
};
class Communicator_client : public CommunicatorBase, public std::enable_shared_from_this<Communicator_client> {
public:
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using QuaternionType                = TypeDefs::QuaternionType;
    using TransformType                 = TypeDefs::TransformType;
    using PointCloudEX  = TypeDefs::PointCloudEX; 
    using PointCloudEXList  = TypeDefs::PointCloudEXList;  
    // struct cmp_by_id{
    //     bool operator() (const std::pair<size_t,KeyFrame*> a, const std::pair<size_t,KeyFrame*> b){
    //         if(a.first < b.first) return true;
    //         else return false;
    // }};

public:

    Communicator_client(std::string server_ip, std::string port);

    virtual auto tryReConnect()      ->void;
    // main function
    virtual auto Run()            ->void;
    
    // virtual auto ClientTest()            ->void;

    // virtual auto PassKfToComm(KeyFrame* kf)                               ->void {
    //     std::unique_lock<std::mutex>(mtx_kf_queue_);
    //     kf_out_buffer_.push_back(kf);
    // }
    virtual auto TryPassKeyPcToComm(PointCloudEX* pc)                                          ->void {

        Vector3Type pos_diff = pc->pos_w-last_pos_;
        // Eigen::Quaternion<double> quan_now=pc->quan_;
        QuaternionType quan_diff =pc->quan_.conjugate()*last_quan_;
        // quan_diff=quan_diff.normalize();
// 四元数的减法运算(逆/共轭)得到的结果不一定是一个合法的四元数。
// 因此，在进行减法运算时，需要对结果进行规范化操作，以确保其仍然是一个合法的四元数。

            // odom_pose_prev = odom_pose_curr;
            // odom_pose_curr = pose_curr;
            // Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

            // double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            // translationAccumulated += delta_translation;
            // rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            // // 关键帧选择
            // if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
            //     isNowKeyFrame = true;
            //     translationAccumulated = 0.0; // reset 
            //     rotaionAccumulated = 0.0; // reset 
            // } else {
            //     isNowKeyFrame = false;
            // }

        PassPcToComm(pc);
        last_pos_=pc->pos_w;
        last_quan_=pc->quan_;

    }
    virtual auto PassPcToComm(PointCloudEX* pc)                                             ->void {
        std::unique_lock<std::mutex>(mtx_pc_queue_);
        pointcloud_out_buffer_.push_back(pc);
    }
protected:

//     // data handling
    virtual auto ProcessAdditional()                                                    ->void;
    virtual auto ProcessPointCloudMessages()->void;
    virtual auto ProcessKeyframeMessages()                                              ->void;
    virtual auto ProcessLandmarkMessages()                                              ->void;
    virtual auto ProcessNewKeyframes()                                                  ->void;
    virtual auto ProcessNewLandmarks()                                                  ->void;

    // virtual auto ProcessPointCloudMessages()                                            ->void;

    virtual auto ProcessKfBuffer()                                                      ->void;
    virtual auto ProcessPointCloudBuffer()                                              ->void;
//     // Infrastructure
//     Atlas*                  map_                                                                = nullptr;  // the map is not necessary to send data to the server. However, we keep a ptr to it to facilitate implementing potetnial interaction


    std::string server_ip_;
    std::string port_;

    bool sending_init_ = false;

    Vector3Type             last_pos_ =Vector3Type::Zero();
    QuaternionType          last_quan_=QuaternionType::Identity(); //https://quaternions.online/

    std::list<KeyFrame*>   kf_out_buffer_;
    std::list<PointCloudEX*>   pointcloud_out_buffer_;
    std::mutex              mtx_pc_queue_;
    std::mutex              mtx_kf_queue_;

    std::mutex              mtx_pointcloud_queue_;

};

} //end ns

