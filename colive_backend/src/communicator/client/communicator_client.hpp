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

#include "typedefs_base.hpp"
#include "config_comm.hpp"

// #include "msgs/msg_landmark.hpp"
#define ContainerSize 10

namespace colive {


// class KeyFrame;
class PointCloud;
// class a{
//     public:
//     int b;
// };
class Communicator_client : public CommunicatorBase, public std::enable_shared_from_this<Communicator_client> {
public:
    using precision_t                   = TypeDefs::precision_t;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using QuaternionType                = TypeDefs::QuaternionType;
    using TransformType                 = TypeDefs::TransformType;
    using PointType                     = TypeDefs::PointType;
    using PointCloud                    = TypeDefs::PointCloud;
    using PointCloudPtr                    = TypeDefs::PointCloudPtr;
    using PointCloudEX  = TypeDefs::PointCloudEX; 
    using PointCloudEXList  = TypeDefs::PointCloudEXList;  
    using ImageEX  = TypeDefs::ImageEX;
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
    virtual auto TryPassKeyPcToComm(PointCloudEX* pc)                                        ->void;
    virtual auto TryPassKeyImgToComm(ImageEX* img)                                        ->void;
    virtual auto PassPcToComm(PointCloudEX pc)                                             ->void {
        std::unique_lock<std::mutex>(mtx_pc_queue_);
        pointcloud_out_buffer_.push_back(pc);
    }
    virtual auto PassImgToComm(ImageEX* img)                                             ->void {
        std::unique_lock<std::mutex>(mtx_img_queue_);
        image_out_buffer_.push_back(img);
    }
protected:

//     // data handling
    virtual auto ProcessAdditional()                                                    ->void;
    virtual auto ProcessPointCloudMessages()->void;
    virtual auto ProcessInstructionMessages()->void;
    // virtual auto ProcessKeyframeMessages()                                              ->void;
    // virtual auto ProcessLandmarkMessages()                                              ->void;
    // virtual auto ProcessNewKeyframes()                                                  ->void;
    // virtual auto ProcessNewLandmarks()                                                  ->void;

    // virtual auto ProcessPointCloudMessages()                                            ->void;

    virtual auto ProcessKfBuffer()                                                      ->void;
    virtual auto ProcessPointCloudBuffer()                                              ->void;
    virtual auto ProcessImageBuffer()                                                   ->void;
//     // Infrastructure
//     Atlas*                  map_                                                                = nullptr;  // the map is not necessary to send data to the server. However, we keep a ptr to it to facilitate implementing potetnial interaction


    std::string server_ip_;
    std::string port_;

    uint32_t send_cnt = 0;
    pcl::PointCloud<PointType>::Ptr pc_final;// 这个是智能指针
    pcl::PointCloud<PointType>::Ptr pc_final_filtered; 
    // PointCloudPtr pc_final_filtered1(new PointCloud); 
    bool sending_init_ = false;
    // 后端反馈的漂移矫正矩阵
    TransformType           drift_corr_ = TransformType::Identity(); 

    Vector3Type             last_pos_ =Vector3Type::Zero();
    QuaternionType          last_quan_=QuaternionType::Identity(); //https://quaternions.online/
    TransformType           last_transform_ = TransformType::Identity(); 
    // 多帧拼接的基础帧的迁移矩阵
    TransformType           base_frame_transform_ = TransformType::Identity(); 
    bool                    base_frame_update_ = true;
    precision_t             last_timestamp_=0;

    // std::list<KeyFrame*>   kf_out_buffer_;
    std::list<PointCloudEX>   pointcloud_out_buffer_;
    std::list<ImageEX*>        image_out_buffer_;
    std::mutex              mtx_pc_queue_;
    std::mutex              mtx_img_queue_;
    std::mutex              mtx_kf_queue_;

    std::mutex              mtx_pointcloud_queue_;
    std::mutex              mtx_image_queue_;

};

} //end ns

