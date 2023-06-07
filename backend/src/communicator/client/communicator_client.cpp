/**
* This file is part of Map_V.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/Map_V>
*
* Map_V is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* Map_V is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Map_V. If not, see <http://www.gnu.org/licenses/>.
*/

#include "communicator_client.hpp"

// Socket Programming
#include <atomic>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

namespace colive {

Communicator_client::Communicator_client(std::string server_ip, std::string port)
    : CommunicatorBase()
{
    colive_params::ShowParamsComm();

    // map_ = map;

    std::cout << "--> Connect to server" << std::endl;
    newfd_ = ConnectToServer(server_ip.c_str(),port);
    if(newfd_ == 2){
        std::cout << COUTFATAL << ": Could no establish connection - exit" << std::endl;
        exit(-1);
    }
    std::cout << "newfd_: " << newfd_ << std::endl;
    std::cout << "--> Connected" << std::endl;
}
auto Communicator_client::ProcessAdditional()->void {

}
auto Communicator_client::ProcessPointCloudBuffer()->void {
    std::unique_lock<std::mutex>(mtx_pointcloud_queue_);
    u16 cnt =0;

    while(!pointcloud_out_buffer_.empty()) {
        auto ptcloud = pointcloud_out_buffer_.front();
        pointcloud_out_buffer_.pop_front();

        // if(kfi->sent_once_ && !Map_V_params::comm::send_updates) continue;
        // if(kfi->sent_once_ && kfi->mnId == 0) continue;
        colive::data_bundle map_chunk;
        colive::MsgPointcloud msg_ptcloud;
        Vector3Type m(1.0,2.0,3.0);
        ptcloud->ConvertToMsg(msg_ptcloud,m ,ptcloud->sent_once_,client_id_);
        ptcloud->sent_once_ = true;
        map_chunk.pointclouds.push_back(msg_ptcloud);

        this->PassDataBundle(map_chunk);

        if(cnt >= colive_params::comm::max_sent_kfs_per_iteration) break;


    }
    // while(!kf_out_buffer_.empty())


}
auto Communicator_client::ProcessKfBuffer()->void {
    std::unique_lock<std::mutex>(mtx_kf_queue_);
    // int cnt = 0;

    // while(!kf_out_buffer_.empty()) {
    //     auto kfi = kf_out_buffer_.front();
    //     kf_out_buffer_.pop_front();
    //     if(kfi->sent_once_ && !Map_V_params::comm::send_updates) continue;
    //     if(kfi->sent_once_ && kfi->mnId == 0) continue;
    //     Map_V::data_bundle map_chunk;
    //     Map_V::MsgKeyframe msg_kf;
    //     kfi->ConvertToMsg(msg_kf,kfi->mPrevKF,kfi->sent_once_,client_id_);
    //     kfi->sent_once_ = true;
    //     map_chunk.keyframes.push_back(msg_kf);
    //     if(!kfi->sent_once_) cnt++;
    //     auto kfi_lms = kfi->GetMapPointMatches();
    //     for(auto lmi : kfi_lms){
    //         if(!lmi) continue;
    //         if(lmi->sent_once_ && !Map_V_params::comm::send_updates) continue;
    //         Map_V::MsgLandmark msg_lm;
    //         lmi->ConvertToMsg(msg_lm,kfi,lmi->sent_once_,client_id_);
    //         lmi->sent_once_ = true;
    //         map_chunk.landmarks.push_back(msg_lm);
    //     }
    //     this->PassDataBundle(map_chunk);
    //     if(cnt >= Map_V_params::comm::max_sent_kfs_per_iteration) break;
    // }
}

auto Communicator_client::ProcessKeyframeMessages()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    // while(!buffer_keyframes_in_.empty()) {
    //     Map_V::MsgKeyframe msg = buffer_keyframes_in_.front();
    //     buffer_keyframes_in_.pop_front();
    //     std::cout << COUTRED("Received KF from server -- define usage") << std::endl;
    //     // Define here what should be done with the received KF
    // }
}

auto Communicator_client::ProcessLandmarkMessages()->void {

}

auto Communicator_client::ProcessNewKeyframes()->void {

}

auto Communicator_client::ProcessNewLandmarks()->void {

}
auto Communicator_client::Run()->void {
    std::thread thread_recv(&Communicator_client::RecvMsg, this);
    thread_recv.detach();

    while(true)
    {
        this->ProcessPointCloudBuffer();
        // this->ProcessKfBuffer();
        this->ProcessBufferOut();
        this->ProcessBufferIn();
        // if(this->TryLock()){//执行tryLock()方法时未获得锁，则会立即返回false,不会阻塞
        //     this->ProcessKeyframeMessages();
        //     this->ProcessLandmarkMessages();
        //     this->ProcessNewKeyframes();
        //     this->ProcessNewLandmarks();
        //     this->ProcessAdditional();
        //     this->UnLock();
        // }

        if(this->ShallFinish()){
            // std::unique_lock<std::mutex>(mtx_kf_queue_);
            // if(!kf_out_buffer_.empty()) {
            //     std::cout << "Comm:: waiting for kf_out_buffer_" << std::endl;
            // } else {
            //     std::cout << "Comm " << client_id_ << ": close" << std::endl;
            //     break;
            // }
        }
        usleep(1000);
    }

    std::cout << "Comm " << client_id_ << ": leave " << __PRETTY_FUNCTION__ << std::endl;

    std::unique_lock<std::mutex> lock(mtx_finish_);
    is_finished_ = true;
}


// auto Communicator_client::Test()->void {
//     std::thread thread_recv(&Communicator::RecvMsg, this);
//     thread_recv.detach();

//     while(true)
//     {
//         // this->ProcessKfBuffer();
//         // this->ProcessBufferOut();
//         // this->ProcessBufferIn();
//         // if(this->TryLock()){//执行tryLock()方法时未获得锁，则会立即返回false,不会阻塞
//         //     this->ProcessKeyframeMessages();
//         //     this->ProcessLandmarkMessages();
//         //     this->ProcessNewKeyframes();
//         //     this->ProcessNewLandmarks();
//         //     this->ProcessAdditional();
//         //     this->UnLock();
//         // }

//         if(this->ShallFinish()){
//             // std::unique_lock<std::mutex>(mtx_kf_queue_);
//             // if(!kf_out_buffer_.empty()) {
//             //     std::cout << "Comm:: waiting for kf_out_buffer_" << std::endl;
//             // } else {
//             //     std::cout << "Comm " << client_id_ << ": close" << std::endl;
//             //     break;
//             // }
//         }
//         usleep(1000);
//     }

//     std::cout << "Comm " << client_id_ << ": leave " << __PRETTY_FUNCTION__ << std::endl;

//     std::unique_lock<std::mutex> lock(mtx_finish_);
//     is_finished_ = true;
// }


} //end ns
