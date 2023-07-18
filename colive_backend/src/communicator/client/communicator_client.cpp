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



#include "pointcloud_ex.hpp"
#include "image_ex.hpp"
// COVINS

#include "map_co.hpp"

// #include "msgs/msg_landmark.hpp"
// #include "msgs/msg_keyframe.hpp"

#include "msgs/msg_pointcloud.hpp"
#include "msgs/msg_image.hpp"
// #include "msgs/msg_odometry.hpp"

namespace colive {




Communicator_client::Communicator_client(std::string server_ip, std::string port)
    : CommunicatorBase(),
    server_ip_(server_ip),
    port_(port)
{
    colive_params::ShowParamsComm();

    // map_ = map;

    std::cout << "--> Connect to server" << std::endl;

    newfd_ = ConnectToServer(server_ip_.c_str(),port_);
    int cnt_retry=0;
    while(newfd_ == 2){
        std::cout << COUTFATAL << ": Could no establish connection - retry "<< (cnt_retry++) << std::endl;
        newfd_ = ConnectToServer(server_ip_.c_str(),port_);

        sleep(5);
        if(cnt_retry >=5){
            std::cout << COUTFATAL << ": Could no establish connection - exit" << std::endl;
            exit(-1);
        }
    }
    std::cout << "newfd_: " << newfd_ << std::endl;
    std::cout << "--> Connected" << std::endl;

    pc_final.reset(new PointCloud);
    pc_final_filtered.reset(new PointCloud);
}
// 用于断线重连
auto Communicator_client::tryReConnect()->void
{
    // colive_params::ShowParamsComm();

    // map_ = map;

    std::cout << "--> try Reconnecting to server" << std::endl;
    // 加入等待重连 TODO：断线重连
    newfd_ = ConnectToServer(server_ip_.c_str(),port_);
    int cnt_retry=0;
    while(newfd_ == 2){// fd=2表示标准错误
        std::cout << COUTFATAL << ": Could no establish connection - retry "<< (cnt_retry++) << std::endl;
        newfd_ = ConnectToServer(server_ip_.c_str(),port_);

        sleep(10);
        if(cnt_retry >=5){
            std::cout << COUTFATAL << ": Could no establish connection - exit" << std::endl;
            exit(-1);
        }
    }
    std::cout << "newfd_: " << newfd_ << std::endl;
    std::cout << "--> Connected" << std::endl;

    this->SetUnFinish();
    std::cout << ">>> COVINS: client id: " << this->GetClientId() << std::endl;
}

auto Communicator_client::ProcessAdditional()->void {

}
auto Communicator_client::ProcessPointCloudBuffer()->void {
    std::unique_lock<std::mutex>(mtx_pointcloud_queue_);
    u16 cnt =0;

    while(!pointcloud_out_buffer_.empty()) {
        // std::cout<< "comm: processing point cloud"<<std::endl;
        auto ptcloud = pointcloud_out_buffer_.front();
        pointcloud_out_buffer_.pop_front();

        // if(kfi->sent_once_ && !Map_V_params::comm::send_updates) continue;
        // if(kfi->sent_once_ && kfi->mnId == 0) continue;
        colive::data_bundle map_chunk;
        colive::MsgPointCloud msg_ptcloud;
        // Vector3Type m(1.0,2.0,3.0);
        ptcloud->ConvertToMsg(msg_ptcloud ,ptcloud->sent_once_,client_id_);
        ptcloud->sent_once_ = true;
        map_chunk.pointclouds.push_back(msg_ptcloud);

        this->PassDataBundle(map_chunk);// 最后加到封包发送缓存中

        if(cnt >= colive_params::comm::max_sent_kfs_per_iteration) break;


    }
    // while(!kf_out_buffer_.empty())


}
auto Communicator_client::ProcessImageBuffer()->void {
    std::unique_lock<std::mutex>(mtx_image_queue_);
    u16 cnt =0;

    while(!image_out_buffer_.empty()) {
        // std::cout<< "comm: processing point cloud"<<std::endl;
        auto img = image_out_buffer_.front();
        image_out_buffer_.pop_front();

        // if(kfi->sent_once_ && !Map_V_params::comm::send_updates) continue;
        // if(kfi->sent_once_ && kfi->mnId == 0) continue;
        colive::data_bundle map_chunk;
        colive::MsgImage msg_img;
        img->ConvertToMsg(msg_img,img->sent_once_,client_id_);
        img->sent_once_ = true;
        map_chunk.images.push_back(msg_img);

        this->PassDataBundle(map_chunk);// 最后加到封包发送缓存中

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

// auto Communicator_client::ProcessKeyframeMessages()->void {
//     std::unique_lock<std::mutex> lock(mtx_in_);
//     // while(!buffer_keyframes_in_.empty()) {
//     //     Map_V::MsgKeyframe msg = buffer_keyframes_in_.front();
//     //     buffer_keyframes_in_.pop_front();
//     //     std::cout << COUTRED("Received KF from server -- define usage") << std::endl;
//     //     // Define here what should be done with the received KF
//     // }
// }
auto Communicator_client::ProcessPointCloudMessages()->void {
    std::unique_lock<std::mutex>(mtx_in_);
    u16 cnt =0;

    while(!buffer_pointclouds_in_.empty()) {

        MsgPointCloud msg = buffer_pointclouds_in_.front();

        // if(msg.id_reference.first > last_processed_kf_msg_.first) break;

        buffer_pointclouds_in_.pop_front();
        if(msg.is_update_msg){
            if(!colive_params::comm::send_updates) continue;
            else {
                // auto kf = map_->GetKeyframe(msg.id,false);
                // if(kf && kf->id_.first == 0) continue;
                // if(kf) kf->UpdatePoseFromMsg(msg,map_);
            }

        }else{

        }
        // auto ptcloud = pointcloud_out_buffer_.front();
        // pointcloud_out_buffer_.pop_front();

        // // if(kfi->sent_once_ && !Map_V_params::comm::send_updates) continue;
        // // if(kfi->sent_once_ && kfi->mnId == 0) continue;
        // colive::data_bundle map_chunk;
        // colive::MsgPointCloud msg_ptcloud;
        // Vector3Type m(1.0,2.0,3.0);
        // ptcloud->ConvertToMsg(msg_ptcloud,m ,ptcloud->sent_once_,client_id_);
        // ptcloud->sent_once_ = true;
        // map_chunk.pointclouds.push_back(msg_ptcloud);

        // this->PassDataBundle(map_chunk);

        // if(cnt >= colive_params::comm::max_sent_kfs_per_iteration) break;


    }
    // while(!kf_out_buffer_.empty())

}
auto Communicator_client::TryPassKeyPcToComm(PointCloudEX* pc)      ->void{

        // lio 一开始可能没初始化好,忽略一开始的5帧
        static uint8_t  ingore_cnt=0;
        if(ingore_cnt< 5){
            ingore_cnt++;
            return;
        }
        auto T= pc->GetPoseTsw();

        Vector3Type pos_diff = T.translation() - last_transform_.translation();

        // TODO: modify keyframe's max pos diff
        precision_t pos_dis = sqrt(pos_diff[0]*pos_diff[0]+pos_diff[1]*pos_diff[1]+pos_diff[2]*pos_diff[2]);


        // QuaternionType quan_diff =pc->quan_*last_quan_.conjugate();
        // Eigen::Matrix3d R = quan_diff.toRotationMatrix();

        // Eigen::Vector3d position = T_curr_loop.translation();
        // Eigen::Vector3d euler = T_curr_loop.rotation().eulerAngles(0, 1, 2);

        Vector3Type rot_euler=T.rotation().eulerAngles(2,1,0); // zyx, euler is [0,pi]
        Vector3Type last_rot_euler=last_transform_.rotation().eulerAngles(2, 1, 0); // zyx, euler is [0,pi]

        // std::cout << "roll_2 pitch_2 yaw_2 = " << rot_euler[2]
        //                                 << " " << rot_euler[1]
	    //                                 << " " << rot_euler[0] << std::endl << std::endl;
        // std::cout << "diff roll_2 pitch_2 yaw_2 = " << rot_euler[2]-last_rot_euler[2]
        //                                 << " " << rot_euler[1]-last_rot_euler[1]
	    //                                 << " " << rot_euler[0]-last_rot_euler[0] << std::endl << std::endl;

        precision_t rot_diff = (fabs(rot_euler[2]-last_rot_euler[2]) + fabs(rot_euler[1]-last_rot_euler[1]) + fabs(rot_euler[0]-last_rot_euler[0]))*(180.0 / M_PI);// TODO:  shoud fix, why max is 532?
        if (rot_diff > 350){
            rot_diff = fabs(rot_diff-360);
        }
        precision_t time_diff = pc->timestamp_- last_timestamp_;
        if (time_diff<0){
                std::cout<<"Error time_diff<0"<< std::endl;
                return;
        }
        // 把多个frame一起打包压缩，减少传输次数的同时可以合并一些重复点
        // std::cout<<"pc->pts_cloud->size():"<<pc->pts_cloud.size()<< std::endl;  6000 per frame
        // if
        // static int tmp=0;
        
        if(base_frame_update_){
            base_frame_transform_=pc->GetPoseTsw();
            base_frame_update_=false;

            *pc_final+=pc->pts_cloud;
        }else{
            PointCloud::Ptr cloud_acc(new PointCloud(pc->pts_cloud));
            pcl::transformPointCloud(*cloud_acc, *cloud_acc, (pc->GetPoseTsw()*base_frame_transform_.inverse()).matrix());
            *pc_final+=*cloud_acc;
        }
        
        // 要基于body坐标系修复
        // pcl::transformPointCloud(*cloud_in, *cloud_in, pc->GetPoseTsg().matrix());
        // if (tmp%2==0){
            
            // tmp++;
        // }

  
        
        if ( (pos_dis > 0.5 || rot_diff > 10 || time_diff > 1 || pc_final->size() >= 50000) ){
            if (pc_final->size() == 0){
                std::cout<<"Error pc_final->size() == 0"<< std::endl;
                // *pc_final+=pc->pts_cloud;
                return;
            }
            
            // pc_sum.pts_cloud
            std::cout<<"pos diff:"<<pos_dis<<", rot_diff:"<<rot_diff<<", time_diff:"<<time_diff<<", pc size:"<<pc_final->size()<< std::endl;
            //  pcl 滤波
            // https://blog.csdn.net/qq_34429849/article/details/128115900
            #if 1
                // 球半径滤波器
                pcl::RadiusOutlierRemoval<PointType> outrem;  //创建滤波器
                outrem.setInputCloud(pc_final);    //设置输入点云
                outrem.setRadiusSearch(0.22);    //设置半径为0.2的范围内找临近点  该项参数越小 越严格
                outrem.setMinNeighborsInRadius (2);//设置查询点的邻域点集数小于2的删除
                outrem.filter (*pc_final);
            #endif
            #if 1
                // 统计滤波器用于去除明显离群点（离群点往往由测量噪声引入）。
                pcl::StatisticalOutlierRemoval<PointType> sor;//创建滤波器对象
                sor.setInputCloud (pc_final); //设置待滤波的点云
                sor.setMeanK (50);//设置在进行统计时考虑查询点临近点数
                sor.setStddevMulThresh (1.0); //设置判断是否为离群点的阀值
                sor.filter (*pc_final);   
            #endif
            #if 1// 降采样

                // 设置降采样的体素大小
                float voxel_size = 0.05f;
                // 创建降采样对象
                pcl::VoxelGrid<PointType> voxel_grid;
                voxel_grid.setInputCloud(pc_final);
                voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
                // 执行降采样操作
                // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
                voxel_grid.filter(*pc_final);
            #endif
            
            pc->id_.second =  GetClientId();
            pc->id_.first = send_cnt++ ;
            pc->pts_cloud=*pc_final;
            pc->SetPoseTsw(base_frame_transform_);
            std::cout << "send new pointcloud "<<pc->id_.first <<", size:"<<pc->pts_cloud.size()<< std::endl;
            PassPcToComm(pc);

            // last_pos_=pc->pos_w;
            // last_quan_=pc->quan_; 
            last_transform_=pc->GetPoseTsw();
            last_timestamp_=pc->timestamp_;          
            pc_final->clear(); // reset
            base_frame_update_=true;
            // pc->pts_cloud.clear();
        }else{
            // pc
            // 把多个frame一起打包压缩，减少传输次数的同时可以合并一些重复点
            // *pc_final+=pc->pts_cloud;
        }
        // pc->pts_cloud.clear(); // shoud not clear it, because comm use 
        

}
auto Communicator_client::TryPassKeyImgToComm(ImageEX* img)      ->void{

        // vio 一开始可能没初始化好,忽略一开始的5帧
        static uint8_t  ingore_cnt=0;
        if(ingore_cnt< 5){
            ingore_cnt++;
            return;
        }
        auto T= img->GetPoseTsw();

        Vector3Type pos_diff = T.translation() - last_transform_.translation();

        // TODO: modify keyframe's max pos diff
        precision_t pos_dis = sqrt(pos_diff[0]*pos_diff[0]+pos_diff[1]*pos_diff[1]+pos_diff[2]*pos_diff[2]);


        // // QuaternionType quan_diff =pc->quan_*last_quan_.conjugate();
        // // Eigen::Matrix3d R = quan_diff.toRotationMatrix();

        // // Eigen::Vector3d position = T_curr_loop.translation();
        // // Eigen::Vector3d euler = T_curr_loop.rotation().eulerAngles(0, 1, 2);

        Vector3Type rot_euler=T.rotation().eulerAngles(2,1,0); // zyx, euler is [0,pi]
        Vector3Type last_rot_euler=last_transform_.rotation().eulerAngles(2, 1, 0); // zyx, euler is [0,pi]

        std::cout << "roll_2 pitch_2 yaw_2 = " << rot_euler[2]
                                        << " " << rot_euler[1]
	                                    << " " << rot_euler[0] << std::endl << std::endl;
        std::cout << "diff roll_2 pitch_2 yaw_2 = " << rot_euler[2]-last_rot_euler[2]
                                        << " " << rot_euler[1]-last_rot_euler[1]
	                                    << " " << rot_euler[0]-last_rot_euler[0] << std::endl << std::endl;

        precision_t rot_diff = (fabs(rot_euler[2]-last_rot_euler[2]) + fabs(rot_euler[1]-last_rot_euler[1]) + fabs(rot_euler[0]-last_rot_euler[0]))*(180.0 / M_PI);// TODO:  shoud fix, why max is 532?
        if (rot_diff > 350){
            rot_diff = fabs(rot_diff-360);
        }
        precision_t time_diff = img->timestamp_- last_timestamp_;
        if (time_diff<0){
                std::cout<<"Error time_diff<0"<< std::endl;
                return;
        }
        // // 把多个frame一起打包压缩，减少传输次数的同时可以合并一些重复点
        // // std::cout<<"pc->pts_cloud->size():"<<pc->pts_cloud.size()<< std::endl;  6000 per frame
        // // if
        // // static int tmp=0;
        
        // if(base_frame_update_){
        //     base_frame_transform_=pc->GetPoseTsw();
        //     base_frame_update_=false;

        //     *pc_final+=pc->pts_cloud;
        // }else{
        //     PointCloud::Ptr cloud_acc(new PointCloud(pc->pts_cloud));
        //     pcl::transformPointCloud(*cloud_acc, *cloud_acc, (pc->GetPoseTsw()*base_frame_transform_.inverse()).matrix());
        //     *pc_final+=*cloud_acc;
        // }
        
        // // 要基于body坐标系修复
        // // pcl::transformPointCloud(*cloud_in, *cloud_in, pc->GetPoseTsg().matrix());
        // // if (tmp%2==0){
            
        //     // tmp++;
        // // }

  
        
        if ( (pos_dis > 0.5 || rot_diff > 10 || time_diff > 1 ) ){
        //     if (pc_final->size() == 0){
        //         std::cout<<"Error pc_final->size() == 0"<< std::endl;
        //         // *pc_final+=pc->pts_cloud;
        //         return;
        //     }
            
       
            
            img->id_.second =  GetClientId();
            img->id_.first = send_cnt++ ;
            // pc->pts_cloud=*pc_final;
            // pc->SetPoseTsw(base_frame_transform_);
            std::cout << "send new pointcloud "<<img->id_.first << std::endl;
            PassImgToComm(img);

        //     // last_pos_=pc->pos_w;
        //     // last_quan_=pc->quan_; 
            last_transform_=img->GetPoseTsw();
            last_timestamp_=img->timestamp_;          
        //     // pc->pts_cloud.clear();
        // }else{
        //     // pc
        //     // 把多个frame一起打包压缩，减少传输次数的同时可以合并一些重复点
        //     // *pc_final+=pc->pts_cloud;
        }
        // // pc->pts_cloud.clear(); // shoud not clear it, because comm use 
        

}
// auto Communicator_client::ProcessLandmarkMessages()->void {

// }

// auto Communicator_client::ProcessNewKeyframes()->void {

// }

// auto Communicator_client::ProcessNewLandmarks()->void {

// }
auto Communicator_client::Run()->void {
    std::thread thread_recv(&Communicator_client::RecvMsg, this);
    thread_recv.detach();

    while(true)
    {
        // std::cout<< "123" << std::endl;
        this->ProcessPointCloudBuffer();
        this->ProcessImageBuffer();
        // this->ProcessKfBuffer();
        this->ProcessBufferOut();
        this->ProcessBufferIn();
        if(this->TryLock()){//执行tryLock()方法时未获得锁，则会立即返回false,不会阻塞
               this->ProcessPointCloudMessages();
        //     this->ProcessKeyframeMessages();
        //     this->ProcessLandmarkMessages();
        //     this->ProcessNewKeyframes();
        //     this->ProcessNewLandmarks();
        //     this->ProcessAdditional();
            this->UnLock();
        }

        if(this->ShallFinish()){
            this->tryReConnect();
            std::unique_lock<std::mutex>(mtx_pointcloud_queue_);
            if(!pointcloud_out_buffer_.empty()) {
                std::cout << "Comm:: waiting for pc_out_buffer_" << std::endl;
            } else {
                std::cout << "Comm " << client_id_ << ": close" << std::endl;
                break;
            }
            
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
