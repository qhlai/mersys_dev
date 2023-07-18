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

#include "communicator_server.hpp"

// Socket Programming
#include <atomic>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "mapmanager.hpp"
#include "pointcloud_ex.hpp"
#include "image_ex.hpp"
#include "visualizer_be.hpp"

// #include "msgs/msg_landmark.hpp"
// #include "msgs/msg_keyframe.hpp"

#include "msgs/msg_pointcloud.hpp"
#include "msgs/msg_image.hpp"
// #include "msgs/msg_odometry.hpp"
#include "place_rec.hpp"

namespace colive {

Communicator_server::Communicator_server(int client_id, int newfd, MapManagerPtr man, PlacerecPtr placerec, VisPtr vis)
    : CommunicatorBase(client_id,newfd),
      mapmanager_(man),
      vis_(vis),
      placerec_(placerec)
{
    //Send client ID
    std::cout << "Pass new ID " << client_id_ << " to client" << std::endl;

    message_container out_container;
    out_container.msg_info.push_back(1);
    out_container.msg_info.push_back(client_id_);
    while(out_container.msg_info.size() != ContainerSize*5)
        out_container.msg_info.push_back(0);
    SendMsgContainer(out_container);

}
auto Communicator_server::CollectDataForAgent()->void {
    // if(most_recent_kf_id_ == defpair) return;
    // KeyframePtr kf_newest = map_->GetKeyframe(most_recent_kf_id_);
    // if(!kf_newest) {
    //     std::cout << COUTWARN << "cannot find KF " << most_recent_kf_id_ << std::endl;
    //     return;
    // }
    // KeyframePtr kf0 = map_->GetKeyframe(0,client_id_);
    // if(!kf_newest) {
    //     std::cout << COUTWARN << "cannot find KF 0" << std::endl;
    //     return;
    // }
    // if(kf_newest == kf0) return;
    // data_bundle map_chunk;
    // MsgKeyframe msg_kf;
    // kf_newest->ConvertToMsg(msg_kf,kf0,true);
    // map_chunk.keyframes.push_back(msg_kf);
    // this->PassDataBundle(map_chunk);
}

auto Communicator_server::ProcessPointCloudMessages()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    while(!buffer_pointclouds_in_.empty()) {
        MsgPointCloud msg = buffer_pointclouds_in_.front();
        // if(msg.id_reference.first > last_processed_kf_msg_.first) break;
        buffer_pointclouds_in_.pop_front();
        if(msg.is_update_msg){
            if(!colive_params::comm::send_updates) continue;
            else{
                // TODO: update
                // auto pc =map_->GetPointCloud();
            }
        }
        else{
            PointCloudEXPtr pc;
            TransformType Twg;
            pc = map_->GetPointCloudEX(msg.id_);// id: id+pointcloud
            Twg = map_->GetFamilyPc(msg.id_.second);
            if(!pc){
                // pc.reset(new PointCloud_ex(msg,map_));
                // map_->AddPointCloud(pc);
                
                pc.reset(new PointCloudEX(msg,map_));
                pc->SetPoseTwg(Twg);
                // pc->map_=map_;
                // std::cout<<"Added point cloud id: "<<pc->id_.first<<" of client:"<<pc->id_.second <<" pc size: "<<pc->pts_cloud.size()<<std::endl;
                pointclouds_new_.push_back(pc);
                last_processed_pc_msg_ = pc->id_;
            }else{
                // TODO: 
            }

        }
    
    
    }
    

}

auto Communicator_server::ProcessNewPointClouds()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);

    while(!buffer_pointclouds_in_.empty()) {
        MsgPointCloud msg = buffer_pointclouds_in_.front();
        // if(msg.id_reference.first > last_processed_kf_msg_.first) break;
        buffer_pointclouds_in_.pop_front();
        if(msg.is_update_msg){
            if(!colive_params::comm::send_updates) continue;
            else{
                // TODO: update
                // auto pc =map_->GetPointCloud();
            }
        }
        else{
            PointCloudEXPtr pc;
            TransformType Twg;
            pc = map_->GetPointCloudEX(msg.id_);// id: id+pointcloud
            Twg = map_->GetFamilyPc(msg.id_.second);
            if(!pc){
                // pc.reset(new PointCloud_ex(msg,map_));
                // map_->AddPointCloud(pc);
                
                pc.reset(new PointCloudEX(msg,map_));
                pc->SetPoseTwg(Twg);
                // pc->map_=map_;
                // std::cout<<"Added point cloud id: "<<pc->id_.first<<" of client:"<<pc->id_.second <<" pc size: "<<pc->pts_cloud.size()<<std::endl;
                pointclouds_new_.push_back(pc);
                last_processed_pc_msg_ = pc->id_;
            }else{
                // TODO: 
            }

        }
    }

    while(!pointclouds_new_.empty()) {
        PointCloudEXPtr pc = pointclouds_new_.front();
        pointclouds_new_.pop_front();

        if(colive_params::placerec::active)
            placerec_->InsertKeyframe(pc);
        // map_->UpdateCovisibilityConnections(kf->id_);

        // Keyframe::LandmarkVector landmarks = kf->GetLandmarks();

        // for(size_t idx=0;idx<landmarks.size();++idx) {
        //     LandmarkPtr i = landmarks[idx];
        //     if(!i) {
        //         continue;
        //     }
        //     i->ComputeDescriptor();
        //     i->UpdateNormal();
        // }

        if(static_cast<int>(pc->id_.second) == client_id_) {
            if(most_recent_pc_id_ == defpair) most_recent_pc_id_ = pc->id_;
            else most_recent_pc_id_.first = std::max(most_recent_pc_id_.first,pc->id_.first);
            // map_->pointclouds_.push_back(pc);
            map_->AddPointCloud(pc);
        }
    }
}

auto Communicator_server::ProcessNewImages()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);

    while(!buffer_images_in_.empty()) {
        MsgImage msg = buffer_images_in_.front();
        // if(msg.id_reference.first > last_processed_kf_msg_.first) break;
        buffer_images_in_.pop_front();
        if(msg.is_update_msg){
            if(!colive_params::comm::send_updates) continue;
            else{
                // TODO: update
                // auto pc =map_->GetPointCloud();
            }
        }
        else{
            ImageEXPtr img;
            TransformType Twg;
            img = map_->GetImageEX(msg.id_);// id: id+pointcloud
            Twg = map_->GetFamilyImg(msg.id_.second);
            if(!img){
                // pc.reset(new PointCloud_ex(msg,map_));
                // map_->AddPointCloud(pc);
                
                img.reset(new ImageEX(msg));
                img->SetPoseTwg(Twg);
                // pc->map_=map_;
                std::cout<<"Added image id: "<<img->id_.first<<" of client:"<<img->id_.second <<" img size: "<<img->img_.rows<<std::endl;
                images_new_.push_back(img);
                last_processed_img_msg_ = img->id_;
            }else{
                // TODO: 
            }

        }
    }

    while(!images_new_.empty()) {
        ImageEXPtr img = images_new_.front();
        images_new_.pop_front();

        // if(colive_params::placerec::active)
            // placerec_->InsertKeyframe(img);
        // map_->UpdateCovisibilityConnections(kf->id_);

        // Keyframe::LandmarkVector landmarks = kf->GetLandmarks();

        // for(size_t idx=0;idx<landmarks.size();++idx) {
        //     LandmarkPtr i = landmarks[idx];
        //     if(!i) {
        //         continue;
        //     }
        //     i->ComputeDescriptor();
        //     i->UpdateNormal();
        // }

        if(static_cast<int>(img->id_.second) == client_id_) {
            if(most_recent_img_id_ == defpair) most_recent_img_id_ = img->id_;
            else most_recent_img_id_.first = std::max(most_recent_img_id_.first,img->id_.first);
            // map_->pointclouds_.push_back(pc);
            map_->AddImage(img);
        }
    }
}

auto Communicator_server::Run()->void {
    std::thread thread_recv(&Communicator_server::RecvMsg, this);
    thread_recv.detach();

    auto last = std::chrono::steady_clock::now();
    double wait_time = 1.0/colive_params::comm::to_agent_freq;
    while(true)
    {
        int check_num_map;
        map_ = mapmanager_->CheckoutMapOrWait(client_id_,check_num_map);

        if(colive_params::comm::data_to_client) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = now-last;
            if(diff.count() > wait_time) {
                this->CollectDataForAgent();
                last = now;
            }
            this->ProcessBufferOut();
        }

        this->ProcessBufferIn();
        if(this->TryLock()){
            // this->ProcessPointCloudMessages();
            this->ProcessNewPointClouds();
            this->ProcessNewImages();


            // this->ProcessPointCloudMessages();
            // this->ProcessNewPointClouds();

            // this->ProcessKeyframeMessages();
            // this->ProcessLandmarkMessages();
            
            // this->ProcessNewKeyframes();
            // this->ProcessNewLandmarks();
            // this->ProcessAdditional();
            this->UnLock();
        }
        vis_->DrawMap(map_);
        // std::cout<< COUTDEBUG <<"return map"<<std::endl;
        mapmanager_->ReturnMap(client_id_,check_num_map);
        map_ = nullptr; //make sure the MapManager is used correctly - this will cause SEGFAULT if accessed

        if(this->ShallFinish()){
            std::cout << "Comm " << client_id_ << ": close" << std::endl;
            break;
        }
        usleep(1000);
    }

    std::unique_lock<std::mutex> lock(mtx_finish_);
    is_finished_ = true;


}

} //end ns
