/**
* This file is part of MERSYS.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/covins>
*
* MERSYS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* MERSYS is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with MERSYS. If not, see <http://www.gnu.org/licenses/>.
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
#include "msgs/msg_instruction.hpp"
// #include "msgs/msg_odometry.hpp"
#include "place_rec.hpp"

namespace mersys {

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

    p_pc_large_tmp=nullptr;

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

auto Communicator_server::ProcessInstructionOut()->void {

}

auto Communicator_server::ProcessInstructionIn()->void {
    std::unique_lock<std::mutex>(mtx_in_);
    u16 cnt =0;

    while(!buffer_pointclouds_in_.empty()) {

        MsgPointCloud msg = buffer_pointclouds_in_.front();

        // if(msg.id_reference.first > last_processed_kf_msg_.first) break;

        buffer_pointclouds_in_.pop_front();
        if(msg.is_update_msg){
            if(!mersys_params::comm::send_updates) continue;
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
        // mersys::data_bundle map_chunk;
        // mersys::MsgPointCloud msg_ptcloud;
        // Vector3Type m(1.0,2.0,3.0);
        // ptcloud->ConvertToMsg(msg_ptcloud,m ,ptcloud->sent_once_,client_id_);
        // ptcloud->sent_once_ = true;
        // map_chunk.pointclouds.push_back(msg_ptcloud);

        // this->PassDataBundle(map_chunk);

        // if(cnt >= mersys_params::comm::max_sent_kfs_per_iteration) break;


    }
    // while(!kf_out_buffer_.empty())

}

auto Communicator_server::ProcessPointCloudIn()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);

    while(!buffer_pointclouds_in_.empty()) {
        MsgPointCloud msg = buffer_pointclouds_in_.front();
        // if(msg.id_reference.first > last_processed_kf_msg_.first) break;
        buffer_pointclouds_in_.pop_front();
        if(msg.is_update_msg){
            if(!mersys_params::comm::send_updates) continue;
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
        bool if_large;
        if(static_cast<int>(pc->id_.second) == client_id_) {
            if(most_recent_pc_id_ == defpair) most_recent_pc_id_ = pc->id_;
            else most_recent_pc_id_.first = std::max(most_recent_pc_id_.first,pc->id_.first);
            // map_->pointclouds_.push_back(pc);

            map_->AddPointCloud(pc);

            if_large=map_->LongTimeStay(pc);

        }

        if(mersys_params::placerec::active){
            // std::cout << "55555555"<<std::endl;
            placerec_->InsertKeyframe(pc);
            // std::cout << "65555555"<<std::endl;
            if(if_large)
                placerec_->InsertLargeKeyframe(map_->p_pc_large_tmp);      
        }
        // std::cout << "75555555"<<std::endl;
    }
}
auto Communicator_server::ProcessImagesIn()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);

    while(!buffer_images_in_.empty()) {
        MsgImage msg = buffer_images_in_.front();
        // if(msg.id_reference.first > last_processed_kf_msg_.first) break;
        buffer_images_in_.pop_front();
        if(msg.is_update_msg){
            if(!mersys_params::comm::send_updates) continue;
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
                mapmanager_->PushImg(img);
                last_processed_img_msg_ = img->id_;
            }else{
                // TODO: 
            }

        }
    }

    while(!images_new_.empty()) {
        ImageEXPtr img = images_new_.front();
        images_new_.pop_front();

        // if(mersys_params::placerec::active)
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
            #ifdef SAVE_FRAMES           
            if(mersys_params::sys::save_frames){
                img->save_to_png( std::string(mersys_params::sys::output_dir).append("/frames/img/").append(std::to_string(img->GetClientID())).append("/"), std::to_string(img->GetTimeStamp()));
            }
            #endif
        }
        if(mersys_params::placerec::active){
            placerec_->InsertKeyframe1(img);
            
        }
    }

    
}


auto Communicator_server::Run()->void {
    std::thread thread_recv(&Communicator_server::RecvMsg, this);
    thread_recv.detach();

    auto last = std::chrono::steady_clock::now();
    double wait_time = 1.0/mersys_params::comm::to_agent_freq;
    while(true)
    {
        // if (client_id_==2){
        //     std::cout << COUTWARN << "client_id_==2: "<< client_id_<< std::endl;
        // }

        int check_num_map;
        map_ = mapmanager_->CheckoutMapOrWait(client_id_,check_num_map);

        
        if(mersys_params::comm::data_to_client) {
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

            // this->ProcessPointCloudIn();
            this->ProcessPointCloudIn();
            this->ProcessImagesIn();


            // this->ProcessPointCloudIn();
            // this->ProcessPointCloudIn();

            // this->ProcessKeyframeMessages();
            // this->ProcessLandmarkMessages();
            
            // this->ProcessNewKeyframes();
            // this->ProcessNewLandmarks();
            // this->ProcessAdditional();
            vis_->DrawMap(map_);
            this->UnLock();
        }
        
        // if(client_id_!=0){
        //     std::cout<<COUTDEBUG<<":"<<client_id_<<std::endl;
        // }
        // std::cout << COUTDEBUG << "ReturnMap "<< client_id_<<check_num_map<< std::endl;
        mapmanager_->ReturnMap(client_id_,check_num_map);// check_num error
        
        map_ = nullptr; //make sure the MapManager is used correctly - this will cause SEGFAULT if accessed

        if(this->ShallFinish()){
            std::cout << "Comm " << client_id_ << ": close" << std::endl;
            break;
        }
        usleep(5000); // 5ms
    }

    std::unique_lock<std::mutex> lock(mtx_finish_);
    is_finished_ = true;


}

} //end ns
