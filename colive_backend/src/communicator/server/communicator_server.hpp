

#pragma once

// C++
#include <unistd.h>
#include <mutex>
#include <thread>
#include <list>
#include <netinet/in.h>
#include <eigen3/Eigen/Core>

#include "communicator_base.hpp"

// COVINS
#include "typedefs_base.hpp"
#include "config_comm.hpp"
#include "map_rgb.hpp"

// #include "msgs/msg_landmark.hpp"
#define ContainerSize 10

namespace colive {



// class KeyFrame;
class Communicator_server : public CommunicatorBase, public std::enable_shared_from_this<Communicator_server> {
public:

    using MapPtr                        = TypeDefs::MapPtr;
    using MapManagerPtr                 = TypeDefs::MapManagerPtr;
    using PlacerecPtr                   = TypeDefs::PlacerecPtr;
    using VisPtr                        = TypeDefs::VisPtr;

    using PointCloudEX          = TypeDefs::PointCloudEX;
    using ImageEX          = TypeDefs::ImageEX;
    using PointCloudEXPtr          = TypeDefs::PointCloudEXPtr;
    using ImageEXPtr          = TypeDefs::ImageEXPtr;
    // using KeyframeList                  = TypeDefs::KeyframeList;
    // using LandmarkList                  = TypeDefs::LandmarkList;
    using PointCloudEXList  = TypeDefs::PointCloudEXList;  
    using ImageEXList  = TypeDefs::ImageEXList;  
    // using MsgPointCloud  = TypeDefs::MsgPointCloud;  

public:
    Communicator_server(int client_id, int newfd, MapManagerPtr man, PlacerecPtr placerec, VisPtr vis);

    // main function
    virtual auto Run()            ->void;

protected:

    virtual auto CollectDataForAgent()              ->void;


    // // data handling
    virtual auto ProcessPointCloudIn()->void;

    virtual auto ProcessImagesIn()->void;

    virtual auto ProcessInstructionIn()->void;    
    virtual auto ProcessInstructionOut()->void;

 
    // LM Culling
    // auto LandmarkCulling(size_t min_obs, size_t max_gap)                                ->int;
    // auto KeyframeCulling(double th_red, int recent_window_size)                         ->void;

    // Infrastructure
    MapPtr                         map_                                                    = nullptr;
    MapManagerPtr                  mapmanager_                                             = nullptr;
    VisPtr                      vis_                                                    = nullptr;
    PlacerecPtr                 placerec_                                               = nullptr;

     //data
    idpair                      most_recent_img_id_                                      = defpair;
    idpair                      most_recent_pc_id_                                      = defpair;

    PointCloudEXList                pointclouds_new_;
    ImageEXList                     images_new_;
    PointCloudEXPtr             p_pc_large_tmp;
    // LandmarkList                landmarks_new_;

    //data
    // idpair                      most_recent_kf_id_                                      = defpair;

    // KeyframeList                keyframes_new_;
    // LandmarkList                landmarks_new_;

    idpair                      last_processed_img_msg_                                  = defpair;
    idpair                      last_processed_pc_msg_                                  = defpair;
    // LM Culling
    // LandmarkList                recent_landmarks_;
    // KeyframeList                recent_keyframes_;


};

} //end ns
