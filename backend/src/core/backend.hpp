#pragma once

// C++
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <iostream>   // std::cout  
#include <string>     // std::string, std::to_string

#include <ros/ros.h>
#include <condition_variable>

#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_data_io.hpp"
#include "tools_timer.hpp"
#include "tools_thread_pool.hpp"
#include "tools_ros.hpp"

#include "config_comm.hpp"
#include "config_backend.hpp"
#include "communicator_server.hpp"
#include "map_co.hpp"

// Socket Programming
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <atomic>

#include "mapmanager.hpp"
#include "typedefs_base.hpp"
#include "communicator_base.hpp"
#include "communicator_server.hpp"
namespace colive {

class Client{
public:
    using CommPtr                       = TypeDefs::CommServerPtr;
    using MapPtr                     = TypeDefs::MapPtr;
    using MapManagerPtr              = TypeDefs::MapManagerPtr;
    using PlacerecPtr                   = TypeDefs::PlacerecPtr;
    using VisPtr                        = TypeDefs::VisPtr;
    using ThreadPtr                     = TypeDefs::ThreadPtr;
    
public:
    Client()=delete;
    Client(size_t client_id_);
    Client(size_t client_id, int newfd, MapManagerPtr man);
    auto Run()->void;

protected:
    size_t client_id_;
    MapPtr  map_ptr;
    CommPtr                     comm_;
    MapManagerPtr mapmanager_;
    PlacerecPtr                 placerec_;
    ThreadPtr                   thread_comm_;
    ThreadPtr                   thread_placerec_;
};

class Backend{
    public:
    using ThreadPtr                  = TypeDefs::ThreadPtr;
    
    using MapPtr                     = TypeDefs::MapPtr;
    using MapManagerPtr              = TypeDefs::MapManagerPtr;
    using ClientPtr                  = TypeDefs::ClientPtr;
    using ClientVector               = TypeDefs::ClientVector;
    public:
    

    ros::NodeHandle             m_ros_node_handle;
    ClientPtr m_client;

    Backend();


    auto Run()->void;

    // void feat_points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg_in);
    // void image_callback(const sensor_msgs::ImageConstPtr &msg);
    // void image_comp_callback(const sensor_msgs::CompressedImageConstPtr &msg);
    
    std::condition_variable sig_buffer;



protected:

    auto AddClient()                ->void;
    auto AcceptClient()             ->void;
    auto ConnectSocket()            ->void;

    int agent_next_id_              = 0;
    //comm
    fd_set                      master_;
    fd_set                      read_fds_;
    int                         listener_, newfd_;

    ThreadPtr                   thread_mapmanager_;

    MapManagerPtr                mapmanager_;
    ClientVector                 clients_;

    // Device Counter
    std::atomic<int>            counter_, overall_counter_;


    std::mutex mtx_buffer;
};

}
