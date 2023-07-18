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


// Socket Programming
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <atomic>

#include "communicator_base.hpp"
#include "communicator_server.hpp"
#include "map_co.hpp"
#include "mapmanager.hpp"
#include "typedefs_base.hpp"
#include "visualizer_be.hpp"
#include "place_rec.hpp"

#include "tools_logger.hpp"
#include "tools_mem_used.h"

// tools_thread_pool.hpp

namespace colive {


// extern std::shared_ptr<Common_tools::ThreadPool> m_thread_pool_ptr
// std::shared_ptr< Common_tools::ThreadPool > m_thread_pool_ptr;

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
    Client(size_t client_id, int newfd, MapManagerPtr man, VisPtr vis);
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
    using VisPtr                        = TypeDefs::VisPtr;
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
    // void print_dash_board();

    std::condition_variable sig_buffer;



protected:

    auto AddClient()                ->void;
    auto AcceptClient()             ->void;
    auto ConnectSocket()            ->void;

 




    MapManagerPtr                mapmanager_;
    VisPtr                      vis_;
    ClientVector                 clients_;

    ThreadPtr                   thread_mapmanager_;
    ThreadPtr                   thread_vis_;

    // std::shared_ptr< Common_tools::ThreadPool > m_thread_pool_ptr;

   int agent_next_id_              = 0;    

   // g_las
    double g_last_stamped_mem_mb = 0;

    //comm
    fd_set                      master_;
    fd_set                      read_fds_;
    int                         listener_, newfd_;

    // Device Counter
    std::atomic<int>            counter_, overall_counter_;


    std::mutex mtx_buffer;
};

}
