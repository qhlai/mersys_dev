#include "test_client.hpp"

#include "map_co.hpp"
#include "pointcloud_ex.hpp"
// #include <ros/ros.h>

// AgentPackage::AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man) {
//     agent_.reset(new AgentHandler(client_id,newfd,vis,man));
// }
namespace colive {



Frontend::Frontend(){
    std::cout<<"hello world!"<<std::endl;
    // some init 
    // colive_params::ShowParamsComm();
    // colive_params::ShowParamsBackend();
}

auto Frontend::Run()->void {
    //+++++ Add the 1st agent +++++
     std::cout<<"client run"<<std::endl;
    // std::cout<<"backend start"<<std::endl;
    // this->AddClient();
    std::cout << ">>> COVINS: Initialize communicator" << std::endl;
    comm_.reset(new Communicator_client(colive_params::sys::server_ip,colive_params::sys::port));
    std::cout << ">>> COVINS: Start comm thread" << std::endl;
    thread_comm_.reset(new std::thread(&Communicator_client::Run,comm_));

    // Get ID from back-end
    std::cout << ">>> COVINS: wait for back-end response" << std::endl;
    while(comm_->GetClientId() < 0){
        usleep(1000); //wait until ID is received from server
    }
    std::cout << ">>> COVINS: client id: " << comm_->GetClientId() << std::endl;

    PointCloud_ex pc;

    // pointcloud_out_buffer_.push_back(

}

// auto Frontend::ConnectSocket()->void {
 
// }


}

