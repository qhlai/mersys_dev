#include "test_frontend.hpp"

#include "map_co.hpp"
// #include <ros/ros.h>

// AgentPackage::AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man) {
//     agent_.reset(new AgentHandler(client_id,newfd,vis,man));
// }
namespace colive {



Frontend::Frontend(){
    std::cout<<"hello world!"<<std::endl;
    // some init 
    colive_params::ShowParamsComm();
    // colive_params::ShowParamsBackend();
}

auto Frontend::Run()->void {
    //+++++ Add the 1st agent +++++
     std::cout<<"client run"<<std::endl;
    // std::cout<<"backend start"<<std::endl;
    // this->AddClient();

}

// auto Frontend::ConnectSocket()->void {
 
// }


}

