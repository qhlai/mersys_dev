#include "test_client.hpp"


// AgentPackage::AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man) {
//     agent_.reset(new AgentHandler(client_id,newfd,vis,man));
// }
namespace mersys {



Frontend::Frontend(){
    std::cout<<"hello world!"<<std::endl;
    // some init 
    // mersys_params::ShowParamsComm();
    // mersys_params::ShowParamsBackend();
}

auto Frontend::Run()->void {
    //+++++ Add the 1st agent +++++
     std::cout<<"client run"<<std::endl;
    // std::cout<<"backend start"<<std::endl;
    // this->AddClient();
    std::cout << ">>> MERSYS: Initialize communicator" << std::endl;
    comm_.reset(new Communicator_client(mersys_params::sys::server_ip,mersys_params::sys::port));
    std::cout << ">>> MERSYS: Start comm thread" << std::endl;
    thread_comm_.reset(new std::thread(&Communicator_client::Run,comm_));

    // Get ID from back-end
    std::cout << ">>> MERSYS: wait for back-end response" << std::endl;
    while(comm_->GetClientId() < 0){
        usleep(1000); //wait until ID is received from server
    }
    std::cout << ">>> MERSYS: client id: " << comm_->GetClientId() << std::endl;
    {
        PointCloud_ex pc1;
        // TypeDefs::Vector3Type m(1.0,2.0,3.0);
        // pc1.pos_w =  m;
        PointCloud_ex* pc;
        pc = &pc1;
        int cnt=0;
    while(true){
        pc->id_.second = comm_->GetClientId() ;
        pc->id_.first = cnt++ ;
        std::cout << "send new pointcloud "<<pc->id_.first << std::endl;
        comm_->PassPcToComm(pc);
        sleep(3);
    }

    }
    // PointCloud_ex* pc;
    // pc
    // TypeDefs::PointCloudEXPtr pc;
    // pc.reset(new TypeDefs::PointCloudEX());
    // TypeDefs::Vector3Type m(1.0,2.0,3.0);
    // pc->pos_w =  m;

    // comm_->pointcloud_out_buffer_.push_back(pc);
    // pointcloud_out_buffer_.push_back(

}

// auto Frontend::ConnectSocket()->void {
 
// }


}

