

#include "config_comm.hpp"

namespace mersys_params {

auto GetPort()->std::string {
    return mersys_params::sys::port;
}

auto GetServerIP()->std::string {
    return mersys_params::sys::server_ip;
}

auto ShowParamsComm()->void
{
    std::cout  << "1111111111111" << s3_comm << std::endl; 

    std::cout << "++++++++++ System ++++++++++" << std::endl;
    std::cout << "server_ip: " << mersys_params::sys::server_ip << std::endl;
    std::cout << "port: " << mersys_params::sys::port << std::endl;
    std::cout << std::endl;
    std::cout << "++++++++++ Communication ++++++++++" << std::endl;
    std::cout << "send_updates: " << (int)mersys_params::comm::send_updates << std::endl;
    std::cout << "data_to_client: " << (int)mersys_params::comm::data_to_client << std::endl;
    std::cout << "start_sending_after_kf: " << mersys_params::comm::start_sending_after_kf << std::endl;
    std::cout << "kf_buffer_withold: " << mersys_params::comm::kf_buffer_withold << std::endl;
    std::cout << "max_sent_kfs_per_iteration: " << mersys_params::comm::max_sent_kfs_per_iteration << std::endl;
    std::cout << "update_window_size: " << mersys_params::comm::update_window_size << std::endl;
    std::cout << "to_agent_freq: " << mersys_params::comm::to_agent_freq << std::endl;
    std::cout << "++++++++++ ORB-SLAM3 ++++++++++" << std::endl;
    std::cout << "activate_visualization: " << (int)mersys_params::orb::activate_visualization << std::endl;
    std::cout << "imu_stamp_max_diff: " << mersys_params::orb::imu_stamp_max_diff << std::endl;
    std::cout << std::endl;
}

} //end ns
