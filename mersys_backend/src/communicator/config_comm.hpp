

#pragma once

//C++
#include <iostream>
#include <opencv2/opencv.hpp>

//MERSYS
#include "tools_read_parm.hpp"
#include "typedefs_base.hpp"

namespace mersys_params {

using precision_t = mersys::TypeDefs::precision_t;

const std::string s0_comm (__FILE__);
const std::size_t p0_comm = s0_comm.find("mersys/mersys_backend/src/communicator");
const std::string s1_comm (s0_comm.substr(0,p0_comm));
const std::string s2_comm ("mersys/mersys_backend/config/config_comm.yaml");
const std::string s3_comm = s1_comm + s2_comm;
const std::string conf_comm (s3_comm);//config_comm_path

// const std::string config_comm(__FILE__);
// opencv 的版本有问题
namespace sys {
    const std::string server_ip      = read_parm::GetStringFromYaml(conf_comm,"sys.server_ip");
    const std::string port           = read_parm::GetStringFromYaml(conf_comm,"sys.port");
}

namespace comm {
    const bool send_updates                             = read_parm::GetValFromYaml<bool>(conf_comm,"comm.send_updates");
    const bool data_to_client                           = read_parm::GetValFromYaml<bool>(conf_comm,"comm.data_to_client");
    const int start_sending_after_kf                    = read_parm::GetValFromYaml<int>(conf_comm,"comm.start_sending_after_kf");
    const int kf_buffer_withold                         = read_parm::GetValFromYaml<int>(conf_comm,"comm.kf_buffer_withold");
    const int max_sent_kfs_per_iteration                = read_parm::GetValFromYaml<int>(conf_comm,"comm.max_sent_kfs_per_iteration");
    const int update_window_size                        = read_parm::GetValFromYaml<int>(conf_comm,"comm.update_window_size");
    const precision_t to_agent_freq                     = read_parm::GetValFromYaml<precision_t>(conf_comm,"comm.to_agent_freq");
}

namespace orb {
    const bool activate_visualization                   = read_parm::GetValFromYaml<bool>(conf_comm,"orb.activate_visualization");
    const precision_t imu_stamp_max_diff                = read_parm::GetValFromYaml<precision_t>(conf_comm,"orb.imu_stamp_max_diff");
}

void ShowParamsComm();
auto GetServerIP()->std::string;
auto GetPort()->std::string;

} //end ns


// namespace sys {
//     const std::string server_ip      = "127.0.0.1";//read_parm::GetStringFromYaml(conf_comm,"sys.server_ip");
//     const std::string port           = "9033";//read_parm::GetStringFromYaml(conf_comm,"sys.port");
// }

// namespace comm {
//     const bool send_updates                             = 1;//read_parm::GetValFromYaml<bool>(conf_comm,"comm.send_updates");
//     const bool data_to_client                           = 0;//read_parm::GetValFromYaml<bool>(conf_comm,"comm.data_to_client");
//     const int start_sending_after_kf                    = 50;//read_parm::GetValFromYaml<int>(conf_comm,"comm.start_sending_after_kf");
//     const int kf_buffer_withold                         = 5;//read_parm::GetValFromYaml<int>(conf_comm,"comm.kf_buffer_withold");
//     const int max_sent_kfs_per_iteration                = 2;//read_parm::GetValFromYaml<int>(conf_comm,"comm.max_sent_kfs_per_iteration");
//     const int update_window_size                        = 5;//read_parm::GetValFromYaml<int>(conf_comm,"comm.update_window_size");
//     const precision_t to_agent_freq                     = 5;//read_parm::GetValFromYaml<precision_t>(conf_comm,"comm.to_agent_freq");
// }

// namespace orb {
//     const bool activate_visualization                   = 1;//read_parm::GetValFromYaml<bool>(conf_comm,"orb.activate_visualization");
//     const precision_t imu_stamp_max_diff                = 2.0;//read_parm::GetValFromYaml<precision_t>(conf_comm,"orb.imu_stamp_max_diff");
// }