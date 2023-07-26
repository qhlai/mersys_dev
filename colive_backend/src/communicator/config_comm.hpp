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

#pragma once

//C++
#include <iostream>
#include <opencv2/opencv.hpp>

//COVINS
#include "tools_read_parm.hpp"
#include "typedefs_base.hpp"

namespace colive_params {

using precision_t = colive::TypeDefs::precision_t;

const std::string s0_comm (__FILE__);
const std::size_t p0_comm = s0_comm.find("colive/colive_backend/src/communicator");
const std::string s1_comm (s0_comm.substr(0,p0_comm));
const std::string s2_comm ("colive/colive_backend/config/config_comm.yaml");
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