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

#include "communicator_server.hpp"

// Socket Programming
#include <atomic>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

namespace colive {

Communicator_server::Communicator_server(int client_id, int newfd, MapManagerPtr man)
    : CommunicatorBase(client_id,newfd),
      mapmanager_(man)
    //   vis_(vis),
    //   placerec_(placerec)
{
    //Send client ID
    // std::cout << "Pass new ID " << client_id_ << " to client" << std::endl;
    // message_container out_container;
    // out_container.msg_info.push_back(1);
    // out_container.msg_info.push_back(client_id_);
    // while(out_container.msg_info.size() != ContainerSize*5)
    //     out_container.msg_info.push_back(0);
    // SendMsgContainer(out_container);
}

auto Communicator_server::Run()->void {
    // std::thread thread_recv(&Communicator_server::RecvMsg, this);
    // thread_recv.detach();

}

} //end ns
