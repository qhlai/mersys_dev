
#include "instruction.hpp"
namespace mersys {
Instruction::Instruction(MsgInstruction msg){
    id_= msg.id_;
    timestamp_=msg.timestamp_; 
    m_comm_direction=                 msg.m_comm_direction;  // src -> dst
    if_from_server= msg.if_from_server;
    T_correction =   msg.T_correction;
    // map_=map;
}
// auto PointCloud_ex::GetPoseTws()->TransformType {
//     std::unique_lock<std::mutex> lock(mtx_pose_);
//     return T_w_s_;
// }
auto Instruction::ConvertToMsg(mersys::MsgInstruction &msg, bool is_update, size_t cliend_id)->void{


    msg.id_= id_;
    msg.timestamp_=timestamp_; 
    msg.m_comm_direction=  m_comm_direction;  // src -> dst
    msg.if_from_server= if_from_server;
    msg.T_correction = T_correction;

}
}