
#include "msg_instruction.hpp"
namespace colive {

MsgInstruction::MsgInstruction()
//  : Image_ex()
{
    //...
}

MsgInstruction::MsgInstruction(bool filesave)
    :  save_to_file(filesave)
{
    //...
}

MsgInstruction::MsgInstruction(MsgTypeVector msgtype)
    : msg_type(msgtype)
{
    //...
}

auto MsgInstruction::SetMsgType(int msg_size)->void {
    msg_type[0] = msg_size;
    msg_type[1] = (int)is_update_msg;
    msg_type[2] = id_.first;
    msg_type[3] = id_.second;
    msg_type[4] = 3;
}

auto MsgInstruction::SetMsgType(MsgTypeVector msgtype)->void {
    msg_type = msgtype;
}

}