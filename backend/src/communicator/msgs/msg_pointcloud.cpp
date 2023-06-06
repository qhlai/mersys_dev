#include "msgs/msg_pointcloud.hpp"

namespace colive {

MsgPointcloud::MsgPointcloud() {
    //...
}

MsgPointcloud::MsgPointcloud(bool filesave)
    : save_to_file(filesave)
{
    //...
}

MsgPointcloud::MsgPointcloud(MsgTypeVector msgtype)
    : msg_type(msgtype)
{
    //...
}

auto MsgPointcloud::SetMsgType(int msg_size)->void {
    msg_type[0] = msg_size;
    msg_type[1] = (int)is_update_msg;
    msg_type[2] = id.first;
    msg_type[3] = id.second;
    msg_type[4] = 1;
}

auto MsgPointcloud::SetMsgType(MsgTypeVector msgtype)->void {
    msg_type = msgtype;
}

} //end ns
