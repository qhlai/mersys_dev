#include "msgs/msg_odometry.hpp"

namespace mersys {

MsgOdometry::MsgOdometry() {
    //...
}

MsgOdometry::MsgOdometry(bool filesave)
    : save_to_file(filesave)
{
    //...
}

MsgOdometry::MsgOdometry(MsgTypeVector msgtype)
    : msg_type(msgtype)
{
    //...
}

auto MsgOdometry::SetMsgType(int msg_size)->void {
    msg_type[0] = msg_size;
    msg_type[1] = (int)is_update_msg;
    msg_type[2] = id.first;
    msg_type[3] = id.second;
    msg_type[4] = 3;
}

auto MsgOdometry::SetMsgType(MsgTypeVector msgtype)->void {
    msg_type = msgtype;
}

} //end ns
