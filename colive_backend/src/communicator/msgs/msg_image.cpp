#include "msgs/msg_image.hpp"
#include "image_ex.hpp"

namespace colive {

MsgImage::MsgImage()
//  : Image_ex()
{
    //...
}

MsgImage::MsgImage(bool filesave)
    :  save_to_file(filesave)
{
    //...
}

MsgImage::MsgImage(MsgTypeVector msgtype)
    : msg_type(msgtype)
{
    //...
}

auto MsgImage::SetMsgType(int msg_size)->void {
    msg_type[0] = msg_size;
    msg_type[1] = (int)is_update_msg;
    msg_type[2] = id_.first;
    msg_type[3] = id_.second;
    msg_type[4] = 0;
}

auto MsgImage::SetMsgType(MsgTypeVector msgtype)->void {
    msg_type = msgtype;
}

} //end ns
