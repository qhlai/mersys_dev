
#include "frame_base.hpp"
namespace colive {
    auto FrameBase::GetClientID()->size_t {
        // std::unique_lock<std::mutex> lock_conn(mtx_connections_);
        return id_.second;
    }
    auto FrameBase::GetFrameID()->size_t {
        // std::unique_lock<std::mutex> lock_conn(mtx_connections_);
        return id_.first;
    }
    auto FrameBase::GetFrameClientID()->idpair {
        // std::unique_lock<std::mutex> lock_conn(mtx_connections_);
        return id_;
    }
    auto FrameBase::GetTimeStamp()->double {
        // std::unique_lock<std::mutex> lock_conn(mtx_connections_);
        return timestamp_;
    }
    auto FrameBase::SetErase()->void {
        std::unique_lock<std::mutex> lock_conn(mtx_connections_);
        not_erase_ = false;
    }
    auto FrameBase::SetNotErase()->void {
        std::unique_lock<std::mutex> lock_conn(mtx_connections_);
        not_erase_ = true;
    }

    auto FrameBase::GetPoseTcw()->TransformType {
        std::unique_lock<std::mutex> lock(mtx_pose_);
        return T_c_w_;
    }
    // nav_msgs::Odometry
    auto FrameBase::convert2T( nav_msgs::Odometry &odometry)->TransformType {
        return convert2T(
        odometry.pose.pose.position.x, 
        odometry.pose.pose.position.y,
        odometry.pose.pose.position.z,
        odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y,
        odometry.pose.pose.orientation.z,
        odometry.pose.pose.orientation.w
        );
    }

    auto FrameBase::convert2T(double x, double y,double z,double qx,double qy,double qz,double qw)->TransformType {
        QuaternionType q;
        Vector3Type pos_w;
        pos_w[0] = x;
        pos_w[1] = y;
        pos_w[2] = z;

        q.coeffs() << qx, qy, qz, qw;
        // transform.setRotation( q );

        TransformType T = TransformType::Identity();
        T.translate(pos_w);
        T.rotate(q);
        return T;
    }

    auto FrameBase::GetPoseTsw()->TransformType {
        std::unique_lock<std::mutex> lock(mtx_pose_);
        return T_s_w_;
    }

    // auto FrameBase::GetPoseTsw_vio()->TransformType {
    //     std::unique_lock<std::mutex> lock(mtx_pose_);
    //     return T_s_w_vio_;
    // }

    auto FrameBase::GetPoseTwc()->TransformType {
        std::unique_lock<std::mutex> lock(mtx_pose_);
        return T_w_c_;
    }

    auto FrameBase::GetPoseTws()->TransformType {
        std::unique_lock<std::mutex> lock(mtx_pose_);
        return T_w_s_;
    }
    auto FrameBase::GetPoseTsg()->TransformType {
        std::unique_lock<std::mutex> lock(mtx_pose_);
        return T_s_w_*T_w_g_;
    }
    auto FrameBase::GetPoseTgs()->TransformType {
        std::unique_lock<std::mutex> lock(mtx_pose_);
        return T_g_w_*T_w_s_;
    }
    auto FrameBase::GetPoseTwg()->TransformType {
        std::unique_lock<std::mutex> lock(mtx_pose_);
        return T_w_g_;
    }
    auto FrameBase::GetPoseTgw()->TransformType {
        std::unique_lock<std::mutex> lock(mtx_pose_);
        return T_g_w_;
    }
    auto FrameBase::SetPoseTsw(TransformType Tsw, bool lock_mtx)->void {
        if(lock_mtx) mtx_pose_.lock();
        // if (!m_lock_Tws_)
        // {
        //     m_lock_Tws_=true;
        // }else{
        //     std::cout<< COUTERROR <<"m_lock_Tws_=true;"<< std::endl;
        //     // if(lock_mtx) mtx_pose_.unlock();
        //     // return;
        // }
        T_s_w_ = Tsw;
        T_w_s_ = T_s_w_.inverse();
        T_w_c_ = T_w_s_*T_s_c_;
        T_c_w_ = T_w_c_.inverse();
        if(lock_mtx) mtx_pose_.unlock();
    }
    auto FrameBase::SetPoseTws(TransformType Tws, bool lock_mtx)->void {
        if(lock_mtx) mtx_pose_.lock();
        // if (!m_lock_Tws_)
        // {
        //     m_lock_Tws_=true;
        // }else{
        //     std::cout<< COUTERROR <<"m_lock_Tws_=true;"<< std::endl;
        //     // if(lock_mtx) mtx_pose_.unlock();
        //     // return;
        // }
        T_w_s_ = Tws;
        T_s_w_ = T_w_s_.inverse();
        T_w_c_ = T_w_s_*T_s_c_;
        T_c_w_ = T_w_c_.inverse();
        if(lock_mtx) mtx_pose_.unlock();
    }
    auto FrameBase::SetPoseTwg(TransformType Twg, bool lock_mtx)->void {
        if(lock_mtx) mtx_pose_.lock();
        T_w_g_ = Twg;
        T_g_w_ = T_w_g_.inverse();
        if(lock_mtx) mtx_pose_.unlock();
    }
}