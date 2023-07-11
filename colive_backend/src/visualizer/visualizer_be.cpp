#include "visualizer_be.hpp"

namespace colive{




Visualizer::Visualizer(std::string topic_prefix)
    : topic_prefix_(topic_prefix)
{
    std::string marker_topic = "colive_markers";
    std::string cloud_topic = "colive_cloud";
    std::string odom_topic = "colive_odom";
    marker_topic += topic_prefix;
    cloud_topic += topic_prefix;
    odom_topic += topic_prefix;
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic,10);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic,10);
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 10);
    pub_odom_vec_.reserve(MAX_CLIENT_NUM);//for publish odoms
    m_pub_rgb_render_pointcloud_ptr_vec.resize(1e3);
    std::cout<<"init vis"<<std::endl;
}
// 废弃的
Visualizer::Visualizer(std::string topic_prefix, MapManagerPtr mapmanager)
    : topic_prefix_(topic_prefix)
{
    std::string marker_topic = "colive_markers";
    std::string cloud_topic = "colive_cloud";
    std::string odom_topic = "colive_odom";
    marker_topic += topic_prefix;
    cloud_topic += topic_prefix;
    odom_topic += topic_prefix;
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic,10);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic,10);
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 10);
    std::cout<<"init vis"<<std::endl;
    mapmanager_=mapmanager;
}
auto Visualizer::CheckVisData()->bool
{
    std::unique_lock<std::mutex> lock(mtx_draw_);
    return !vis_data_.empty();
}

auto Visualizer::CreatePoint3D(Eigen::Vector3d &p3D, size_t client_id)->pcl::PointXYZRGB {
    pcl::PointXYZRGB p;
    p.x = colive_params::vis::scalefactor*p3D(0);
    p.y = colive_params::vis::scalefactor*p3D(1);
    p.z = colive_params::vis::scalefactor*p3D(2);

    if(client_id < 12) {
        p.r = static_cast<uint8_t>(colive_params::colors::col_vec[client_id].mu8R);
        p.g = static_cast<uint8_t>(colive_params::colors::col_vec[client_id].mu8G);
        p.b = static_cast<uint8_t>(colive_params::colors::col_vec[client_id].mu8B);
    } else {
        p.r = 127;
        p.g = 127;
        p.b = 127;
    }

    return p;
}

auto Visualizer::MakeColorMsg(float fR,float fG, float fB)->std_msgs::ColorRGBA {
    std_msgs::ColorRGBA msg;
    msg.r = fR;
    msg.g = fG;
    msg.b = fB;
    msg.a = 1.0;
    return msg;
}

auto Visualizer::RequestReset()->void {
    {
        std::unique_lock<std::mutex> lock(mtx_reset_);
        reset_ = true;
    }

    while(1){
        {
            std::unique_lock<std::mutex> lock(mtx_reset_);
            if(!reset_)
                break;
        }
        usleep(1000);
    }
}

auto Visualizer::ResetIfRequested()->void
{
    std::unique_lock<std::mutex> lock(mtx_reset_);

    if(reset_) {
        std::unique_lock<std::mutex> lock(mtx_draw_);
        vis_data_.clear();
        reset_=false;
    }
}


auto Visualizer::DrawMap(MapPtr map)->void {
    // 
    if(!map) return;
    // std::cout<<"draw vis"<<std::endl;
    std::unique_lock<std::mutex> lock(mtx_draw_);
    VisBundle vb;
    vb.pointCloud =map->GetPointCloudEXs();
    // vb.keyframes = map->GetKeyframes();
    // vb.keyframes_erased = map->GetKeyframesErased();
    // vb.landmarks = map->GetLandmarks();
    vb.id_map = map->id_map_;
    vb.associated_clients = map->associated_clients_;
    vb.loops = map->GetLoopConstraints();
    vb.frame = "camera_init";

    // if(vb.keyframes.size() < 3) return;
    if(vb.pointCloud.size() < 3) return;
    vis_data_[map->id_map_] = vb;
}
void pointcloud_convert(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_in,pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out){

    for (size_t i = 0; i < pc_in->size(); ++i)
    {
    pcl::PointXYZI point;
    point.x = pc_in->points[i].x;
    point.y = pc_in->points[i].y;
    point.z = pc_in->points[i].z;
    point.intensity = pc_in->points[i].intensity;
    pc_out->push_back(point);
    }
}
auto Visualizer::PubTrajectories()->void {


    PointCloudEXSetById pctraj;
    std::set<int> pccids;

    for(PointCloudEXMap::const_iterator mit=curr_bundle_.pointCloud.begin();mit!=curr_bundle_.pointCloud.end();++mit){
        // KeyframePtr kf = mit->second;
        PointCloudEXPtr pc = mit->second;

        pctraj.insert(pc);
        pccids.insert(pc->id_.second);
    }

    std::map<int,visualization_msgs::Marker> msgs;
    for(std::set<int>::iterator sit = pccids.begin(); sit != pccids.end(); ++sit){
        int cid = *sit;
        visualization_msgs::Marker traj;
        traj.header.frame_id = curr_bundle_.frame;
        traj.header.stamp = ros::Time::now();
        std::stringstream ss;
        ss << "Traj" << cid << topic_prefix_;
        traj.ns = ss.str();
        traj.id=0;
        traj.type = visualization_msgs::Marker::LINE_STRIP;
        traj.scale.x=colive_params::vis::trajmarkersize;
        traj.action=visualization_msgs::Marker::ADD;

        traj.color = MakeColorMsg(0.5,0.5,0.5);
        // if(cid < 12){
        //     covins_params::VisColorRGB col = covins_params::colors::col_vec[cid];
        //     traj.color = MakeColorMsg(col.mfR,col.mfG,col.mfB);
        // }
        // else
        //     traj.color = MakeColorMsg(0.5,0.5,0.5);

        msgs[cid] = traj;
    }

    precision_t scale = colive_params::vis::scalefactor;

    for(PointCloudEXSetById::iterator sit = pctraj.begin(); sit != pctraj.end(); ++sit){
        PointCloudEXPtr  pc = *sit;
        // Eigen::Matrix3d T = pc->pos_w;
        // Eigen::Matrix4d T = pc->GetPoseTws();
        Eigen::Matrix4d T = pc->GetPoseTsw().matrix();
        geometry_msgs::Point p;

        p.x = scale*(T(0,3));
        p.y = scale*(T(1,3));
        p.z = scale*(T(2,3));
// std::cout<<"x:"<<pc->pos_w[0]<<" "<<(T(0,3))<<"y:"<<pc->pos_w[1]<<" "<<(T(1,3))<<std::endl;
        // p.x = scale*pc->pos_w[0];
        // p.y = scale*pc->pos_w[1];
        // p.z = scale*pc->pos_w[2];
        msgs[pc->id_.second].points.push_back(p);
    }

    for(std::map<int,visualization_msgs::Marker>::iterator mit = msgs.begin(); mit != msgs.end(); ++mit){
        visualization_msgs::Marker msg = mit->second;
        msg.pose.orientation.w = 1.0;
        pub_marker_.publish(msg);
    }

}
// void R3LIVE::service_pub_rgb_maps()
auto Visualizer::PubOdometries()->void {
    // TODO: multi client odom pub
    PointCloudEXPtr pc = curr_bundle_.pointCloud.rbegin()->second;
    TransformType T = pc->GetPoseTsw();
    Eigen::Vector3d position = T.translation();
    // Eigen::Matrix3d euler = T.rotation();
    Eigen::Quaterniond q(T.rotation());

    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.child_frame_id = "body";
    
    odom.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.position.x = position[0];
    odom.pose.pose.position.y = position[1];
    odom.pose.pose.position.z = position[2];
    pub_odom_.publish( odom );

}
auto Visualizer::PubLoopEdges()->void {
    if(curr_bundle_.pointCloud.empty() && curr_bundle_.keyframes.empty()){
        std::cout << COUTWARN << "no KFs on VisBundle" << std::endl;
        return;
    }

    LoopVector loops = curr_bundle_.loops;

    if(loops.empty())
        return;

    visualization_msgs::Marker msg_intra; // loops between KFs from the same agent
    visualization_msgs::Marker msg_inter; // loops between KFs from different agents

    msg_intra.header.frame_id = curr_bundle_.frame;
    msg_intra.header.stamp = ros::Time::now();
    std::stringstream ss;
    ss << "LoopEdges_intra_" << curr_bundle_.id_map << topic_prefix_ << std::endl;
    msg_intra.ns = ss.str();
    msg_intra.type = visualization_msgs::Marker::LINE_LIST;
    msg_intra.color = MakeColorMsg(1.0,0.0,0.0);
    msg_intra.action = visualization_msgs::Marker::ADD;
    msg_intra.scale.x = colive_params::vis::loopmarkersize;
    msg_intra.id = 0;

    msg_inter.header.frame_id = curr_bundle_.frame;
    msg_inter.header.stamp = ros::Time::now();
    ss = std::stringstream();
    ss << "LoopEdges_inter_" << curr_bundle_.id_map << topic_prefix_ << std::endl;
    msg_inter.ns = ss.str();
    msg_inter.type = visualization_msgs::Marker::LINE_LIST;
    msg_inter.color = MakeColorMsg(1.0,0.0,0.0);
    msg_inter.action = visualization_msgs::Marker::ADD;
    msg_inter.scale.x = colive_params::vis::loopmarkersize;
    msg_inter.id = 1;

    precision_t scale = colive_params::vis::scalefactor;


    for(size_t idx = 0; idx<loops.size(); ++idx) {
        TransformType T1, T2;

        bool b_intra = false;
        if(loops[idx].type == 0){
            PointCloudEXPtr pc1 = loops[idx].pc1;
            PointCloudEXPtr pc2 = loops[idx].pc2;
            b_intra = (pc1->GetClientID() == pc2->GetClientID());
            T1 = pc1->GetPoseTsw();
            T2 = pc2->GetPoseTsw();
            std::cout <<COUTDEBUG << pc1->GetFrameID()<<"<->" << pc2->GetFrameID() << std::endl;
        }
        else{
            //TODO::
        }


        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        Eigen::Vector3d pos1, pos2;
        pos1= T1.translation();
        pos2= T2.translation();
        p1.x = scale*((double)(pos1[0]));
        p1.y = scale*((double)(pos1[1]));
        p1.z = scale*((double)(pos1[2]));

        p2.x = scale*((double)(pos2[0]));
        p2.y = scale*((double)(pos2[1]));
        p2.z = scale*((double)(pos2[2]));

        if(b_intra){
            msg_intra.points.push_back(p1);
            msg_intra.points.push_back(p2);
        } else {
            msg_inter.points.push_back(p1);
            msg_inter.points.push_back(p2);
        }
    }

    pub_marker_.publish(msg_intra);
    pub_marker_.publish(msg_inter);
}
// void R3LIVE::service_pub_rgb_maps()
auto Visualizer::PubPointCloud_service()->void {
    pcl::PointCloud< pcl::PointXYZI> pc_sum;
    pcl::PointCloud< pcl::PointXYZRGB > pc_rgb;
    pcl::PointCloud< pcl::PointXYZI> pc;
    pcl::PointCloud< pcl::PointXYZI>::Ptr pc_tmp;
    sensor_msgs::PointCloud2            ros_pc_msg;

    // pc.resize( number_of_pts_per_topic);
    uint64_t pub_idx_size = 0;
    int cur_topic_idx = 0;

    for(PointCloudEXMap::const_iterator mit=curr_bundle_.pointCloud.begin();mit!=curr_bundle_.pointCloud.end();++mit) {
        PointCloudEXPtr pc_ex = mit->second;
        PointCloud::Ptr pc_tmp(new PointCloud(pc_ex->pts_cloud));
        pcl::transformPointCloud(*pc_tmp, *pc_tmp, pc_ex->GetPoseTsg().matrix());
        pc_sum+=*pc_tmp;
    }
    std::cout << COUTNOTICE <<"rviz:pointcloud points sum:"<< pc_sum.points.size() << std::endl;
    for(uint64_t i=0;i<pc_sum.points.size();i++) {
        // PointCloudEXPtr pc_ex = mit->second;

        // PointCloud::Ptr pc_tmp(new PointCloud(pc_ex->pts_cloud));

        // std::cout<<"pc pos_w:"<<pc_i->pos_w<<std::endl;
        // std::cout<<"pc size:"<<pc_i->pts_cloud.size()<<std::endl;

        // pcl::transformPointCloud(*pc_tmp, *pc_tmp, pc_ex->GetPoseTsg().matrix());
        pc.points.push_back(pc_sum.points[i]);
        // pc.points[i].x=pc_sum.points[i].x;
        // pc.points[i].y=pc_sum.points[i].y;
        // pc.points[i].z=pc_sum.points[i].z;
        pub_idx_size++;
        if(pub_idx_size == number_of_pts_per_topic){
            pub_idx_size = 0;
            pcl::toROSMsg( pc, ros_pc_msg );
            ros_pc_msg.header.frame_id = "camera_init";       
            ros_pc_msg.header.stamp = ros::Time::now(); 
            if ( m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] == nullptr )
            {
                m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] =
                        std::make_shared< ros::Publisher >( nh_.advertise< sensor_msgs::PointCloud2 >(
                            std::string( "/RGB_map_" ).append( std::to_string( cur_topic_idx ) ), 100 ) );
            }
            m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ]->publish( ros_pc_msg );
            pc.clear();
            std::this_thread::sleep_for( std::chrono::microseconds( sleep_time_aft_pub ) );
            // ros::spinOnce();
            // ros::spinOnce();
            cur_topic_idx++;

        }
    }
    pc.resize( pub_idx_size );
    pcl::toROSMsg( pc, ros_pc_msg );
    ros_pc_msg.header.frame_id = "world";       
    ros_pc_msg.header.stamp = ros::Time::now(); 
    if ( m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] == nullptr )
    {
        m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] =
            std::make_shared< ros::Publisher >( nh_.advertise< sensor_msgs::PointCloud2 >(
                std::string( "/RGB_map_" ).append( std::to_string( cur_topic_idx ) ), 100 ) );
    }
    std::this_thread::sleep_for( std::chrono::microseconds( sleep_time_aft_pub ) );
    // ros::spinOnce();
    m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ]->publish( ros_pc_msg );
    pc.clear();
    cur_topic_idx++;

    if ( cur_topic_idx >= 40 ) // Maximum pointcloud topics = 45.
    {
        number_of_pts_per_topic *= 1.5;
        sleep_time_aft_pub *= 1.5;
    }

}

auto Visualizer::PubPointCloud_service_bak()->void {
    pcl::PointCloud< pcl::PointXYZRGB > pc_rgb;
    pcl::PointCloud< pcl::PointXYZI> pc;
    pcl::PointCloud< pcl::PointXYZI>::Ptr pc_tmp;
    sensor_msgs::PointCloud2            ros_pc_msg;

    pc.resize( number_of_pts_per_topic);
    int pub_idx_size = 0;
    int cur_topic_idx = 0;

    for(PointCloudEXMap::const_iterator mit=curr_bundle_.pointCloud.begin();mit!=curr_bundle_.pointCloud.end();++mit) {
        PointCloudEXPtr pc_ex = mit->second;

        PointCloud::Ptr pc_tmp(new PointCloud(pc_ex->pts_cloud));

        // std::cout<<"pc pos_w:"<<pc_i->pos_w<<std::endl;
        // std::cout<<"pc size:"<<pc_i->pts_cloud.size()<<std::endl;

        pcl::transformPointCloud(*pc_tmp, *pc_tmp, pc_ex->GetPoseTsg().matrix());
        pc+=*pc_tmp;
        if(pc.points.size() >= number_of_pts_per_topic){
            pcl::toROSMsg( pc, ros_pc_msg );
            ros_pc_msg.header.frame_id = "camera_init";       
            ros_pc_msg.header.stamp = ros::Time::now(); 
            if ( m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] == nullptr )
            {
                m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] =
                        std::make_shared< ros::Publisher >( nh_.advertise< sensor_msgs::PointCloud2 >(
                            std::string( "/RGB_map_" ).append( std::to_string( cur_topic_idx ) ), 100 ) );
            }
            m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ]->publish( ros_pc_msg );
            pc.clear();
            std::this_thread::sleep_for( std::chrono::microseconds( sleep_time_aft_pub ) );
            // ros::spinOnce();
            // ros::spinOnce();
            cur_topic_idx++;

        }
    }

    pcl::toROSMsg( pc, ros_pc_msg );
    ros_pc_msg.header.frame_id = "world";       
    ros_pc_msg.header.stamp = ros::Time::now(); 
    if ( m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] == nullptr )
    {
        m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] =
            std::make_shared< ros::Publisher >( nh_.advertise< sensor_msgs::PointCloud2 >(
                std::string( "/RGB_map_" ).append( std::to_string( cur_topic_idx ) ), 100 ) );
    }
    std::this_thread::sleep_for( std::chrono::microseconds( sleep_time_aft_pub ) );
    // ros::spinOnce();
    m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ]->publish( ros_pc_msg );
    pc.clear();
    cur_topic_idx++;

    if ( cur_topic_idx >= 40 ) // Maximum pointcloud topics = 45.
    {
        number_of_pts_per_topic *= 1.5;
        sleep_time_aft_pub *= 1.5;
    }

}
auto Visualizer::PubPointCloud()->void {



    // for(PointCloudEXMap::const_iterator mit=curr_bundle_.pointCloud.begin();mit!=curr_bundle_.pointCloud.end();++mit) {
    //     PointCloudEXPtr pc_i = mit->second;

    //     std::cout<<"pc pos_w:"<<pc_i->pos_w<<std::endl;
    //     std::cout<<"pc size:"<<pc_i->pts_cloud.size()<<std::endl;
    //     // pcl::PointXYZRGB p;
    //     // Eigen::Vector3d PosWorld = pc_i->GetWorldPos();
    //     // p = CreatePoint3D(PosWorld,pc_i->id_.second);

    //     // cloud.points.push_back(p);
    // }

    PointCloudEXPtr pc = curr_bundle_.pointCloud.rbegin()->second;
    
    // PointCloud cloud_in_=pc->pts_cloud;
    // PointCloud cloud_in=pc->pts_cloud;
    // PointCloud::Ptr cloud_in = pc->pts_cloud;
    // PointCloud::Ptr cloud_in=&(pc->pts_cloud);
    
    PointCloud::Ptr cloud_in(new PointCloud(pc->pts_cloud));
    // pcl::transformPointCloud(*cloud_in, *cloud_in, (pc->T_s_lm_*pc->T_lm_w_).matrix());
    pcl::transformPointCloud(*cloud_in, *cloud_in, pc->GetPoseTsg().matrix());
    // int size = cloud_in->points.size();
    // pcl::PointCloudXYZI::Ptr laserCloudIMUWorld(new PointCloudXYZI(size, 1));
    // for (int i = 0; i < size; i++)
    // {
    //     // RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
    //     //                     &laserCloudIMUBody->points[i]);
    //         // V3D p_body_lidar(pi->x, pi->y, pi->z);
    //         // V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);
    //         cloud_in
    //         laserCloudIMUWorld->points[i].x = p_body_imu(0);
    //         laserCloudIMUWorld->points[i].y = p_body_imu(1);
    //         laserCloudIMUWorld->z = p_body_imu(2);
    //         laserCloudIMUWorld->intensity = pi->intensity;
    // }
    // cloud_in.reset(pc->pts_cloud);
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZINormal>);
    // *cloud_in=pc->pts_cloud;
// #if PointType == pcl::PointXYZI

// #endif
# if 0
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    pointcloud_convert(cloud_in,cloud_out);
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(*cloud_out,pcl_msg);    
#else
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(*cloud_in,pcl_msg);    
# endif

    pcl_msg.header.frame_id = curr_bundle_.frame;
    // pcl_msg.header.stamp = ros::Time::now();
    pcl_msg.header.stamp = ros::Time().fromSec(pc->timestamp_);

    pub_cloud_.publish(pcl_msg);
    // std::cout<<"pub vis"<<std::endl;
        
}

std::string append_space_to_bits( std::string & in_str, int bits )
{
    while( in_str.length() < bits )
    {
        in_str.append(" ");
    }
    return in_str;
}

auto Visualizer::Run()->void{
    // std::cout<<"run vis"<<std::endl;

    while(1) {

        if(this->CheckVisData()) {
        std::unique_lock<std::mutex> lock(mtx_draw_);
        //  std::cout<<"run vis 1"<<std::endl;
       
        g_camera_frame_num=0;
        g_lidar_frame_num=0;
        g_pointcloud_pts_num =0;
         // 每个agent
        for(std::map<size_t,VisBundle>::iterator mit = vis_data_.begin();mit!=vis_data_.end();++mit){
            curr_bundle_ = mit->second;
            g_camera_frame_num+=curr_bundle_.keyframes.size();
            g_lidar_frame_num+=curr_bundle_.pointCloud.size();

            for(PointCloudEXMap::const_iterator mit=curr_bundle_.pointCloud.begin();mit!=curr_bundle_.pointCloud.end();++mit) {
                PointCloudEXPtr pc_i = mit->second;
                g_pointcloud_pts_num += pc_i->pts_cloud.size();
            }
            // std::cout<<"run vis2"<<std::endl;
            // this->PubPointCloud();
            this->PubPointCloud_service();
            this->PubTrajectories();
            this->PubOdometries();
            //  std::cout<<"run vis3"<<std::endl;
        //     if(colive_params::vis::showkeyframes)
        //         this->PubKeyframesAsFrusta();

        //     if(covins_params::vis::showtraj)
        //         this->PubTrajectories();

        //     if(covins_params::vis::showlandmarks)
        //         this->PubLandmarksAsCloud();

        //     if(covins_params::vis::showcovgraph)
        //         this->PubCovGraph();

            this->PubLoopEdges();
        }
        static uint32_t board_cnt=0;
        if(board_cnt%10==0){
            print_dash_board();
        }
        board_cnt++;
        vis_data_.clear();
        // limit frequency, TODO: should be modified later may like this

            // // float loopClosureFrequency = 3.0; // can change 
            // ros::Rate rate(loopClosureFrequency);
            // while (ros::ok())
            // {
            //     rate.sleep();
            //     // performSCLoopClosure();
            //     performRSLoopClosure(); // TODO
            //     visualizeLoopClosure();
            // }
        usleep(200000);// limit frequency, TODO
    }
    this->ResetIfRequested();
    usleep(5000);
    }
    
}

void Visualizer::print_dash_board()
{
#if DEBUG_PHOTOMETRIC
    return;
#endif

    // 获取当前进程的物理内存占用量。
    int mem_used_mb = ( int ) ( Common_tools::get_RSS_Mb() );
    // clang-format off
    if( (mem_used_mb - g_last_stamped_mem_mb < 1024 ) && g_last_stamped_mem_mb != 0 )
    {
        std::cout  << ANSI_DELETE_CURRENT_LINE << ANSI_DELETE_LAST_LINE ;
    }
    else
    {
        std::cout << "\r\n" << endl;
        std::cout << ANSI_COLOR_WHITE_BOLD << "======================= COLIVE Dashboard ======================" << ANSI_COLOR_RESET << std::endl;
        g_last_stamped_mem_mb = mem_used_mb ;
    }
    
    std::string out_str_line_1, out_str_line_2;
    out_str_line_1 = std::string(        "| System-time | LiDAR-frame | Camera-frame |  Pts in maps | Memory used (Mb) |") ;
    //                                    1             16            30             45             60     
    // clang-format on
    out_str_line_2.reserve( 1e3 );
    out_str_line_2.append( "|   " ).append( Common_tools::get_current_time_str() );
    append_space_to_bits( out_str_line_2, 14 );
    out_str_line_2.append( "|    " ).append( std::to_string( g_lidar_frame_num ) );
    append_space_to_bits( out_str_line_2, 28 );
    out_str_line_2.append( "|    " ).append( std::to_string( g_camera_frame_num ) );
    append_space_to_bits( out_str_line_2, 43 );
    // out_str_line_2.append( "| " ).append( std::to_string( m_map_rgb_pts.m_rgb_pts_vec.size() ) );
    out_str_line_2.append( "| " ).append( std::to_string( g_pointcloud_pts_num) );
    append_space_to_bits( out_str_line_2, 58 );
    out_str_line_2.append( "|    " ).append( std::to_string( mem_used_mb ) );

    out_str_line_2.insert( 58, ANSI_COLOR_YELLOW, 7 );
    out_str_line_2.insert( 43, ANSI_COLOR_BLUE, 7 );
    out_str_line_2.insert( 28, ANSI_COLOR_GREEN, 7 );
    out_str_line_2.insert( 14, ANSI_COLOR_RED, 7 );
    out_str_line_2.insert( 0, ANSI_COLOR_WHITE, 7 );

    out_str_line_1.insert( 58, ANSI_COLOR_YELLOW_BOLD, 7 );
    out_str_line_1.insert( 43, ANSI_COLOR_BLUE_BOLD, 7 );
    out_str_line_1.insert( 28, ANSI_COLOR_GREEN_BOLD, 7 );
    out_str_line_1.insert( 14, ANSI_COLOR_RED_BOLD, 7 );
    out_str_line_1.insert( 0, ANSI_COLOR_WHITE_BOLD, 7 );

    std::cout << out_str_line_1 << std::endl;
    std::cout << out_str_line_2 << ANSI_COLOR_RESET << "          ";
    ANSI_SCREEN_FLUSH;
    // std::cout     
}

// void R3LIVE::service_pub_rgb_maps()
// {
//     int last_publish_map_idx = -3e8;
//     int sleep_time_aft_pub = 10;
//     int number_of_pts_per_topic = 1000;
//     if ( number_of_pts_per_topic < 0 )
//     {
//         return;
//     }
//     while ( 1 )
//     {
//         ros::spinOnce();
//         std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
//         pcl::PointCloud< pcl::PointXYZRGB > pc_rgb;
//         sensor_msgs::PointCloud2            ros_pc_msg;
//         int pts_size = m_map_rgb_pts.m_rgb_pts_vec.size();
//         pc_rgb.resize( number_of_pts_per_topic );
//         // for (int i = pts_size - 1; i > 0; i--)
//         int pub_idx_size = 0;
//         int cur_topic_idx = 0;
//         if ( last_publish_map_idx == m_map_rgb_pts.m_last_updated_frame_idx )
//         {
//             continue;
//         }
//         last_publish_map_idx = m_map_rgb_pts.m_last_updated_frame_idx;
//         for ( int i = 0; i < pts_size; i++ )
//         {
//             if ( m_map_rgb_pts.m_rgb_pts_vec[ i ]->m_N_rgb < 1 )// if is empty
//             {
//                 continue;
//             }
//             pc_rgb.points[ pub_idx_size ].x = m_map_rgb_pts.m_rgb_pts_vec[ i ]->m_pos[ 0 ];
//             pc_rgb.points[ pub_idx_size ].y = m_map_rgb_pts.m_rgb_pts_vec[ i ]->m_pos[ 1 ];
//             pc_rgb.points[ pub_idx_size ].z = m_map_rgb_pts.m_rgb_pts_vec[ i ]->m_pos[ 2 ];
//             pc_rgb.points[ pub_idx_size ].r = m_map_rgb_pts.m_rgb_pts_vec[ i ]->m_rgb[ 2 ];
//             pc_rgb.points[ pub_idx_size ].g = m_map_rgb_pts.m_rgb_pts_vec[ i ]->m_rgb[ 1 ];
//             pc_rgb.points[ pub_idx_size ].b = m_map_rgb_pts.m_rgb_pts_vec[ i ]->m_rgb[ 0 ];
//             // pc_rgb.points[i].intensity = m_map_rgb_pts.m_rgb_pts_vec[i]->m_obs_dis;
//             pub_idx_size++;
//             if ( pub_idx_size == number_of_pts_per_topic )
//             {
//                 pub_idx_size = 0;
//                 pcl::toROSMsg( pc_rgb, ros_pc_msg );
//                 ros_pc_msg.header.frame_id = "world";       
//                 ros_pc_msg.header.stamp = ros::Time::now(); 
//                 if ( m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] == nullptr )//create a new point cloud publisher
//                 {
//                     m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] =
//                         std::make_shared< ros::Publisher >( m_ros_node_handle.advertise< sensor_msgs::PointCloud2 >(
//                             std::string( "/RGB_map_" ).append( std::to_string( cur_topic_idx ) ), 100 ) );
//                 }
//                 m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ]->publish( ros_pc_msg );
//                 std::this_thread::sleep_for( std::chrono::microseconds( sleep_time_aft_pub ) );
//                 ros::spinOnce();
//                 cur_topic_idx++;
//             }
//         }
//         // pub remaining pts or pts_size < number_of_pts_per_topic
//         pc_rgb.resize( pub_idx_size );
//         pcl::toROSMsg( pc_rgb, ros_pc_msg );
//         ros_pc_msg.header.frame_id = "world";       
//         ros_pc_msg.header.stamp = ros::Time::now(); 
//         if ( m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] == nullptr )
//         {
//             m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] =
//                 std::make_shared< ros::Publisher >( m_ros_node_handle.advertise< sensor_msgs::PointCloud2 >(
//                     std::string( "/RGB_map_" ).append( std::to_string( cur_topic_idx ) ), 100 ) );
//         }
//         std::this_thread::sleep_for( std::chrono::microseconds( sleep_time_aft_pub ) );
//         ros::spinOnce();
//         m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ]->publish( ros_pc_msg );
//         cur_topic_idx++;
//         if ( cur_topic_idx >= 45 ) // Maximum pointcloud topics = 45.
//         {
//             number_of_pts_per_topic *= 1.5;//next should larger
//             sleep_time_aft_pub *= 1.5;
//         }
//     }
// }

// void R3LIVE::publish_render_pts( ros::Publisher &pts_pub, Global_map &m_map_rgb_pts )
// {
//     pcl::PointCloud< pcl::PointXYZRGB > pc_rgb;
//     sensor_msgs::PointCloud2            ros_pc_msg;
//     pc_rgb.reserve( 1e7 );
//     m_map_rgb_pts.m_mutex_m_box_recent_hitted->lock();
//     std::unordered_set< std::shared_ptr< RGB_Voxel > > boxes_recent_hitted = m_map_rgb_pts.m_voxels_recent_visited;
//     m_map_rgb_pts.m_mutex_m_box_recent_hitted->unlock();

//     for ( Voxel_set_iterator it = boxes_recent_hitted.begin(); it != boxes_recent_hitted.end(); it++ )
//     {
//         for ( int pt_idx = 0; pt_idx < ( *it )->m_pts_in_grid.size(); pt_idx++ )
//         {
//             pcl::PointXYZRGB           pt;
//             std::shared_ptr< RGB_pts > rgb_pt = ( *it )->m_pts_in_grid[ pt_idx ];
//             pt.x = rgb_pt->m_pos[ 0 ];
//             pt.y = rgb_pt->m_pos[ 1 ];
//             pt.z = rgb_pt->m_pos[ 2 ];
//             pt.r = rgb_pt->m_rgb[ 2 ];
//             pt.g = rgb_pt->m_rgb[ 1 ];
//             pt.b = rgb_pt->m_rgb[ 0 ];
//             if ( rgb_pt->m_N_rgb > m_pub_pt_minimum_views )
//             {
//                 pc_rgb.points.push_back( pt );
//             }
//         }
//     }
//     pcl::toROSMsg( pc_rgb, ros_pc_msg );
//     ros_pc_msg.header.frame_id = "world";       // world; camera_init
//     ros_pc_msg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
//     pts_pub.publish( ros_pc_msg );
// }



}