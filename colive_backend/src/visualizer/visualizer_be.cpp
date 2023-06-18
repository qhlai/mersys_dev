#include "visualizer_be.hpp"

namespace colive{



Visualizer::Visualizer()
{
    std::string topic_prefix ="_be";
    topic_prefix_=topic_prefix;

    std::string marker_topic = "colive_markers";
    std::string cloud_topic = "colive_cloud";
    marker_topic += topic_prefix;
    cloud_topic += topic_prefix;
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic,10);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic,10);
}

Visualizer::Visualizer(std::string topic_prefix)
    : topic_prefix_(topic_prefix)
{
    std::string marker_topic = "colive_markers";
    std::string cloud_topic = "colive_cloud";
    marker_topic += topic_prefix;
    cloud_topic += topic_prefix;
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic,10);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic,10);
    std::cout<<"init vis"<<std::endl;
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
    // vb.associated_clients = map->associated_clients_;
    // vb.loops = map->GetLoopConstraints();
    vb.frame = "body";

    // if(vb.keyframes.size() < 3) return;
    if(vb.pointCloud.size() < 3) return;
    vis_data_[map->id_map_] = vb;
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

    PointCloudEXPtr p = curr_bundle_.pointCloud.rbegin()->second;
    PointCloud tmp=p->pts_cloud;
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(tmp,pcl_msg);
    pcl_msg.header.frame_id = curr_bundle_.frame;
    pcl_msg.header.stamp = ros::Time::now();

    pub_cloud_.publish(pcl_msg);
    std::cout<<"pub vis"<<std::endl;
        
}

auto Visualizer::Run()->void{
    // std::cout<<"run vis"<<std::endl;
    while(1) {
        if(this->CheckVisData()) {
        std::unique_lock<std::mutex> lock(mtx_draw_);
        //  std::cout<<"run vis 1"<<std::endl;
        for(std::map<size_t,VisBundle>::iterator mit = vis_data_.begin();mit!=vis_data_.end();++mit){
            curr_bundle_ = mit->second;
            // std::cout<<"run vis2"<<std::endl;
            this->PubPointCloud();
            //  std::cout<<"run vis3"<<std::endl;
        //     if(colive_params::vis::showkeyframes)
        //         this->PubKeyframesAsFrusta();

        //     if(covins_params::vis::showtraj)
        //         this->PubTrajectories();

        //     if(covins_params::vis::showlandmarks)
        //         this->PubLandmarksAsCloud();

        //     if(covins_params::vis::showcovgraph)
        //         this->PubCovGraph();

        //     this->PubLoopEdges();
        }
        vis_data_.clear();
    }
    this->ResetIfRequested();
    usleep(5000);
    }
    
}




}