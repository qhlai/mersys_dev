#include "backend.hpp"
// #include "image_ex.hpp"
#include "service_manager.hpp"

#include "tools_logger.hpp"
#include "tools_color_printf.hpp"

// #include <ros/ros.h>

// AgentPackage::AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man) {
//     agent_.reset(new AgentHandler(client_id,newfd,vis,man));
// }
namespace mersys {

std::shared_ptr< Common_tools::ThreadPool > m_thread_pool_ptr;
Common_tools::Cost_time_logger g_cost_time_logger;


// void vis(){
//     while (true)
//     {
//         mapmanager_->Display();
//         sleep(3);
//     } 
// }

Client::Client(size_t client_id, int newfd, MapManagerPtr man, VisPtr vis)
:   client_id_(client_id),
    mapmanager_(man)
{
    std::cout << "Client:"<<client_id_ <<" start"<< std::endl;
    mapmanager_->InitializeMap(client_id);
    mapmanager_->Display();
    std::cout << "map init done"<< std::endl;

    

    placerec_.reset(new PlaceRecognition(mapmanager_,true));
    m_thread_pool_ptr->commit_task( &PlaceRecognition::Run,placerec_ );
    // thread_placerec_.reset(new std::thread(&PlaceRecognition::Run,placerec_));
    // thread_placerec_->detach();

    comm_.reset(new Communicator_server(client_id_,newfd,man,placerec_,vis));
    m_thread_pool_ptr->commit_task(&Communicator_server::Run,comm_);
    // thread_comm_.reset(new std::thread(&Communicator_server::Run,comm_));
    // thread_comm_->detach();
    std::cout << "thread init done"<< std::endl;
}

Client::Client(size_t client_id_)
:client_id_(client_id_)
{
    map_ptr.reset(new Map(client_id_));
}

auto Client::Run()->void {



}

Backend::Backend(){
    std::cout<<"hello world!"<<std::endl;
    // some init 
    printf_program("mersys backend");
    Common_tools::printf_software_version();
    Eigen::initParallel();
    mersys_params::ShowParamsComm();
    mersys_params::ShowParamsBackend();

    m_thread_pool_ptr = std::make_shared<Common_tools::ThreadPool>(mersys_params::sys::threads_pool, true, false); 

    if(!Common_tools::if_file_exist(mersys_params::sys::output_path))
    {
        cout << ANSI_COLOR_BLUE_BOLD << "Create r3live output dir: " << mersys_params::sys::output_path << ANSI_COLOR_RESET << endl;
        Common_tools::create_dir(mersys_params::sys::output_path);
    }
        
    g_cost_time_logger.init_log( std::string(mersys_params::sys::output_path).append("cost_time_logger.log"));
    
    mapmanager_.reset(new MapManager());
    m_thread_pool_ptr->commit_task(&MapManager::Run,mapmanager_);

    m_service_mapmanager_.reset(new ServiceManager(mapmanager_));
    m_thread_pool_ptr->commit_task( &ServiceManager::Run,m_service_mapmanager_);
    // thread_mapmanager_.reset(new std::thread(&MapManager::Run,mapmanager_));
    // thread_mapmanager_->detach(); // Thread will be cleaned up when exiting main()



        //+++++ Create Viewer +++++
    if(mersys_params::vis::active){
        // auto sb =new mersys::Visualizer();
        // std::string a;
        // a="_be";
        vis_.reset(new Visualizer("_be"));
        // thread_vis_.reset(new std::thread(&Visualizer::Run,vis_));
        // thread_vis_->detach(); // Thread will be cleaned up when exiting main()
        m_thread_pool_ptr->commit_task(&Visualizer::Run,vis_);
    }

    if ( Common_tools::get_total_phy_RAM_size_in_GB() < 16 )
    {
        scope_color( ANSI_COLOR_RED_BOLD );
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
        cout << "I have detected your physical memory smaller than 16GB (currently: " << Common_tools::get_total_phy_RAM_size_in_GB()
             << "GB). I recommend you to add more physical memory for improving the overall performance of mersys." << endl;
        cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
        std::this_thread::sleep_for( std::chrono::seconds( 5 ) );
        // m_rgb_pts_vec.reserve( 1e8 );
    }
    else
    {
        // m_rgb_pts_vec.reserve( 1e9 );
    }
}

auto Backend::Run()->void {
    //+++++ Add the 1st agent +++++
    std::cout<<"backend start"<<std::endl;
    this->AddClient();

}

auto Backend::ConnectSocket()->void {
    struct addrinfo hints, *ai, *p;
    int rv;
    int yes = 1;

    FD_ZERO(&master_);    // clear the master and temp sets
    FD_ZERO(&read_fds_);

    // get us a socket and bind it
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;//表示可以使用 IPv4 或 IPv6 地址。
    hints.ai_socktype = SOCK_STREAM;//将套接字类型设置为 SOCK_STREAM，表示使用 TCP 协议进行流式传输。
    hints.ai_flags = AI_PASSIVE;//设置为允许获取地址信息用于监听（作为服务器）。
    // 这里将主机名设置为 NULL，表示使用本地地址。服务名设置为 "10086"，即指定使用端口号为 10086。hints 参数传递了之前设置的地址信息的提示。函数调用的结果将存储在 ai 指向的 addrinfo 结构中。
    if ((rv = getaddrinfo(NULL, mersys_params::sys::port.c_str(), &hints, &ai)) != 0) {//socket port  
        fprintf(stderr, "selectserver: %s\n", gai_strerror(rv));
        exit(1);
    }

    for (p = ai; p != NULL; p = p->ai_next) {
        listener_ = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (listener_ < 0) {
            continue;
        }
        // SOL_SOCKET 表示设置的是套接字级别的选项，SO_REUSEADDR 表示允许重用地址。这一步旨在解决 "address already in use" 的错误信息。
        // lose the pesky "address already in use" error message
        setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

        if (bind(listener_, p->ai_addr, p->ai_addrlen) < 0) {
            //close(listener);
            continue;
        }

        break;
    }

    // if we got here, it means we didn't get bound
    if (p == NULL) {
        fprintf(stderr, "selectserver: failed to bind\n");
        exit(2);
    }
    std::cout << "socket bind success" << std::endl;
    freeaddrinfo(ai); // all done with this
}
auto Backend::AddClient()->void {
    //connect
    this->ConnectSocket(); //get a socket

    //listen and accept
    this->AcceptClient();
}
auto Backend::AcceptClient()->void {
    socklen_t addrlen;
    struct sockaddr_storage remoteaddr; // client address
    char remoteIP[INET6_ADDRSTRLEN];
    struct timeval tv;
    int temp;

    if (listen(listener_, 10) == -1) {
        perror("listen");
        exit(3);
    }
    // 需要将要监听的文件描述符添加到文件描述符集合中，以便在后续的监视过程中跟踪其状态变化。
    // add the listener to the master set
    FD_SET(listener_, &master_);

    std::cout << "waiting for connection" << std::endl;
    // main loop
    for(;;) {
        read_fds_ = master_; // copy it
        tv.tv_sec = 5; // 超时时间
        tv.tv_usec = 0;

        //将阻塞程序执行，直到满足以下任一条件：listener_ 中的文件描述符准备好进行读取操作。超时时间到达。 出现错误。
        if ((temp = select(listener_ + 1, &read_fds_, NULL, NULL, &tv)) == -1) {
            perror("select");
            exit(4);
        }
        //listener_ 是一个文件描述符，read_fds_ 是一个文件描述符集合。FD_ISSET(listener_, &read_fds_) 用于检查 listener_ 是否在 read_fds_ 中被设置。
        if (FD_ISSET(listener_, &read_fds_)) {
            addrlen = sizeof remoteaddr;
            if ((newfd_ = accept(listener_, (struct sockaddr *) &remoteaddr, &addrlen)) == -1) {
                perror("accept failed");
            } else {
                // ok
                printf("selectserver: new connection from %s on socket %d\n", inet_ntop(remoteaddr.ss_family, CommunicatorBase::GetInAddr((struct sockaddr *) &remoteaddr), remoteIP, INET6_ADDRSTRLEN), newfd_);
            }

            //Creating new threads for every agent
            ClientPtr client{new Client(agent_next_id_++,newfd_,mapmanager_,vis_)};
            clients_.push_back(client);
        }
        usleep(100);
    }
}


}

    // ros::Subscriber sub_pcl;
    // ros::Subscriber sub_imu;
    // ros::Subscriber sub_img, sub_img_comp;
    // ros::Subscriber sub_odom;

    // std::string LiDAR_pointcloud_topic, IMU_topic, IMAGE_topic, IMAGE_topic_compressed, ODOM_topic;

    // get_ros_parameter<std::string>(m_ros_node_handle, "basic/LiDAR_pointcloud_topic", LiDAR_pointcloud_topic, std::string("/laser_cloud_flat") );

    // get_ros_parameter<std::string>(m_ros_node_handle, "basic/IMU_topic", IMU_topic, std::string("/livox/imu") );
    // get_ros_parameter<std::string>(m_ros_node_handle, "basic/Image_topic", IMAGE_topic, std::string("/camera/image_color") );
    // IMAGE_topic_compressed = std::string(IMAGE_topic).append("/compressed");
    
    // get_ros_parameter<std::string>(m_ros_node_handle, "basic/ODOM_topic", ODOM_topic, std::string("/odom") );