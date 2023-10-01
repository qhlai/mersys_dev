


// C++
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <ros/ros.h>


// #include "backend.hpp"
// #include "backend.hpp"

// #include "tools_logger.hpp"
// #include "tools_color_printf.hpp"
// #include "tools_eigen.hpp"
// #include "tools_data_io.hpp"
// #include "tools_timer.hpp"
// #include "tools_thread_pool.hpp"
// #include "tools_ros.hpp"

#include "typedefs_base.hpp"
#include "test_client.hpp"

auto SignalHandler(int signum)->void {
   std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
   exit(signum);
}

// Common_tools::Cost_time_logger g_cost_time_logger;


// namespace mersys{


int main(int argc, char* argv[]) {
    std::cout << "+++ mersys Back-End +++" << std::endl;

    // if(argc != 1){
    //     std::cout << "Error: " << argc-1 << " arguments - 0 required" << std::endl;
    //     return 0;
    // }

    ros::init(argc, argv, "mersys_BackEnd");

    signal(SIGINT, SignalHandler);
    // extern Common_tools::Cost_time_logger g_cost_time_logger;
    // std::string m_map_output_dir;
    // m_map_output_dir =Common_tools::get_home_folder().append( "/r3live_output" );
    // g_cost_time_logger.init_log( std::string(m_map_output_dir).append("/cost_time_logger.log"));

    // std::shared_ptr<mersys::Backend> backend(new mersys::Backend());
    // mersys::Backend::ThreadPtr main_thread(new std::thread(&mersys::Backend::Run,backend));
    // main_thread->detach(); // Thread will be cleaned up when exiting main()
    
    std::shared_ptr<mersys::Frontend> frontend(new mersys::Frontend());
    std::unique_ptr<std::thread> sub_thread(new std::thread(&mersys::Frontend::Run,frontend));
    sub_thread->detach(); // Thread will be cleaned up when exiting main()
    // std::shared_ptr<mersys::Communicator_client> comm_;

    // comm_.reset(new Communicator_client("127.0.0.1","80"));
    



    ros::spin();

    return 0;
}


// }

