


// C++
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <ros/ros.h>

// Thirdparty
#include <ros/ros.h>


#include "backend.hpp"
// #include "backend.hpp"

#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_data_io.hpp"
#include "tools_timer.hpp"
#include "tools_thread_pool.hpp"
#include "tools_ros.hpp"

auto SignalHandler(int signum)->void {
   std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
   exit(signum);
}

Common_tools::Cost_time_logger g_cost_time_logger;


int main(int argc, char* argv[]) {
    std::cout << "+++ COLIVE Back-End +++" << std::endl;

    // if(argc != 1){
    //     std::cout << "Error: " << argc-1 << " arguments - 0 required" << std::endl;
    //     return 0;
    // }

    ros::init(argc, argv, "COLIVE_BackEnd");

    signal(SIGINT, SignalHandler);
    // extern Common_tools::Cost_time_logger g_cost_time_logger;
    std::string m_map_output_dir;
    m_map_output_dir =Common_tools::get_home_folder().append( "/r3live_output" );
    g_cost_time_logger.init_log( std::string(m_map_output_dir).append("/cost_time_logger.log"));

    std::shared_ptr<colive::Backend> backend(new colive::Backend());
    colive::Backend::ThreadPtr main_thread(new std::thread(&colive::Backend::Run,backend));
    main_thread->detach(); // Thread will be cleaned up when exiting main()
    
    // std::shared_ptr<Communicator> comm_;
    // covins::TypeDefs::ThreadPtr thread_comm_;


    ros::spin();

    return 0;
}
