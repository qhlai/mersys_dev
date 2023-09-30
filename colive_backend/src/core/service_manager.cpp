// #include "backend.hpp"
// #include "image_ex.hpp"


#include "service_manager.hpp"
#include "config_comm.hpp"
// class Mapmanager{
//     auto SaveAllMapPath()                                                  ->void;
// }
#include "mapmanager.hpp"
// using namespace web;
// using namespace http;
// using namespace utility;
// using namespace http::experimental::listener;

// #include <ros/ros.h>

// AgentPackage::AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man) {
//     agent_.reset(new AgentHandler(client_id,newfd,vis,man));
// }
namespace colive {

    ServiceManager::ServiceManager(MapManagerPtr mapmanager){
        m_mapmanager = mapmanager;
    }
    auto ServiceManager::Run()->void{

    using namespace httplib;
    svr.Get("/hi", [](const Request& req, Response& res) {
        res.set_content("Hello World!", "text/plain");
    });

    svr.Get("/savepath", [&](const Request& req, Response& res) {
        m_mapmanager->SaveAllMapPath();
    });

    svr.Get("/stop", [&](const Request& req, Response& res) {
        svr.stop();
    });

// stoi(colive_params::sys::port)
    std::cout << "svr.listen:"<<"localhost, " <<stoi(colive_params::sys::port)+1<< std::endl;
    svr.listen("localhost", stoi(colive_params::sys::port)+1);
    //     // // 创建HTTP监听器
    //     // web::http::experimental::listener::http_listener listener("http://0.0.0.0:9034/");
    //     // // http_listener listener("http://localhost:".append(colive_params::sys::port.c_str()).append("/"));
    //     // // 设置请求处理回调函数
    //     // listener.support(web::http::methods::GET, ServiceManager::staticHandleRequest);
        
    //     // try {
    //     //     // 开始监听HTTP请求
    //     //     listener.open().wait();

    //     //     std::cout << "正在监听HTTP请求..." << std::endl;

    //     //     // 保持主线程运行
    //     //     while (true);

    //     //     // 停止监听HTTP请求
    //     //     listener.close().wait();
    //     // }
    //     // catch (const std::exception& ex) {
    //     //     std::cerr << "发生异常： " << ex.what() << std::endl;
    //     // }

    //     return ;
    }

}
