#include <iostream>
// #include <boost/asio.hpp>
#include "httplib.h"
#include "typedefs_base.hpp"
// #include <cpprest/http_listener.h>
// #include "mapmanager.hpp"
// AgentPackage::AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man) {
//     agent_.reset(new AgentHandler(client_id,newfd,vis,man));
// }
class httplib{
    class Server{

    };
};
namespace colive {

class ServiceManager {
    // using namespace httplib;

    public:
    using MapManagerPtr                 = TypeDefs::MapManagerPtr;

    
    public:
    ServiceManager(MapManagerPtr mapmanager);
    // static void staticHandleRequest(const web::http::http_request& request);
    // auto handleRequest(const web::http::http_request& request)->void;
    auto Run()->void;

    MapManagerPtr m_mapmanager;
    httplib::Server svr;

};


}

   