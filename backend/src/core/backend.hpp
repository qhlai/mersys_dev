#pragma once

// C++
#include <memory>
#include <mutex>
#include <vector>
#include <thread>


#include <ros/ros.h>

#include "typedefs_base.hpp"

// namespace colive {

// class AgentHandler;
// class Map;
// class MapManager;
// class Visualizer;

// class AgentPackage : public std::enable_shared_from_this<AgentPackage> {
// public:
//     // using MapPtr                        = TypeDefs::MapPtr;
//     // using ManagerPtr                    = TypeDefs::ManagerPtr;
//     using HandlerPtr                    = std::shared_ptr<AgentHandler>;
//     // using VisPtr                        = TypeDefs::VisPtr;

// public:
//     // AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man);

// protected:
//     HandlerPtr agent_;
// };

// class CovinsBackend {
// public:
//     using AgentPtr                      = std::shared_ptr<AgentPackage>;
//     using AgentVector                   = std::vector<AgentPtr>;
//     // using ManagerPtr                    = TypeDefs::ManagerPtr;
//     // using VisPtr                        = TypeDefs::VisPtr;
//     using ThreadPtr                     = TypeDefs::ThreadPtr;
//     using VocabularyPtr                 = CovinsVocabulary::VocabularyPtr;

// public:
//     CovinsBackend();
//     auto Run()                                                                          ->void;

//     // Service Interfaces
//     auto CallbackGBA(covins_backend::ServiceGBA::Request &req,
//                      covins_backend::ServiceGBA::Response &res)                         ->bool;

//     auto CallbackSaveMap(covins_backend::ServiceSaveMap::Request &req,
//                          covins_backend::ServiceSaveMap::Response &res)                 ->bool;

//     auto CallbackLoadMap(covins_backend::ServiceLoadMap::Request &req,
//                                covins_backend::ServiceLoadMap::Response &res)           ->bool;

//     auto CallbackPruneMap(covins_backend::ServicePruneMap::Request &req,
//                          covins_backend::ServicePruneMap::Response &res)                ->bool;

// protected:
//     auto AddAgent()                                                                     ->void;
//     auto AcceptAgent()                                                                  ->void;
//     auto ConnectSocket()                                                                ->void;
//     auto LoadVocabulary()                                                               ->void;

//     auto add_counter()                                                                  ->void;
//     auto sub_counter()                                                                  ->void;
//     auto disp_counter()                                                                 ->void;
//     auto get_counter()                                                                  ->int;

//     // Infrastructure
//     AgentVector                 agents_;
//     ManagerPtr                  mapmanager_;
//     VisPtr                      vis_;
//     VocabularyPtr               voc_;

//     ThreadPtr                   thread_mapmanager_;
//     ThreadPtr                   thread_vis_;

//     int                         agent_next_id_                                          = 0;

//     ros::NodeHandle             nh_;
//     ros::ServiceServer          service_gba_;
//     ros::ServiceServer          service_savemap_;
//     ros::ServiceServer          service_loadmap_;
//     ros::ServiceServer          service_prune_;

//     fd_set                      master_;
//     fd_set                      read_fds_;
//     int                         listener_, newfd_;

//     // Device Counter
//     std::atomic<int>            counter_, overall_counter_;

//     // Sync
//     std::mutex                  mtx_num_agents_;
// };

// } //end ns


// // class AgentHandler;
// // class Map;
// // class MapManager;
// // class Visualizer;

// // class AgentPackage : public std::enable_shared_from_this<AgentPackage> {
// // public:
// //     using MapPtr                        = TypeDefs::MapLIVPtr;
// //     // using ManagerPtr                    = TypeDefs::ManagerPtr;
// //     using HandlerPtr                    = std::shared_ptr<AgentHandler>;
// //     // using VisPtr                        = TypeDefs::VisPtr;

// // public:
// //     // AgentPackage(size_t client_id, int newfd, VisPtr vis, ManagerPtr man);

// // protected:
// //     HandlerPtr agent_;
// // };

// // class CovinsBackend {
// // public:
// //     using AgentPtr                      = std::shared_ptr<AgentPackage>;
// //     using AgentVector                   = std::vector<AgentPtr>;
// //     // using ManagerPtr                    = TypeDefs::ManagerPtr;
// //     // using VisPtr                        = TypeDefs::VisPtr;
// //     using ThreadPtr                     = TypeDefs::ThreadPtr;
// //     // using VocabularyPtr                 = CovinsVocabulary::VocabularyPtr;

// // public:
// //     CovinsBackend();
// //     auto Run()                                                                          ->void;

// //     // Service Interfaces
// //     // auto CallbackGBA(covins_backend::ServiceGBA::Request &req,
// //     //                  covins_backend::ServiceGBA::Response &res)                         ->bool;

// //     // auto CallbackSaveMap(covins_backend::ServiceSaveMap::Request &req,
// //     //                      covins_backend::ServiceSaveMap::Response &res)                 ->bool;

// //     // auto CallbackLoadMap(covins_backend::ServiceLoadMap::Request &req,
// //     //                            covins_backend::ServiceLoadMap::Response &res)           ->bool;

// //     // auto CallbackPruneMap(covins_backend::ServicePruneMap::Request &req,
// //     //                      covins_backend::ServicePruneMap::Response &res)                ->bool;

// // protected:
// //     auto AddAgent()                                                                     ->void;
// //     auto AcceptAgent()                                                                  ->void;
// //     auto ConnectSocket()                                                                ->void;
// //     auto LoadVocabulary()                                                               ->void;

// //     auto add_counter()                                                                  ->void;
// //     auto sub_counter()                                                                  ->void;
// //     auto disp_counter()                                                                 ->void;
// //     auto get_counter()                                                                  ->int;

// //     // Infrastructure
// //     AgentVector                 agents_;
// //     // ManagerPtr                  mapmanager_;
// //     // VisPtr                      vis_;
// //     // VocabularyPtr               voc_;

// //     ThreadPtr                   thread_mapmanager_;
// //     ThreadPtr                   thread_vis_;

// //     int                         agent_next_id_                                          = 0;

// //     ros::NodeHandle             nh_;
// //     ros::ServiceServer          service_gba_;
// //     ros::ServiceServer          service_savemap_;
// //     ros::ServiceServer          service_loadmap_;
// //     ros::ServiceServer          service_prune_;

// //     fd_set                      master_;
// //     fd_set                      read_fds_;
// //     int                         listener_, newfd_;

// //     // Device Counter
// //     std::atomic<int>            counter_, overall_counter_;

// //     // Sync
// //     std::mutex                  mtx_num_agents_;
// // };
