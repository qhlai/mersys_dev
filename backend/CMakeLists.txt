cmake_minimum_required(VERSION 2.8.3)
project(colive_backend)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -O3 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Add OpenCV if available
# set(OpenCV_DIR "/home/zivlin/opt/opencv/build")
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()
find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    std_msgs
                    geometry_msgs
                    nav_msgs
                    tf
                    cv_bridge
                    livox_ros_driver
            )

# find_package(catkin_simple REQUIRED)
# catkin_simple(ALL_DEPS_REQUIRED)

# add_compile_options(-std=c++1z -pthread -pipe -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-function -Wno-vla)

#ROS_BUILD_TYPE: Set the build type. Options are (default: RelWithDebInfo):
#Debug : w/ debug symbols, w/o optimization
#Release : w/o debug symbols, w/ optimization
#RelWithDebInfo : w/ debug symbols, w/ optimization
#RelWithAsserts : w/o debug symbols, w/ optimization, w/ assertions (i.e., w/o -DNDEBUG). New in ros 1.1.
#MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries

# IF(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE RelWithDebInfo)
# ENDIF()

# find_package(Eigen3 3.3.4 EXACT REQUIRED)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
# find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    "src/base"
    "src/core"
    "src/map"
    "src/tools"
    "src/communicator"
    # "src/communicator/msg"
    # ${CMAKE_SOURCE_DIR}/thirdparty/cereal
    # "include/covins"
    # ${PROJECT_SOURCE_DIR}/thirdparty
)

catkin_package()
message(STATUS "===== ${PROJECT_NAME}: OpenCV library status: =====")
message(STATUS "===== OpenCV ersion: ${OpenCV_VERSION}  =====")
message(STATUS "===== OpenCV libraries: ${OpenCV_LIBS} =====")
message(STATUS "===== OpenCV include path: ${OpenCV_INCLUDE_DIRS} =====")

# set(EXTERNAL_LIBRARIES
#   pthread
# )

# set(BASE_SOURCE_FILES
    
# )

# set(BASE_HEADER_FILES
#     # include/covins/covins_base/keyframe_base.hpp
#     # include/covins/covins_base/landmark_base.hpp
#     # include/covins/covins_base/map_base.hpp
#     # include/covins/covins_base/optimization_base.hpp
#     # include/covins/covins_base/visualization_base.hpp

#     # # config
#     # include/covins/covins_base/config_backend.hpp
# )

# set(BACKEND_SOURCE_FILES
#     src/backend/backend/backend.cpp

# )
# set(BACKEND_HEADER_FILES
#     src/backend/backend/backend.hpp

# )
file(GLOB_RECURSE COMM_SRC_FILES     
# ${PROJECT_SOURCE_DIR}/src/communicator/*.cc 
${PROJECT_SOURCE_DIR}/src/communicator/*.cpp
${PROJECT_SOURCE_DIR}/src/communicator/msg/*.cpp
${PROJECT_SOURCE_DIR}/src/communicator/server/*.cpp
${PROJECT_SOURCE_DIR}/src/communicator/client/*.cpp
)

add_executable(colive_test_timer src/tools/test_timer.cpp)

add_executable(colive_backend_node src/core/backend_node.cpp
    src/core/backend.cpp 
    src/core/config_backend.cpp
    src/map/map_co.cpp 
    # src/communicator/*.cpp
    # src/communicator/client/*.cpp
    # src/communicator/msgs/*.cpp
    ${COMM_SRC_FILES}   
    # src/map/rgb_map.cpp 
    # src/map/image_frame.cpp 
)

target_link_libraries(colive_backend_node
                          ${catkin_LIBRARIES}
                          ${Boost_LIBRARIES}
                          ${Boost_FILESYSTEM_LIBRARY}
                          ${Boost_SERIALIZATION_LIBRARY} # serialization
                          ${OpenCV_LIBRARIES}
                          # ${OpenMVS_LIBRARIES}
                          pcl_common 
                          pcl_io) 


# if (NOT BUILD_LIBRARY)
    # cs_add_library(covins_base ${BASE_SOURCE_FILES} ${BASE_HEADER_FILES})
    # target_link_libraries(covins_base ${catkin_LIBRARIES} ${EXTERNAL_LIBRARIES})

    # cs_add_library(covins_backend ${BACKEND_SOURCE_FILES} ${BACKEND_HEADER_FILES})
    # target_link_libraries(covins_backend covins_base ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so)

    # cs_add_executable(covins_backend_node
    #     covins_sys/src/covins_backend_node.cpp
    # )
    # target_link_libraries(covins_backend_node covins_backend)

# else()
#     if (NOT USE_CATKIN)
        # include_directories(${CMAKE_SOURCE_DIR}/thirdparty/cereal)
        # # create library
        # message("${PROJECT_SOURCE_DIR}/lib")
        # set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

        # add_library(${PROJECT_NAME} SHARED ${COMM_SOURCE_FILES} ${COMM_HEADER_FILES})
        # option(BUILD_SHARED_LIBS "Build shared library" ON)

        # target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
        # target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
        # target_link_libraries(${PROJECT_NAME} ${GLOG_LIBRARIES})
#     endif()

# endif()

# add_doxygen(NOT_AUTOMATIC)
