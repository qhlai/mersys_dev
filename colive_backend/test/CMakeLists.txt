cmake_minimum_required(VERSION 2.8.3)
project(colive_backend)


# if()
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

if("${CMAKE_BUILD_TYPE}" MATCHES "Debug")
    message(STATUS "building for: debug")
    add_definitions("-g")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++17 -O1 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
else()
    message(STATUS "building for: release")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++17 -O3 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
endif()
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -O1 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
# 

set(CMAKE_CXX_STANDARD 17)
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
                    # livox_ros_driver
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

if(TARGET ${PROJECT_NAME}_package)
    message(WARNING "Could not create target '${${PROJECT_NAME}_package}' for project ${PROJECT_NAME}, as it already exists.")
endif()
add_custom_target(${PROJECT_NAME}_package)
set(${PROJECT_NAME}_TARGETS)
set(${PROJECT_NAME}_LIBRARIES)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
find_package(PCL REQUIRED)
# set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.10) 
find_package(OpenCV REQUIRED)
find_package(ZLIB)
find_package(GTSAM REQUIRED QUIET)

set (INCLUDE_DIRS

${PROJECT_SOURCE_DIR}/src
${PROJECT_SOURCE_DIR}/src/base
${PROJECT_SOURCE_DIR}/src/core
${PROJECT_SOURCE_DIR}/src/map
${PROJECT_SOURCE_DIR}/src/map/frames
${PROJECT_SOURCE_DIR}/src/tools
${PROJECT_SOURCE_DIR}/src/communicator
${PROJECT_SOURCE_DIR}/src/communicator/server
${PROJECT_SOURCE_DIR}/src/communicator/client
# ${PROJECT_SOURCE_DIR}/src/communicator/msgs
# ${PROJECT_SOURCE_DIR}/src/communicator/client/colive_backend
${PROJECT_SOURCE_DIR}/src/place_rec
${PROJECT_SOURCE_DIR}/src/visualizer
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${INCLUDE_DIRS}

)


message(STATUS "===== ${PROJECT_NAME}: OpenCV library status: =====")
message(STATUS "===== OpenCV ersion: ${OpenCV_VERSION}  =====")
message(STATUS "===== OpenCV libraries: ${OpenCV_LIBS} =====")
message(STATUS "===== OpenCV include path: ${OpenCV_INCLUDE_DIRS} =====")


add_executable(main main.cpp)
target_link_libraries(main
                          ${catkin_LIBRARIES}
                          ${Boost_LIBRARIES}
                          ${Boost_FILESYSTEM_LIBRARY}
                          ${Boost_SERIALIZATION_LIBRARY} # serialization
                          ${OpenCV_LIBRARIES}
                          # ${OpenMVS_LIBRARIES}
                          ${PCL_LIBRARIES}
                          ZLIB::ZLIB
                          gtsam
                        #   pcl_common 
                        #   pcl_io
                          ) 




