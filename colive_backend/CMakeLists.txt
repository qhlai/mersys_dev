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
${PROJECT_SOURCE_DIR}/src/communicator/msgs
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

# set(EXTERNAL_LIBRARIES
#   pthread
# )
file(GLOB_RECURSE BACKEND_INC_FILES     
${PROJECT_SOURCE_DIR}/src/core/*.hpp
)
file(GLOB_RECURSE BACKEND_SRC_FILES     
${PROJECT_SOURCE_DIR}/src/core/*.cpp
)
file(GLOB_RECURSE FRAME_INC_FILES     
${PROJECT_SOURCE_DIR}/src/map/frames/*.cpp
)
file(GLOB_RECURSE FRAME_SRC_FILES     
${PROJECT_SOURCE_DIR}/src/map/frames/*.cpp
)
file(GLOB_RECURSE COMM_INC_FILES     
${PROJECT_SOURCE_DIR}/src/communicator/*.hpp
)
file(GLOB_RECURSE COMM_SRC_FILES     
${PROJECT_SOURCE_DIR}/src/communicator/*.cpp
)

file(GLOB_RECURSE MAP_INC_FILES     
${PROJECT_SOURCE_DIR}/src/map/*.hpp
)
file(GLOB_RECURSE MAP_SRC_FILES     
${PROJECT_SOURCE_DIR}/src/map/map_co.cpp
${PROJECT_SOURCE_DIR}/src/map/mapmanager.cpp
${PROJECT_SOURCE_DIR}/src/map/frames/*.cpp
)
file(GLOB_RECURSE PLACE_INC_FILES     
# ${PROJECT_SOURCE_DIR}/src/place_rec/*.hpp
${PROJECT_SOURCE_DIR}/src/place_rec/place_rec.hpp
${PROJECT_SOURCE_DIR}/src/place_rec/scancontext/*.hpp
${PROJECT_SOURCE_DIR}/src/place_rec/scancontext/*.h
# ${PROJECT_SOURCE_DIR}/src/visualizer/visualizer.cpp
)

file(GLOB_RECURSE PLACE_SRC_FILES     
${PROJECT_SOURCE_DIR}/src/place_rec/place_rec.cpp
${PROJECT_SOURCE_DIR}/src/place_rec/scancontext/Scancontext.cpp
# ${PROJECT_SOURCE_DIR}/src/visualizer/visualizer.cpp
)
file(GLOB_RECURSE VIS_INC_FILES     
${PROJECT_SOURCE_DIR}/src/visualizer/*.cpp
# ${PROJECT_SOURCE_DIR}/src/visualizer/visualizer.cpp
)

file(GLOB_RECURSE VIS_SRC_FILES     
${PROJECT_SOURCE_DIR}/src/visualizer/*.cpp
# ${PROJECT_SOURCE_DIR}/src/visualizer/visualizer.cpp
)
message("${PROJECT_SOURCE_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(${PROJECT_NAME}_LIBRARIES
    ${PROJECT_NAME}
)
# This function must be called before declaring any targets with add_library() or add_executable().
catkin_package(CATKIN_DEPENDS
    INCLUDE_DIRS ${INCLUDE_DIRS}
    LIBRARIES ${${PROJECT_NAME}_LIBRARIES}
    # LIBRARIES  colive_backend

	)

add_executable(colive_test_timer src/tools/test_timer.cpp)


# add_library(colive_backend_lib SHARED src/core/backend_node.cpp
# ${BACKEND_SRC_FILES}
# ${MAP_SRC_FILES}
# ${COMM_SRC_FILES}   
# ${VIS_SRC_FILES}
# # ${PROJECT_SOURCE_DIR}/src/core/sb.cpp
# )

add_executable(colive_backend_node src/core/backend_node.cpp
    ${BACKEND_SRC_FILES}
    
    ${MAP_SRC_FILES}
    ${COMM_SRC_FILES}
    ${PLACE_SRC_FILES}   
    ${VIS_SRC_FILES}
    # ${PROJECT_SOURCE_DIR}/src/core/sb.cpp
)
target_link_libraries(colive_backend_node
                          ${catkin_LIBRARIES}
                          ${Boost_LIBRARIES}
                          ${Boost_FILESYSTEM_LIBRARY}
                          ${Boost_SERIALIZATION_LIBRARY} # serialization
                          ${OpenCV_LIBRARIES}
                          # ${OpenMVS_LIBRARIES}
                          ${PCL_LIBRARIES}
                          ZLIB::ZLIB
                        #   pcl_common 
                        #   pcl_io
                          ) 

add_executable(colive_test_node src/test/test_node.cpp
    ${PROJECT_SOURCE_DIR}/src/core/backend.cpp
    ${PROJECT_SOURCE_DIR}/src/core/config_backend.cpp
    ${PROJECT_SOURCE_DIR}/src/test/test_client.cpp
    ${MAP_SRC_FILES}
    ${COMM_SRC_FILES}   
    ${PLACE_SRC_FILES}
    ${VIS_SRC_FILES}
)

target_link_libraries(colive_test_node
                        ${catkin_LIBRARIES}
                        ${Boost_LIBRARIES}
                        ${Boost_FILESYSTEM_LIBRARY}
                        ${Boost_SERIALIZATION_LIBRARY} # serialization
                        ${OpenCV_LIBRARIES}
                        # ${OpenMVS_LIBRARIES}
                        ${PCL_LIBRARIES}
                        ZLIB::ZLIB
                        #   pcl_common 
                        #   pcl_io
) 

add_library(${PROJECT_NAME} SHARED 
    ${BACKEND_INC_FILES}
    ${BACKEND_SRC_FILES}
    ${FRAME_INC_FILES}
    ${FRAME_SRC_FILES}  
    ${MAP_INC_FILES}
    ${MAP_SRC_FILES}
    ${COMM_INC_FILES}  
    ${COMM_SRC_FILES}  
    ${PLACE_INC_FILES}
    ${PLACE_SRC_FILES}
    ${VIS_INC_FILES} 
    ${VIS_SRC_FILES}
)
target_link_libraries(${PROJECT_NAME}
                        ${catkin_LIBRARIES}
                        ${Boost_LIBRARIES}
                        ${Boost_FILESYSTEM_LIBRARY}
                        ${Boost_SERIALIZATION_LIBRARY} # serialization
                        ${OpenCV_LIBRARIES}
                        ZLIB::ZLIB
                        # ${OpenMVS_LIBRARIES}
                        # ${PCL_LIBRARIES}
                        #   pcl_common 
                        #   pcl_io
) 
message(STATUS "=======================================")




set(${PROJECT_NAME}_TARGETS
    ${${PROJECT_NAME}_LIBRARYS}
    colive_backend_node
    colive_test_node
)

# install(TARGETS ${${PROJECT_NAME}_TARGETS}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# install(DIRECTORY
#   launch
#   config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# 	)


