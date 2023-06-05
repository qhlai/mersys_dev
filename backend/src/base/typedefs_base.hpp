#pragma once
#include <thread>
#include <cstdint>
#include <list>
#include <vector>
#include <map>
#include <set>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>


class Map;
class MapLIV;

namespace TypeDefs {

    using precision_t                   = double;   // system-wide precision type
    using keypoint_precision_t          = float;    // for 2D keypoint coordinates - float to reduce network traffic
    using idpair                        = std::pair<size_t,size_t>;
    using ThreadPtr                     = std::unique_ptr<std::thread>;

    // using CommPtr                       = std::shared_ptr<Communicator>;
    // using KeyframePtr                   = std::shared_ptr<Keyframe>;
    // using LandmarkPtr                   = std::shared_ptr<Landmark>;
    using MapPtr                           = std::shared_ptr<Map>;
    using MapLIVPtr                        = std::shared_ptr<MapLIV>;
    // using ManagerPtr                    = std::shared_ptr<MapManager>;
    // using PlacerecPtr                   = std::shared_ptr<PlacerecBase>;
    // using VisPtr                        = std::shared_ptr<Visualizer>;

    // using KeypointType                  = Eigen::Matrix<keypoint_precision_t,2,1>;
    // using AorsType                      = Eigen::Matrix<keypoint_precision_t,4,1>;

    // using KeypointVector                = std::vector<KeypointType,Eigen::aligned_allocator<KeypointType>>;
    // using AorsVector                    = std::vector<AorsType,Eigen::aligned_allocator<AorsType>>;

    using Vector2Type                   = Eigen::Matrix<precision_t,2,1>;
    using Vector3Type                   = Eigen::Matrix<precision_t,3,1>;
    using Vector4Type                   = Eigen::Matrix<precision_t,4,1>;
    using DynamicVectorType             = Eigen::Matrix<precision_t,Eigen::Dynamic,1>;
    using QuaternionType                = Eigen::Quaternion<precision_t>;

    using Matrix3Type                   = Eigen::Matrix<precision_t,3,3>;
    using Matrix4Type                   = Eigen::Matrix<precision_t,4,4>;
    using Matrix6Type                   = Eigen::Matrix<precision_t,6,6>;
    using TransformType                 = Matrix4Type;
    using DynamicMatrixType             = Eigen::Matrix<precision_t,Eigen::Dynamic,Eigen::Dynamic>;

    using Vector2Vector                 = std::vector<Vector2Type,Eigen::aligned_allocator<Vector2Type>>;
    using Vector3Vector                 = std::vector<Vector3Type,Eigen::aligned_allocator<Vector3Type>>;
    using Vector4Vector                 = std::vector<Vector4Type,Eigen::aligned_allocator<Vector4Type>>;
    using Matrix3Vector                 = std::vector<Vector4Type,Eigen::aligned_allocator<Matrix3Type>>;
    using Matrix4Vector                 = std::vector<Vector4Type,Eigen::aligned_allocator<Matrix4Type>>;
    using TransformVector               = std::vector<Vector4Type,Eigen::aligned_allocator<TransformType>>;
    using PoseMap                       = std::map<idpair,TransformType,std::less<idpair>,Eigen::aligned_allocator<std::pair<const idpair,TransformType>>>;
    // using VectorIMU                     = std::vector<IMU_Measurement>;

    // using KeyframeVector                = std::vector<KeyframePtr,Eigen::aligned_allocator<KeyframePtr>>;
    // using LandmarkVector                = std::vector<LandmarkPtr,Eigen::aligned_allocator<LandmarkPtr>>;

    // using KeyframeList                  = std::list<KeyframePtr,Eigen::aligned_allocator<KeyframePtr>>;
    // using LandmarkList                  = std::list<LandmarkPtr,Eigen::aligned_allocator<LandmarkPtr>>;

    // using KeyframeSet                   = std::set<KeyframePtr,std::less<KeyframePtr>,Eigen::aligned_allocator<KeyframePtr>>;
    // using LandmarkSet                   = std::set<LandmarkPtr,std::less<LandmarkPtr>,Eigen::aligned_allocator<LandmarkPtr>>;

    // using KeyframeMap                   = std::map<idpair,KeyframePtr,std::less<idpair>,Eigen::aligned_allocator<std::pair<const idpair,KeyframePtr>>>; // map allocator: first element of pair must be declared const
    // using LandmarkMap                   = std::map<idpair,LandmarkPtr,std::less<idpair>,Eigen::aligned_allocator<std::pair<const idpair,LandmarkPtr>>>; // map allocator: first element of pair must be declared const

    // using KeyframePairVector            = std::vector<std::pair<KeyframePtr,KeyframePtr>, Eigen::aligned_allocator<std::pair<KeyframePtr,KeyframePtr>>>;
    // using KeyframeIntMap                = std::map<KeyframePtr,int,std::less<KeyframePtr>,Eigen::aligned_allocator<std::pair<const KeyframePtr,int>>>;
    // using LoopVector                    = std::vector<LoopConstraint, Eigen::aligned_allocator<LoopConstraint>>;
    // using LoopConnectionType            = std::map<KeyframePtr,KeyframeSet,std::less<KeyframePtr>,Eigen::aligned_allocator<std::pair<const KeyframePtr,KeyframeSet>>>;
    // using IntKfPair                     = std::pair<int,KeyframePtr>;
    // using VectorIntKfPair               = std::vector<IntKfPair,Eigen::aligned_allocator<IntKfPair>>;
    // using KfObservations                = std::map<KeyframePtr,size_t,std::less<KeyframePtr>,Eigen::aligned_allocator<std::pair<const KeyframePtr,size_t>>>;

    // communication-specific
    using MsgTypeVector                 = std::vector<uint32_t>;
    using LandmarksMinimalType          = std::map<int, idpair>;
    using ObservationsMinimalType       = std::map<idpair,int>;

    // using KeyframeMsgList               = std::list<MsgKeyframe,Eigen::aligned_allocator<MsgKeyframe>>;
    // using LandmarkMsgList               = std::list<MsgLandmark,Eigen::aligned_allocator<MsgLandmark>>;
}
