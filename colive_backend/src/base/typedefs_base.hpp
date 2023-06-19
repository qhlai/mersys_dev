#pragma once
#include <thread>
#include <cstdint>
#include <list>
#include <vector>
#include <map>
#include <set>

#include "print_enhancement.hpp"
#include "value_redefine.hpp"
#include "read_parm.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

#define COLIVE_MOD

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/concepts/pair_associative_container.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/access.hpp>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define MAPRANGE std::numeric_limits<uint8_t>::max()
#define KFRANGE std::numeric_limits<uint16_t>::max()
#define MPRANGE std::numeric_limits<uint32_t>::max()
#define UIDRANGE std::numeric_limits<uint32_t>::max()

#define defpair std::make_pair((size_t)KFRANGE,(size_t)MAPRANGE) //default pair
#define defid -1 //default id

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64



namespace colive {
struct IMU_Measurement;

// Forward Decs
class Communicator_server;
class Communicator_client;
class Keyframe;
class Landmark;
class Map;
class MapInstance;  
class MapManager;
class PlacerecBase;
class Visualizer;
class Client;
class PointCloud_ex;

struct MsgKeyframe;
struct MsgLandmark;
struct MsgPointCloud;
struct MsgOdometry;


// typedef pcl::PointXYZINormal PointType;

namespace TypeDefs {

    using precision_t                   = double;   // system-wide precision type
    using keypoint_precision_t          = float;    // for 2D keypoint coordinates - float to reduce network traffic
    using idpair                        = std::pair<size_t,size_t>;
    using ThreadPtr                     = std::unique_ptr<std::thread>;

    using PointType                     = pcl::PointXYZINormal;
    using RGBPointType                  = pcl::PointXYZINormal;
    using VoxelGrid                     = pcl::VoxelGrid<PointType>;
    using PointCloud                    = pcl::PointCloud<PointType>;// pcl::PointCloud<PointXYZINormal>
    using PointCloudRBG                 = pcl::PointCloud<pcl::PointXYZRGB>;
    using PointCloudEX                  = PointCloud_ex;

    using CommClientPtr                       = std::shared_ptr<Communicator_client>;
    using CommServerPtr                       = std::shared_ptr<Communicator_server>;
    using KeyframePtr                   = std::shared_ptr<Keyframe>;
    using LandmarkPtr                   = std::shared_ptr<Landmark>;
    using PointCloudPtr                 = std::shared_ptr<PointCloud>;
    using PointCloudEXPtr                 = std::shared_ptr<PointCloudEX>;

    using ClientPtr                        = std::shared_ptr<Client>;
    using ClientVector                     = std::vector<ClientPtr>;
    using MapPtr                           = std::shared_ptr<Map>;
    using MapInstancePtr                   = std::shared_ptr<MapInstance>;
    using MapManagerPtr                    = std::shared_ptr<MapManager>;

    using PlacerecPtr                   = std::shared_ptr<PlacerecBase>;

    using VisPtr                        = std::shared_ptr<Visualizer>;

    using KeypointType                  = Eigen::Matrix<keypoint_precision_t,2,1>;
    using AorsType                      = Eigen::Matrix<keypoint_precision_t,4,1>;

    using KeypointVector                = std::vector<KeypointType,Eigen::aligned_allocator<KeypointType>>;
    using AorsVector                    = std::vector<AorsType,Eigen::aligned_allocator<AorsType>>;

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

    using KeyframeVector                = std::vector<KeyframePtr,Eigen::aligned_allocator<KeyframePtr>>;
    using LandmarkVector                = std::vector<LandmarkPtr,Eigen::aligned_allocator<LandmarkPtr>>;

    using KeyframeList                  = std::list<KeyframePtr,Eigen::aligned_allocator<KeyframePtr>>;
    using LandmarkList                  = std::list<LandmarkPtr,Eigen::aligned_allocator<LandmarkPtr>>;
    using PointCloudEXList              = std::list<PointCloudEXPtr,Eigen::aligned_allocator<PointCloudEXPtr>>;

    using KeyframeSet                   = std::set<KeyframePtr,std::less<KeyframePtr>,Eigen::aligned_allocator<KeyframePtr>>;
    using LandmarkSet                   = std::set<LandmarkPtr,std::less<LandmarkPtr>,Eigen::aligned_allocator<LandmarkPtr>>;
    using PointCloudEXSet                 = std::set<PointCloudEXPtr,std::less<PointCloudEXPtr>,Eigen::aligned_allocator<PointCloudEXPtr>>;

    using KeyframeMap                   = std::map<idpair,KeyframePtr,std::less<idpair>,Eigen::aligned_allocator<std::pair<const idpair,KeyframePtr>>>; // map allocator: first element of pair must be declared const
    using LandmarkMap                   = std::map<idpair,LandmarkPtr,std::less<idpair>,Eigen::aligned_allocator<std::pair<const idpair,LandmarkPtr>>>; // map allocator: first element of pair must be declared const
    using PointCloudMap                 = std::map<idpair,PointCloudEXPtr,std::less<idpair>,Eigen::aligned_allocator<std::pair<const idpair,PointCloudEXPtr>>>; // map allocator: first element of pair must be declared const
    using PointCloudEXMap                 = std::map<idpair,PointCloudEXPtr,std::less<idpair>,Eigen::aligned_allocator<std::pair<const idpair,PointCloudEXPtr>>>; // map allocator: first element of pair must be declared const
    
    using KeyframePairVector            = std::vector<std::pair<KeyframePtr,KeyframePtr>, Eigen::aligned_allocator<std::pair<KeyframePtr,KeyframePtr>>>;
    using KeyframeIntMap                = std::map<KeyframePtr,int,std::less<KeyframePtr>,Eigen::aligned_allocator<std::pair<const KeyframePtr,int>>>;
    // using LoopVector                    = std::vector<LoopConstraint, Eigen::aligned_allocator<LoopConstraint>>;
    using LoopConnectionType            = std::map<KeyframePtr,KeyframeSet,std::less<KeyframePtr>,Eigen::aligned_allocator<std::pair<const KeyframePtr,KeyframeSet>>>;
    using IntKfPair                     = std::pair<int,KeyframePtr>;
    using VectorIntKfPair               = std::vector<IntKfPair,Eigen::aligned_allocator<IntKfPair>>;
    using KfObservations                = std::map<KeyframePtr,size_t,std::less<KeyframePtr>,Eigen::aligned_allocator<std::pair<const KeyframePtr,size_t>>>;

    // communication-specific
    using MsgTypeVector                 = std::vector<uint32_t>;
    using LandmarksMinimalType          = std::map<int, idpair>;
    using ObservationsMinimalType       = std::map<idpair,int>;

    using KeyframeMsgList               = std::list<MsgKeyframe,Eigen::aligned_allocator<MsgKeyframe>>;
    using LandmarkMsgList               = std::list<MsgLandmark,Eigen::aligned_allocator<MsgLandmark>>;
    using PointCloudMsgList             = std::list<MsgPointCloud,Eigen::aligned_allocator<MsgPointCloud>>;
    using OdometryMsgList               = std::list<MsgOdometry,Eigen::aligned_allocator<MsgOdometry>>;
}



inline std::ostream &operator<<(std::ostream &out, const TypeDefs::idpair id) {
    return out << id.first << "|" << id.second;
}

struct IMU_Measurement
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;

public:
    IMU_Measurement() = delete;
    IMU_Measurement(precision_t lax, precision_t lay, precision_t laz,
                    precision_t avx, precision_t avy, precision_t avz,
                    precision_t dt)
        : lin_acc_x(lax),
          lin_acc_y(lay),
          lin_acc_z(laz),
          ang_vel_x(avx),
          ang_vel_y(avy),
          ang_vel_z(avz),
          delta_t(dt)
    {}

    IMU_Measurement(precision_t lax, precision_t lay, precision_t laz,
                    precision_t avx, precision_t avy, precision_t avz,
                    IMU_Measurement& imu_last,
                    precision_t dt_before, precision_t dt_after)
        : delta_t(dt_before)
    {
        precision_t w1 = dt_after / (dt_before + dt_after);       //If timestamp is closer to the new message, the old message has less weight
        precision_t w2 = dt_before  / (dt_before + dt_after);
        lin_acc_x = w1 * imu_last.lin_acc_x + w2 * lax;
        lin_acc_y = w1 * imu_last.lin_acc_y + w2 * lay;
        lin_acc_z = w1 * imu_last.lin_acc_z + w2 * laz;
        ang_vel_x = w1 * imu_last.ang_vel_x + w2 * avx;
        ang_vel_y = w1 * imu_last.ang_vel_y + w2 * avy;
        ang_vel_z = w1 * imu_last.ang_vel_z + w2 * avz;
    }

    precision_t                 lin_acc_x;
    precision_t                 lin_acc_y;
    precision_t                 lin_acc_z;
    precision_t                 ang_vel_x;
    precision_t                 ang_vel_y;
    precision_t                 ang_vel_z;
    precision_t                 delta_t;
};

enum eDistortionModel
{
    NOTSET_DIST     = -1,
    RADTAN          =  0,
    EQUI            =  1,
    PLUMBBOB        =  2
};

enum eCamModel
{
    NOTSET_CAM      = -1,
    PINHOLE         =  0,
    OMNI            =  1
};

class Keyframe;

struct LoopConstraint {
    using TransformType             = TypeDefs::TransformType;
    using KeyframePtr               = std::shared_ptr<Keyframe>;
    using Matrix6Type               = TypeDefs::Matrix6Type;

    LoopConstraint(KeyframePtr k1, KeyframePtr k2, TransformType T_12,
                   Matrix6Type covm = Matrix6Type::Identity())
        : kf1(k1), kf2(k2), T_s1_s2(T_12), cov_mat(covm) {}
    
    KeyframePtr         kf1;
    KeyframePtr         kf2;
    TransformType       T_s1_s2;
    Matrix6Type         cov_mat;
};

struct VICalibration {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using Vector2Type                   = TypeDefs::Vector2Type;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;
    using DynamicVectorType             = TypeDefs::DynamicVectorType;

public:

    VICalibration()
        : T_SC(Eigen::Matrix4d::Zero()),cam_model(static_cast<eCamModel>(-1)),dist_model(static_cast<eDistortionModel>(-1)),
          img_dims((Eigen::Matrix<double,2,1>::Zero())),
          dist_coeffs(Eigen::Vector4d::Zero()),
          intrinsics(Eigen::Matrix<double,4,1>::Zero()),
          K((Eigen::Matrix<double,3,3>() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished()),
          a_max(0.0),g_max(0.0),
          sigma_a_c(0.0),sigma_g_c(0.0),sigma_ba(0.0),sigma_bg(0.0),sigma_aw_c(0.0),sigma_gw_c(0.0),
          tau(0.0),g(0.0),a0(Eigen::Vector3d::Zero()),rate(0),
          delay_cam0_to_imu(0.0),delay_cam1_to_imu(0.0)
    {}

    VICalibration(Eigen::Matrix4d Tsc,eCamModel cmodel, eDistortionModel dmodel,
                  Eigen::VectorXd DistCoeffs,
                  double dw,double dh,
                  double dfx,double dfy,double dcx,double dcy,
                  double damax,double dgmax,double dsigmaac,double dsigmagc,double dsigmaba,double dsigmabg,double dsigmaawc,double dsigmagwc,
                  double dtau,double dg,Eigen::Vector3d va0,int irate,
                  double dDelayC0toIMU,double dDelayC1toIMU)
        : T_SC(Tsc),cam_model(cmodel),dist_model(dmodel),
          img_dims((Eigen::Matrix<precision_t,2,1>() << dw,dh).finished()),
          dist_coeffs(DistCoeffs),
          intrinsics((Eigen::Matrix<precision_t,4,1>() << dfx,dfy,dcx,dcy).finished()),
          K((Eigen::Matrix<precision_t,3,3>() << dfx, 0.0, dcx, 0.0, dfy, dcy, 0.0, 0.0, 1.0).finished()),
          a_max(damax),g_max(dgmax),
          sigma_a_c(dsigmaac),sigma_g_c(dsigmagc),sigma_ba(dsigmaba),sigma_bg(dsigmabg),sigma_aw_c(dsigmaawc),sigma_gw_c(dsigmagwc),
          tau(dtau),g(dg),a0(va0),rate(irate),
          delay_cam0_to_imu(dDelayC0toIMU),delay_cam1_to_imu(dDelayC1toIMU)
    {}

    //Cam
    Eigen::Matrix4d             T_SC;                                                                                           ///< Transformation from camera to sensor (IMU) frame.
    eCamModel                   cam_model;                                                                                      ///< Distortion type. ('pinhole' 'omni')
    eDistortionModel            dist_model;                                                                                     ///< Distortion type. ('radialtangential' 'plumb_bob' 'equidistant')
    Vector2Type                 img_dims;                                                                                       ///< Image dimension. [pixels] (width;height)
    DynamicVectorType           dist_coeffs;                                                                                    ///< Distortion Coefficients.
    DynamicVectorType           intrinsics;                                                                                     ///< fx fy cx cy
    Matrix3Type                 K;
    //IMU
    precision_t                 a_max;                                                                                          ///< Accelerometer saturation. [m/s^2] -- not used
    precision_t                 g_max;                                                                                          ///< Gyroscope saturation. [rad/s] -- not used
    precision_t                 sigma_a_c;                                                                                      ///< Accelerometer noise density.
    precision_t                 sigma_g_c;                                                                                      ///< Gyroscope noise density.
    precision_t                 sigma_ba;                                                                                       ///< Initial accelerometer bias -- not used
    precision_t                 sigma_bg;                                                                                       ///< Initial gyroscope bias. -- not used
    precision_t                 sigma_aw_c;                                                                                     ///< Accelerometer drift noise density.
    precision_t                 sigma_gw_c;                                                                                     ///< Gyroscope drift noise density.
    precision_t                 tau;                                                                                            ///< Reversion time constant of accerometer bias. [s] -- not used
    precision_t                 g;                                                                                              ///< Earth acceleration.
    Vector3Type                 a0;                                                                                             ///< Mean of the prior accelerometer bias. -- not used
    int                         rate;                                                                                           ///< IMU rate in Hz.
    precision_t                 delay_cam0_to_imu;                                                                              ///< Timestamp shift. Timu = Timage + image_delay
    precision_t                 delay_cam1_to_imu;                                                                              ///< Timestamp shift. Timu = Timage + image_delay

    void show()
    {
        std::cout << "--- Cam ---" << std::endl;
        std::cout << "T_SC: \n" << T_SC << std::endl;
        if(cam_model == eCamModel::PINHOLE)                 std::cout << "CamModel: pinhole" << std::endl;
        else if(cam_model == eCamModel::OMNI)               std::cout << "CamModel: omni" << std::endl;
        if(dist_model == eDistortionModel::RADTAN)          std::cout << "DistortionModel: radialtangential" << std::endl;
        else if(dist_model == eDistortionModel::EQUI)       std::cout << "DistortionModel: equidistant" << std::endl;
        else if(dist_model == eDistortionModel::PLUMBBOB)   std::cout << "DistortionModel: plumb_bob" << std::endl;
        std::cout << "Image Dimensions: \n" << img_dims << std::endl;
        std::cout << "Distortion Coefficients: \n" << dist_coeffs << std::endl;
        std::cout << "Intrinsics: \n" << intrinsics << std::endl;
        std::cout << "K: \n" << K << std::endl;
        std::cout << "--- IMU ---" << std::endl;
        std::cout << "a_max: " << a_max << std::endl;
        std::cout << "g_max: " << g_max << std::endl;
        std::cout << "sigma_g_c: " << sigma_g_c << std::endl;
        std::cout << "sigma_bg: " << sigma_bg << std::endl;
        std::cout << "sigma_a_c: " << sigma_a_c << std::endl;
        std::cout << "sigma_ba: " << sigma_ba << std::endl;
        std::cout << "sigma_gw_c: " << sigma_gw_c << std::endl;
        std::cout << "sigma_aw_c: " << sigma_aw_c << std::endl;
        std::cout << "tau: " << tau << std::endl;
        std::cout << "g: " << g << std::endl;
        std::cout << "a0: \n" << a0 << std::endl;
        std::cout << "rate: " << rate << std::endl;
        std::cout << "delay_cam0_to_imu: " << delay_cam0_to_imu << std::endl;
        std::cout << "delay_cam1_to_imu: " << delay_cam1_to_imu << std::endl;
    }

    template<class Archive> auto serialize( Archive & archive )->void {
        archive(T_SC, cam_model, dist_model, img_dims, dist_coeffs, intrinsics, K,
                a_max, g_max, sigma_a_c, sigma_g_c, sigma_ba, sigma_bg, sigma_aw_c, sigma_gw_c, tau, g, a0,
                rate, delay_cam0_to_imu, delay_cam1_to_imu);
    }
};


}