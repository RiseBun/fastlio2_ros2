#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iomanip>
#include <sstream>

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

// 6DOF Pose point type for trajectory storage (compatible with fast_lio_sam)
struct PointTypePose {
    PCL_ADD_POINT4D;                  // x, y, z, padding
    float intensity;                   // keyframe index
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointTypePose,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)
    (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)
    (double, time, time)
)

using PoseCloud = pcl::PointCloud<PointTypePose>;

// Utility: pad zeros for file naming (e.g., 1 -> "000001")
inline std::string padZeros(int val, int width = 6) {
    std::ostringstream ss;
    ss << std::setw(width) << std::setfill('0') << val;
    return ss.str();
}

// Pose struct for KITTI format trajectory saving
struct KittiPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    
    KittiPose() : R(Eigen::Matrix3d::Identity()), t(Eigen::Vector3d::Zero()) {}
    KittiPose(const Eigen::Matrix3d& _R, const Eigen::Vector3d& _t) : R(_R), t(_t) {}
};

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M2D = Eigen::Matrix2d;
using V2D = Eigen::Vector2d;
using M2F = Eigen::Matrix2f;
using V2F = Eigen::Vector2f;
using M4D = Eigen::Matrix4d;
using V4D = Eigen::Vector4d;


template <typename T>
using Vec = std::vector<T>;


bool esti_plane(PointVec &points, const double &thresh, V4D &out);

float sq_dist(const PointType &p1, const PointType &p2);

struct Config
{
    int lidar_filter_num = 3;
    double lidar_min_range = 0.5;
    double lidar_max_range = 20.0;
    double scan_resolution = 0.15;
    double map_resolution = 0.3;

    double cube_len = 300;
    double det_range = 60;
    double move_thresh = 1.5;

    double na = 0.01;
    double ng = 0.01;
    double nba = 0.0001;
    double nbg = 0.0001;
    int imu_init_num = 20;
    int near_search_num = 5;
    int ieskf_max_iter = 5;
    bool gravity_align = true;
    bool esti_il = false;
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();

    double lidar_cov_inv = 1000.0;
};

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    double time;
    IMUData() = default;
    IMUData(const V3D &a, const V3D &g, double &t) : acc(a), gyro(g), time(t) {}
};

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset;
    V3D acc;
    V3D gyro;
    V3D vel;
    V3D trans;
    M3D rot;
    Pose() = default;
    Pose(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p, const M3D &r) : offset(t), acc(a), gyro(g), vel(v), trans(p), rot(r) {}
};

struct SyncPackage
{
    Vec<IMUData> imus;
    CloudType::Ptr cloud;
    double cloud_start_time = 0.0;
    double cloud_end_time = 0.0;
};