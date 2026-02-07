#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <sophus/so3.hpp>

#include "../map_builder/commons.h"

struct KeyFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    size_t id;
    double timestamp;
    M3D rotation;           // world -> imu rotation
    V3D position;           // world -> imu translation
    M3D original_rotation;  // unoptimized rotation (before loop closure)
    V3D original_position;  // unoptimized position (before loop closure)
    CloudType::Ptr cloud;   // undistorted point cloud in body frame (may be nullptr if cleaned)
    bool cloud_valid;       // whether cloud is still in memory
    
    KeyFrame() : id(0), timestamp(0.0), rotation(M3D::Identity()), 
                 position(V3D::Zero()), original_rotation(M3D::Identity()),
                 original_position(V3D::Zero()), cloud(nullptr), cloud_valid(false) {}
    
    KeyFrame(size_t _id, double _time, const M3D& _rot, const V3D& _pos, 
             CloudType::Ptr _cloud)
        : id(_id), timestamp(_time), rotation(_rot), position(_pos),
          original_rotation(_rot), original_position(_pos), cloud(_cloud),
          cloud_valid(_cloud != nullptr && !_cloud->empty()) {}
};

struct KeyFrameConfig {
    double dist_threshold = 1.0;      // keyframe distance threshold (m)
    double angle_threshold = 0.2;     // keyframe angle threshold (rad)
    
    // Memory management config
    int max_keyframes_with_cloud = 500;   // max keyframes that keep cloud in memory
    bool enable_cloud_cleanup = true;     // enable automatic cloud cleanup
    double cloud_downsample_res = 0.15;   // downsample resolution for keyframe cloud (0=disable)
};

class KeyFrameManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    KeyFrameManager() = default;
    explicit KeyFrameManager(const KeyFrameConfig& config);
    
    // Check if current pose should be added as keyframe
    bool shouldAddKeyFrame(const M3D& rotation, const V3D& position);
    
    // Add a new keyframe
    void addKeyFrame(const KeyFrame& kf);
    
    // Add keyframe with components
    void addKeyFrame(size_t id, double timestamp, const M3D& rotation, 
                     const V3D& position, CloudType::Ptr cloud);
    
    // Search for nearby keyframes within radius
    std::vector<int> searchNearbyKeyFrames(const V3D& position, float radius);
    
    // Search for nearby keyframes excluding recent ones
    std::vector<int> searchNearbyKeyFrames(const V3D& position, float radius, 
                                           int exclude_recent);
    
    // Get keyframe by index
    const KeyFrame& getKeyFrame(size_t idx) const;
    
    // Get all keyframes
    const std::vector<KeyFrame>& getAllKeyFrames() const { return keyframes_; }
    
    // Get number of keyframes
    size_t size() const { return keyframes_.size(); }
    
    // Check if empty
    bool empty() const { return keyframes_.empty(); }
    
    // Get latest keyframe
    const KeyFrame& getLatestKeyFrame() const;
    
    // Build submap from keyframe indices
    CloudType::Ptr buildSubmap(const std::vector<int>& indices);
    
    // Build submap around a keyframe (in center frame's body coordinate)
    CloudType::Ptr buildSubmapInBodyFrame(int center_idx, int num_neighbors);
    
    // Legacy function - calls buildSubmapInBodyFrame
    CloudType::Ptr buildSubmapAroundKeyFrame(int kf_idx, int num_neighbors) {
        return buildSubmapInBodyFrame(kf_idx, num_neighbors);
    }
    
    // Update keyframe pose (for loop closure correction)
    void updateKeyFramePose(size_t idx, const M3D& rotation, const V3D& position);
    
    // Get all positions as point cloud (for visualization)
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPositionsAsCloud();
    
    // Get trajectory clouds (compatible with fast_lio_sam)
    pcl::PointCloud<PointType>::Ptr getCloudKeyPoses3D() const;
    PoseCloud::Ptr getCloudKeyPoses6D() const;
    PoseCloud::Ptr getUnoptimizedKeyPoses6D() const;
    
    // Memory management
    size_t getKeyframesWithCloudCount() const;
    void cleanupOldClouds();
    
    // Thread-safe access
    std::mutex& getMutex() { return mutex_; }

private:
    void updateKdTree();
    void updatePoseClouds(size_t idx);
    void downsampleCloud(CloudType::Ptr& cloud);
    
    // Euler angle conversion helper
    static void rotationToEuler(const M3D& rotation, float& roll, float& pitch, float& yaw);
    
    KeyFrameConfig config_;
    std::vector<KeyFrame> keyframes_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr positions_cloud_;
    
    // Trajectory point clouds (compatible with fast_lio_sam format)
    pcl::PointCloud<PointType>::Ptr cloud_key_poses_3d_;      // position only
    PoseCloud::Ptr cloud_key_poses_6d_;                        // full 6DOF pose
    PoseCloud::Ptr unoptimized_key_poses_6d_;                  // original unoptimized poses
    
    M3D last_kf_rotation_ = M3D::Identity();
    V3D last_kf_position_ = V3D::Zero();
    
    std::mutex mutex_;
    bool kdtree_updated_ = false;
};
