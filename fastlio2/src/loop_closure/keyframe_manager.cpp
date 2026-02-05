#include "keyframe_manager.h"
#include <iostream>
#include <cmath>

KeyFrameManager::KeyFrameManager(const KeyFrameConfig& config)
    : config_(config),
      kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()),
      positions_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
      cloud_key_poses_3d_(new pcl::PointCloud<PointType>()),
      cloud_key_poses_6d_(new PoseCloud()),
      unoptimized_key_poses_6d_(new PoseCloud()) {
}

void KeyFrameManager::rotationToEuler(const M3D& rotation, float& roll, float& pitch, float& yaw) {
    // Convert rotation matrix to Euler angles (roll, pitch, yaw)
    // Using ZYX convention (yaw-pitch-roll)
    Eigen::Vector3d euler = rotation.eulerAngles(2, 1, 0);  // ZYX order
    yaw = static_cast<float>(euler(0));
    pitch = static_cast<float>(euler(1));
    roll = static_cast<float>(euler(2));
}

bool KeyFrameManager::shouldAddKeyFrame(const M3D& rotation, const V3D& position) {
    // First frame is always a keyframe
    if (keyframes_.empty()) {
        return true;
    }
    
    // Calculate relative pose from last keyframe
    M3D rel_rotation = last_kf_rotation_.transpose() * rotation;
    V3D rel_position = position - last_kf_position_;
    
    // Calculate translation distance
    double dist = rel_position.norm();
    
    // Calculate rotation angle using Sophus
    Sophus::SO3d rel_so3(rel_rotation);
    V3D angle_axis = rel_so3.log();
    double angle = angle_axis.norm();
    
    // Check thresholds
    if (dist >= config_.dist_threshold || angle >= config_.angle_threshold) {
        return true;
    }
    
    return false;
}

void KeyFrameManager::addKeyFrame(const KeyFrame& kf) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    keyframes_.push_back(kf);
    
    // Update last keyframe pose
    last_kf_rotation_ = kf.rotation;
    last_kf_position_ = kf.position;
    
    // Add position to cloud for kdtree
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(kf.position.x());
    pt.y = static_cast<float>(kf.position.y());
    pt.z = static_cast<float>(kf.position.z());
    positions_cloud_->push_back(pt);
    
    // Add to 3D pose cloud (position only)
    PointType pose3d;
    pose3d.x = static_cast<float>(kf.position.x());
    pose3d.y = static_cast<float>(kf.position.y());
    pose3d.z = static_cast<float>(kf.position.z());
    pose3d.intensity = static_cast<float>(kf.id);
    cloud_key_poses_3d_->push_back(pose3d);
    
    // Add to 6D pose cloud
    PointTypePose pose6d;
    pose6d.x = static_cast<float>(kf.position.x());
    pose6d.y = static_cast<float>(kf.position.y());
    pose6d.z = static_cast<float>(kf.position.z());
    pose6d.intensity = static_cast<float>(kf.id);
    rotationToEuler(kf.rotation, pose6d.roll, pose6d.pitch, pose6d.yaw);
    pose6d.time = kf.timestamp;
    cloud_key_poses_6d_->push_back(pose6d);
    
    // Add to unoptimized pose cloud (same as original at creation time)
    PointTypePose unopt_pose6d;
    unopt_pose6d.x = static_cast<float>(kf.original_position.x());
    unopt_pose6d.y = static_cast<float>(kf.original_position.y());
    unopt_pose6d.z = static_cast<float>(kf.original_position.z());
    unopt_pose6d.intensity = static_cast<float>(kf.id);
    rotationToEuler(kf.original_rotation, unopt_pose6d.roll, unopt_pose6d.pitch, unopt_pose6d.yaw);
    unopt_pose6d.time = kf.timestamp;
    unoptimized_key_poses_6d_->push_back(unopt_pose6d);
    
    kdtree_updated_ = false;
    
    std::cout << "[KeyFrameManager] Added keyframe " << kf.id 
              << " at position (" << kf.position.x() << ", " 
              << kf.position.y() << ", " << kf.position.z() << ")" 
              << ", total: " << keyframes_.size() << std::endl;
}

void KeyFrameManager::addKeyFrame(size_t id, double timestamp, const M3D& rotation,
                                   const V3D& position, CloudType::Ptr cloud) {
    KeyFrame kf(id, timestamp, rotation, position, cloud);
    addKeyFrame(kf);
}

void KeyFrameManager::updateKdTree() {
    if (positions_cloud_->empty()) {
        return;
    }
    
    if (!kdtree_updated_) {
        kdtree_->setInputCloud(positions_cloud_);
        kdtree_updated_ = true;
    }
}

std::vector<int> KeyFrameManager::searchNearbyKeyFrames(const V3D& position, float radius) {
    return searchNearbyKeyFrames(position, radius, 0);
}

std::vector<int> KeyFrameManager::searchNearbyKeyFrames(const V3D& position, float radius,
                                                         int exclude_recent) {
    std::vector<int> result;
    
    if (positions_cloud_->empty()) {
        return result;
    }
    
    updateKdTree();
    
    pcl::PointXYZ search_point;
    search_point.x = static_cast<float>(position.x());
    search_point.y = static_cast<float>(position.y());
    search_point.z = static_cast<float>(position.z());
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    kdtree_->radiusSearch(search_point, radius, indices, distances);
    
    // Filter out recent keyframes
    int max_valid_idx = static_cast<int>(keyframes_.size()) - exclude_recent - 1;
    
    for (int idx : indices) {
        if (idx <= max_valid_idx) {
            result.push_back(idx);
        }
    }
    
    return result;
}

const KeyFrame& KeyFrameManager::getKeyFrame(size_t idx) const {
    if (idx >= keyframes_.size()) {
        throw std::out_of_range("KeyFrame index out of range");
    }
    return keyframes_[idx];
}

const KeyFrame& KeyFrameManager::getLatestKeyFrame() const {
    if (keyframes_.empty()) {
        throw std::runtime_error("No keyframes available");
    }
    return keyframes_.back();
}

CloudType::Ptr KeyFrameManager::buildSubmap(const std::vector<int>& indices) {
    CloudType::Ptr submap(new CloudType());
    
    for (int idx : indices) {
        if (idx < 0 || idx >= static_cast<int>(keyframes_.size())) {
            continue;
        }
        
        const KeyFrame& kf = keyframes_[idx];
        if (!kf.cloud) {
            continue;
        }
        
        // Transform cloud to world frame
        CloudType::Ptr transformed(new CloudType());
        for (const auto& pt : kf.cloud->points) {
            PointType new_pt = pt;
            V3D p_body(pt.x, pt.y, pt.z);
            V3D p_world = kf.rotation * p_body + kf.position;
            new_pt.x = static_cast<float>(p_world.x());
            new_pt.y = static_cast<float>(p_world.y());
            new_pt.z = static_cast<float>(p_world.z());
            transformed->push_back(new_pt);
        }
        
        *submap += *transformed;
    }
    
    return submap;
}

CloudType::Ptr KeyFrameManager::buildSubmapAroundKeyFrame(int kf_idx, int num_neighbors) {
    std::vector<int> indices;
    
    int start_idx = std::max(0, kf_idx - num_neighbors);
    int end_idx = std::min(static_cast<int>(keyframes_.size()) - 1, kf_idx + num_neighbors);
    
    for (int i = start_idx; i <= end_idx; ++i) {
        indices.push_back(i);
    }
    
    return buildSubmap(indices);
}

void KeyFrameManager::updateKeyFramePose(size_t idx, const M3D& rotation, const V3D& position) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (idx >= keyframes_.size()) {
        return;
    }
    
    keyframes_[idx].rotation = rotation;
    keyframes_[idx].position = position;
    
    // Update position in kdtree cloud
    positions_cloud_->points[idx].x = static_cast<float>(position.x());
    positions_cloud_->points[idx].y = static_cast<float>(position.y());
    positions_cloud_->points[idx].z = static_cast<float>(position.z());
    
    // Update 3D pose cloud
    cloud_key_poses_3d_->points[idx].x = static_cast<float>(position.x());
    cloud_key_poses_3d_->points[idx].y = static_cast<float>(position.y());
    cloud_key_poses_3d_->points[idx].z = static_cast<float>(position.z());
    
    // Update 6D pose cloud
    cloud_key_poses_6d_->points[idx].x = static_cast<float>(position.x());
    cloud_key_poses_6d_->points[idx].y = static_cast<float>(position.y());
    cloud_key_poses_6d_->points[idx].z = static_cast<float>(position.z());
    rotationToEuler(rotation, 
                    cloud_key_poses_6d_->points[idx].roll,
                    cloud_key_poses_6d_->points[idx].pitch,
                    cloud_key_poses_6d_->points[idx].yaw);
    
    // Note: unoptimized_key_poses_6d_ is NOT updated - it keeps the original poses
    
    kdtree_updated_ = false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KeyFrameManager::getPositionsAsCloud() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud = *positions_cloud_;
    return cloud;
}

pcl::PointCloud<PointType>::Ptr KeyFrameManager::getCloudKeyPoses3D() const {
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    *cloud = *cloud_key_poses_3d_;
    return cloud;
}

PoseCloud::Ptr KeyFrameManager::getCloudKeyPoses6D() const {
    PoseCloud::Ptr cloud(new PoseCloud());
    *cloud = *cloud_key_poses_6d_;
    return cloud;
}

PoseCloud::Ptr KeyFrameManager::getUnoptimizedKeyPoses6D() const {
    PoseCloud::Ptr cloud(new PoseCloud());
    *cloud = *unoptimized_key_poses_6d_;
    return cloud;
}
