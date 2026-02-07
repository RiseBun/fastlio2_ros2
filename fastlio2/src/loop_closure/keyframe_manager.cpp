#include "keyframe_manager.h"
#include <iostream>
#include <cmath>
#include <pcl/common/transforms.h>

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
    
    // Downsample keyframe cloud if configured
    KeyFrame new_kf = kf;
    if (new_kf.cloud && !new_kf.cloud->empty() && config_.cloud_downsample_res > 0) {
        downsampleCloud(new_kf.cloud);
    }
    new_kf.cloud_valid = (new_kf.cloud != nullptr && !new_kf.cloud->empty());
    
    keyframes_.push_back(new_kf);
    
    // Update last keyframe pose
    last_kf_rotation_ = new_kf.rotation;
    last_kf_position_ = new_kf.position;
    
    // Add position to cloud for kdtree
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(new_kf.position.x());
    pt.y = static_cast<float>(new_kf.position.y());
    pt.z = static_cast<float>(new_kf.position.z());
    positions_cloud_->push_back(pt);
    
    // Add to 3D pose cloud (position only)
    PointType pose3d;
    pose3d.x = static_cast<float>(new_kf.position.x());
    pose3d.y = static_cast<float>(new_kf.position.y());
    pose3d.z = static_cast<float>(new_kf.position.z());
    pose3d.intensity = static_cast<float>(new_kf.id);
    cloud_key_poses_3d_->push_back(pose3d);
    
    // Add to 6D pose cloud
    PointTypePose pose6d;
    pose6d.x = static_cast<float>(new_kf.position.x());
    pose6d.y = static_cast<float>(new_kf.position.y());
    pose6d.z = static_cast<float>(new_kf.position.z());
    pose6d.intensity = static_cast<float>(new_kf.id);
    rotationToEuler(new_kf.rotation, pose6d.roll, pose6d.pitch, pose6d.yaw);
    pose6d.time = new_kf.timestamp;
    cloud_key_poses_6d_->push_back(pose6d);
    
    // Add to unoptimized pose cloud (same as original at creation time)
    PointTypePose unopt_pose6d;
    unopt_pose6d.x = static_cast<float>(new_kf.original_position.x());
    unopt_pose6d.y = static_cast<float>(new_kf.original_position.y());
    unopt_pose6d.z = static_cast<float>(new_kf.original_position.z());
    unopt_pose6d.intensity = static_cast<float>(new_kf.id);
    rotationToEuler(new_kf.original_rotation, unopt_pose6d.roll, unopt_pose6d.pitch, unopt_pose6d.yaw);
    unopt_pose6d.time = new_kf.timestamp;
    unoptimized_key_poses_6d_->push_back(unopt_pose6d);
    
    kdtree_updated_ = false;
    
    // Memory management: cleanup old keyframe clouds
    if (config_.enable_cloud_cleanup && 
        keyframes_.size() > static_cast<size_t>(config_.max_keyframes_with_cloud)) {
        cleanupOldClouds();
    }
    
    std::cout << "[KeyFrameManager] Added keyframe " << new_kf.id 
              << " at position (" << new_kf.position.x() << ", " 
              << new_kf.position.y() << ", " << new_kf.position.z() << ")" 
              << ", total: " << keyframes_.size() 
              << ", with_cloud: " << getKeyframesWithCloudCount() << std::endl;
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
        if (!kf.cloud || !kf.cloud_valid) {
            continue;
        }
        
        // Add cloud directly in body frame (NO transformation to world frame)
        // This is important for correct loop closure constraint calculation
        *submap += *(kf.cloud);
    }
    
    return submap;
}

// Build submap in the center keyframe's body coordinate system
// This is compatible with fast_lio_sam's loopFindNearKeyframes function
CloudType::Ptr KeyFrameManager::buildSubmapInBodyFrame(int center_idx, int num_neighbors) {
    CloudType::Ptr submap(new CloudType());
    
    if (center_idx < 0 || center_idx >= static_cast<int>(keyframes_.size())) {
        return submap;
    }
    
    const KeyFrame& center_kf = keyframes_[center_idx];
    
    // Check if center frame has valid cloud
    if (!center_kf.cloud_valid || !center_kf.cloud || center_kf.cloud->empty()) {
        std::cout << "[KeyFrameManager] Center keyframe " << center_idx 
                  << " has no valid cloud (cleaned up by memory management)" << std::endl;
        return submap;  // Return empty - caller should check
    }
    
    Eigen::Matrix4f center_transform = Eigen::Matrix4f::Identity();
    center_transform.block<3, 3>(0, 0) = center_kf.rotation.cast<float>();
    center_transform.block<3, 1>(0, 3) = center_kf.position.cast<float>();
    Eigen::Matrix4f center_transform_inv = center_transform.inverse();
    
    int start_idx = std::max(0, center_idx - num_neighbors);
    int end_idx = std::min(static_cast<int>(keyframes_.size()) - 1, center_idx + num_neighbors);
    
    int valid_cloud_count = 0;
    for (int i = start_idx; i <= end_idx; ++i) {
        const KeyFrame& kf = keyframes_[i];
        if (!kf.cloud_valid || !kf.cloud || kf.cloud->empty()) {
            continue;  // Skip keyframes with released clouds
        }
        
        if (i == center_idx) {
            // Center frame: keep in its own body frame (no transform)
            *submap += *(kf.cloud);
            valid_cloud_count++;
        } else {
            // Neighbor frames: transform to center frame's body coordinate
            // T_center_body_to_neighbor_body = T_world_to_center_body * T_neighbor_body_to_world
            Eigen::Matrix4f kf_transform = Eigen::Matrix4f::Identity();
            kf_transform.block<3, 3>(0, 0) = kf.rotation.cast<float>();
            kf_transform.block<3, 1>(0, 3) = kf.position.cast<float>();
            
            Eigen::Matrix4f relative_transform = center_transform_inv * kf_transform;
            
            CloudType::Ptr transformed(new CloudType());
            pcl::transformPointCloud(*(kf.cloud), *transformed, relative_transform);
            *submap += *transformed;
            valid_cloud_count++;
        }
    }
    
    if (valid_cloud_count < 3) {
        std::cout << "[KeyFrameManager] Submap around " << center_idx 
                  << " has only " << valid_cloud_count << " valid clouds (insufficient)" << std::endl;
        submap->clear();  // Not enough clouds for reliable ICP
    }
    
    return submap;
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

void KeyFrameManager::downsampleCloud(CloudType::Ptr& cloud) {
    if (!cloud || cloud->empty() || config_.cloud_downsample_res <= 0) {
        return;
    }
    
    CloudType::Ptr downsampled(new CloudType());
    pcl::VoxelGrid<PointType> filter;
    filter.setLeafSize(static_cast<float>(config_.cloud_downsample_res),
                       static_cast<float>(config_.cloud_downsample_res),
                       static_cast<float>(config_.cloud_downsample_res));
    filter.setInputCloud(cloud);
    filter.filter(*downsampled);
    cloud = downsampled;
}

size_t KeyFrameManager::getKeyframesWithCloudCount() const {
    size_t count = 0;
    for (const auto& kf : keyframes_) {
        if (kf.cloud_valid && kf.cloud && !kf.cloud->empty()) {
            ++count;
        }
    }
    return count;
}

void KeyFrameManager::cleanupOldClouds() {
    // Release point clouds from oldest keyframes, keeping only max_keyframes_with_cloud
    // Note: This is called from addKeyFrame which already holds the lock
    
    size_t current_with_cloud = getKeyframesWithCloudCount();
    if (current_with_cloud <= static_cast<size_t>(config_.max_keyframes_with_cloud)) {
        return;
    }
    
    size_t to_cleanup = current_with_cloud - config_.max_keyframes_with_cloud;
    size_t cleaned = 0;
    
    for (size_t i = 0; i < keyframes_.size() && cleaned < to_cleanup; ++i) {
        if (keyframes_[i].cloud_valid && keyframes_[i].cloud) {
            keyframes_[i].cloud.reset();  // Release the point cloud memory
            keyframes_[i].cloud_valid = false;
            ++cleaned;
        }
    }
    
    if (cleaned > 0) {
        std::cout << "[KeyFrameManager] Cleaned up " << cleaned 
                  << " old keyframe clouds for memory management" << std::endl;
    }
}
