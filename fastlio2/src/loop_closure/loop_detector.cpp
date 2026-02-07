#include "loop_detector.h"
#include <iostream>

LoopDetector::LoopDetector(const LoopDetectorConfig& config)
    : config_(config) {
    // Configure ICP
    icp_.setMaxCorrespondenceDistance(config_.icp_max_correspondence_dist);
    icp_.setMaximumIterations(config_.icp_max_iterations);
    icp_.setTransformationEpsilon(1e-6);
    icp_.setEuclideanFitnessEpsilon(1e-6);
    
    // Configure ScanContext threshold
    sc_manager_.SC_DIST_THRES = config_.sc_dist_threshold;
}

void LoopDetector::setKeyFrameManager(std::shared_ptr<KeyFrameManager> kf_manager) {
    kf_manager_ = kf_manager;
}

void LoopDetector::addKeyFrameSC(CloudType::Ptr cloud) {
    if (!cloud || cloud->empty()) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    sc_manager_.makeAndSaveScancontextAndKeys(*cloud);
}

bool LoopDetector::detectLoopClosureDistance(int& current_idx, int& loop_idx) {
    if (!kf_manager_ || kf_manager_->size() < static_cast<size_t>(config_.exclude_recent + 1)) {
        return false;
    }
    
    current_idx = static_cast<int>(kf_manager_->size()) - 1;
    const KeyFrame& current_kf = kf_manager_->getKeyFrame(current_idx);
    
    // Search for nearby keyframes (excluding recent ones)
    std::vector<int> candidates = kf_manager_->searchNearbyKeyFrames(
        current_kf.position, 
        static_cast<float>(config_.search_radius),
        config_.exclude_recent
    );
    
    if (candidates.empty()) {
        return false;
    }
    
    // Find the closest candidate that also satisfies time difference
    double min_dist = std::numeric_limits<double>::max();
    int best_candidate = -1;
    
    for (int idx : candidates) {
        const KeyFrame& candidate_kf = kf_manager_->getKeyFrame(idx);
        
        // Check time difference
        double time_diff = std::abs(current_kf.timestamp - candidate_kf.timestamp);
        if (time_diff < config_.time_diff_threshold) {
            continue;
        }
        
        // Calculate distance
        double dist = (current_kf.position - candidate_kf.position).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_candidate = idx;
        }
    }
    
    if (best_candidate < 0) {
        return false;
    }
    
    loop_idx = best_candidate;
    std::cout << "[LoopDetector] Distance candidate found: current=" << current_idx 
              << " loop=" << loop_idx << " dist=" << min_dist << std::endl;
    
    return true;
}

bool LoopDetector::verifyScanContext(int current_idx, int loop_idx,
                                      float& sc_score, float& yaw_offset) {
    if (current_idx < 0 || loop_idx < 0) return false;
    if (static_cast<size_t>(current_idx) >= sc_manager_.polarcontexts_.size() ||
        static_cast<size_t>(loop_idx) >= sc_manager_.polarcontexts_.size()) {
        return false;
    }
    
    // Get ScanContext descriptors
    Eigen::MatrixXd& current_sc = sc_manager_.polarcontexts_[current_idx];
    Eigen::MatrixXd& loop_sc = sc_manager_.polarcontexts_[loop_idx];
    
    // Calculate SC distance
    auto result = sc_manager_.distanceBtnScanContext(current_sc, loop_sc);
    sc_score = static_cast<float>(result.first);
    yaw_offset = static_cast<float>(result.second) * 
                 static_cast<float>(sc_manager_.PC_UNIT_SECTORANGLE) * M_PI / 180.0f;
    
    std::cout << "[LoopDetector] SC verification: score=" << sc_score 
              << " threshold=" << config_.sc_dist_threshold 
              << " yaw_offset=" << yaw_offset * 180.0 / M_PI << " deg" << std::endl;
    
    return sc_score < config_.sc_dist_threshold;
}

CloudType::Ptr LoopDetector::buildSubmap(int kf_idx) {
    if (!kf_manager_) return nullptr;
    
    return kf_manager_->buildSubmapAroundKeyFrame(kf_idx, config_.submap_size);
}

bool LoopDetector::verifyWithICP(CloudType::Ptr source_cloud,
                                  CloudType::Ptr target_cloud,
                                  float yaw_offset,
                                  Eigen::Affine3f& transform,
                                  float& fitness_score) {
    if (!source_cloud || !target_cloud || 
        source_cloud->empty() || target_cloud->empty()) {
        return false;
    }
    
    // Apply yaw offset as initial guess
    Eigen::Affine3f initial_guess = Eigen::Affine3f::Identity();
    initial_guess.rotate(Eigen::AngleAxisf(yaw_offset, Eigen::Vector3f::UnitZ()));
    
    // Downsample for faster ICP
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(0.5f, 0.5f, 0.5f);
    
    CloudType::Ptr source_down(new CloudType());
    CloudType::Ptr target_down(new CloudType());
    
    voxel_filter.setInputCloud(source_cloud);
    voxel_filter.filter(*source_down);
    
    voxel_filter.setInputCloud(target_cloud);
    voxel_filter.filter(*target_down);
    
    // Run ICP
    CloudType::Ptr aligned(new CloudType());
    icp_.setInputSource(source_down);
    icp_.setInputTarget(target_down);
    icp_.align(*aligned, initial_guess.matrix());
    
    fitness_score = static_cast<float>(icp_.getFitnessScore());
    
    std::cout << "[LoopDetector] ICP verification: converged=" << icp_.hasConverged()
              << " fitness=" << fitness_score 
              << " threshold=" << config_.icp_fitness_threshold << std::endl;
    
    if (!icp_.hasConverged() || fitness_score > config_.icp_fitness_threshold) {
        return false;
    }
    
    transform.matrix() = icp_.getFinalTransformation();
    return true;
}

LoopClosureResult LoopDetector::detectLoopClosure() {
    LoopClosureResult result;
    
    if (!kf_manager_ || !config_.enable) {
        return result;
    }
    
    // Step 1: Distance-based candidate search
    int current_idx, loop_idx;
    if (!detectLoopClosureDistance(current_idx, loop_idx)) {
        return result;
    }
    
    // Step 1.5: Check for duplicate detection and cooldown
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Create normalized pair (smaller first)
        std::pair<int, int> loop_pair = std::make_pair(
            std::min(current_idx, loop_idx),
            std::max(current_idx, loop_idx)
        );
        
        // Check if this pair was already detected
        if (detected_loop_pairs_.count(loop_pair) > 0) {
            return result;  // Skip duplicate
        }
        
        // Check cooldown: don't detect new loops too soon after last detection
        if (last_loop_detected_idx_ >= 0 && 
            current_idx - last_loop_detected_idx_ < LOOP_COOLDOWN_FRAMES) {
            return result;  // Still in cooldown period
        }
    }
    
    // Step 2: ScanContext verification
    float sc_score, yaw_offset;
    if (!verifyScanContext(current_idx, loop_idx, sc_score, yaw_offset)) {
        std::cout << "[LoopDetector] SC verification failed" << std::endl;
        return result;
    }
    
    // Step 3: Build submaps in body coordinate frame (following fast_lio_sam approach)
    // Each submap is built in its center frame's body coordinate
    CloudType::Ptr source_submap = kf_manager_->buildSubmapInBodyFrame(current_idx, config_.submap_size);
    CloudType::Ptr target_submap = kf_manager_->buildSubmapInBodyFrame(loop_idx, config_.submap_size);
    
    if (!source_submap || !target_submap || source_submap->empty() || target_submap->empty()) {
        std::cout << "[LoopDetector] Failed to build submaps" << std::endl;
        return result;
    }
    
    // Step 4: ICP verification
    // ICP finds transform from source_body to target_body
    Eigen::Affine3f icp_transform;
    float icp_score;
    if (!verifyWithICP(source_submap, target_submap, yaw_offset, 
                       icp_transform, icp_score)) {
        std::cout << "[LoopDetector] ICP verification failed" << std::endl;
        return result;
    }
    
    // Get world poses for both keyframes
    const KeyFrame& current_kf = kf_manager_->getKeyFrame(current_idx);
    const KeyFrame& loop_kf = kf_manager_->getKeyFrame(loop_idx);
    
    // Success - fill result with all information needed for constraint calculation
    result.valid = true;
    result.current_idx = current_idx;
    result.loop_idx = loop_idx;
    result.icp_transform = icp_transform;
    result.fitness_score = icp_score;
    result.sc_score = sc_score;
    
    // Store world poses for constraint calculation in lio_node
    result.current_rotation = current_kf.rotation.cast<float>();
    result.current_position = current_kf.position.cast<float>();
    result.loop_rotation = loop_kf.rotation.cast<float>();
    result.loop_position = loop_kf.position.cast<float>();
    
    // Record this loop pair and update cooldown
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::pair<int, int> loop_pair = std::make_pair(
            std::min(current_idx, loop_idx),
            std::max(current_idx, loop_idx)
        );
        detected_loop_pairs_.insert(loop_pair);
        last_loop_detected_idx_ = current_idx;
    }
    
    std::cout << "[LoopDetector] Loop closure detected! current=" << current_idx 
              << " loop=" << loop_idx << " fitness=" << icp_score << std::endl;
    
    // Add to pending queue
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_loops_.push_back(result);
    }
    
    return result;
}

std::vector<LoopClosureResult> LoopDetector::getPendingLoopClosures() {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_loops_;
}

void LoopDetector::clearPendingLoopClosures() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_loops_.clear();
}

bool LoopDetector::hasPendingLoopClosures() const {
    return !pending_loops_.empty();
}
