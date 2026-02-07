#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <queue>
#include <set>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include "../map_builder/commons.h"
#include "keyframe_manager.h"
#include "../../include/sc-relo/Scancontext.h"

struct LoopClosureResult {
    bool valid = false;
    int current_idx = -1;
    int loop_idx = -1;
    Eigen::Affine3f icp_transform;      // ICP result: source_body -> target_body
    float fitness_score = 0.0f;
    float sc_score = 0.0f;
    
    // World poses for constraint calculation (following fast_lio_sam approach)
    Eigen::Matrix3f current_rotation;   // Current frame rotation in world
    Eigen::Vector3f current_position;   // Current frame position in world
    Eigen::Matrix3f loop_rotation;      // Loop frame rotation in world
    Eigen::Vector3f loop_position;      // Loop frame position in world
    
    LoopClosureResult() : icp_transform(Eigen::Affine3f::Identity()),
                          current_rotation(Eigen::Matrix3f::Identity()),
                          current_position(Eigen::Vector3f::Zero()),
                          loop_rotation(Eigen::Matrix3f::Identity()),
                          loop_position(Eigen::Vector3f::Zero()) {}
};

struct LoopDetectorConfig {
    bool enable = true;
    double frequency = 1.0;              // Hz
    double search_radius = 10.0;         // m
    double time_diff_threshold = 30.0;   // s
    double sc_dist_threshold = 0.3;      // ScanContext threshold
    double icp_fitness_threshold = 0.5;  // ICP convergence threshold
    int submap_size = 25;                // number of keyframes in submap
    int exclude_recent = 30;             // exclude recent keyframes
    double icp_max_correspondence_dist = 100.0;
    int icp_max_iterations = 100;
};

class LoopDetector {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    LoopDetector() = default;
    explicit LoopDetector(const LoopDetectorConfig& config);
    
    // Set the keyframe manager reference
    void setKeyFrameManager(std::shared_ptr<KeyFrameManager> kf_manager);
    
    // Add scan context for a keyframe
    void addKeyFrameSC(CloudType::Ptr cloud);
    
    // Detect loop closure for the latest keyframe
    LoopClosureResult detectLoopClosure();
    
    // Detect loop closure with distance-based candidate search
    bool detectLoopClosureDistance(int& current_idx, int& loop_idx);
    
    // Verify loop with ScanContext
    bool verifyScanContext(int current_idx, int loop_idx, 
                           float& sc_score, float& yaw_offset);
    
    // Verify loop with ICP
    bool verifyWithICP(CloudType::Ptr source_cloud, 
                       CloudType::Ptr target_cloud,
                       float yaw_offset,
                       Eigen::Affine3f& transform,
                       float& fitness_score);
    
    // Get pending loop closures
    std::vector<LoopClosureResult> getPendingLoopClosures();
    
    // Clear pending loop closures
    void clearPendingLoopClosures();
    
    // Check if there are pending loop closures
    bool hasPendingLoopClosures() const;
    
    // Get SC manager reference (for external access if needed)
    ScanContext::SCManager& getSCManager() { return sc_manager_; }
    
    // Thread-safe access
    std::mutex& getMutex() { return mutex_; }

private:
    CloudType::Ptr buildSubmap(int kf_idx);
    
    LoopDetectorConfig config_;
    std::shared_ptr<KeyFrameManager> kf_manager_;
    ScanContext::SCManager sc_manager_;
    
    std::vector<LoopClosureResult> pending_loops_;
    // History of detected loops to prevent duplicates
    std::set<std::pair<int, int>> detected_loop_pairs_;
    int last_loop_detected_idx_ = -1;
    const int LOOP_COOLDOWN_FRAMES = 20;  // Wait N frames before detecting new loops
    
    std::mutex mutex_;
    
    // ICP object
    pcl::IterativeClosestPoint<PointType, PointType> icp_;
};
