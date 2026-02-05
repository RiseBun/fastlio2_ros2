#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <Eigen/Eigen>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

#include "../map_builder/commons.h"

struct PoseGraphConfig {
    double prior_noise_rotation = 1e-12;
    double prior_noise_translation = 1e-12;
    double odom_noise_rotation = 1e-6;
    double odom_noise_translation = 1e-4;
    double isam2_relinearize_threshold = 0.01;
    int isam2_relinearize_skip = 1;
};

struct OptimizedPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t id;
    M3D rotation;
    V3D position;
};

class PoseGraphOptimizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PoseGraphOptimizer();
    explicit PoseGraphOptimizer(const PoseGraphConfig& config);
    ~PoseGraphOptimizer();
    
    // Add first keyframe with prior factor
    void addPriorFactor(size_t id, const M3D& rotation, const V3D& position);
    
    // Add odometry factor between consecutive keyframes
    void addOdomFactor(size_t from_id, size_t to_id,
                       const M3D& from_rotation, const V3D& from_position,
                       const M3D& to_rotation, const V3D& to_position);
    
    // Add loop closure factor
    void addLoopFactor(size_t from_id, size_t to_id,
                       const Eigen::Affine3f& relative_transform,
                       float fitness_score);
    
    // Run optimization
    void optimize();
    
    // Get optimized pose for a specific keyframe
    OptimizedPose getOptimizedPose(size_t id);
    
    // Get all optimized poses
    std::vector<OptimizedPose> getAllOptimizedPoses();
    
    // Check if loop closure happened
    bool hasLoopClosure() const { return loop_closed_; }
    
    // Reset loop closure flag
    void resetLoopClosureFlag() { loop_closed_ = false; }
    
    // Get correction for latest pose (for state feedback)
    bool getLatestCorrection(size_t latest_id, 
                             const M3D& current_rotation, const V3D& current_position,
                             M3D& corrected_rotation, V3D& corrected_position);
    
    // Get number of nodes
    size_t numNodes() const { return num_nodes_; }
    
    // Save pose graph to file (g2o format)
    void savePoseGraph(const std::string& filepath);
    
    // Thread-safe access
    std::mutex& getMutex() { return mutex_; }

private:
    gtsam::Pose3 eigenToPose3(const M3D& rotation, const V3D& position);
    void pose3ToEigen(const gtsam::Pose3& pose, M3D& rotation, V3D& position);
    
    PoseGraphConfig config_;
    
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    gtsam::ISAM2* isam2_;
    gtsam::Values current_estimate_;
    
    size_t num_nodes_ = 0;
    bool loop_closed_ = false;
    
    std::mutex mutex_;
};
