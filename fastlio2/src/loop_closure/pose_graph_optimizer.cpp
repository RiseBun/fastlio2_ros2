#include "pose_graph_optimizer.h"
#include <iostream>
#include <fstream>
#include <iomanip>

PoseGraphOptimizer::PoseGraphOptimizer() : PoseGraphOptimizer(PoseGraphConfig()) {}

PoseGraphOptimizer::PoseGraphOptimizer(const PoseGraphConfig& config)
    : config_(config) {
    // Initialize ISAM2
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = config_.isam2_relinearize_threshold;
    params.relinearizeSkip = config_.isam2_relinearize_skip;
    isam2_ = new gtsam::ISAM2(params);
}

PoseGraphOptimizer::~PoseGraphOptimizer() {
    if (isam2_) {
        delete isam2_;
        isam2_ = nullptr;
    }
}

gtsam::Pose3 PoseGraphOptimizer::eigenToPose3(const M3D& rotation, const V3D& position) {
    gtsam::Rot3 rot(rotation);
    gtsam::Point3 trans(position.x(), position.y(), position.z());
    return gtsam::Pose3(rot, trans);
}

void PoseGraphOptimizer::pose3ToEigen(const gtsam::Pose3& pose, M3D& rotation, V3D& position) {
    rotation = pose.rotation().matrix();
    position = V3D(pose.translation().x(), 
                   pose.translation().y(), 
                   pose.translation().z());
}

void PoseGraphOptimizer::addPriorFactor(size_t id, const M3D& rotation, const V3D& position) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Create prior noise model (very small variance to fix first pose)
    gtsam::Vector6 prior_noise_vec;
    prior_noise_vec << config_.prior_noise_rotation, config_.prior_noise_rotation, 
                       config_.prior_noise_rotation, config_.prior_noise_translation, 
                       config_.prior_noise_translation, config_.prior_noise_translation;
    auto prior_noise = gtsam::noiseModel::Diagonal::Variances(prior_noise_vec);
    
    gtsam::Pose3 pose = eigenToPose3(rotation, position);
    
    // Add prior factor
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(id, pose, prior_noise));
    
    // Add initial estimate
    initial_estimate_.insert(id, pose);
    
    num_nodes_ = std::max(num_nodes_, id + 1);
    
    std::cout << "[PoseGraphOptimizer] Added prior factor for node " << id << std::endl;
}

void PoseGraphOptimizer::addOdomFactor(size_t from_id, size_t to_id,
                                        const M3D& from_rotation, const V3D& from_position,
                                        const M3D& to_rotation, const V3D& to_position) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Create odometry noise model
    gtsam::Vector6 odom_noise_vec;
    odom_noise_vec << config_.odom_noise_rotation, config_.odom_noise_rotation,
                      config_.odom_noise_rotation, config_.odom_noise_translation,
                      config_.odom_noise_translation, config_.odom_noise_translation;
    auto odom_noise = gtsam::noiseModel::Diagonal::Variances(odom_noise_vec);
    
    gtsam::Pose3 pose_from = eigenToPose3(from_rotation, from_position);
    gtsam::Pose3 pose_to = eigenToPose3(to_rotation, to_position);
    
    // Calculate relative pose: pose_from.between(pose_to) = pose_from^(-1) * pose_to
    gtsam::Pose3 relative_pose = pose_from.between(pose_to);
    
    // Add between factor
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(from_id, to_id, relative_pose, odom_noise));
    
    // Add initial estimate for new node
    if (!initial_estimate_.exists(to_id)) {
        initial_estimate_.insert(to_id, pose_to);
    }
    
    num_nodes_ = std::max(num_nodes_, to_id + 1);
}

void PoseGraphOptimizer::addLoopFactor(size_t from_id, size_t to_id,
                                        const Eigen::Affine3f& relative_transform,
                                        float fitness_score) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Create loop closure noise model (based on ICP fitness score)
    double noise_val = static_cast<double>(fitness_score);
    noise_val = std::max(noise_val, 1e-6);  // Prevent zero noise
    
    gtsam::Vector6 loop_noise_vec;
    loop_noise_vec << noise_val, noise_val, noise_val, noise_val, noise_val, noise_val;
    auto loop_noise = gtsam::noiseModel::Diagonal::Variances(loop_noise_vec);
    
    // Convert Affine3f to Pose3
    Eigen::Matrix3d rot = relative_transform.rotation().cast<double>();
    Eigen::Vector3d trans = relative_transform.translation().cast<double>();
    gtsam::Pose3 relative_pose{gtsam::Rot3(rot), gtsam::Point3(trans)};
    
    // Add loop closure factor
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(from_id, to_id, relative_pose, loop_noise));
    
    loop_closed_ = true;
    
    std::cout << "[PoseGraphOptimizer] Added loop factor: " << from_id << " <-> " << to_id 
              << " (fitness=" << fitness_score << ")" << std::endl;
}

void PoseGraphOptimizer::optimize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (graph_.empty() && initial_estimate_.empty()) {
        return;
    }
    
    try {
        // ISAM2 incremental update
        isam2_->update(graph_, initial_estimate_);
        isam2_->update();
        
        // Additional iterations if loop closure happened
        if (loop_closed_) {
            isam2_->update();
            isam2_->update();
            isam2_->update();
            isam2_->update();
        }
        
        // Get current estimate
        current_estimate_ = isam2_->calculateEstimate();
        
        // Clear graph and initial estimate (already incorporated into ISAM2)
        graph_.resize(0);
        initial_estimate_.clear();
        
        std::cout << "[PoseGraphOptimizer] Optimization completed, " 
                  << current_estimate_.size() << " nodes" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[PoseGraphOptimizer] Optimization error: " << e.what() << std::endl;
    }
}

OptimizedPose PoseGraphOptimizer::getOptimizedPose(size_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    OptimizedPose result;
    result.id = id;
    result.rotation = M3D::Identity();
    result.position = V3D::Zero();
    
    if (current_estimate_.exists(id)) {
        gtsam::Pose3 pose = current_estimate_.at<gtsam::Pose3>(id);
        pose3ToEigen(pose, result.rotation, result.position);
    }
    
    return result;
}

std::vector<OptimizedPose> PoseGraphOptimizer::getAllOptimizedPoses() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<OptimizedPose> poses;
    
    for (size_t i = 0; i < num_nodes_; ++i) {
        if (current_estimate_.exists(i)) {
            OptimizedPose pose;
            pose.id = i;
            gtsam::Pose3 gtsam_pose = current_estimate_.at<gtsam::Pose3>(i);
            pose3ToEigen(gtsam_pose, pose.rotation, pose.position);
            poses.push_back(pose);
        }
    }
    
    return poses;
}

bool PoseGraphOptimizer::getLatestCorrection(size_t latest_id,
                                              const M3D& current_rotation, 
                                              const V3D& current_position,
                                              M3D& corrected_rotation, 
                                              V3D& corrected_position) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!current_estimate_.exists(latest_id)) {
        return false;
    }
    
    gtsam::Pose3 optimized_pose = current_estimate_.at<gtsam::Pose3>(latest_id);
    pose3ToEigen(optimized_pose, corrected_rotation, corrected_position);
    
    return true;
}

void PoseGraphOptimizer::savePoseGraph(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[PoseGraphOptimizer] Failed to open file for saving: " << filepath << std::endl;
        return;
    }
    
    // Write vertices (poses) in g2o format
    // Format: VERTEX_SE3:QUAT id x y z qx qy qz qw
    for (size_t i = 0; i < num_nodes_; ++i) {
        if (current_estimate_.exists(i)) {
            gtsam::Pose3 pose = current_estimate_.at<gtsam::Pose3>(i);
            gtsam::Point3 t = pose.translation();
            gtsam::Quaternion q = pose.rotation().toQuaternion();
            
            file << "VERTEX_SE3:QUAT " << i << " "
                 << std::fixed << std::setprecision(9)
                 << t.x() << " " << t.y() << " " << t.z() << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
    }
    
    file.close();
    std::cout << "[PoseGraphOptimizer] Saved pose graph with " << num_nodes_ << " vertices to " << filepath << std::endl;
}
