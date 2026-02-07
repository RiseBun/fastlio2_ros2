#include "pose_graph_optimizer.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <map>

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
    
    // Store initial pose for re-optimization
    initial_poses_.push_back({id, pose});
    
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
    
    // Store edge for serialization
    PoseGraphEdge edge;
    edge.type = EdgeType::ODOMETRY;
    edge.from_id = from_id;
    edge.to_id = to_id;
    edge.relative_rotation = relative_pose.rotation().matrix();
    edge.relative_translation = V3D(relative_pose.translation().x(),
                                     relative_pose.translation().y(),
                                     relative_pose.translation().z());
    edge.noise_rotation = config_.odom_noise_rotation;
    edge.noise_translation = config_.odom_noise_translation;
    edge.fitness_score = 0.0f;
    edges_.push_back(edge);
    
    // Store initial pose
    initial_poses_.push_back({to_id, pose_to});
    
    num_nodes_ = std::max(num_nodes_, to_id + 1);
}

void PoseGraphOptimizer::addLoopFactor(size_t from_id, size_t to_id,
                                        const Eigen::Affine3f& relative_transform,
                                        float fitness_score) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Validate fitness score - reject low quality constraints
    if (fitness_score > 1.0f) {
        std::cout << "[PoseGraphOptimizer] Warning: High fitness score " << fitness_score 
                  << " for loop " << from_id << " <-> " << to_id 
                  << ", using higher noise variance" << std::endl;
    }
    
    // Create loop closure noise model (based on ICP fitness score)
    // Use a more robust noise model: lower fitness = tighter constraint
    // Map fitness [0, 1.5] to noise variance [0.01, 0.5]
    double base_rot_noise = config_.loop_noise_rotation;
    double base_trans_noise = config_.loop_noise_translation;
    
    // Scale noise by fitness score - higher fitness means less reliable, so more noise
    double scale_factor = 1.0 + std::min(static_cast<double>(fitness_score), 2.0);
    double rot_noise = base_rot_noise * scale_factor;
    double trans_noise = base_trans_noise * scale_factor;
    
    // Ensure minimum noise to prevent numerical issues
    rot_noise = std::max(rot_noise, 1e-4);
    trans_noise = std::max(trans_noise, 1e-4);
    
    gtsam::Vector6 loop_noise_vec;
    loop_noise_vec << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise;
    auto loop_noise = gtsam::noiseModel::Diagonal::Variances(loop_noise_vec);
    
    // Convert Affine3f to Pose3
    Eigen::Matrix3d rot = relative_transform.rotation().cast<double>();
    Eigen::Vector3d trans = relative_transform.translation().cast<double>();
    gtsam::Pose3 relative_pose{gtsam::Rot3(rot), gtsam::Point3(trans)};
    
    // Add loop closure factor
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(from_id, to_id, relative_pose, loop_noise));
    
    // Store edge for serialization
    PoseGraphEdge edge;
    edge.type = EdgeType::LOOP_CLOSURE;
    edge.from_id = from_id;
    edge.to_id = to_id;
    edge.relative_rotation = rot;
    edge.relative_translation = trans;
    edge.noise_rotation = rot_noise;
    edge.noise_translation = trans_noise;
    edge.fitness_score = fitness_score;
    edges_.push_back(edge);
    
    loop_closed_ = true;
    
    std::cout << "[PoseGraphOptimizer] Added loop factor: " << from_id << " <-> " << to_id 
              << " (fitness=" << fitness_score << ", noise_scale=" << scale_factor << ")" << std::endl;
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

void PoseGraphOptimizer::savePoseGraphFull(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[PoseGraphOptimizer] Failed to open file for saving: " << filepath << std::endl;
        return;
    }
    
    file << std::fixed << std::setprecision(9);
    
    // Write header comment
    file << "# FASTLIO2 Pose Graph\n";
    file << "# Vertices: " << num_nodes_ << "\n";
    file << "# Edges: " << edges_.size() << "\n";
    
    // Write vertices (poses) in g2o format
    // Format: VERTEX_SE3:QUAT id x y z qx qy qz qw
    for (size_t i = 0; i < num_nodes_; ++i) {
        if (current_estimate_.exists(i)) {
            gtsam::Pose3 pose = current_estimate_.at<gtsam::Pose3>(i);
            gtsam::Point3 t = pose.translation();
            gtsam::Quaternion q = pose.rotation().toQuaternion();
            
            file << "VERTEX_SE3:QUAT " << i << " "
                 << t.x() << " " << t.y() << " " << t.z() << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
    }
    
    // Write edges in g2o format
    // Format: EDGE_SE3:QUAT from_id to_id x y z qx qy qz qw info_11 info_22 ... info_66
    // We use diagonal information matrix
    for (const auto& edge : edges_) {
        Eigen::Quaterniond q(edge.relative_rotation);
        
        // Compute information matrix (inverse of covariance)
        double info_rot = 1.0 / edge.noise_rotation;
        double info_trans = 1.0 / edge.noise_translation;
        
        // Add edge type tag as comment for easier parsing
        if (edge.type == EdgeType::LOOP_CLOSURE) {
            file << "# LOOP fitness=" << edge.fitness_score << "\n";
        }
        
        file << "EDGE_SE3:QUAT " << edge.from_id << " " << edge.to_id << " "
             << edge.relative_translation.x() << " " 
             << edge.relative_translation.y() << " " 
             << edge.relative_translation.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
             // Upper triangular of 6x6 information matrix (diagonal only for simplicity)
             << info_trans << " 0 0 0 0 0 "
             << info_trans << " 0 0 0 0 "
             << info_trans << " 0 0 0 "
             << info_rot << " 0 0 "
             << info_rot << " 0 "
             << info_rot << "\n";
    }
    
    file.close();
    std::cout << "[PoseGraphOptimizer] Saved complete pose graph: " 
              << num_nodes_ << " vertices, " << edges_.size() << " edges to " << filepath << std::endl;
}

bool PoseGraphOptimizer::loadPoseGraph(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[PoseGraphOptimizer] Failed to open file for loading: " << filepath << std::endl;
        return false;
    }
    
    // Clear existing data
    edges_.clear();
    initial_poses_.clear();
    num_nodes_ = 0;
    current_estimate_.clear();
    
    // Temporary storage
    std::map<size_t, gtsam::Pose3> vertices;
    bool next_is_loop = false;
    float loop_fitness = 0.0f;
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        // Check for loop closure comment
        if (line.find("# LOOP") != std::string::npos) {
            next_is_loop = true;
            size_t pos = line.find("fitness=");
            if (pos != std::string::npos) {
                loop_fitness = std::stof(line.substr(pos + 8));
            }
            continue;
        }
        
        // Skip other comments
        if (line[0] == '#') continue;
        
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        
        if (type == "VERTEX_SE3:QUAT") {
            size_t id;
            double x, y, z, qx, qy, qz, qw;
            iss >> id >> x >> y >> z >> qx >> qy >> qz >> qw;
            
            gtsam::Pose3 pose(gtsam::Rot3::Quaternion(qw, qx, qy, qz),
                              gtsam::Point3(x, y, z));
            vertices[id] = pose;
            current_estimate_.insert(id, pose);
            num_nodes_ = std::max(num_nodes_, id + 1);
        }
        else if (type == "EDGE_SE3:QUAT") {
            size_t from_id, to_id;
            double x, y, z, qx, qy, qz, qw;
            double info[21];  // Upper triangular 6x6 = 21 elements
            
            iss >> from_id >> to_id >> x >> y >> z >> qx >> qy >> qz >> qw;
            for (int i = 0; i < 21; ++i) {
                iss >> info[i];
            }
            
            // Extract diagonal elements for noise
            double info_trans = info[0];   // (0,0)
            double info_rot = info[15];    // (3,3)
            
            PoseGraphEdge edge;
            edge.type = next_is_loop ? EdgeType::LOOP_CLOSURE : EdgeType::ODOMETRY;
            edge.from_id = from_id;
            edge.to_id = to_id;
            edge.relative_translation = V3D(x, y, z);
            edge.relative_rotation = gtsam::Rot3::Quaternion(qw, qx, qy, qz).matrix();
            edge.noise_rotation = (info_rot > 0) ? 1.0 / info_rot : config_.odom_noise_rotation;
            edge.noise_translation = (info_trans > 0) ? 1.0 / info_trans : config_.odom_noise_translation;
            edge.fitness_score = next_is_loop ? loop_fitness : 0.0f;
            
            edges_.push_back(edge);
            
            // Store initial pose
            if (vertices.count(to_id)) {
                initial_poses_.push_back({to_id, vertices[to_id]});
            }
            
            next_is_loop = false;
            loop_fitness = 0.0f;
        }
    }
    
    file.close();
    
    std::cout << "[PoseGraphOptimizer] Loaded pose graph: " 
              << num_nodes_ << " vertices, " << edges_.size() << " edges from " << filepath << std::endl;
    
    return true;
}

bool PoseGraphOptimizer::reoptimize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (edges_.empty() || current_estimate_.empty()) {
        std::cerr << "[PoseGraphOptimizer] No data to reoptimize" << std::endl;
        return false;
    }
    
    std::cout << "[PoseGraphOptimizer] Starting batch re-optimization with LevenbergMarquardt..." << std::endl;
    
    try {
        // Build factor graph from stored edges
        gtsam::NonlinearFactorGraph batch_graph;
        gtsam::Values batch_initial;
        
        // Add prior factor for first node
        gtsam::Vector6 prior_noise_vec;
        prior_noise_vec << config_.prior_noise_rotation, config_.prior_noise_rotation, 
                           config_.prior_noise_rotation, config_.prior_noise_translation, 
                           config_.prior_noise_translation, config_.prior_noise_translation;
        auto prior_noise = gtsam::noiseModel::Diagonal::Variances(prior_noise_vec);
        
        if (current_estimate_.exists(0)) {
            gtsam::Pose3 first_pose = current_estimate_.at<gtsam::Pose3>(0);
            batch_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, first_pose, prior_noise));
        }
        
        // Add all edges as between factors
        for (const auto& edge : edges_) {
            gtsam::Vector6 noise_vec;
            noise_vec << edge.noise_rotation, edge.noise_rotation, edge.noise_rotation,
                         edge.noise_translation, edge.noise_translation, edge.noise_translation;
            auto noise_model = gtsam::noiseModel::Diagonal::Variances(noise_vec);
            
            gtsam::Pose3 relative_pose(gtsam::Rot3(edge.relative_rotation),
                                        gtsam::Point3(edge.relative_translation));
            
            batch_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                edge.from_id, edge.to_id, relative_pose, noise_model));
        }
        
        // Use current estimate as initial values
        for (size_t i = 0; i < num_nodes_; ++i) {
            if (current_estimate_.exists(i)) {
                batch_initial.insert(i, current_estimate_.at<gtsam::Pose3>(i));
            }
        }
        
        // Run LevenbergMarquardt optimization
        gtsam::LevenbergMarquardtParams params;
        params.maxIterations = 100;
        params.relativeErrorTol = 1e-8;
        params.absoluteErrorTol = 1e-8;
        
        gtsam::LevenbergMarquardtOptimizer optimizer(batch_graph, batch_initial, params);
        gtsam::Values result = optimizer.optimize();
        
        // Calculate error reduction
        double initial_error = batch_graph.error(batch_initial);
        double final_error = batch_graph.error(result);
        
        std::cout << "[PoseGraphOptimizer] LM optimization completed:" << std::endl;
        std::cout << "  - Initial error: " << initial_error << std::endl;
        std::cout << "  - Final error: " << final_error << std::endl;
        std::cout << "  - Error reduction: " << (1.0 - final_error/initial_error) * 100.0 << "%" << std::endl;
        std::cout << "  - Iterations: " << optimizer.iterations() << std::endl;
        
        // Update current estimate
        current_estimate_ = result;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "[PoseGraphOptimizer] Re-optimization error: " << e.what() << std::endl;
        return false;
    }
}
