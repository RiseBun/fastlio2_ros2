#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"
#include "loop_closure/keyframe_manager.h"
#include "loop_closure/loop_detector.h"
#include "loop_closure/pose_graph_optimizer.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "interface/srv/save_maps.hpp"

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
    
    // Keyframe config
    double keyframe_dist_threshold = 1.0;
    double keyframe_angle_threshold = 0.2;
    
    // Loop closure config (will be used in later modules)
    bool loop_closure_enable = false;
    double loop_closure_frequency = 1.0;
    double loop_search_radius = 10.0;
    double loop_time_diff_threshold = 30.0;
    double sc_dist_threshold = 0.3;
    double icp_fitness_threshold = 0.5;
    int submap_size = 25;
    
    // Map saving config
    bool save_map_on_exit = true;
    std::string map_save_path = "/home/li/maps";
    bool save_patches = false;
    
    // Extended saving config (compatible with fast_lio_sam)
    bool save_trajectory_txt = true;     // Save KITTI format trajectory
    bool save_scd_files = true;          // Save ScanContext descriptors
    bool save_pose_graph = true;         // Save pose graph file
    bool save_performance_log = true;    // Save performance CSV log
    double map_resolution = 0.1;         // Voxel filter resolution for saved map
};

// Performance logging data
struct PerformanceLog {
    double timestamp;
    double total_time_ms;
    int scan_points;
    double preprocess_time_ms;
    double ikdtree_time_ms;
    double ieskf_time_ms;
};

struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node"), m_loop_thread_running(false)
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Create save map service
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>(
            "lio/save_map",
            std::bind(&LIONode::saveMapCB, this, std::placeholders::_1, std::placeholders::_2));

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));
        
        // Start loop closure thread if enabled
        if (m_node_config.loop_closure_enable) {
            startLoopClosureThread();
        }
    }
    
    ~LIONode() {
        stopLoopClosureThread();
        
        // Auto save map on exit
        if (m_node_config.save_map_on_exit) {
            RCLCPP_INFO(this->get_logger(), "Auto-saving map on exit...");
            bool success = saveMapInternal(m_node_config.map_save_path, m_node_config.save_patches);
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Map saved to: %s", m_node_config.map_save_path.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to save map");
            }
        }
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
        
        // Keyframe parameters
        if (config["keyframe_dist_threshold"]) {
            m_node_config.keyframe_dist_threshold = config["keyframe_dist_threshold"].as<double>();
        }
        if (config["keyframe_angle_threshold"]) {
            m_node_config.keyframe_angle_threshold = config["keyframe_angle_threshold"].as<double>();
        }
        
        // Loop closure parameters (for later modules)
        if (config["loop_closure_enable"]) {
            m_node_config.loop_closure_enable = config["loop_closure_enable"].as<bool>();
        }
        if (config["loop_closure_frequency"]) {
            m_node_config.loop_closure_frequency = config["loop_closure_frequency"].as<double>();
        }
        if (config["loop_search_radius"]) {
            m_node_config.loop_search_radius = config["loop_search_radius"].as<double>();
        }
        if (config["loop_time_diff_threshold"]) {
            m_node_config.loop_time_diff_threshold = config["loop_time_diff_threshold"].as<double>();
        }
        if (config["sc_dist_threshold"]) {
            m_node_config.sc_dist_threshold = config["sc_dist_threshold"].as<double>();
        }
        if (config["icp_fitness_threshold"]) {
            m_node_config.icp_fitness_threshold = config["icp_fitness_threshold"].as<double>();
        }
        if (config["submap_size"]) {
            m_node_config.submap_size = config["submap_size"].as<int>();
        }
        
        // Map saving parameters
        if (config["save_map_on_exit"]) {
            m_node_config.save_map_on_exit = config["save_map_on_exit"].as<bool>();
        }
        if (config["map_save_path"]) {
            m_node_config.map_save_path = config["map_save_path"].as<std::string>();
        }
        if (config["save_patches"]) {
            m_node_config.save_patches = config["save_patches"].as<bool>();
        }
        
        // Extended saving parameters (compatible with fast_lio_sam)
        if (config["save_trajectory_txt"]) {
            m_node_config.save_trajectory_txt = config["save_trajectory_txt"].as<bool>();
        }
        if (config["save_scd_files"]) {
            m_node_config.save_scd_files = config["save_scd_files"].as<bool>();
        }
        if (config["save_pose_graph"]) {
            m_node_config.save_pose_graph = config["save_pose_graph"].as<bool>();
        }
        if (config["save_performance_log"]) {
            m_node_config.save_performance_log = config["save_performance_log"].as<bool>();
        }
        if (config["map_resolution"]) {
            m_node_config.map_resolution = config["map_resolution"].as<double>();
        }
        
        RCLCPP_INFO(this->get_logger(), "Map save config: on_exit=%s, path=%s",
                    m_node_config.save_map_on_exit ? "true" : "false",
                    m_node_config.map_save_path.c_str());
        
        // Initialize keyframe manager
        KeyFrameConfig kf_config;
        kf_config.dist_threshold = m_node_config.keyframe_dist_threshold;
        kf_config.angle_threshold = m_node_config.keyframe_angle_threshold;
        m_keyframe_manager = std::make_shared<KeyFrameManager>(kf_config);
        
        RCLCPP_INFO(this->get_logger(), "Keyframe config: dist_threshold=%.2f, angle_threshold=%.2f",
                    kf_config.dist_threshold, kf_config.angle_threshold);
        
        // Initialize loop detector
        LoopDetectorConfig ld_config;
        ld_config.enable = m_node_config.loop_closure_enable;
        ld_config.frequency = m_node_config.loop_closure_frequency;
        ld_config.search_radius = m_node_config.loop_search_radius;
        ld_config.time_diff_threshold = m_node_config.loop_time_diff_threshold;
        ld_config.sc_dist_threshold = m_node_config.sc_dist_threshold;
        ld_config.icp_fitness_threshold = m_node_config.icp_fitness_threshold;
        ld_config.submap_size = m_node_config.submap_size;
        m_loop_detector = std::make_shared<LoopDetector>(ld_config);
        m_loop_detector->setKeyFrameManager(m_keyframe_manager);
        
        // Initialize pose graph optimizer
        PoseGraphConfig pg_config;
        m_pose_graph = std::make_shared<PoseGraphOptimizer>(pg_config);
        
        RCLCPP_INFO(this->get_logger(), "Loop closure: enable=%s, frequency=%.1fHz, search_radius=%.1fm",
                    m_node_config.loop_closure_enable ? "true" : "false",
                    m_node_config.loop_closure_frequency,
                    m_node_config.loop_search_radius);
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
        
        if (m_node_config.print_time_cost)
        {
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }
        
        // Collect performance log
        if (m_node_config.save_performance_log) {
            PerformanceLog log;
            log.timestamp = m_package.cloud_end_time;
            log.total_time_ms = time_used;
            log.scan_points = static_cast<int>(m_package.cloud->size());
            log.preprocess_time_ms = 0.0;  // Can be filled if builder exposes timing
            log.ikdtree_time_ms = 0.0;
            log.ieskf_time_ms = 0.0;
            m_performance_logs.push_back(log);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
        
        // Try to add keyframe
        tryAddKeyFrame(body_cloud, m_package.cloud_end_time);
    }
    
    void tryAddKeyFrame(CloudType::Ptr cloud, double timestamp)
    {
        if (!m_keyframe_manager) return;
        
        const M3D& rotation = m_kf->x().r_wi;
        const V3D& position = m_kf->x().t_wi;
        
        if (m_keyframe_manager->shouldAddKeyFrame(rotation, position))
        {
            // Make a copy of the cloud for keyframe storage
            CloudType::Ptr kf_cloud(new CloudType());
            *kf_cloud = *cloud;
            
            size_t kf_id = m_keyframe_manager->size();
            
            // Add to pose graph
            if (m_pose_graph) {
                if (kf_id == 0) {
                    // First keyframe: add prior factor
                    m_pose_graph->addPriorFactor(kf_id, rotation, position);
                } else {
                    // Subsequent keyframes: add odometry factor
                    const KeyFrame& prev_kf = m_keyframe_manager->getLatestKeyFrame();
                    m_pose_graph->addOdomFactor(prev_kf.id, kf_id,
                                                 prev_kf.rotation, prev_kf.position,
                                                 rotation, position);
                }
                
                // Run optimization
                m_pose_graph->optimize();
            }
            
            // Add keyframe
            m_keyframe_manager->addKeyFrame(kf_id, timestamp, rotation, position, kf_cloud);
            
            // Add ScanContext descriptor
            if (m_loop_detector) {
                m_loop_detector->addKeyFrameSC(kf_cloud);
            }
            
            // Apply loop correction if needed
            applyLoopCorrection();
        }
    }
    
    void applyLoopCorrection()
    {
        if (!m_pose_graph || !m_pose_graph->hasLoopClosure()) {
            return;
        }
        
        // Get optimized poses and update keyframes (DO NOT modify ESKF state!)
        // Reference: fast_lio_sam only updates keyframe poses, not filter state
        auto optimized_poses = m_pose_graph->getAllOptimizedPoses();
        for (const auto& opt_pose : optimized_poses) {
            m_keyframe_manager->updateKeyFramePose(opt_pose.id, 
                                                    opt_pose.rotation, 
                                                    opt_pose.position);
        }
        
        // Rebuild path for visualization
        m_state_data.path.poses.clear();
        for (size_t i = 0; i < m_keyframe_manager->size(); ++i) {
            const KeyFrame& kf = m_keyframe_manager->getKeyFrame(i);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = m_node_config.world_frame;
            pose.pose.position.x = kf.position.x();
            pose.pose.position.y = kf.position.y();
            pose.pose.position.z = kf.position.z();
            Eigen::Quaterniond q(kf.rotation);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            m_state_data.path.poses.push_back(pose);
        }
        
        RCLCPP_INFO(this->get_logger(), "[LoopClosure] Updated %zu keyframe poses", 
                    optimized_poses.size());
        
        m_pose_graph->resetLoopClosureFlag();
    }
    
    void startLoopClosureThread()
    {
        m_loop_thread_running = true;
        m_loop_thread = std::thread(&LIONode::loopClosureThread, this);
        RCLCPP_INFO(this->get_logger(), "Loop closure thread started");
    }
    
    void stopLoopClosureThread()
    {
        m_loop_thread_running = false;
        if (m_loop_thread.joinable()) {
            m_loop_thread.join();
        }
    }
    
    void loopClosureThread()
    {
        double period_ms = 1000.0 / m_node_config.loop_closure_frequency;
        
        while (m_loop_thread_running && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(period_ms)));
            
            if (!m_loop_detector || !m_pose_graph) {
                continue;
            }
            
            // Detect loop closure
            LoopClosureResult result = m_loop_detector->detectLoopClosure();
            
            if (result.valid) {
                // Add loop factor to pose graph
                m_pose_graph->addLoopFactor(result.loop_idx, result.current_idx,
                                             result.transform, result.fitness_score);
                
                // Run optimization
                m_pose_graph->optimize();
                
                RCLCPP_INFO(this->get_logger(), 
                           "[LoopClosure] Loop detected: %d <-> %d (fitness=%.3f)",
                           result.loop_idx, result.current_idx, result.fitness_score);
            }
        }
    }
    
    // Internal map saving function - enhanced version compatible with fast_lio_sam
    bool saveMapInternal(const std::string& save_path, bool save_patches)
    {
        if (!m_keyframe_manager) {
            RCLCPP_WARN(this->get_logger(), "KeyFrame manager not initialized");
            return false;
        }
        
        const auto& keyframes = m_keyframe_manager->getAllKeyFrames();
        if (keyframes.empty()) {
            RCLCPP_WARN(this->get_logger(), "No keyframes available to save");
            return false;
        }
        
        try {
            // Create output directories
            std::filesystem::path save_dir(save_path);
            std::filesystem::create_directories(save_dir);
            std::filesystem::create_directories(save_dir / "pcd");
            std::filesystem::create_directories(save_dir / "scd");
            std::filesystem::create_directories(save_dir / "log");
            
            RCLCPP_INFO(this->get_logger(), "****************************************************");
            RCLCPP_INFO(this->get_logger(), "Saving map to: %s", save_path.c_str());
            
            // === 1. Save trajectory PCD files (compatible with fast_lio_sam) ===
            auto poses3d = m_keyframe_manager->getCloudKeyPoses3D();
            auto poses6d = m_keyframe_manager->getCloudKeyPoses6D();
            auto unopt_poses6d = m_keyframe_manager->getUnoptimizedKeyPoses6D();
            
            pcl::io::savePCDFileBinary((save_dir / "trajectory.pcd").string(), *poses3d);
            pcl::io::savePCDFileBinary((save_dir / "transformations.pcd").string(), *poses6d);
            RCLCPP_INFO(this->get_logger(), "Saved trajectory.pcd and transformations.pcd");
            
            // === 2. Save KITTI format trajectory TXT files ===
            if (m_node_config.save_trajectory_txt) {
                saveTrajectoryKITTI(save_dir, poses6d, unopt_poses6d);
            }
            
            // === 3. Build and save global maps ===
            CloudType::Ptr global_map(new CloudType());
            CloudType::Ptr global_map_ds(new CloudType());
            
            for (size_t i = 0; i < keyframes.size(); i++) {
                const auto& kf = keyframes[i];
                if (!kf.cloud || kf.cloud->empty()) continue;
                
                // Transform cloud from body to world frame
                CloudType::Ptr transformed_cloud(new CloudType());
                Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                transform.block<3, 3>(0, 0) = kf.rotation.cast<float>();
                transform.block<3, 1>(0, 3) = kf.position.cast<float>();
                pcl::transformPointCloud(*kf.cloud, *transformed_cloud, transform);
                
                *global_map += *transformed_cloud;
                
                // Save individual keyframe patches with padded naming
                if (save_patches) {
                    std::string idx_str = padZeros(static_cast<int>(i), 6);
                    std::string patch_path = (save_dir / "pcd" / (idx_str + ".pcd")).string();
                    pcl::io::savePCDFileBinary(patch_path, *transformed_cloud);
                }
                
                std::cout << "\r" << std::flush << "Processing keyframe " << i << " of " << keyframes.size() << " ...";
            }
            std::cout << std::endl;
            
            // Downsample global map
            pcl::VoxelGrid<PointType> voxel_filter;
            float res = static_cast<float>(m_node_config.map_resolution);
            voxel_filter.setInputCloud(global_map);
            voxel_filter.setLeafSize(res, res, res);
            voxel_filter.filter(*global_map_ds);
            
            // Save multiple map formats
            pcl::io::savePCDFileBinary((save_dir / "GlobalMap.pcd").string(), *global_map);
            pcl::io::savePCDFileBinary((save_dir / "filterGlobalMap.pcd").string(), *global_map_ds);
            pcl::io::savePCDFileBinary((save_dir / "SurfMap.pcd").string(), *global_map_ds);
            
            RCLCPP_INFO(this->get_logger(), "Saved GlobalMap.pcd (%zu pts), filterGlobalMap.pcd (%zu pts)",
                        global_map->size(), global_map_ds->size());
            
            // === 4. Save ScanContext descriptors ===
            if (m_node_config.save_scd_files && m_loop_detector) {
                saveSCDFiles(save_dir / "scd");
            }
            
            // === 5. Save pose graph file ===
            if (m_node_config.save_pose_graph && m_pose_graph) {
                m_pose_graph->savePoseGraph((save_dir / "pose_graph.g2o").string());
                RCLCPP_INFO(this->get_logger(), "Saved pose_graph.g2o");
            }
            
            // === 6. Save performance log ===
            if (m_node_config.save_performance_log && !m_performance_logs.empty()) {
                savePerformanceLog(save_dir / "log");
            }
            
            RCLCPP_INFO(this->get_logger(), "****************************************************");
            RCLCPP_INFO(this->get_logger(), "Map saving completed: %zu keyframes", keyframes.size());
            
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", e.what());
            return false;
        }
    }
    
    // Save trajectory in KITTI format (3x4 transformation matrix per line)
    void saveTrajectoryKITTI(const std::filesystem::path& save_dir, 
                              PoseCloud::Ptr poses6d, 
                              PoseCloud::Ptr unopt_poses6d)
    {
        std::ofstream file_opt((save_dir / "optimized_pose.txt").string());
        std::ofstream file_unopt((save_dir / "without_optimized_pose.txt").string());
        
        if (!file_opt.is_open() || !file_unopt.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to create trajectory TXT files");
            return;
        }
        
        file_opt << std::fixed << std::setprecision(9);
        file_unopt << std::fixed << std::setprecision(9);
        
        for (size_t i = 0; i < poses6d->size(); i++) {
            // Optimized pose
            const auto& p = poses6d->points[i];
            Eigen::AngleAxisd rollAngle(p.roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(p.pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(p.yaw, Eigen::Vector3d::UnitZ());
            Eigen::Matrix3d R = (yawAngle * pitchAngle * rollAngle).matrix();
            
            file_opt << R(0,0) << " " << R(0,1) << " " << R(0,2) << " " << p.x << " "
                     << R(1,0) << " " << R(1,1) << " " << R(1,2) << " " << p.y << " "
                     << R(2,0) << " " << R(2,1) << " " << R(2,2) << " " << p.z << "\n";
            
            // Unoptimized pose
            if (i < unopt_poses6d->size()) {
                const auto& up = unopt_poses6d->points[i];
                Eigen::AngleAxisd urollAngle(up.roll, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd upitchAngle(up.pitch, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd uyawAngle(up.yaw, Eigen::Vector3d::UnitZ());
                Eigen::Matrix3d uR = (uyawAngle * upitchAngle * urollAngle).matrix();
                
                file_unopt << uR(0,0) << " " << uR(0,1) << " " << uR(0,2) << " " << up.x << " "
                           << uR(1,0) << " " << uR(1,1) << " " << uR(1,2) << " " << up.y << " "
                           << uR(2,0) << " " << uR(2,1) << " " << uR(2,2) << " " << up.z << "\n";
            }
        }
        
        file_opt.close();
        file_unopt.close();
        RCLCPP_INFO(this->get_logger(), "Saved optimized_pose.txt and without_optimized_pose.txt");
    }
    
    // Save ScanContext descriptor files
    void saveSCDFiles(const std::filesystem::path& scd_dir)
    {
        if (!m_loop_detector) return;
        
        // Use the SCManager's built-in save function for each keyframe
        auto& sc_manager = m_loop_detector->getSCManager();
        const auto& scd_list = sc_manager.polarcontexts_;
        
        for (size_t i = 0; i < scd_list.size(); i++) {
            std::string idx_str = padZeros(static_cast<int>(i), 6);
            std::string scd_path = (scd_dir / (idx_str + ".scd")).string();
            
            std::ofstream file(scd_path);
            if (file.is_open()) {
                const auto& scd = scd_list[i];
                for (int r = 0; r < scd.rows(); r++) {
                    for (int c = 0; c < scd.cols(); c++) {
                        file << std::fixed << std::setprecision(3) << scd(r, c);
                        if (c < scd.cols() - 1) file << " ";
                    }
                    file << "\n";
                }
                file.close();
            }
        }
        RCLCPP_INFO(this->get_logger(), "Saved %zu SCD files", scd_list.size());
    }
    
    // Save performance log to CSV
    void savePerformanceLog(const std::filesystem::path& log_dir)
    {
        std::string log_path = (log_dir / "fast_lio_time_log.csv").string();
        std::ofstream file(log_path);
        
        if (!file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to create performance log file");
            return;
        }
        
        file << "timestamp,total_time_ms,scan_points,preprocess_time_ms,ikdtree_time_ms,ieskf_time_ms\n";
        
        for (const auto& log : m_performance_logs) {
            file << std::fixed << std::setprecision(8) << log.timestamp << ","
                 << std::setprecision(4) << log.total_time_ms << ","
                 << log.scan_points << ","
                 << log.preprocess_time_ms << ","
                 << log.ikdtree_time_ms << ","
                 << log.ieskf_time_ms << "\n";
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Saved performance log: %zu entries", m_performance_logs.size());
    }
    
    void saveMapCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request,
                   std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to: %s", request->file_path.c_str());
        
        bool success = saveMapInternal(request->file_path, request->save_patches);
        
        if (success) {
            response->success = true;
            response->message = "Map saved successfully";
        } else {
            response->success = false;
            response->message = "Failed to save map";
        }
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<KeyFrameManager> m_keyframe_manager;
    std::shared_ptr<LoopDetector> m_loop_detector;
    std::shared_ptr<PoseGraphOptimizer> m_pose_graph;
    
    // Performance logging
    std::vector<PerformanceLog> m_performance_logs;
    
    // Loop closure thread
    std::thread m_loop_thread;
    std::atomic<bool> m_loop_thread_running;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}