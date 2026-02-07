#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <condition_variable>
#include <sstream>
#include <fstream>
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
#include "../include/sc-relo/Scancontext.h"

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
    
    // Memory management config (for long-time running)
    int max_keyframes_with_cloud = 500;  // Max keyframes that keep point cloud in memory
    bool enable_cloud_cleanup = true;    // Enable automatic cloud cleanup
    double keyframe_cloud_res = 0.15;    // Keyframe cloud downsample resolution (0=disable)
    
    // Incremental save config
    bool enable_incremental_save = false;
    int incremental_save_interval = 500; // Save every N keyframes
    std::string incremental_save_path = "/home/li/maps/incremental";
    
    // System monitoring
    bool enable_system_monitor = true;
    double monitor_frequency = 0.2;      // Hz (every 5 seconds)
    
    // Relocalization config
    bool reloc_enable_on_startup = false;
    std::string reloc_prior_map_path = "";
    double reloc_sc_match_threshold = 0.25;
    double reloc_icp_refine_threshold = 0.5;
    int reloc_max_attempts = 10;
    double reloc_timeout_sec = 30.0;
    bool reloc_use_global_map_icp = true;
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

// System monitoring data
struct SystemMonitor {
    double frontend_time_ms = 0.0;
    double loop_time_ms = 0.0;
    size_t keyframe_count = 0;
    size_t keyframe_with_cloud_count = 0;
    size_t ikdtree_size = 0;
    size_t estimated_memory_mb = 0;
    int dropped_frames = 0;
    int loop_closure_count = 0;
    bool frontend_healthy = true;
    
    void updateMemoryEstimate(size_t kf_count, size_t kf_cloud_count, size_t tree_size) {
        keyframe_count = kf_count;
        keyframe_with_cloud_count = kf_cloud_count;
        ikdtree_size = tree_size;
        // Rough estimate: 3MB per keyframe cloud, 32 bytes per tree point
        estimated_memory_mb = (kf_cloud_count * 3) + (tree_size * 32 / 1024 / 1024);
    }
};

// Async keyframe task
struct KeyFrameTask {
    size_t id;
    double timestamp;
    M3D rotation;
    V3D position;
    CloudType::Ptr cloud;
    size_t prev_id;
    M3D prev_rotation;
    V3D prev_position;
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

// Relocalization state
struct RelocalizationState {
    bool enabled = false;
    bool completed = false;
    bool failed = false;
    int attempts = 0;
    double start_time = 0.0;
    
    // Prior map data
    std::vector<Eigen::Matrix4d> prior_poses;          // Keyframe poses from prior map
    CloudType::Ptr prior_global_map;                   // Global map for ICP (optional)
    ScanContext::SCManager prior_sc_manager;           // SC manager for prior map
    
    // Result
    Eigen::Matrix3d initial_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
    
    RelocalizationState() : prior_global_map(new CloudType()) {}
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node"), m_loop_thread_running(false), 
                m_keyframe_thread_running(false), m_last_incremental_save_idx(0)
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
        
        // Start keyframe processing thread (always enabled for async processing)
        startKeyframeThread();
        
        // Start loop closure thread if enabled
        if (m_node_config.loop_closure_enable) {
            startLoopClosureThread();
        }
        
        // Start system monitor if enabled
        if (m_node_config.enable_system_monitor) {
            double monitor_period = 1.0 / m_node_config.monitor_frequency;
            m_monitor_timer = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(monitor_period * 1000)),
                std::bind(&LIONode::monitorCB, this));
        }
        
        // Load prior map for relocalization if enabled
        if (m_node_config.reloc_enable_on_startup && !m_node_config.reloc_prior_map_path.empty()) {
            if (loadPriorMap()) {
                RCLCPP_INFO(this->get_logger(), "Prior map loaded, relocalization will start on first scan");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to load prior map, relocalization disabled");
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "LIO Node initialization completed");
    }
    
    ~LIONode() {
        stopKeyframeThread();
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
        
        // Memory management parameters
        if (config["max_keyframes_with_cloud"]) {
            m_node_config.max_keyframes_with_cloud = config["max_keyframes_with_cloud"].as<int>();
        }
        if (config["enable_cloud_cleanup"]) {
            m_node_config.enable_cloud_cleanup = config["enable_cloud_cleanup"].as<bool>();
        }
        if (config["keyframe_cloud_res"]) {
            m_node_config.keyframe_cloud_res = config["keyframe_cloud_res"].as<double>();
        }
        
        // Incremental save parameters
        if (config["enable_incremental_save"]) {
            m_node_config.enable_incremental_save = config["enable_incremental_save"].as<bool>();
        }
        if (config["incremental_save_interval"]) {
            m_node_config.incremental_save_interval = config["incremental_save_interval"].as<int>();
        }
        if (config["incremental_save_path"]) {
            m_node_config.incremental_save_path = config["incremental_save_path"].as<std::string>();
        }
        
        // System monitoring parameters
        if (config["enable_system_monitor"]) {
            m_node_config.enable_system_monitor = config["enable_system_monitor"].as<bool>();
        }
        if (config["monitor_frequency"]) {
            m_node_config.monitor_frequency = config["monitor_frequency"].as<double>();
        }
        
        // Relocalization parameters
        if (config["relocalization"]) {
            auto reloc_config = config["relocalization"];
            if (reloc_config["enable_on_startup"]) {
                m_node_config.reloc_enable_on_startup = reloc_config["enable_on_startup"].as<bool>();
            }
            if (reloc_config["prior_map_path"]) {
                m_node_config.reloc_prior_map_path = reloc_config["prior_map_path"].as<std::string>();
            }
            if (reloc_config["sc_match_threshold"]) {
                m_node_config.reloc_sc_match_threshold = reloc_config["sc_match_threshold"].as<double>();
            }
            if (reloc_config["icp_refine_threshold"]) {
                m_node_config.reloc_icp_refine_threshold = reloc_config["icp_refine_threshold"].as<double>();
            }
            if (reloc_config["max_attempts"]) {
                m_node_config.reloc_max_attempts = reloc_config["max_attempts"].as<int>();
            }
            if (reloc_config["timeout_sec"]) {
                m_node_config.reloc_timeout_sec = reloc_config["timeout_sec"].as<double>();
            }
            if (reloc_config["use_global_map_icp"]) {
                m_node_config.reloc_use_global_map_icp = reloc_config["use_global_map_icp"].as<bool>();
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Map save config: on_exit=%s, path=%s",
                    m_node_config.save_map_on_exit ? "true" : "false",
                    m_node_config.map_save_path.c_str());
        
        RCLCPP_INFO(this->get_logger(), "Memory management: max_kf_cloud=%d, cleanup=%s, cloud_res=%.2f",
                    m_node_config.max_keyframes_with_cloud,
                    m_node_config.enable_cloud_cleanup ? "true" : "false",
                    m_node_config.keyframe_cloud_res);
        
        // Initialize keyframe manager with memory management config
        KeyFrameConfig kf_config;
        kf_config.dist_threshold = m_node_config.keyframe_dist_threshold;
        kf_config.angle_threshold = m_node_config.keyframe_angle_threshold;
        kf_config.max_keyframes_with_cloud = m_node_config.max_keyframes_with_cloud;
        kf_config.enable_cloud_cleanup = m_node_config.enable_cloud_cleanup;
        kf_config.cloud_downsample_res = m_node_config.keyframe_cloud_res;
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
        
        // Log relocalization config
        if (m_node_config.reloc_enable_on_startup && !m_node_config.reloc_prior_map_path.empty()) {
            RCLCPP_INFO(this->get_logger(), "Relocalization: enabled, prior_map=%s", 
                        m_node_config.reloc_prior_map_path.c_str());
        }
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
        
        // === Relocalization logic ===
        // Try relocalization on first valid mapping frame if enabled
        if (m_reloc_state.enabled && !m_reloc_state.completed && !m_reloc_state.failed) {
            CloudType::Ptr body_cloud_reloc = m_builder->lidar_processor()->transformCloud(
                m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
            
            if (tryRelocalize(body_cloud_reloc)) {
                // Apply relocalization result to ESKF state
                initializeFromRelocalization();
                RCLCPP_INFO(this->get_logger(), "[Relocalization] Initial pose applied to ESKF");
            }
        }

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
        
        // Update frontend time for monitoring
        m_system_monitor.frontend_time_ms = time_used;
        
        // Try to add keyframe (async)
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
            
            // Create async task
            KeyFrameTask task;
            task.id = kf_id;
            task.timestamp = timestamp;
            task.rotation = rotation;
            task.position = position;
            task.cloud = kf_cloud;
            
            // Get previous keyframe info for odometry factor
            if (kf_id > 0) {
                const KeyFrame& prev_kf = m_keyframe_manager->getLatestKeyFrame();
                task.prev_id = prev_kf.id;
                task.prev_rotation = prev_kf.rotation;
                task.prev_position = prev_kf.position;
            } else {
                task.prev_id = 0;
                task.prev_rotation = M3D::Identity();
                task.prev_position = V3D::Zero();
            }
            
            // Add to async queue (non-blocking)
            {
                std::lock_guard<std::mutex> lock(m_keyframe_mutex);
                m_keyframe_queue.push(std::move(task));
            }
            m_keyframe_cv.notify_one();
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
        m_system_monitor.loop_closure_count++;
    }
    
    // === Keyframe processing thread ===
    void startKeyframeThread()
    {
        m_keyframe_thread_running = true;
        m_keyframe_thread = std::thread(&LIONode::keyframeProcessThread, this);
        RCLCPP_INFO(this->get_logger(), "Keyframe processing thread started");
    }
    
    void stopKeyframeThread()
    {
        m_keyframe_thread_running = false;
        m_keyframe_cv.notify_all();
        if (m_keyframe_thread.joinable()) {
            m_keyframe_thread.join();
        }
    }
    
    void keyframeProcessThread()
    {
        while (m_keyframe_thread_running && rclcpp::ok()) {
            KeyFrameTask task;
            
            {
                std::unique_lock<std::mutex> lock(m_keyframe_mutex);
                m_keyframe_cv.wait(lock, [this] {
                    return !m_keyframe_queue.empty() || !m_keyframe_thread_running;
                });
                
                if (!m_keyframe_thread_running) break;
                if (m_keyframe_queue.empty()) continue;
                
                task = std::move(m_keyframe_queue.front());
                m_keyframe_queue.pop();
            }
            
            // Process keyframe asynchronously
            processKeyFrameTask(task);
        }
    }
    
    void processKeyFrameTask(const KeyFrameTask& task)
    {
        // Add to pose graph
        if (task.id == 0) {
            m_pose_graph->addPriorFactor(task.id, task.rotation, task.position);
        } else {
            m_pose_graph->addOdomFactor(task.prev_id, task.id,
                                         task.prev_rotation, task.prev_position,
                                         task.rotation, task.position);
        }
        
        // Run incremental optimization (without loop closure)
        m_pose_graph->optimize();
        
        // Add keyframe to manager
        m_keyframe_manager->addKeyFrame(task.id, task.timestamp, 
                                         task.rotation, task.position, task.cloud);
        
        // Add ScanContext descriptor
        if (m_loop_detector) {
            m_loop_detector->addKeyFrameSC(task.cloud);
        }
        
        // Apply loop correction if needed
        applyLoopCorrection();
        
        // Check for incremental save
        checkIncrementalSave();
    }
    
    void checkIncrementalSave()
    {
        if (!m_node_config.enable_incremental_save) return;
        
        size_t current_size = m_keyframe_manager->size();
        if (current_size - m_last_incremental_save_idx >= 
            static_cast<size_t>(m_node_config.incremental_save_interval)) {
            
            // Run incremental save in detached thread to not block
            size_t start_idx = m_last_incremental_save_idx;
            m_last_incremental_save_idx = current_size;
            
            std::thread([this, start_idx, current_size]() {
                saveIncrementalData(start_idx, current_size);
            }).detach();
        }
    }
    
    void saveIncrementalData(size_t start_idx, size_t end_idx)
    {
        try {
            std::filesystem::path save_dir(m_node_config.incremental_save_path);
            std::filesystem::create_directories(save_dir);
            std::filesystem::create_directories(save_dir / "pcd");
            std::filesystem::create_directories(save_dir / "scd");
            
            const auto& keyframes = m_keyframe_manager->getAllKeyFrames();
            
            // Save point clouds
            for (size_t i = start_idx; i < end_idx && i < keyframes.size(); ++i) {
                const auto& kf = keyframes[i];
                if (kf.cloud_valid && kf.cloud && !kf.cloud->empty()) {
                    // Transform to world frame and save
                    CloudType::Ptr transformed_cloud(new CloudType());
                    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                    transform.block<3, 3>(0, 0) = kf.rotation.cast<float>();
                    transform.block<3, 1>(0, 3) = kf.position.cast<float>();
                    pcl::transformPointCloud(*kf.cloud, *transformed_cloud, transform);
                    
                    std::string idx_str = padZeros(static_cast<int>(i), 6);
                    std::string pcd_path = (save_dir / "pcd" / (idx_str + ".pcd")).string();
                    pcl::io::savePCDFileBinaryCompressed(pcd_path, *transformed_cloud);
                }
            }
            
            // Save SCD files (for relocalization compatibility)
            if (m_loop_detector) {
                auto& sc_manager = m_loop_detector->getSCManager();
                const auto& scd_list = sc_manager.polarcontexts_;
                
                for (size_t i = start_idx; i < end_idx && i < scd_list.size(); ++i) {
                    std::string idx_str = padZeros(static_cast<int>(i), 6);
                    std::string scd_path = (save_dir / "scd" / (idx_str + ".scd")).string();
                    
                    std::ofstream scd_file(scd_path);
                    if (scd_file.is_open()) {
                        const auto& scd = scd_list[i];
                        for (int r = 0; r < scd.rows(); r++) {
                            for (int c = 0; c < scd.cols(); c++) {
                                scd_file << std::fixed << std::setprecision(3) << scd(r, c);
                                if (c < scd.cols() - 1) scd_file << " ";
                            }
                            scd_file << "\n";
                        }
                        scd_file.close();
                    }
                }
            }
            
            // Save KITTI format trajectory (overwrite with all poses for consistency)
            std::ofstream pose_kitti((save_dir / "optimized_pose.txt").string());
            if (pose_kitti.is_open()) {
                for (size_t i = 0; i < keyframes.size(); ++i) {
                    const auto& kf = keyframes[i];
                    const auto& R = kf.rotation;
                    const auto& t = kf.position;
                    pose_kitti << std::fixed << std::setprecision(9)
                               << R(0,0) << " " << R(0,1) << " " << R(0,2) << " " << t.x() << " "
                               << R(1,0) << " " << R(1,1) << " " << R(1,2) << " " << t.y() << " "
                               << R(2,0) << " " << R(2,1) << " " << R(2,2) << " " << t.z() << "\n";
                }
                pose_kitti.close();
            }
            
            // Also save simple incremental log
            std::ofstream pose_log((save_dir / "poses_incremental.txt").string(), std::ios::app);
            if (pose_log.is_open()) {
                for (size_t i = start_idx; i < end_idx && i < keyframes.size(); ++i) {
                    const auto& kf = keyframes[i];
                    pose_log << i << " " 
                             << std::fixed << std::setprecision(9)
                             << kf.position.x() << " " << kf.position.y() << " " << kf.position.z() << "\n";
                }
                pose_log.close();
            }
            
            RCLCPP_INFO(this->get_logger(), "[IncrementalSave] Saved keyframes %zu-%zu (PCD+SCD+Poses)", 
                        start_idx, end_idx - 1);
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "[IncrementalSave] Error: %s", e.what());
        }
    }
    
    // === System monitoring ===
    void monitorCB()
    {
        // Update monitor statistics
        m_system_monitor.keyframe_count = m_keyframe_manager ? m_keyframe_manager->size() : 0;
        m_system_monitor.keyframe_with_cloud_count = m_keyframe_manager ? 
            m_keyframe_manager->getKeyframesWithCloudCount() : 0;
        
        // Get ikd-tree size from builder
        if (m_builder && m_builder->lidar_processor()) {
            // Note: ikd-tree size access would need to be exposed from lidar_processor
            // For now, estimate based on keyframes
            m_system_monitor.ikdtree_size = m_system_monitor.keyframe_count * 5000; // rough estimate
        }
        
        m_system_monitor.updateMemoryEstimate(
            m_system_monitor.keyframe_count,
            m_system_monitor.keyframe_with_cloud_count,
            m_system_monitor.ikdtree_size
        );
        
        // Check health
        m_system_monitor.frontend_healthy = (m_system_monitor.frontend_time_ms < 18.0);
        
        // Log status periodically
        RCLCPP_INFO(this->get_logger(), 
            "[Monitor] KF: %zu (cloud: %zu), Memory: ~%zu MB, Frontend: %.1f ms, Loops: %d",
            m_system_monitor.keyframe_count,
            m_system_monitor.keyframe_with_cloud_count,
            m_system_monitor.estimated_memory_mb,
            m_system_monitor.frontend_time_ms,
            m_system_monitor.loop_closure_count
        );
        
        // Warn if memory is high
        if (m_system_monitor.estimated_memory_mb > 6000) {
            RCLCPP_WARN(this->get_logger(), 
                "[Monitor] High memory usage detected: ~%zu MB", 
                m_system_monitor.estimated_memory_mb);
        }
    }
    
    // === Loop closure thread ===
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
                // === Correct loop constraint calculation ===
                // 
                // ICP was performed with submaps in their respective body frames:
                // - source_submap: current frame's body coordinate
                // - target_submap: loop frame's body coordinate  
                // - icp_transform: transforms points from current_body to loop_body
                //
                // Mathematical derivation:
                // Let T_w_c = current frame pose in world (world -> current_body)
                // Let T_w_l = loop frame pose in world (world -> loop_body)
                // 
                // For a point p:
                // - p_c = T_w_c^(-1) * p_w  (point in current_body)
                // - p_l = T_w_l^(-1) * p_w  (point in loop_body)
                // - ICP: p_l = T_icp * p_c
                //
                // Therefore: T_icp = T_w_l^(-1) * T_w_c
                //            T_icp = (T_w_c^(-1) * T_w_l)^(-1)
                //
                // For GTSAM BetweenFactor(from, to, measurement):
                // - measurement = T_from^(-1) * T_to
                // - from = current_idx, to = loop_idx
                // - measurement = T_w_c^(-1) * T_w_l = T_icp^(-1)
                //
                // CONCLUSION: The correct constraint is icp_transform.inverse()
                
                Eigen::Affine3f relative_constraint = result.icp_transform.inverse();
                
                // Add loop factor to pose graph with the correctly computed constraint
                m_pose_graph->addLoopFactor(result.current_idx, result.loop_idx,
                                             relative_constraint, result.fitness_score);
                
                // Run optimization
                m_pose_graph->optimize();
                
                RCLCPP_INFO(this->get_logger(), 
                           "[LoopClosure] Loop detected: %d <-> %d (fitness=%.3f)",
                           result.current_idx, result.loop_idx, result.fitness_score);
            }
        }
    }
    
    // === Relocalization functions ===
    bool loadPriorMap()
    {
        if (m_node_config.reloc_prior_map_path.empty()) {
            RCLCPP_WARN(this->get_logger(), "[Relocalization] Prior map path is empty");
            return false;
        }
        
        std::filesystem::path map_dir(m_node_config.reloc_prior_map_path);
        if (!std::filesystem::exists(map_dir)) {
            RCLCPP_ERROR(this->get_logger(), "[Relocalization] Prior map path does not exist: %s",
                         m_node_config.reloc_prior_map_path.c_str());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "[Relocalization] Loading prior map from: %s",
                    m_node_config.reloc_prior_map_path.c_str());
        
        // 1. Load SCD files
        std::filesystem::path scd_dir = map_dir / "scd";
        if (!std::filesystem::exists(scd_dir)) {
            RCLCPP_ERROR(this->get_logger(), "[Relocalization] SCD directory not found: %s",
                         scd_dir.string().c_str());
            return false;
        }
        
        // Count SCD files
        int scd_count = 0;
        for (const auto& entry : std::filesystem::directory_iterator(scd_dir)) {
            if (entry.path().extension() == ".scd") {
                scd_count++;
            }
        }
        
        if (scd_count == 0) {
            RCLCPP_ERROR(this->get_logger(), "[Relocalization] No SCD files found");
            return false;
        }
        
        // Load SCD files into prior SC manager
        m_reloc_state.prior_sc_manager.SC_DIST_THRES = m_node_config.reloc_sc_match_threshold;
        m_reloc_state.prior_sc_manager.loadPriorSCD(scd_dir.string(), 6, scd_count);
        RCLCPP_INFO(this->get_logger(), "[Relocalization] Loaded %d SCD descriptors", scd_count);
        
        // 2. Load poses from optimized_pose.txt
        std::filesystem::path pose_file = map_dir / "optimized_pose.txt";
        if (!std::filesystem::exists(pose_file)) {
            RCLCPP_ERROR(this->get_logger(), "[Relocalization] Pose file not found: %s",
                         pose_file.string().c_str());
            return false;
        }
        
        std::ifstream ifs(pose_file.string());
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "[Relocalization] Failed to open pose file");
            return false;
        }
        
        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty()) continue;
            std::istringstream iss(line);
            
            // KITTI format: R(0,0) R(0,1) R(0,2) tx R(1,0) R(1,1) R(1,2) ty R(2,0) R(2,1) R(2,2) tz
            double r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz;
            if (!(iss >> r00 >> r01 >> r02 >> tx >> r10 >> r11 >> r12 >> ty >> r20 >> r21 >> r22 >> tz)) {
                continue;
            }
            
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose(0, 0) = r00; pose(0, 1) = r01; pose(0, 2) = r02; pose(0, 3) = tx;
            pose(1, 0) = r10; pose(1, 1) = r11; pose(1, 2) = r12; pose(1, 3) = ty;
            pose(2, 0) = r20; pose(2, 1) = r21; pose(2, 2) = r22; pose(2, 3) = tz;
            
            m_reloc_state.prior_poses.push_back(pose);
        }
        ifs.close();
        
        RCLCPP_INFO(this->get_logger(), "[Relocalization] Loaded %zu poses", 
                    m_reloc_state.prior_poses.size());
        
        if (m_reloc_state.prior_poses.size() != static_cast<size_t>(scd_count)) {
            RCLCPP_WARN(this->get_logger(), "[Relocalization] Pose count (%zu) != SCD count (%d)",
                        m_reloc_state.prior_poses.size(), scd_count);
        }
        
        // 3. Optionally load global map for ICP refinement
        if (m_node_config.reloc_use_global_map_icp) {
            std::filesystem::path global_map_file = map_dir / "GlobalMap.pcd";
            if (std::filesystem::exists(global_map_file)) {
                if (pcl::io::loadPCDFile<PointType>(global_map_file.string(), 
                                                     *m_reloc_state.prior_global_map) == 0) {
                    RCLCPP_INFO(this->get_logger(), "[Relocalization] Loaded GlobalMap.pcd (%zu points)",
                                m_reloc_state.prior_global_map->size());
                    
                    // Downsample for faster ICP
                    pcl::VoxelGrid<PointType> voxel_filter;
                    voxel_filter.setLeafSize(0.5f, 0.5f, 0.5f);
                    CloudType::Ptr downsampled(new CloudType());
                    voxel_filter.setInputCloud(m_reloc_state.prior_global_map);
                    voxel_filter.filter(*downsampled);
                    m_reloc_state.prior_global_map = downsampled;
                    
                    RCLCPP_INFO(this->get_logger(), "[Relocalization] Downsampled to %zu points",
                                m_reloc_state.prior_global_map->size());
                } else {
                    RCLCPP_WARN(this->get_logger(), "[Relocalization] Failed to load GlobalMap.pcd");
                }
            } else {
                // Try filterGlobalMap.pcd
                global_map_file = map_dir / "filterGlobalMap.pcd";
                if (std::filesystem::exists(global_map_file)) {
                    pcl::io::loadPCDFile<PointType>(global_map_file.string(), 
                                                     *m_reloc_state.prior_global_map);
                    RCLCPP_INFO(this->get_logger(), "[Relocalization] Loaded filterGlobalMap.pcd (%zu points)",
                                m_reloc_state.prior_global_map->size());
                }
            }
        }
        
        // Configure ICP for relocalization
        m_reloc_icp.setMaxCorrespondenceDistance(100.0);
        m_reloc_icp.setMaximumIterations(100);
        m_reloc_icp.setTransformationEpsilon(1e-6);
        m_reloc_icp.setEuclideanFitnessEpsilon(1e-6);
        
        m_reloc_state.enabled = true;
        RCLCPP_INFO(this->get_logger(), "[Relocalization] Prior map loaded successfully");
        return true;
    }
    
    bool tryRelocalize(CloudType::Ptr current_scan)
    {
        if (!m_reloc_state.enabled || m_reloc_state.completed || m_reloc_state.failed) {
            return false;
        }
        
        // Check timeout
        double current_time = this->now().seconds();
        if (m_reloc_state.start_time == 0.0) {
            m_reloc_state.start_time = current_time;
        }
        
        if (current_time - m_reloc_state.start_time > m_node_config.reloc_timeout_sec) {
            RCLCPP_ERROR(this->get_logger(), "[Relocalization] Timeout after %.1f seconds",
                         m_node_config.reloc_timeout_sec);
            m_reloc_state.failed = true;
            return false;
        }
        
        // Check max attempts
        m_reloc_state.attempts++;
        if (m_reloc_state.attempts > m_node_config.reloc_max_attempts) {
            RCLCPP_ERROR(this->get_logger(), "[Relocalization] Max attempts (%d) exceeded",
                         m_node_config.reloc_max_attempts);
            m_reloc_state.failed = true;
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "[Relocalization] Attempt %d/%d",
                    m_reloc_state.attempts, m_node_config.reloc_max_attempts);
        
        // Step 1: ScanContext matching
        auto sc_result = m_reloc_state.prior_sc_manager.relocalize(*current_scan);
        int matched_kf_id = sc_result.first;
        float yaw_offset = sc_result.second;
        
        if (matched_kf_id < 0) {
            RCLCPP_WARN(this->get_logger(), "[Relocalization] No SC match found");
            return false;
        }
        
        if (static_cast<size_t>(matched_kf_id) >= m_reloc_state.prior_poses.size()) {
            RCLCPP_WARN(this->get_logger(), "[Relocalization] Matched KF ID out of range");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "[Relocalization] SC matched keyframe %d, yaw_offset=%.1f deg",
                    matched_kf_id, yaw_offset * 180.0 / M_PI);
        
        // Get matched keyframe pose
        const Eigen::Matrix4d& matched_pose = m_reloc_state.prior_poses[matched_kf_id];
        
        // Step 2: ICP refinement (if global map available)
        Eigen::Matrix4f refined_transform = Eigen::Matrix4f::Identity();
        float icp_fitness = 0.0f;
        
        if (m_reloc_state.prior_global_map && !m_reloc_state.prior_global_map->empty()) {
            // Create initial guess: matched_pose with yaw correction
            Eigen::Matrix4f initial_guess = matched_pose.cast<float>();
            Eigen::AngleAxisf yaw_correction(yaw_offset, Eigen::Vector3f::UnitZ());
            initial_guess.block<3, 3>(0, 0) = initial_guess.block<3, 3>(0, 0) * yaw_correction.matrix();
            
            // Downsample current scan
            pcl::VoxelGrid<PointType> voxel_filter;
            voxel_filter.setLeafSize(0.5f, 0.5f, 0.5f);
            CloudType::Ptr scan_down(new CloudType());
            voxel_filter.setInputCloud(current_scan);
            voxel_filter.filter(*scan_down);
            
            // Run ICP
            CloudType::Ptr aligned(new CloudType());
            m_reloc_icp.setInputSource(scan_down);
            m_reloc_icp.setInputTarget(m_reloc_state.prior_global_map);
            m_reloc_icp.align(*aligned, initial_guess);
            
            icp_fitness = static_cast<float>(m_reloc_icp.getFitnessScore());
            
            if (!m_reloc_icp.hasConverged() || icp_fitness > m_node_config.reloc_icp_refine_threshold) {
                RCLCPP_WARN(this->get_logger(), "[Relocalization] ICP failed: converged=%d, fitness=%.3f",
                            m_reloc_icp.hasConverged(), icp_fitness);
                return false;
            }
            
            refined_transform = m_reloc_icp.getFinalTransformation();
            RCLCPP_INFO(this->get_logger(), "[Relocalization] ICP refinement successful, fitness=%.4f",
                        icp_fitness);
        } else {
            // No global map, use SC result directly with yaw correction
            refined_transform = matched_pose.cast<float>();
            Eigen::AngleAxisf yaw_correction(yaw_offset, Eigen::Vector3f::UnitZ());
            refined_transform.block<3, 3>(0, 0) = refined_transform.block<3, 3>(0, 0) * yaw_correction.matrix();
            RCLCPP_INFO(this->get_logger(), "[Relocalization] Using SC result directly (no ICP)");
        }
        
        // Step 3: Set initial pose
        m_reloc_state.initial_rotation = refined_transform.block<3, 3>(0, 0).cast<double>();
        m_reloc_state.initial_position = refined_transform.block<3, 1>(0, 3).cast<double>();
        m_reloc_state.completed = true;
        
        RCLCPP_INFO(this->get_logger(), 
                    "[Relocalization] SUCCESS! Position: (%.2f, %.2f, %.2f), matched_kf=%d, fitness=%.4f",
                    m_reloc_state.initial_position.x(),
                    m_reloc_state.initial_position.y(),
                    m_reloc_state.initial_position.z(),
                    matched_kf_id, icp_fitness);
        
        return true;
    }
    
    void initializeFromRelocalization()
    {
        if (!m_reloc_state.completed) return;
        
        // Set ESKF state from relocalization result
        m_kf->x().r_wi = m_reloc_state.initial_rotation;
        m_kf->x().t_wi = m_reloc_state.initial_position;
        
        RCLCPP_INFO(this->get_logger(), "[Relocalization] ESKF state initialized from relocalization");
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
            
            // === 5. Save pose graph file (with edges) ===
            if (m_node_config.save_pose_graph && m_pose_graph) {
                m_pose_graph->savePoseGraphFull((save_dir / "pose_graph.g2o").string());
                RCLCPP_INFO(this->get_logger(), "Saved pose_graph.g2o (with edges)");
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
    
    // Keyframe processing thread
    std::thread m_keyframe_thread;
    std::atomic<bool> m_keyframe_thread_running;
    std::queue<KeyFrameTask> m_keyframe_queue;
    std::mutex m_keyframe_mutex;
    std::condition_variable m_keyframe_cv;
    
    // System monitoring
    SystemMonitor m_system_monitor;
    rclcpp::TimerBase::SharedPtr m_monitor_timer;
    
    // Incremental save tracking
    size_t m_last_incremental_save_idx;
    
    // Relocalization state
    RelocalizationState m_reloc_state;
    pcl::IterativeClosestPoint<PointType, PointType> m_reloc_icp;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}