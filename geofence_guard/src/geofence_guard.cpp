#include "geofence_guard/geofence_guard.h"
#include <cmath>

GeoFenceGuard::GeoFenceGuard() : nh_private_("~"), has_odom_(false), has_pose_(false), 
    is_armed_(false), return_state_(IDLE), fence_triggered_(false), was_outside_fence_(false) {
    
    // ========== 加载参数 ==========
    int shape_param;
    nh_private_.param("fence_shape", shape_param, 0);
    fence_shape_ = static_cast<FenceShape>(shape_param);
    
    nh_private_.param("fence_center_x", fence_center_x_, 0.0);
    nh_private_.param("fence_center_y", fence_center_y_, 0.0);
    
    nh_private_.param("fence_half_width", fence_half_width_, 3.0);   
    nh_private_.param("fence_half_height", fence_half_height_, 3.0); 
    nh_private_.param("fence_radius", fence_radius_, 3.0);  
    nh_private_.param("max_altitude", max_altitude_, 2.0); 
    
    // ========== 初始化订阅 & 服务 ==========
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "mavros/local_position/odom", 10, &GeoFenceGuard::odom_cb, this);
    
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, &GeoFenceGuard::pose_cb, this);
    
    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        "mavros/state", 10, &GeoFenceGuard::state_cb, this);
    
    commander_pub_ = nh_.advertise<std_msgs::Int32>("/commander/set_mode", 1);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    
    ROS_INFO("=== GeoFence Guard Initialized ===");
    ROS_INFO("ACTION: IMMEDIATE AUTO.LAND UPON BREACH");
    ROS_INFO("==================================");
}

void GeoFenceGuard::odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    has_odom_ = true;
}

void GeoFenceGuard::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (has_odom_) return; 
    current_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    has_pose_ = true;
}

void GeoFenceGuard::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    is_armed_ = msg->armed;
    current_mode_ = msg->mode;
}

bool GeoFenceGuard::check_inside_fence(const Eigen::Vector3d& pos) {
    bool inside_xy = false;
    if (fence_shape_ == RECTANGLE) {
        double dx = std::abs(pos.x() - fence_center_x_);
        double dy = std::abs(pos.y() - fence_center_y_);
        inside_xy = (dx <= fence_half_width_) && (dy <= fence_half_height_);
    } else {
        double dx = pos.x() - fence_center_x_;
        double dy = pos.y() - fence_center_y_;
        double dist = std::sqrt(dx*dx + dy*dy);
        inside_xy = (dist <= fence_radius_);
    }
    return inside_xy && (pos.z() <= max_altitude_);
}

void GeoFenceGuard::set_mode(const std::string& mode_str) {
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode_str;
    if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("Switched to %s", mode_str.c_str());
    }
}

void GeoFenceGuard::execute_land() {
    if (current_mode_ != "AUTO.LAND") {
        ROS_WARN_THROTTLE(1.0, "[EMERGENCY] Forcing PX4 to AUTO.LAND...");
        set_mode("AUTO.LAND");
    }
    ROS_INFO_THROTTLE(3.0, "[LANDING] Auto landing in progress... Alt: %.2f m", current_pos_.z());
}

void GeoFenceGuard::run() {
    ros::Rate rate(20.0);  // 20Hz
    
    while (ros::ok()) {
        ros::spinOnce();
        
        if (!has_odom_ && !has_pose_) {
            rate.sleep();
            continue;
        }
        
        bool inside_fence = check_inside_fence(current_pos_);
        
        // 检测出界触发
        if (!inside_fence && !was_outside_fence_ && is_armed_) {
            ROS_ERROR("GEOFENCE BREACHED! FORCING AUTO.LAND!");
            fence_triggered_ = true;
            return_state_ = LANDING; 
        }
        
        was_outside_fence_ = !inside_fence;
        
        // 执行降落逻辑
        if (fence_triggered_ && is_armed_ && return_state_ == LANDING) {
            execute_land();
        }
        
        // 降落完成上锁后重置
        if (!is_armed_ && fence_triggered_) {
            ROS_INFO("Flight ended. Resetting guard...");
            fence_triggered_ = false;
            return_state_ = IDLE;
            was_outside_fence_ = false;
        }
        
        rate.sleep();
    }
}