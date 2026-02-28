#ifndef GEOFENCE_GUARD_H
#define GEOFENCE_GUARD_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Int32.h>
#include <eigen3/Eigen/Dense>

// 区域形状枚举
enum FenceShape {
    RECTANGLE = 0,  
    CIRCLE = 1      
};

// 状态精简：只有 正常 和 降落
enum ReturnState {
    IDLE = 0,           // 正常飞行
    LANDING = 1         // 正在降落
};

class GeoFenceGuard {
public:
    GeoFenceGuard();
    void run();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    ros::Subscriber odom_sub_;      
    ros::Subscriber pose_sub_;      
    ros::Subscriber state_sub_;     
    
    // 注意：删除了 local_pos_pub_，因为不需要发位置指令了
    ros::Publisher commander_pub_;  
    
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    
    bool check_inside_fence(const Eigen::Vector3d& pos);
    void execute_land();
    void set_mode(const std::string& mode_str);
    
    FenceShape fence_shape_;        
    double fence_center_x_;         
    double fence_center_y_;         
    double fence_half_width_;       
    double fence_half_height_;      
    double fence_radius_;           
    double max_altitude_;           
    
    Eigen::Vector3d current_pos_;   
    bool has_odom_;                 
    bool has_pose_;                 
    bool is_armed_;                 
    std::string current_mode_;      
    
    ReturnState return_state_;      
    bool fence_triggered_;          
    bool was_outside_fence_;        
    
    ros::Time return_start_time_;
};

#endif // GEOFENCE_GUARD_H