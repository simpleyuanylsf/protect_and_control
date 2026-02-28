#ifndef DRONE_COMMANDER_H
#define DRONE_COMMANDER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense> // 需要 Eigen 库进行向量计算

// 定义模式枚举
enum Mode {
    IDLE = 0,// 待机模式
    TAKEOFF = 1,// 起飞模式
    MISSION_SQUARE = 2,// 任务模式：正方形轨迹
    LAND = 3,//  降落模式
    EXPLORE = 4 // 【新增】探索模式
};

class DroneCommander {
public:
    DroneCommander(ros::NodeHandle& nh);
    void run(); // 主循环

private:
    // 回调函数
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mode_input_cb(const std_msgs::Int32::ConstPtr& msg);
    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg); // 【新增】

    // 动作函数
    // 核心功能函数
    void perform_explore(); // 【新增】探索模式逻辑
    void perform_takeoff();
    void perform_mission();
    void perform_land();
    void publish_status(std::string status_str);

    // 核心控制函数：P控制位置平滑器
    // target: 最终目标, current_setpoint: 当前发布的中间点
    void update_setpoint_p_control(const Eigen::Vector3d& target, double dt);

    ros::NodeHandle nh_;

    ros::Time last_req_;//ros时间的类
    
    // 订阅者 & 发布者
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber mode_input_sub_;
    ros::Subscriber cmd_vel_sub_; // 【新增】

    ros::Publisher local_pos_pub_;
    ros::Publisher setpoint_raw_pub_; // 【新增】发布给 mavros/setpoint_raw/local
    ros::Publisher status_pub_;
    
    // 服务客户端
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    // 状态变量
    mavros_msgs::State current_state_;
    Eigen::Vector3d current_pos_;      // 当前无人机真实位置
    Eigen::Vector3d start_pos_;        // 起飞时的原点记录
    Eigen::Vector3d target_pos_;       // 最终想要去的目标
    Eigen::Vector3d current_setpoint_; // 发送给飞控的中间设定点 (平滑后) // 在探索模式下，用来存储目标高度

    // 【新增/修改】姿态相关变量
    Eigen::Quaterniond current_att_; // 保存当前的实时朝向
    Eigen::Quaterniond target_att_;  // 保存我们要锁定的目标朝向
    bool has_pose_data_;             // 安全标志位
    
// 【新增】导航控制变量
    geometry_msgs::Twist latest_cmd_vel_;
    ros::Time last_vel_time_;

    int current_mode_;
    int mission_waypoint_index_;       // 正方形走到第几个点了
    bool is_offboard_active_;
    bool home_recorded_;

    // P控制参数
    double p_gain_;    // P系数
    double max_vel_;   // 最大速度限制 (m/s)
    double acceptance_radius_; // 到达判定半径 (m)
};

#endif