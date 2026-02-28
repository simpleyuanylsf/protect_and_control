#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h> // 【新增】需要知道位置
#include <std_msgs/Int32.h>
#include <eigen3/Eigen/Dense>          // 用来计算距离

class BatteryFailSafe {
public:
    BatteryFailSafe() {
        ros::NodeHandle nh_private("~");
        
        // === 参数配置 ===
        nh_private.param("low_battery_percent", low_batt_threshold_, 0.20); 
        nh_private.param("low_battery_voltage", low_volt_threshold_, 22.0); 
        nh_private.param("trigger_duration", trigger_duration_, 5.0);       

        // === 通信接口 ===
        // 1. 监听电池
        batt_sub_ = nh_.subscribe<sensor_msgs::BatteryState>
            ("mavros/battery", 10, &BatteryFailSafe::battery_cb, this);
        
        // 2. 监听连接状态
        state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &BatteryFailSafe::state_cb, this);

        // 3. 【新增】监听位置 (为了自己控制返航)
        pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &BatteryFailSafe::pos_cb, this);

        // 4. 【新增】发布控制设定点 (接管 Offboard 控制)
        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

        // 5. 发布指令给 DroneCommander
        commander_pub_ = nh_.advertise<std_msgs::Int32>("/commander/set_mode", 1);

        // 6. MAVROS 服务客户端
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // 初始化
        low_batt_start_time_ = ros::Time(0);
        is_low_battery_ = false;
        failsafe_triggered_ = false;
        is_armed_ = false;
        has_pos_ = false;

        ROS_INFO("Battery FailSafe Node Started (Advanced Version).");
    }

    void run() {
        // 【重要】必须使用循环，因为接管 Offboard 后需要持续发送 Setpoint
        ros::Rate rate(20.0); 

        while (ros::ok()) {
            // 如果触发了保护逻辑，执行核心状态机
            if (failsafe_triggered_ && is_armed_) {
                execute_failsafe_logic();
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber batt_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pos_sub_; // 【新增】
    ros::Publisher commander_pub_;
    ros::Publisher local_pos_pub_; // 【新增】
    ros::ServiceClient set_mode_client_;

    double low_batt_threshold_;
    double low_volt_threshold_;
    double trigger_duration_;
    
    ros::Time low_batt_start_time_;
    bool is_low_battery_;
    bool failsafe_triggered_;
    bool is_armed_;
    bool has_pos_;
    std::string current_px4_mode_;
    
    // 当前位置
    Eigen::Vector3d current_pos_;

    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        is_armed_ = msg->armed;
        current_px4_mode_ = msg->mode;
        
        if (!is_armed_ && failsafe_triggered_) {
             failsafe_triggered_ = false;
             is_low_battery_ = false;
             ROS_INFO("Drone Disarmed. Failsafe Reset.");
        }
    }

    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        has_pos_ = true;
    }

    void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg) {
        ROS_INFO_THROTTLE(2, "Battery Status: Voltage=%.2fV, Level=%.1f%%", 
                          msg->voltage, msg->percentage * 100.0);
                          
        if (!is_armed_) return;
        if (failsafe_triggered_) return; 

        bool current_low = false;
        // 这里的判断逻辑保持不变
        if (msg->percentage >= 0 && msg->percentage < low_batt_threshold_) {
            current_low = true;
            ROS_WARN_THROTTLE(5, "Low Battery: %.1f%%", msg->percentage * 100);
        } 
        else if (msg->voltage > 0 && msg->voltage < low_volt_threshold_) {
            current_low = true;
            ROS_WARN_THROTTLE(5, "Low Voltage: %.2fV", msg->voltage);
        }

        if (current_low) {
            if (!is_low_battery_) {
                low_batt_start_time_ = ros::Time::now();
                is_low_battery_ = true;
            } else {
                if (ros::Time::now() - low_batt_start_time_ > ros::Duration(trigger_duration_)) {
                    // 确认触发保护！
                    failsafe_triggered_ = true;
                    ROS_ERROR("CRITICAL BATTERY! Failsafe Triggered!");
                }
            }
        } else {
            is_low_battery_ = false;
        }
    }

    // --- 核心故障保护执行循环 ---
    void execute_failsafe_logic() {
        
        // === 策略 A: 指挥官在线 (优先级最高) ===
        // 如果 DroneCommander 节点活着，就让它干活
        if (commander_pub_.getNumSubscribers() > 0) {
            ROS_INFO_THROTTLE(2, "[Strategy A] Requesting Commander to LAND...");
            std_msgs::Int32 msg;
            msg.data = 3; // MODE_LAND
            commander_pub_.publish(msg);
            return; 
        }

        // === 策略 B: 指挥官挂了，但我还在 OFFBOARD 模式 ===
        // 这时候我要充当“临时指挥官”，控制飞机回家
        if (current_px4_mode_ == "OFFBOARD") {
            if (!has_pos_) {
                ROS_WARN("[Strategy B] No Position Data! Switching to AUTO.LAND immediately.");
                set_mode("AUTO.LAND");
                return;
            }

            // 1. 设定目标：(0, 0, 保持当前高度)
            geometry_msgs::PoseStamped target_pose;
            target_pose.header.stamp = ros::Time::now();
            target_pose.header.frame_id = "map";
            target_pose.pose.position.x = 0.0;
            target_pose.pose.position.y = 0.0;
            target_pose.pose.position.z = current_pos_.z(); // 保持高度飞回去
            
            // 保持当前的朝向，或者你可以让它转头朝向家
            target_pose.pose.orientation.w = 1.0; 

            // 2. 发布控制指令 (维持 Offboard 心跳)
            local_pos_pub_.publish(target_pose);

            // 3. 计算距离，判断是否到达
            double dist_xy = sqrt(pow(current_pos_.x(), 2) + pow(current_pos_.y(), 2));
            
            ROS_INFO_THROTTLE(1, "[Strategy B] Solo Return Home... Dist: %.2fm", dist_xy);

            // 4. 如果到达 (误差小于 0.2m)，切换自动降落
            if (dist_xy < 0.2) {
                ROS_INFO("[Strategy B] Home Reached. Switching to AUTO.LAND.");
                set_mode("AUTO.LAND");
            }
            return;
        }

        // === 策略 C: 既没指挥官，又不是 OFFBOARD (保底) ===
        // 比如你在 Manual 手动飞，或者定高模式
        ROS_WARN_THROTTLE(2, "[Strategy C] Manual Mode detected. Forcing AUTO.RTL.");
        if (current_px4_mode_ != "AUTO.RTL" && current_px4_mode_ != "AUTO.LAND") {
            set_mode("AUTO.RTL");
        }
    }

    // 辅助函数：切模式
    void set_mode(std::string mode_str) {
        mavros_msgs::SetMode mode_cmd;
        mode_cmd.request.custom_mode = mode_str;
        if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
            ROS_INFO("Switched to %s", mode_str.c_str());
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "battery_fail_safe_node");
    BatteryFailSafe failsafe;
    failsafe.run();
    return 0;
}