#include "simple_planner/drone_commander.h"

DroneCommander::DroneCommander(ros::NodeHandle& nh) : nh_(nh) {
    // 参数初始化
    current_mode_ = IDLE;
    mission_waypoint_index_ = 0;
    is_offboard_active_ = false;
    home_recorded_ = false;
    
// 【新增】初始化标志位和姿态
    has_pose_data_ = false;
    current_att_ = Eigen::Quaterniond::Identity();
    target_att_  = Eigen::Quaterniond::Identity();

    // 调参区域：稳一点、慢一点的关键
    p_gain_ = 1.0;       // P参数，越大反应越快
    max_vel_ = 0.3;      // 最大速度 0.3 m/s (非常慢，适合室内测试)
    acceptance_radius_ = 0.1; // 误差小于 10cm 认为到达

    // 初始化订阅和发布
    state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &DroneCommander::state_cb, this);
    local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &DroneCommander::pose_cb, this);
    mode_input_sub_ = nh_.subscribe<std_msgs::Int32>
            ("/commander/set_mode", 10, &DroneCommander::mode_input_cb, this);
            // 【新增】订阅 move_base 输出的速度
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>
            ("/cmd_vel", 10, &DroneCommander::cmd_vel_cb, this);

    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    status_pub_ = nh_.advertise<std_msgs::String>
            ("/commander/current_status", 10);
    setpoint_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void DroneCommander::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void DroneCommander::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    
    // 【新增】读取当前姿态
    current_att_ = Eigen::Quaterniond(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );

    // 【新增】标记已收到数据，并做第一次初始化
    if (!has_pose_data_) {
        has_pose_data_ = true;
        target_att_ = current_att_; // 初始对齐
    }

    if (current_mode_ == IDLE) {
        start_pos_ = current_pos_;
        current_setpoint_ = current_pos_;
        // 【关键】IDLE模式下，目标姿态实时跟随当前姿态（软跟随）
        // 这样当你切 Takeoff 的瞬间，目标姿态就是当前的姿态，不会跳变
        target_att_ = current_att_; 
    }
}

// 【新增】cmd_vel 回调函数
void DroneCommander::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    latest_cmd_vel_ = *msg;
    last_vel_time_ = ros::Time::now(); // 更新最后接收时间，用于安全检查
}

void DroneCommander::mode_input_cb(const std_msgs::Int32::ConstPtr& msg) {
    // 简单的状态机切换
    if (msg->data != current_mode_) {
        // 【安全】没数据不许切模式
        if (!has_pose_data_) {
            ROS_WARN("Waiting for Pose Data...");
            return;
        }
        ROS_INFO("Switching Mode from %d to %d", current_mode_, msg->data);
        current_mode_ = msg->data;
        
        // 模式初始化逻辑
        if (current_mode_ == TAKEOFF) {
            home_recorded_ = true; // 锁定起飞点
            // 目标：原点上方1米
            target_pos_ = start_pos_; 
            target_pos_.z() += 1.0; 
            // 【核心】起飞瞬间，target_att_ 已经被定格在了 IDLE 最后一刻的值
            // 这里不需要额外写代码，因为 pose_cb 里的跟随逻辑在切出 IDLE 后就停止了
            // target_att_ 自然就锁住了。
        } 
        else if (current_mode_ == MISSION_SQUARE) {
            mission_waypoint_index_ = 0; // 重置任务进度
        }
        else if (current_mode_ == LAND) {
            // 目标：回到起飞原点 (保留高度)
            target_pos_ = start_pos_;
            target_pos_.z() = current_pos_.z(); 
        }
        else if (current_mode_ == EXPLORE) {
            // 【新增】切入探索模式时，锁定当前高度作为目标高度
            ROS_INFO("Entering Explore Mode. Holding Altitude: %.2f", current_pos_.z());
            target_pos_ = current_pos_; // 保持当前位置，重点是Z
        }
    }
}

// 融合了参考代码思路的 P 控制函数
void DroneCommander::update_setpoint_p_control(const Eigen::Vector3d& target, double dt) {
    // 1. 位置控制逻辑 (保持你原来的逻辑，这部分没问题)
    Eigen::Vector3d error = target - current_setpoint_;
    Eigen::Vector3d velocity_cmd = error * p_gain_;
    if (velocity_cmd.norm() > max_vel_) {
        velocity_cmd = velocity_cmd.normalized() * max_vel_;
    }
    current_setpoint_ += velocity_cmd * dt;

    // 2. 构造消息
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = current_setpoint_.x();
    pose_msg.pose.position.y = current_setpoint_.y();
    pose_msg.pose.position.z = current_setpoint_.z();

    // 【核心修正】始终发布锁定的 target_att_
    // 在 IDLE 时，它是实时的；在 TAKEOFF 后，它是锁死的。
    pose_msg.pose.orientation.w = target_att_.w();
    pose_msg.pose.orientation.x = target_att_.x();
    pose_msg.pose.orientation.y = target_att_.y();
    pose_msg.pose.orientation.z = target_att_.z();

    local_pos_pub_.publish(pose_msg);
}

// 【新增】探索模式的核心逻辑
void DroneCommander::perform_explore() {
    publish_status("MODE: EXPLORATION");

    // 安全检查：如果超过 0.5秒 没收到 cmd_vel，就悬停
    if (ros::Time::now() - last_vel_time_ > ros::Duration(0.5)) {
        // 超时悬停：发送零速度，保持高度
        latest_cmd_vel_.linear.x = 0;
        latest_cmd_vel_.linear.y = 0;
        latest_cmd_vel_.angular.z = 0;
    }

    mavros_msgs::PositionTarget raw_target;
    raw_target.header.stamp = ros::Time::now();
// 【关键点 1】坐标系选择
    // 使用 FRAME_BODY_NED。
    // 这意味着 velocity.x 是 "机头向前"，velocity.y 是 "机身向左/右" (取决于NED定义，通常右为正)
    // cmd_vel 的 x 是前，y 是左。move_base 输出通常适配 Body Frame。
    raw_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; 

    // 【关键点 2】Type Mask (混合控制模式)
    // 我们想要: 
    // - 控制 Vx, Vy (忽略 Px, Py)
    // - 控制 Pz (忽略 Vz, Az) -> 这样才能定高！
    // - 控制 YawRate (忽略 Yaw)
    raw_target.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_VZ |  // 忽略 Z 速度 (让 Pz 生效)
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW;

    // 2. 平面速度控制 (来自 move_base)
    // 限制幅度，防止过快
    double cmd_x = std::max(std::min(latest_cmd_vel_.linear.x, max_vel_), -max_vel_);
    double cmd_y = std::max(std::min(latest_cmd_vel_.linear.y, max_vel_), -max_vel_);
    
    // 注意：ROS (ENU) y轴向左，PX4 (NED) y轴向右。
    // 如果你发现左右飞反了，把下面这行改为 -cmd_y
    // 根据你之前的反馈 "Z轴不取反能用"，这里先保持直传，实飞微调
    raw_target.velocity.x = cmd_x;
    raw_target.velocity.y = cmd_y; 
    raw_target.velocity.z = 0; // 这个值会被忽略，因为 IGNORE_VZ 没设，啊不对，上面设了IGNORE_VZ，所以这里填啥都行

    // 3. 高度控制 (锁定位置)
    // 使用切入模式时锁定的高度 target_pos_.z()
    raw_target.position.z = target_pos_.z(); 

    // 4. 偏航控制 (来自 move_base)
    // 同理，ROS逆时针为正，PX4顺时针为正。
    // 如果发现一直转圈，把这里改为 -latest_cmd_vel_.angular.z
    raw_target.yaw_rate = latest_cmd_vel_.angular.z;

    // 发布给 MAVROS
    setpoint_raw_pub_.publish(raw_target);
    
    // 【重要】同步平滑处理
    // 在探索模式下，飞机位置在变。
    // 我们需要实时更新 current_setpoint_ 为当前真实位置。
    // 否则当你切回 HOVER 或 MISSION 模式时，飞机可能会突然试图飞回几分钟前的位置。
    current_setpoint_ = current_pos_;
    
    // 同时更新 target_att_，防止切出模式时姿态跳变
    target_att_ = current_att_;
}

void DroneCommander::perform_takeoff() {
    publish_status("MODE: TAKEOFF");

    // 1. 尝试切 OFFBOARD 和 解锁 (如果是第一次)
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_req_ > ros::Duration(5.0))) {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if(set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_req_ = ros::Time::now();
    } else {
        if (!current_state_.armed && (ros::Time::now() - last_req_ > ros::Duration(5.0))) {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if(arming_client_.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_req_ = ros::Time::now();
        }
    }

    // 2. 执行 P 控制飞向 1m 高度
    update_setpoint_p_control(target_pos_, 0.05); // 假定 20Hz, dt=0.05
}

void DroneCommander::perform_mission() {
    publish_status("MODE: MISSION SQUARE");

    // 定义正方形的四个角点 (相对于 start_pos_)
    // 假设 start_pos_ 是 (0,0,1)，正方形边长 1m
    Eigen::Vector3d waypoints[4];
    waypoints[0] = start_pos_ + Eigen::Vector3d(0, 0, 1.0); // 确保高度
    waypoints[1] = start_pos_ + Eigen::Vector3d(1.0, 0, 1.0);
    waypoints[2] = start_pos_ + Eigen::Vector3d(1.0, 1.0, 1.0);
    waypoints[3] = start_pos_ + Eigen::Vector3d(0, 1.0, 1.0);
    // 闭环回到点0
    
    // 获取当前目标点
    Eigen::Vector3d current_target = waypoints[mission_waypoint_index_ % 4];

    // 判断是否到达
    double dist = (current_pos_ - current_target).norm();
    if (dist < acceptance_radius_) {
        ROS_INFO("Reached Waypoint %d", mission_waypoint_index_);
        mission_waypoint_index_++;
        ros::Duration(1.0).sleep(); // 停顿1秒再走下一步
    }

    update_setpoint_p_control(current_target, 0.05);
}

void DroneCommander::perform_land() {
    publish_status("MODE: LANDING");

    // 1. 先回到原点上方 (x,y 回归, z 保持)
    Eigen::Vector3d hover_above_home = start_pos_;
    hover_above_home.z() = current_pos_.z(); // 保持当前高度平移回去

    double dist_to_home_xy = (Eigen::Vector2d(current_pos_.x(), current_pos_.y()) - 
                              Eigen::Vector2d(start_pos_.x(), start_pos_.y())).norm();

    if (dist_to_home_xy > 0.2) {
        // 如果水平距离还远，先平移回去
        update_setpoint_p_control(hover_above_home, 0.05);
    } else {
        // 2. 水平到位了，开始降落
        // 简单做法：切换到 AUTO.LAND 模式
        if (current_state_.mode != "AUTO.LAND") {
            mavros_msgs::SetMode land_set_mode;
            land_set_mode.request.custom_mode = "AUTO.LAND";
            if(set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent){
                ROS_INFO("Auto Land enabled");
            }
        }
    }
}

void DroneCommander::publish_status(std::string status_str) {
    std_msgs::String msg;
    msg.data = status_str;
    status_pub_.publish(msg);
}

void DroneCommander::run() {
    ros::Rate rate(20.0); // 20Hz
    last_req_ = ros::Time::now();

    while(ros::ok()){
        // 必须有位置信号且已经记录了原点才能切模式 (IDLE除外)
        if(current_mode_ != IDLE && !home_recorded_){
             ROS_WARN("No Home Position Recorded! Cannot execute.");
             current_mode_ = IDLE;
        }

        switch (current_mode_) {
            case IDLE:
                publish_status("MODE: IDLE (Waiting for command)");
                // 在 IDLE 模式下，我们要不断发送当前位置作为 setpoint，防止切 OFFBOARD 时乱飞
                update_setpoint_p_control(current_pos_, 0.05); 
                break;
            case TAKEOFF:
                perform_takeoff();
                break;
            case MISSION_SQUARE:
                perform_mission();
                break;
            case LAND:
                perform_land();
                break;
            case EXPLORE: // 【新增】调用探索逻辑
            perform_explore();
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }
}