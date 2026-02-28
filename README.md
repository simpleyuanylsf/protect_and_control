# 无人机保护与控制套件 


基于 ROS + MAVROS 的无人机安全保护与控制系统，提供键盘控制、电子围栏保护、低电压自动返航等功能，支持与大语言模型集成实现智能控制。

---

## 📋 功能概览

| 模块 | 功能描述 | 状态 |
|------|----------|------|
| **键盘控制节点** | 通过键盘控制无人机起飞、任务执行、探索、降落 | ✅ 完成 |
| **电子围栏** | 实时监控无人机位置，越界自动触发降落保护 | ✅ 完成 |
| **低电压返航** | 电池电量/电压监控，低电量自动返航降落 | 未测试 |

---

## 🏗️ 项目结构

```
protect_and_control/
├── simple_planner/          # 键盘控制与飞行控制节点
│   ├── src/
│   │   ├── keyboard_control.cpp   # 键盘输入节点
│   │   ├── uavkey_control.cpp     # 无人机控制核心
│   │   └── main.cpp
│   ├── include/
│   │   └── simple_planner/
│   │       └── drone_commander.h
│   └── launch/
│       ├── keyboard_control.launch
│       └── uavkey_control.launch
│
├── geofence_guard/          # 电子围栏保护节点
│   ├── src/
│   │   ├── geofence_guard.cpp
│   │   └── main.cpp
│   ├── include/
│   │   └── geofence_guard/
│   │       └── geofence_guard.h
│   └── launch/
│       └── geofence_guard.launch
│
└── lowbattery_RTL/          # 低电压自动返航节点
    ├── src/
    │   └── low_battery.cpp
    └── launch/
        └── low_battery.launch
```

---

## 🚀 快速开始

### 环境要求

- **ROS**: Melodic 或 Noetic
- **MAVROS**: 用于与 PX4 飞控通信
- **Eigen3**: 用于向量计算
- **PX4 Autopilot**: 无人机飞控固件

### 编译安装

```bash

# 编译
cd ~/catkin_ws
catkin_make

#  source 环境
source devel/setup.bash
```

---

## 🎮 键盘控制节点 (simple_planner)

### 功能特点

- **起飞模式 (T)**: 自动解锁并起飞到 1 米高度
- **任务模式 (M)**: 执行预设的正方形轨迹任务
- **探索模式 (E)**: **2D 自主导航，接收 `/cmd_vel` 指令进行平面移动**
- **降落模式 (L)**: 返回起飞点并自动降落
- **待机模式 (I)**: 悬停在当前位置等待指令

### 启动方法

```bash
# 启动键盘控制节点
roslaunch simple_planner keyboard_control.launch

# 启动无人机控制核心
roslaunch simple_planner uavkey_control.launch
```

### 键盘控制说明

```
========== 无人机大模型模拟控制器 ==========
请按键盘控制模式切换 (无需回车):
  [T] -> 起飞模式 (Takeoff - 1m)
  [M] -> 任务模式 (Mission - 正方形轨迹)
  [E] -> 探索模式 (Explore - 2D自主导航)
  [L] -> 降落模式 (Land - 返回原点)
  [I] -> 待机模式 (Idle)
  [Esc] -> 退出程序
============================================
```

### 🔗 探索模式集成说明

**探索模式需要配合 [erwei_navigatin](https://github.com/simpleyuanylsf/erwei_navigatin) 项目一起使用。**

探索模式下，无人机接收 `/cmd_vel` 话题的速度指令，实现 2D 平面自主导航，同时保持当前高度不变。

**集成步骤：**

```bash
# 1. 克隆并编译 erwei_navigatin 项目
cd ~/catkin_ws/src
git clone https://github.com/simpleyuanylsf/erwei_navigatin.git
cd ~/catkin_ws && catkin_make

```

---

## 🛡️ 电子围栏 (geofence_guard)

### 功能特点

- **实时监控**: 20Hz 频率监控无人机位置
- **形状支持**: 支持矩形和圆形两种围栏形状
- **越界保护**: 飞出围栏时立即触发 `AUTO.LAND` 自动降落
- **高度限制**: 可设置最大飞行高度

### 启动方法

```bash
roslaunch geofence_guard geofence_guard.launch
```

### 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `fence_shape` | int | 0 | 围栏形状: 0=矩形, 1=圆形 |
| `fence_center_x` | double | 0.0 | 围栏中心 X 坐标 |
| `fence_center_y` | double | 0.0 | 围栏中心 Y 坐标 |
| `fence_half_width` | double | 1.0 | 矩形半宽（米） |
| `fence_half_height` | double | 1.0 | 矩形半高（米） |
| `fence_radius` | double | 1.0 | 圆形半径（米） |
| `max_altitude` | double | 2.0 | 最大飞行高度（米） |

### 自定义配置示例

```xml
<launch>
    <node pkg="geofence_guard" type="geofence_guard_node" name="geofence_guard" output="screen">
        <!-- 圆形围栏，半径 5 米 -->
        <param name="fence_shape" value="1"/>
        <param name="fence_center_x" value="0.0"/>
        <param name="fence_center_y" value="0.0"/>
        <param name="fence_radius" value="5.0"/>
        <param name="max_altitude" value="3.0"/>
    </node>
</launch>
```

---

## 🔋 低电压返航 (lowbattery_RTL)

### 功能特点

- **双重检测**: 支持电量百分比和电压阈值检测
- **延时触发**: 低电量持续一定时间后才触发，避免误报
- **智能返航**: 三种策略自动选择：
  - **策略 A**: 指挥官在线，发送降落指令
  - **策略 B**: 独立控制，自主返航到起飞点
  - **策略 C**: 手动模式，强制切换 `AUTO.RTL`

### 启动方法

```bash
roslaunch lowbattery_RTL low_battery.launch
```

### 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `low_battery_percent` | double | 0.20 | 低电量阈值（20%） |
| `low_battery_voltage` | double | 22.0 | 低电压阈值（V） |
| `trigger_duration` | double | 5.0 | 触发延时（秒） |

---

## 🤖 大语言模型控制集成

如需通过大语言模型（LLM）控制无人机，可参考以下项目：

🔗 **参考项目**: [github.com/simpleyuanylsf]((https://github.com/simpleyuanylsf/language_demo))

### 集成思路

1. **模式映射**: 将 LLM 输出的自然语言指令映射到模式 ID
   - "起飞" → `MODE_TAKEOFF (1)`
   - "执行任务" → `MODE_MISSION (2)`
   - "探索环境" → `MODE_EXPLORE (4)`
   - "降落" → `MODE_LAND (3)`

2. **话题接口**: 发布 `std_msgs/Int32` 到 `/commander/set_mode`

```python
# Python 示例
import rospy
from std_msgs.msg import Int32

mode_pub = rospy.Publisher('/commander/set_mode', Int32, queue_size=10)

# LLM 解析用户指令后发布对应模式
def execute_command(command):
    mode_mapping = {
        'takeoff': 1,
        'mission': 2,
        'land': 3,
        'explore': 4,
        'idle': 0
    }
    mode_id = mode_mapping.get(command, 0)
    mode_pub.publish(Int32(mode_id))
```

---

## 📡 ROS 话题接口

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/mavros/state` | `mavros_msgs/State` | 飞控状态 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 本地位置 |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | 里程计数据 |
| `/mavros/battery` | `sensor_msgs/BatteryState` | 电池状态 |
| `/cmd_vel` | `geometry_msgs/Twist` | 探索模式速度指令 |

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/commander/set_mode` | `std_msgs/Int32` | 模式切换指令 |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 位置设定点 |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 原始控制指令 |

### 服务模式

| 服务名 | 消息类型 | 说明 |
|--------|----------|------|
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 解锁/上锁 |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 模式切换 |

---

## ⚠️ 安全提示

1. **首次测试请在仿真环境进行**
2. **确认 GPS 或定位系统正常工作后再解锁**
3. **设置合理的电子围栏范围**
4. **定期检查电池状态**
5. **准备遥控器随时接管**

---

## 📝 许可证

MIT License

---

## 🤝 贡献与支持

如有问题或建议，欢迎提交 Issue 或 Pull Request。

---

## 🔗 相关链接

- **2D 导航依赖**: [erwei_navigatin](https://github.com/simpleyuanylsf/erwei_navigatin)
- **LLM 控制参考**: [github.com/simpleyuanylsf](https://github.com/simpleyuanylsf)
- **PX4 官方文档**: https://docs.px4.io
- **MAVROS 文档**: https://github.com/mavlink/mavros
