#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

// 定义模式常量，必须与 drone_commander.h 保持一致
const int MODE_IDLE = 0;
const int MODE_TAKEOFF = 1;
const int MODE_MISSION = 2;
const int MODE_LAND = 3;
const int MODE_EXPLORE = 4; // 【新增】探索模式 ID

// Linux下实现无需回车直接读取按键的函数
int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);           // 保存旧的终端设置
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);         // 取消行缓冲和回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 应用新设置

    int c = getchar();                        // 读取字符

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 恢复旧设置
    return c;
}

void print_menu() {
    std::cout << "\n\033[1;32m========== 无人机大模型模拟控制器 ==========\033[0m" << std::endl;
    std::cout << "请按键盘控制模式切换 (无需回车):" << std::endl;
    std::cout << "  [T] -> 起飞模式 (Takeoff - 1m)" << std::endl;
    std::cout << "  [M] -> 任务模式 (Mission - 正方形轨迹)" << std::endl;
    std::cout << "  [E] -> 探索模式 (Explore - 2D自主导航)" << std::endl; // 【新增】菜单显示
    std::cout << "  [L] -> 降落模式 (Land - 返回原点)" << std::endl;
    std::cout << "  [I] -> 待机模式 (Idle)" << std::endl;
    std::cout << "  [Esc] -> 退出程序" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "等待指令 >> ";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_control_node");
    ros::NodeHandle nh;

    // 发布到 /commander/set_mode 话题
    ros::Publisher mode_pub = nh.advertise<std_msgs::Int32>("/commander/set_mode", 10);

    // 打印菜单
    print_menu();

    std_msgs::Int32 msg;
    int c;

    while (ros::ok()) {
        // 读取按键 (阻塞直到有按键按下)
        c = getch();

        // 这里的 0x03 是 Ctrl+C 的 ASCII 码
        if (c == 0x03) {
            ROS_INFO("Exiting...");
            break;
        }

        bool valid_key = false;
        std::string mode_str = "";

        // 处理大小写
        switch (c) {
            case 't':
            case 'T':
                msg.data = MODE_TAKEOFF;
                mode_str = "TAKEOFF (起飞)";
                valid_key = true;
                break;
            case 'm':
            case 'M':
                msg.data = MODE_MISSION;
                mode_str = "MISSION (执行任务)";
                valid_key = true;
                break;
            // 【新增】处理 'E' 键
            case 'e':
            case 'E':
                msg.data = MODE_EXPLORE; // 发送 4
                mode_str = "EXPLORE (开始探索)";
                valid_key = true;
                break;
            case 'l':
            case 'L':
                msg.data = MODE_LAND;
                mode_str = "LAND (降落)";
                valid_key = true;
                break;
            case 'i':
            case 'I':
                msg.data = MODE_IDLE;
                mode_str = "IDLE (待机)";
                valid_key = true;
                break;
            default:
                // 如果按了其他键，不处理
                break;
        }

        if (valid_key) {
            // 在终端输出切换信息
            std::cout << "\r\033[K"; // 清除当前行
            std::cout << "\033[1;33m[发送指令] 切换到: " << mode_str << "\033[0m" << std::endl;
            
            // 发布话题
            mode_pub.publish(msg);
            
            // 重新打印提示符
            std::cout << "等待指令 >> "; 
            std::cout.flush(); // 刷新缓冲区确保显示
        }
        
        ros::spinOnce();
    }

    return 0;
}