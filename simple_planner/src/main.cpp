#include "simple_planner/drone_commander.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_commander_node");
    ros::NodeHandle nh;

    DroneCommander commander(nh);
    commander.run();

    return 0;
}