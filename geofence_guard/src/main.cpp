#include "geofence_guard/geofence_guard.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "geofence_guard_node");
    
    GeoFenceGuard guard;
    guard.run();
    
    return 0;
}
