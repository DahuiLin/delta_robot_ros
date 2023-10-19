#include "delta_robot/delta_ros_interface.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "delta_robot");
    ros::NodeHandle nh;
    delta_ros_interface delta(nh);
    delta.run();
    return 0;
}