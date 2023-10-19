#ifndef DELTA_ROBOT_DELTA_ROS_INTERFACE_HPP
#define DELTA_ROBOT_DELTA_ROS_INTERFACE_HPP

#include "delta_robot/delta_model.hpp"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <memory>
#include <ros/ros.h>

class delta_ros_interface
{
private:
    
    ros::NodeHandle nh_;
    ros::Subscriber joints_states_sub_;
    ros::Publisher join1_vel_pub_, join2_vel_pub_, join3_vel_pub_;

    std::shared_ptr<delta_model> model_;
    
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);


public:
    delta_ros_interface(ros::NodeHandle nh);
    ~delta_ros_interface();
    void publishJointVelocities(double* dq);

    void run();


};




#endif