#include "delta_robot/delta_ros_interface.hpp"

delta_ros_interface::delta_ros_interface(ros::NodeHandle nh): nh_(nh){

    joints_states_sub_ = nh_.subscribe("/delta_robot/joint_states", 1, &delta_ros_interface::jointStatesCallback, this);
    join1_vel_pub_ = nh_.advertise<std_msgs::Float64>("/delta_robot/joint1_velocity_controller/command", 1);
    join2_vel_pub_ = nh_.advertise<std_msgs::Float64>("/delta_robot/joint2_velocity_controller/command", 1);
    join3_vel_pub_ = nh_.advertise<std_msgs::Float64>("/delta_robot/joint3_velocity_controller/command", 1);

    model_ = std::make_shared<delta_model>(0.2, 0.2, 0.2, 0.2, 100.0);

}

delta_ros_interface::~delta_ros_interface(){
}

void delta_ros_interface::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
    double q[3], dq[3];
    q[0] = msg->position[0];
    q[1] = msg->position[1];
    q[2] = msg->position[2];
    dq[0] = msg->velocity[0];
    dq[1] = msg->velocity[1];
    dq[2] = msg->velocity[2];

    if (!model_->setJointState(q, dq))
    {
        std::cout << "ERROR SET JOINT STATE";
    }
}

void delta_ros_interface::publishJointVelocities(double* dq){
    std_msgs::Float64 dq1, dq2, dq3;
    dq1.data = dq[0];
    dq2.data = dq[1];
    dq3.data = dq[2];
    join1_vel_pub_.publish(dq1);
    join2_vel_pub_.publish(dq2);
    join3_vel_pub_.publish(dq3);

}

void delta_ros_interface::run(){
    
    ros::spin();
    
}