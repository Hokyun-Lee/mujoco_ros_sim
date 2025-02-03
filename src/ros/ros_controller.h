#ifndef ROSCONTROLLER_H
#define ROSCONTROLLER_H

#include <mujoco/mjdata.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

class MujocoRos : public rclcpp::Node
{
public:
    MujocoRos();


    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscriber_;


    sensor_msgs::msg::JointState joint_state_msg;
    sensor_msgs::msg::Imu imu_msg;

    Eigen::VectorXd e_ctrl;
    Eigen::VectorXd eq_pos;
    Eigen::VectorXd eq_vel;
    Eigen::VectorXd eq_acc;

    void initialize(const mjModel *m, mjData *d);
    void ros_sync(const mjModel *m, mjData *d);
    // void setup_executor();
    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);


};

// void initialize_roscontroller(const mjModel *m, mjData *d);
// void roscontroller(const mjModel* m, mjData* d);

#endif