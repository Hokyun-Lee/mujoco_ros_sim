#ifndef ROSCONTROLLER_H
#define ROSCONTROLLER_H

#include <mujoco/mjdata.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
// #include <simulate/simulate.h"

#include <Eigen/Dense>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <mujoco_data.h>

using namespace std::chrono_literals;

class MujocoRos : public rclcpp::Node
{
public:
    MujocoRos();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sim_command_subscriber_;

    sensor_msgs::msg::JointState joint_state_msg;
    sensor_msgs::msg::Imu imu_msg;

    bool command_received;
    int mujoco_count;

    double command_time;

    Eigen::VectorXd e_ctrl;
    Eigen::VectorXd m_ctrl;
    Eigen::VectorXd eq_pos;
    Eigen::VectorXd eq_vel;
    Eigen::VectorXd eq_acc;

    mjtNum *ctrl_command;

    int actuator_dof;

    void initialize(const mjModel *m, mjData *d);
    void ros_sync(const mjModel *m, mjData *d);
    // void setup_executor();
    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    mujoco_shm *shm;

    spsc_mujoco_joint_command *joint_command_buffer;
    spsc_mujoco_joint_status *joint_status_buffer;
};

// void initialize_roscontroller(const mjModel *m, mjData *d);
// void roscontroller(const mjModel* m, mjData* d);

#endif