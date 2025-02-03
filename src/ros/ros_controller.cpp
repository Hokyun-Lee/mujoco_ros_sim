#include "ros_controller.h"


MujocoRos::MujocoRos() : rclcpp::Node("mujoco_ros")
{

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/mujoco/joint_states", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/mujoco/imu", 10);
    joint_command_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/mujoco/joint_command", 10, std::bind(&MujocoRos::joint_command_callback, this, std::placeholders::_1));

    std::cout << "MujocoRos initialized" << std::endl;
}

void MujocoRos::initialize(const mjModel *m, mjData *d)
{
    // initialize the controller

    if (m->nkey > 0)
    {
        mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
    }
    // mju_copy(d->qpos,m->key_qpos + i*m->nq,m->nq)
    // control the joint

    e_ctrl.setZero(m->nu);
    eq_pos.setZero(m->nq);
    eq_vel.setZero(m->nv);
    eq_acc.setZero(m->nv);

    
    joint_state_msg.name.clear();
    // for (int i = 0; i < m->nu; i++)
    // {
    //     joint_state_msg.name.push_back(m->names + m->name_actuatoradr[i] * mjOBJ_SIZE);
    // }

    //get init joint status
    joint_state_msg.position.resize(m->nq);
    joint_state_msg.velocity.resize(m->nv);
    joint_state_msg.effort.resize(m->nv);

    std::copy(d->qpos, d->qpos + m->nq, eq_pos.data());
    std::copy(d->qvel, d->qvel + m->nv, eq_vel.data());
    std::copy(d->qacc, d->qacc + m->nv, eq_acc.data());

    joint_state_msg.header.stamp = rclcpp::Time(this->now());

    for ( int i=0;i<m->nu;i++)
    {
        joint_state_msg.position[i] = eq_pos[i+7];
        joint_state_msg.velocity[i] = eq_vel[i+6];
        joint_state_msg.effort[i] = eq_acc[i+6];
    }
\
    joint_state_publisher_->publish(joint_state_msg);

}

void MujocoRos::ros_sync(const mjModel *m, mjData *d)
{

    //get current joint status
    std::copy(d->qpos, d->qpos + m->nq, eq_pos.data());
    std::copy(d->qvel, d->qvel + m->nv, eq_vel.data());
    std::copy(d->qacc, d->qacc + m->nv, eq_acc.data());

    joint_state_msg.header.stamp = rclcpp::Time(this->now());

    for ( int i=0;i<m->nu;i++)
    {
        joint_state_msg.position[i] = eq_pos[i+7];
        joint_state_msg.velocity[i] = eq_vel[i+6];
        joint_state_msg.effort[i] = eq_acc[i+6];
    }
    joint_state_publisher_->publish(joint_state_msg);

    // executor_.spin_once(std::chrono::nanoseconds(1000000));

    // std::cout << "qpos : "<< joint_state_msg.position[0] << joint_state_msg.position[1] << joint_state_msg.position[2]<<std::endl;
    // ctrl_vec.setZero();
    // e_ctrl(12) = sin(d->time);



    mju_copy(d->ctrl, e_ctrl.data(), m->nu);
}

void MujocoRos::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{  
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->effort[0]);

    for(int i=0;i<msg->effort.size();i++)
    {
        e_ctrl(i) = msg->effort[i];
    }

    std::cout << "Joint command received : "<<msg->effort.size() << std::endl;
}

// void MujocoRos::setup_executor()
// {
//     executor_.add_node(shared_from_this());
// }
