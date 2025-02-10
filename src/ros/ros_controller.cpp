#include "ros_controller.h"

MujocoRos::MujocoRos() : rclcpp::Node("mujoco_ros")
{
    command_received = false;
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

    actuator_dof = m->nu;

    joint_state_msg.name.clear();
    // for (int i = 0; i < m->nu; i++)
    // {
    //     joint_state_msg.name.push_back(m->names + m->name_actuatoradr[i] * mjOBJ_SIZE);
    // }

    // get init joint status
    joint_state_msg.position.resize(m->nu);
    joint_state_msg.velocity.resize(m->nu);
    joint_state_msg.effort.resize(m->nu);

    std::copy(d->qpos, d->qpos + m->nq, eq_pos.data());
    std::copy(d->qvel, d->qvel + m->nv, eq_vel.data());
    std::copy(d->qacc, d->qacc + m->nv, eq_acc.data());

    // Publish sim time
    joint_state_msg.header.stamp.sec = (int)d->time;
    joint_state_msg.header.stamp.nanosec = (int)((d->time - (int)d->time) * 1e9);

    joint_state_msg.name.resize(m->nu);

    int qdiff = m->nq - m->nu;
    int qdiff2 = m->nv - m->nu;

    for (int i = 0; i < m->nu; i++)
    {

        std::string buffer(m->names + m->name_actuatoradr[i]);
        joint_state_msg.name[i] = buffer;

        joint_state_msg.position[i] = eq_pos[i + qdiff];
        joint_state_msg.velocity[i] = eq_vel[i + qdiff2];
        joint_state_msg.effort[i] = eq_acc[i + qdiff2];
    }

    joint_state_publisher_->publish(joint_state_msg);

    imu_msg.header.frame_id = "base_link";
    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 1;

    imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = 0;

    imu_msg.linear_acceleration.x = 0;
    imu_msg.linear_acceleration.y = 0;
    imu_msg.linear_acceleration.z = 0;

    imu_publisher_->publish(imu_msg);
}

void MujocoRos::ros_sync(const mjModel *m, mjData *d)
{

    // get current joint status
    std::copy(d->qpos, d->qpos + m->nq, eq_pos.data());
    std::copy(d->qvel, d->qvel + m->nv, eq_vel.data());
    std::copy(d->qacc, d->qacc + m->nv, eq_acc.data());

    joint_state_msg.header.stamp.sec = (int)d->time;
    joint_state_msg.header.stamp.nanosec = (int)((d->time - (int)d->time) * 1e9);

    int qdiff = m->nq - m->nu;
    int qdiff2 = m->nv - m->nu;

    for (int i = 0; i < m->nu; i++)
    {
        joint_state_msg.position[i] = eq_pos[i + qdiff];
        joint_state_msg.velocity[i] = eq_vel[i + qdiff2];
        joint_state_msg.effort[i] = eq_acc[i + qdiff2];
    }

    // executor_.spin_once(std::chrono::nanoseconds(1000000));

    // std::cout << "qpos : "<< joint_state_msg.position[0] << joint_state_msg.position[1] << joint_state_msg.position[2]<<std::endl;
    // ctrl_vec.setZero();
    // e_ctrl(12) = sin(d->time);

    imu_msg.header.stamp.sec = (int)d->time;
    imu_msg.header.stamp.nanosec = (int)((d->time - (int)d->time) * 1e9);
    imu_msg.header.frame_id = "base_link";

    if (qdiff == 0)
    {

        imu_msg.orientation.w = 1;
        imu_msg.orientation.x = 0;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
    }
    else
    {

        imu_msg.orientation.w = eq_pos[3];
        imu_msg.orientation.x = eq_pos[4];
        imu_msg.orientation.y = eq_pos[5];
        imu_msg.orientation.z = eq_pos[6];
    }

    for (int i = 0; i < m->nsensor; i++)
    {
        if (m->sensor_type[i] == mjSENS_ACCELEROMETER)
        {
            imu_msg.linear_acceleration.x = d->sensordata[m->sensor_adr[i]];
            imu_msg.linear_acceleration.y = d->sensordata[m->sensor_adr[i] + 1];
            imu_msg.linear_acceleration.z = d->sensordata[m->sensor_adr[i] + 2];
        }
        if (m->sensor_type[i] == mjSENS_GYRO)
        {
            imu_msg.angular_velocity.x = d->sensordata[m->sensor_adr[i]];
            imu_msg.angular_velocity.y = d->sensordata[m->sensor_adr[i] + 1];
            imu_msg.angular_velocity.z = d->sensordata[m->sensor_adr[i] + 2];
        }
    }

    static double prev_time = 0;
    double current_time = d->time;

    if (current_time == prev_time)
    {
        return;
    }

    joint_state_publisher_->publish(joint_state_msg);
    imu_publisher_->publish(imu_msg);

    if (command_received)
    {
        command_received = false;
        mju_copy(d->ctrl, e_ctrl.data(), m->nu);
    }
    prev_time = current_time;
}

void MujocoRos::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->effort[0]);

    if (msg->effort.size() != actuator_dof)
    {
        std::cout << " ERR: recevied commmand has different joint size " << std::endl;
    }

    command_received = true;
    for (int i = 0; i < msg->effort.size(); i++)
    {
        e_ctrl(i) = msg->effort[i];
    }

    // std::cout << "Joint command received : "<<msg->effort.size() << std::endl;
}

// void MujocoRos::setup_executor()
// {
//     executor_.add_node(shared_from_this());
// }
