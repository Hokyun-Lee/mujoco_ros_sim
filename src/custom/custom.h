#ifndef MYCONTROLLER_H
#define MYCONTROLLER_H

#include <mujoco/mjdata.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <iostream>

// #ifdef MUJOCO_ROS
// #include <ros/ros.h>
// #include <mujoco_ros_msgs/SensorState.h>
// #include <mujoco_ros_msgs/JointSet.h>
// #include <mujoco_ros_msgs/SimStatus.h>
// #include <mujoco_ros_msgs/applyforce.h>

// #include <std_msgs/String.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <sensor_msgs/JointState.h>
// #include <tf/transform_datatypes.h>
// #endif


void initialize_mycontroller(const mjModel *m, mjData *d);
void mycontroller(const mjModel* m, mjData* d);

#endif