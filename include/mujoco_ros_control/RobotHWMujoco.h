/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 8/2/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 *
 *     Register Joint State interface and Joint Effort interface for every joint in MuJoCo model.
 *
 */

#ifndef MUJOCO_ROS_CONTROL_ROBOTHWMUJOCO_H
#define MUJOCO_ROS_CONTROL_ROBOTHWMUJOCO_H

#include <common_robot_functions/Mujoco/mujoco_parser.h>
#include <common_robot_functions/Transformations/transformation.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class RobotHWMujoco : public hardware_interface::RobotHW {
public:

    /** @brief Register interfaces for each joint in a MuJoCo model. */
    explicit RobotHWMujoco(MujocoParser * m);

    /** @brief From mjdata extract position,velocity and acceleration into the internal state */
    void read();

    /** @brief Write to mjData computed command */
    void write();

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;

    /** @brief Defines position in the array of MuJoCo data, qadr for qpos array and vadr for qvel, qacc, and xfrc_applied */
    std::vector<size_t> qadr, vadr;
    std::vector<double> cmd, cur_pos, cur_vel, cur_eff;
    MujocoParser * mujoco_parser;
};

#endif //MUJOCO_ROS_CONTROL_ROBOTHWMUJOCO_H
