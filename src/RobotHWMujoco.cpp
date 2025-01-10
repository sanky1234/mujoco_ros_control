#include <mujoco_ros_control/RobotHWMujoco.h>

RobotHWMujoco::RobotHWMujoco(MujocoParser * m) : mujoco_parser(m)
{
    assert(mujoco_parser->n_ctrl_ > 0);
    const auto n = (size_t) mujoco_parser->n_ctrl_;
    cmd.resize(n, 0.0);
    cur_pos.resize(n, 0.0);
    cur_vel.resize(n, 0.0);
    cur_eff.resize(n, 0.0);
    qadr.resize(n, 0);
    vadr.resize(n, 0);

    for (size_t i = 0; i < n; ++i) {
        // const auto joint_type = m.jnt_type[i];
        // if (joint_type == mjJNT_FREE || joint_type == mjJNT_BALL) {
        //     continue;
        // }
        int joint_id = mujoco_parser->m_->actuator_trnid[i*2];
        const auto joint_name = mj_id2name(mujoco_parser->m_, mjOBJ_JOINT, joint_id);
        ROS_INFO_STREAM("Registering joint: " << joint_name);
        ROS_INFO_STREAM("Registering joint: " << joint_id);

        qadr[i] = (size_t) mujoco_parser->m_->jnt_qposadr[joint_id];
        vadr[i] = (size_t) mujoco_parser->m_->jnt_dofadr[joint_id];

        hardware_interface::JointStateHandle state_handle_a(joint_name, &cur_pos[i], &cur_vel[i], &cur_eff[i]);
        jnt_state_interface.registerHandle(state_handle_a);

        hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle(joint_name), &cmd[i]);
        jnt_pos_interface.registerHandle(pos_handle_a);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
}

void RobotHWMujoco::read() {
    for (size_t i = 0; i < qadr.size(); ++i) {
        cur_pos[i] = mujoco_parser->d_->qpos[qadr[i]];
        cur_vel[i] = mujoco_parser->d_->qvel[vadr[i]];
        cur_eff[i] = mujoco_parser->d_->qfrc_applied[vadr[i]];
    }
}

void RobotHWMujoco::write() {
    for (size_t i = 0; i < vadr.size(); ++i) {
        mujoco_parser->d_->ctrl[i] = cmd[i];
    }
}
