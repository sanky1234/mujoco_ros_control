/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 8/2/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include "stdio.h"
#include <mutex>
#include <common_robot_functions/Mujoco/mujoco_parser.h>
#include <common_robot_functions/Transformations/transformation.h>

#include <GLFW/glfw3.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include <mujoco_ros_control/RobotHWMujoco.h>
#include <controller_manager/controller_manager.h>


std::unique_ptr<RobotHWMujoco> hw;
std::unique_ptr<controller_manager::ControllerManager> cm;

void cb_controller(MujocoParser & m, const float time_step) {
    hw->read();
    const float duration = m.GetSimTime(false);
    int32_t sec = static_cast<int32_t>(duration); // Extract seconds
    int32_t nsec = static_cast<int32_t>((duration - sec) * 1e9);
    cm->update(ros::Time(sec, nsec), ros::Duration(time_step));
    hw->write();
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "mujoco_control");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    bool verbose = false;
    int window_width = 1400;
    int window_height = 1000; 
    bool hide_menus = false; 
    mjtFontScale font_scale = mjFONTSCALE_200;
    double azimuth = 170;
    double distance = 5.0;
    double elevation = -20.0;
    Eigen::Vector3d lookat = Eigen::Vector3d(0.01,0.11,0.5);
    std::vector<double> lookat_vec = {0.01,0.11,0.5};
    bool transparent = false;
    bool contactpoint = false;
    float contactwidth = 0.0;
    float contactheight = 0.0;
    std::vector<float> contact_rgba = {};
    bool joint = false;
    float jointlength = 0.0;
    float jointwidth = 0.0;
    std::vector<float> joint_rgba = {};
    int geomgroup_0 = 0;
    int geomgroup_1 = 0;
    int geomgroup_2 = 0;
    bool update = false;
    int maxgeom = 10000;

    node.param<bool>("mujoco_sim/verbose", verbose, false);
    node.param<int>("mujoco_sim/window_width", window_width, 1400);
    node.param<int>("mujoco_sim/window_height", window_height, 1000);
    node.param<bool>("mujoco_sim/hide_menus", hide_menus, false);
    node.param<double>("mujoco_sim/azimuth", azimuth, 170);
    node.param<double>("mujoco_sim/distance", distance, 5.0);
    node.param<double>("mujoco_sim/elevation", elevation, -20.0);
    node.param<std::vector<double>>("mujoco_sim/lookat", lookat_vec, {0.01,0.11,0.5});
    node.param<bool>("mujoco_sim/transparent", transparent, false);
    node.param<bool>("mujoco_sim/contactpoint", contactpoint, false);
    node.param<float>("mujoco_sim/contactwidth", contactwidth, 0.0);
    node.param<float>("mujoco_sim/contactheight", contactheight, 0.0);
    node.param<std::vector<float>>("mujoco_sim/contact_rgba", contact_rgba, {});
    node.param<bool>("mujoco_sim/joint", joint, false);
    node.param<float>("mujoco_sim/jointlength", jointlength, 0.0);
    node.param<float>("mujoco_sim/jointwidth", jointwidth, 0.0);
    node.param<std::vector<float>>("mujoco_sim/joint_rgba", joint_rgba, {});
    node.param<int>("mujoco_sim/geomgroup_0", geomgroup_0, 0);
    node.param<int>("mujoco_sim/geomgroup_1", geomgroup_1, 0);
    node.param<int>("mujoco_sim/geomgroup_2", geomgroup_2, 0);
    node.param<bool>("mujoco_sim/update", update, false);
    node.param<int>("mujoco_sim/maxgeom", maxgeom, 10000);

    lookat = pose_eigen(lookat_vec[0],lookat_vec[1],lookat_vec[2]);

    const auto default_model_path = ros::package::getPath("mujoco_ros_control") + "/model/simple_robot.urdf";
    const auto model_path = node.param("model", default_model_path);

    std::string test_model_path = "/home/sankalp/ws_spot_catching/src/papras-spot/papras_spot_demo/description/mujoco/spot_catching_update.xml";


    std::string window_name = "Mujoco Simulation";

    MujocoParser mujoco_parser(window_name, test_model_path, verbose);
    
    mujoco_parser.InitViewer(window_name, 
                             window_width,
                             window_height,
                             hide_menus,
                             font_scale, 
                             azimuth,
                             distance,
                             elevation,
                             lookat, 
                             transparent, 
                             contactpoint, 
                             contactwidth,
                             contactheight,
                             contact_rgba, 
                             joint, 
                             jointlength,
                             jointwidth,
                             joint_rgba,
                             geomgroup_0,
                             geomgroup_1,
                             geomgroup_2,
                             update,
                             maxgeom);


    mujoco_parser.Reset(true);

    hw.reset(new RobotHWMujoco(&mujoco_parser));
    cm.reset(new controller_manager::ControllerManager(hw.get(), node));
    ros::CallbackQueue queue;

    ros::TimerOptions timer_options(
                                    ros::Duration(0.008), // 8ms
                                    boost::bind(cb_controller, boost::ref(mujoco_parser), 0.008),
                                    &queue);
    
    ros::Timer timer = node.createTimer(timer_options);



    while(ros::ok() && mujoco_parser.IsViewerAlive()){
         mujoco_parser.Step();

        if(mujoco_parser.LoopEvery(0,50, true)){
            // current_joint_poses = mujoco_parser.GetQposJoints(joint_names);

            // mujoco_parser.PlotT(pr2t(pose_eigen(0,0,0), Eigen::Matrix3d::Identity()));
            mujoco_parser.PlotTime(pose_eigen(0,0,1));
            mujoco_parser.PlotContactInfo("", 0.005, 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), 0.02, true, false, false, false,"");
            mujoco_parser.PlotJointAxis(0.1, 0.005, {}, 0.2);
            mujoco_parser.PlotLinksBetweenBodies({"world"}, 0.001, Eigen::Vector4d(0.0,0.0,0.0,1), "");
            mujoco_parser.Render();

        }
    }   

    return 0;
}
