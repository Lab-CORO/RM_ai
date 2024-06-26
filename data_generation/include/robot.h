//
// Created by will on 02/04/24.
//

#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit_msgs/GetPositionIK.h"
#include <moveit_msgs/ApplyPlanningScene.h>
#include <Eigen/Dense>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "master_ik_data.h"

class Robot {

public:
    Robot(const std::string& planning_group);
    ~Robot();
    bool get_all_ik( const std::vector<double> &pose, std::vector<joint> &joints, int &numOfSolns);
    bool move_joint(joint &j);
    bool move_cartesian(geometry_msgs::Pose &pose);
    bool checkCollision(const std::vector<double>& joint_positions);

private:
    ros::NodeHandle node_handle;
    moveit::planning_interface::MoveGroupInterface move_group_;
    const robot_state::JointModelGroup* joint_model_group_;
    planning_scene::PlanningScenePtr planning_scene_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
//    moveit::core::RobotState &current_state_NonConst;
    moveit::planning_interface::PlanningSceneInterface current_state_;
    bool box_init;



};


#endif //SRC_ROBOT_H
