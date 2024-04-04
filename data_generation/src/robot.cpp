//
// Created by will on 02/04/24.
//

#include "robot.h"

//Robot::Robot()
//        : move_group_interface(PLANNING_GROUP),
//          current_state(move_group_interface.getCurrentState()) {
//
//    // Initialisations supplémentaires si nécessaire
//
//}
Robot::Robot(const std::string& planning_group)
        : move_group_(planning_group),
          joint_model_group_(move_group_.getCurrentState()->getJointModelGroup(planning_group)) {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
}
Robot::~Robot() {}

bool Robot::move_joint(joint &j) {
    ros::AsyncSpinner spinner(1);
    spinner.start();
//    const moveit::core::JointModelGroup* joint_model_group =
//            move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
//    //
//    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
//    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//

    joint_group_positions.push_back(j.j1);
    joint_group_positions.push_back(j.j2);
    joint_group_positions.push_back(j.j3);
    joint_group_positions.push_back(j.j4);
    joint_group_positions.push_back(j.j5);
    joint_group_positions.push_back(j.j6);

    this->move_group_.setJointValueTarget(joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
//    move_group_interface.setMaxVelocityScalingFactor(0.05);
//    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    bool success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    return success;
}


bool Robot::get_all_ik(const std::vector<double> &pose,
                       std::vector<joint> &joints, int &numOfSolns) {


    const moveit::core::RobotStatePtr& current_state = move_group_.getCurrentState();

    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = pose[0];
    goal_pose.position.y = pose[1];
    goal_pose.position.z = pose[2];
    goal_pose.orientation.x = pose[3];
    goal_pose.orientation.y = pose[4];
    goal_pose.orientation.z = pose[5];
    goal_pose.orientation.w = pose[6];

    joint j;
    std::vector<std::vector<double>> joints_array;

//    get time
//    ros::Time begin = ros::Time::now();
    bool found_ik = current_state->setFromIK(joint_model_group_, goal_pose);
//    ros::Time end = ros::Time::now();
//    ros::Duration duration = end - begin;
//    ROS_INFO("IK duration: %f", duration.toSec());
    if (found_ik) {
        std::vector<double> joint_values;
        current_state->copyJointGroupPositions(joint_model_group_, joint_values);
//        get time
//        begin = ros::Time::now();
        bool is_collide = checkCollision(joint_values);
//        end = ros::Time::now();
//        duration = end - begin;
//        ROS_INFO("Collision check duration: %f", duration.toSec());
        if (!is_collide) {

            // add the joint solution to the vector
            j.j1 = joint_values[0];
            j.j2 = joint_values[1];
            j.j3 = joint_values[2];
            j.j4 = joint_values[3];
            j.j5 = joint_values[4];
            j.j6 = joint_values[5];
//            get time
//            begin = ros::Time::now();
            bool get_path = move_cartesian(goal_pose);
//            end = ros::Time::now();
//            duration = end - begin;
//            ROS_INFO("Move joint duration: %f", duration.toSec());

            if(get_path){
                joints.push_back(j);
            }else{
                return false;
            }

        }else{
            return false;
        }
    } else {
        return false;
    }
    return true;
}

bool Robot::checkCollision(const std::vector<double>& joint_positions) {
    robot_state::RobotState& current_state = planning_scene_->getCurrentStateNonConst();
    current_state.setJointGroupPositions(joint_model_group_, joint_positions);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene_->checkSelfCollision(collision_request, collision_result, current_state, planning_scene_->getAllowedCollisionMatrix());

    return collision_result.collision;
}

bool Robot::move_cartesian(geometry_msgs::Pose &pose) {
    move_group_.setPoseTarget(pose);
    bool success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    return success;

}

