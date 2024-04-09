//
// Created by will on 02/04/24.
//

#include "robot.h"


Robot::Robot(const std::string &planning_group)
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
    std::vector<double> joint_group_positions;

    joint_group_positions.push_back(j.j1);
    joint_group_positions.push_back(j.j2);
    joint_group_positions.push_back(j.j3);
    joint_group_positions.push_back(j.j4);
    joint_group_positions.push_back(j.j5);
    joint_group_positions.push_back(j.j6);

    this->move_group_.setJointValueTarget(joint_group_positions);

    double planning_timeout = 0.2;
    move_group_.setPlanningTime(planning_timeout);
    bool success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    return success;
}


bool Robot::get_all_ik(const std::vector<double> &pose,
                       std::vector<joint> &joints, int &numOfSolns) {


    const moveit::core::RobotStatePtr &current_state = move_group_.getCurrentState();

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

            if (get_path) {
                joints.push_back(j);
            } else {
                return false;
            }

        } else {
            return false;
        }
    } else {
        return false;
    }
    return true;
}


bool Robot::checkCollision(const std::vector<double> &joint_positions) {
    // Obtenez l'état actuel du robot
    moveit::core::RobotState &current_state = planning_scene_->getCurrentStateNonConst();

    // Appliquez les positions des joints fournies à l'état actuel du robot
    current_state.setJointGroupPositions(joint_model_group_, joint_positions);

    // Assurez-vous que la nouvelle configuration des joints est dans les limites acceptables
    if (!current_state.satisfiesBounds(joint_model_group_)) {
        ROS_INFO_STREAM("La configuration des joints est hors des limites acceptables.");
        return false; // ou vous pouvez choisir de retourner true, selon la manière dont vous souhaitez gérer cette situation
    }

    // Préparation de la requête et du résultat de collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true; // Option pour obtenir des détails sur les contacts en cas de collision
    collision_request.max_contacts = 1000; // Nombre maximal de contacts à rapporter

    // Vérification des collisions avec l'environnement
    planning_scene_->checkCollision(collision_request, collision_result, current_state, planning_scene_->getAllowedCollisionMatrix());

    // Log des contacts en cas de collision
    if (collision_result.collision) {
        return true;
    } else {
        return false;
    }
}





bool Robot::move_cartesian(geometry_msgs::Pose &pose) {
    move_group_.setPoseTarget(pose);
    double planning_timeout = 0.2; // Par exemple, 10 secondes
    move_group_.setPlanningTime(planning_timeout);
    bool success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    return success;

}