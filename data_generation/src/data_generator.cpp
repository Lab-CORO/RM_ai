//
// Created by will on 04/04/24.
//

#include "../include/data_generator.h"

DataGenerator::DataGenerator(std::string filename): robot("manipulator") {
    this->filename = filename;
    this->ik_data.load_data(this->filename);
    this->ik_data_result = MasterIkData();
    this->ik_data_result.resolution = this->ik_data.resolution;
    this->ik_data_result.radius = this->ik_data.radius;
    this->ik_data_result.sphere_sample = this->ik_data.sphere_sample;

}

DataGenerator::~DataGenerator() {
}

void DataGenerator::data_comparator() {
    // for each sphere in the data
    for (int i = 0; i < this->ik_data.spheres.size(); i++) {
        Sphere s = this->ik_data.spheres[i];
        // create a sphere
        Sphere sphere_result;
        sphere_result.x = s.x;
        sphere_result.y = s.y;
        sphere_result.z = s.z;
        for (int k = 0; k < s.poses.size(); k++) {
            PoseOnSphere pose_on_sphere_result;
            PoseOnSphere pose_on_sphere = s.poses[k];
            pose_on_sphere_result.x = pose_on_sphere.x;
            pose_on_sphere_result.y = pose_on_sphere.y;
            pose_on_sphere_result.z = pose_on_sphere.z;
            pose_on_sphere_result.theta_x = pose_on_sphere.theta_x;
            pose_on_sphere_result.theta_y = pose_on_sphere.theta_y;
            pose_on_sphere_result.theta_z = pose_on_sphere.theta_z;
            pose_on_sphere_result.theta_w = pose_on_sphere.theta_w;

            // if p has a joint, test it
            if (pose_on_sphere.has_joints()) {
            // test the joint pose with plan
                for (int l = 0; l < pose_on_sphere.joints.size(); l++) {
                    joint j = pose_on_sphere.joints[l];
                    if (!this->robot.checkCollision(j.toVector())) {
                        // add this joint to the poseOnSphere of ik_data_result
                        pose_on_sphere_result.joints.push_back(j);
//                        ROS_INFO("Joint move done");
                        break;
                    }
                    else {
                        ROS_INFO("Joint move failed");
                    }
                }
            }
//            else {
//                // test the cartesian pose with plan
//                ROS_INFO("PointOnSphere has no joints");
//            }
            // add the poseOnSphere to the sphere
            sphere_result.add(pose_on_sphere_result);
        }
        // add the sphere to the ik_data_result
        this->ik_data_result.add(sphere_result);
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "data_generate");
    ros::NodeHandle n;
//    sleep 2 sec
    ros::Duration(2).sleep();
    ros::Time begin = ros::Time::now();
    ros::AsyncSpinner spinner(1);
    spinner.start();

    DataGenerator data_generator("/home/will/master_ik_data.json");
    data_generator.data_comparator();
    data_generator.ik_data_result.write_data("/home/will/master_ik_data_result.json");
    // get time
    ros::Time end = ros::Time::now();
    ros::Duration duration = end - begin;
    ROS_INFO("Total time taken: %f", duration.toSec());
    ROS_INFO("Data generation done");
    return 0;
}

