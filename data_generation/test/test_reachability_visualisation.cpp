//
// Created by will on 09/04/24.
//
#include "../include/reachability_visualisation.h"
#include <ros/ros.h>



int main(int argc, char **argv) {
    ros::init(argc, argv, "data_generate");
    ros::NodeHandle n;
    std::string filename = "/home/will/master_ik_data_result.json";
    ReachabilityVisualisation reachabilityVisualisation;
    reachabilityVisualisation.show_map(filename);
    return 0;
//ros::spin();
}