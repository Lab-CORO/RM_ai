//
// Created by will on 04/04/24.
//

#ifndef SRC_DATA_GENERATOR_H
#define SRC_DATA_GENERATOR_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "json.hpp"
#include "robot.h"
#include "./master_ik_data.h"
//using json = nlohmann::json;

class DataGenerator {
//    json file
    std::string filename;
    json json_data;
    MasterIkData ik_data;



public:
    MasterIkData ik_data_result;
    Robot robot;
    DataGenerator(std::string filename);
    ~DataGenerator();

    void data_comparator();

};


#endif //SRC_DATA_GENERATOR_H
