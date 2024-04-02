//
// Created by will on 29/03/24.
//

#ifndef SRC_MASTER_IK_DATA_H
#define SRC_MASTER_IK_DATA_H

//#include <jsoncpp/json/json.h>
#include "json.hpp"
using json = nlohmann::json;
#include <iostream>
#include <fstream>




typedef struct joint {
    double j1;
    double j2;
    double j3;
    double j4;
    double j5;
    double j6;
} joint;


class pose {

public:
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    std::vector<joint> joints;

    void to_json(json &j);
};
class sphere {

public:
    double x;
    double y;
    double z;
    std::vector<pose> poses;

    void to_json(json &j);
};


class master_ik_data {
/***
 * This class is used to store the data for the master_ik node
 * the first object is compose of a sphere with x,y,z. each sphere is compose of poses (x,y,z, roll, pitch, yaw) and each pose is compose of a vector of 6 doubles
 * A methode save the data in a file json
 * A methode load the data from a file json
 */
public:
    double resolution;
    double radius;
    int sphere_sample;
    std::vector<sphere> spheres;
    json json_data;

    void write_data(std::string filename);
    void load_data(json &j);
};


#endif //SRC_MASTER_IK_DATA_H
