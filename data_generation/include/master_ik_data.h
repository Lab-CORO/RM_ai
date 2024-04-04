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

//    joint array2joint(std::vector<double> array);
        std::vector<double> joint2array();

    } joint;


    class PoseOnSphere {

    public:
        double x;
        double y;
        double z;
        double theta_x;
        double theta_y;
        double theta_z;
        double theta_w;

        std::vector<joint> joints;

        void to_json(json &j);

        void add(std::vector<joint> &j);
    };

    class Sphere {

    public:
        double x;
        double y;
        double z;
        std::vector<PoseOnSphere> poses;

        void to_json(json &j);

        void add(PoseOnSphere &p);
    };


    class MasterIkData {
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
        std::vector<Sphere> spheres;
        json json_data;

        void write_data(std::string filename);

        void load_data(json &j);

        void add(Sphere &s);
    };


#endif //SRC_MASTER_IK_DATA_H
