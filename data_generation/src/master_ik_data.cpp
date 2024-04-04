//
// Created by will on 29/03/24.
//

#include "../include/master_ik_data.h"


void MasterIkData::write_data(std::string filename) {
    json j;
    j["resolution"] = resolution;
    j["radius"] = radius;
    j["sphere_sample"] = sphere_sample;
    // add spheres in the json
    for (int i = 0; i < this->spheres.size(); i++) {
        spheres[i].to_json(j["spheres"][i]);
    }

    this->json_data = j;
    std::ofstream o(filename);
    o << j.dump(4) << std::endl;
}

void MasterIkData::load_data(json &j) {
    std::ifstream i("./master_ik_data.json");
    j = json::parse(i);
    resolution = j["resolution"];
    radius = j["radius"];
    sphere_sample = j["sphere_sample"];


    // load spheres from the json
}

void MasterIkData::add(Sphere &s) {
    this->spheres.push_back(s);
}

void PoseOnSphere::to_json(json &j) {

    j["x"] = this->x;
    j["y"] = this->y;
    j["z"] = this->z;
    j["theta_x"] = this->theta_x;
    j["theta_y"] = this->theta_y;
    j["theta_z"] = this->theta_z;
    j["theta_w"] = this->theta_w;
    // add all joints in the json
    for (int i = 0; i < this->joints.size(); i++) {
        j["joints"][i]["j1"] = this->joints[i].j1;
        j["joints"][i]["j2"] = this->joints[i].j2;
        j["joints"][i]["j3"] = this->joints[i].j3;
        j["joints"][i]["j4"] = this->joints[i].j4;
        j["joints"][i]["j5"] = this->joints[i].j5;
        j["joints"][i]["j6"] = this->joints[i].j6;
    }


}

void PoseOnSphere::add(std::vector<joint> &j) {
    for (int i = 0; i < j.size(); i++) {
        this->joints.push_back(j[i]);
    }
}

void Sphere::to_json(json &j) {
    j["x"] = this->x;
    j["y"] = this->y;
    j["z"] = this->z;
    // add all poses in the json
    for (int i = 0; i < this->poses.size(); i++) {
        this->poses[i].to_json(j["poses"][i]);
    }
}

void Sphere::add(PoseOnSphere &p) {
    this->poses.push_back(p);

}
/**
//main
int main() {
    MasterIkData data;
//    create 3 sphere with 3 poses each and 3 joints each
    for (int i = 0; i < 3; i++) {
        Sphere s;
        s.x = i;
        s.y = i;
        s.z = i;
        for (int j = 0; j < 3; j++) {
            PoseOnSphere p;
            p.x = j;
            p.y = j;
            p.z = j;
            p.theta_x = j;
            p.theta_y = j;
            p.theta_z = j;
            p.theta_w = j;
            for (int k = 0; k < 3; k++) {
                joint j;
                j.j1 = k;
                j.j2 = k;
                j.j3 = k;
                j.j4 = k;
                j.j5 = k;
                j.j6 = k;
                p.joints.push_back(j);
            }
            s.add(p);
        }
        data.add(s);
    }
    data.write_data("/home/will/master_ik_data.json");
    data.load_data(data.json_data);
//    print the data sphere 2 pose 1 joint 2
    std::cout << data.spheres[2].poses[1].joints[0].j1 << std::endl;


    return 0;
}**/

//   joint joint::array2joint(std::vector<double> array) {
//    /*
//     * This function is used to convert an float64[] to a joint object
//     */
//    for(int i = 0; i < 6; i++) {
//        this->j1 = array[0];
//        this->j2 = array[1];
//        this->j3 = array[2];
//        this->j4 = array[3];
//        this->j5 = array[4];
//        this->j6 = array[5];
//    }
//    return *this;
//}


std::vector<double> joint::joint2array() {
    /*
     * This function is used to convert a joint object to a float64[]
     */
    std::vector<double> array;
    array.push_back(this->j1);
    array.push_back(this->j2);
    array.push_back(this->j3);
    array.push_back(this->j4);
    array.push_back(this->j5);
    array.push_back(this->j6);
    return array;
}
