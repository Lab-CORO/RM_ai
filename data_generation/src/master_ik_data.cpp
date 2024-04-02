//
// Created by will on 29/03/24.
//

#include "../include/master_ik_data.h"

void master_ik_data::write_data(std::string filename) {
    json j;
    j["resolution"] = resolution;
    j["radius"] = radius;
    j["sphere_sample"] = sphere_sample;
    // add spheres in the json
    for (int i = 0; i < this->spheres.size(); i++) {
        spheres[i].to_json(j["spheres"][i]);
    }

    this->json_data = j;
    std::ofstream o("master_ik_data.json");
    o << j << std::endl;
}

void master_ik_data::load_data(json &j) {
    std::ifstream i("./master_ik_data.json");
    j = json::parse(i);
    resolution = j["resolution"];
    radius = j["radius"];
    sphere_sample = j["sphere_sample"];


    // load spheres from the json
}

void pose::to_json(json &j) {

    j["x"] = x;
    j["y"] = y;
    j["z"] = z;
    j["roll"] = roll;
    j["pitch"] = pitch;
    j["yaw"] = yaw;
    // add all joints in the json
    for (int i = 0; i < this->joints.size(); i++) {
        j["joints"][i]["j1"] = joints[i].j1;
        j["joints"][i]["j2"] = joints[i].j2;
        j["joints"][i]["j3"] = joints[i].j3;
        j["joints"][i]["j4"] = joints[i].j4;
        j["joints"][i]["j5"] = joints[i].j5;
        j["joints"][i]["j6"] = joints[i].j6;
    }


}

void sphere::to_json(json &j) {
    j["x"] = x;
    j["y"] = y;
    j["z"] = z;
    // add all poses in the json
    for (int i = 0; i < this->poses.size(); i++) {
        poses[i].to_json(j["poses"][i]);
    }
}

//main
int main() {
    master_ik_data data;
//    create 3 sphere with 3 poses each and 3 joints each
    for (int i = 0; i < 3; i++) {
        sphere s;
        s.x = i;
        s.y = i;
        s.z = i;
        for (int j = 0; j < 3; j++) {
            pose p;
            p.x = j;
            p.y = j;
            p.z = j;
            p.roll = j;
            p.pitch = j;
            p.yaw = j;
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
            s.poses.push_back(p);
        }
        data.spheres.push_back(s);
    }
    data.write_data();
    data.load_data(data.json_data);
//    print the data sphere 2 pose 1 joint 2
    std::cout << data.spheres[2].poses[1].joints[0].j1 << std::endl;


    return 0;
}