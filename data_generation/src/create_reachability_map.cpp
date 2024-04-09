// The spheres and poses are fused in a single dataset, instead of two datasets for sphere and poses

#include "../include/sphere_discretization.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <iostream>
#include "moveit_msgs/GetPositionIK.h"
#include "../include/progressbar.hpp"
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>


#include "../include/master_ik_data.h"
#include "../include/robot.h"
//struct stat st;

typedef std::multimap<const std::vector<double> *, const std::vector<double> *> MultiMapPtr;
typedef std::map<const std::vector<double> *, double> MapVecDoublePtr;
typedef std::multimap<std::vector<double>, std::vector<double> > MultiMap;
typedef std::map<std::vector<double>, double> MapVecDouble;
typedef std::vector<std::vector<double> > VectorOfVectors;
struct stat st;
typedef std::vector<std::pair<std::vector<double>, const std::vector<double> *> > MultiVector;
//typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMap;




bool isFloat(std::string s) {
    std::istringstream iss(s);
    float dummy;
    iss >> std::noskipws >> dummy;
    return iss && iss.eof();  // Result converted to bool
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "workspace");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //create the robot
    Robot robot("manipulator");

    bool debug = false;

    ros::Time startit = ros::Time::now();
    float resolution = 0.3;  //previous 0.08
    static ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10, true);
    static ros::Publisher cube_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker_cube", 10, true);
    ros::Rate loop_rate(10);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = cloud.makeShared();

    int count = 0;
//    get time
    ros::Time begin = ros::Time::now();


    unsigned char max_depth = 16;
    unsigned char minDepth = 0;

    // A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of
    // specified resolution

    // TODO resolution will be user argument
    // The center of every voxels are stored in a vector

    sphere_discretization::SphereDiscretization sd;
    float r = 0.5;
    octomap::point3d origin = octomap::point3d(0, 0, 0);  // This point will be the base of the robot
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector<octomap::point3d> new_data;
    ROS_INFO("Creating the box and discretizing with resolution: %f", resolution);
    int sphere_count = 0;
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it) {
        sphere_count++;
    }
    new_data.reserve(sphere_count);
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it) {
        new_data.push_back(it.getCoordinate());
    }
    if (debug) {
//         show the octree to rviz
        visualization_msgs::Marker points;
        points.header.frame_id = "base_link";
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.01;
        points.scale.y = 0.01;
        points.color.g = 1.0f;
        points.color.a = 1.0;
        for (int i = 0; i < new_data.size(); i++) {
            geometry_msgs::Point p;
            p.x = new_data[i].x();
            p.y = new_data[i].y();
            p.z = new_data[i].z();
            points.points.push_back(p);
        }
        cube_pub.publish(points);
    }

    ros::Duration(10).sleep();

    ROS_INFO("Total no of spheres now: %lu", new_data.size());
    ROS_INFO("Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some "
             "time");

    // A sphere is created in every voxel. The sphere may be created by default or other techniques.
    // TODO Other techniques need to be modified. the user can specifiy which technique they want to use
    // TODO The sphere discretization parameter and rotation of every poses will be taken as argument. If the final
    // joints can rotate (0, 2pi) we dont need to rotate the poses.
    // Every discretized points on spheres are converted to pose and all the poses are saved in a multimap with their
    // corresponding sphere centers
    // If the resolution is 0.01 the programs not responds
    //TODO seperate raduise and resolutiion
    float radius = resolution / 10;

    VectorOfVectors sphere_coord;
    sphere_coord.resize(new_data.size());

    MultiVector pose_col;
    pose_col.reserve(new_data.size() * 50);

    MasterIkData data_ik;

    for (int i = 0; i < new_data.size(); i++) {
//        progressbar bar(new_data.size());
        static std::vector<geometry_msgs::Pose> pose;
        sd.convertPointToVector(new_data[i], sphere_coord[i]);
        // create a sphere in the master_ik_data
        Sphere sphere;
        sphere.x = sphere_coord[i][0];
        sphere.y = sphere_coord[i][1];
        sphere.z = sphere_coord[i][2];


        sd.make_sphere_poses(new_data[i], radius, pose);
        for (int j = 0; j < pose.size(); j++) {
            static std::vector<double> point_on_sphere;
            sd.convertPoseToVector(pose[j], point_on_sphere);
            pose_col.push_back(std::make_pair(point_on_sphere, &sphere_coord[i]));

            // create a pose in the sphere
            PoseOnSphere p;
            p.x = point_on_sphere[0];
            p.y = point_on_sphere[1];
            p.z = point_on_sphere[2];
            p.theta_x = point_on_sphere[3];
            p.theta_y = point_on_sphere[4];
            p.theta_z = point_on_sphere[5];
            p.theta_w = point_on_sphere[6];

            // call the ik_solvers
            std::vector<joint> joints;
            int solns;
            if (debug) {
                // rviz arrow marker for the point on the sphere
                visualization_msgs::Marker arrow;
                arrow.header.frame_id = "base_link";
                arrow.header.stamp = ros::Time::now();
                arrow.ns = "points_and_lines";
                arrow.action = visualization_msgs::Marker::ADD;
                arrow.pose.orientation.w = 1.0;
                arrow.id = 0;
                arrow.type = visualization_msgs::Marker::ARROW;
                arrow.scale.x = 0.05;
                arrow.scale.y = 0.01;
                arrow.scale.z = 0.01;
                arrow.color.b = 1.0f;
                arrow.color.a = 1.0;
                geometry_msgs::Pose pose_arrow;
                pose_arrow.position.x = point_on_sphere[0];
                pose_arrow.position.y = point_on_sphere[1];
                pose_arrow.position.z = point_on_sphere[2];
                pose_arrow.orientation.x = point_on_sphere[3];
                pose_arrow.orientation.y = point_on_sphere[4];
                pose_arrow.orientation.z = point_on_sphere[5];
                pose_arrow.orientation.w = point_on_sphere[6];
                arrow.pose = pose_arrow;

                marker_pub.publish(arrow);
            }
//            ros::Duration(0.1).sleep();
//            get time
//            ros::Time begin = ros::Time::now();
            bool does_have_ik = robot.get_all_ik(point_on_sphere, joints, solns);
//            ros::Time end = ros::Time::now();
//            ros::Duration duration = end - begin;
//            ROS_INFO("IK duration: %f", duration.toSec());

            if (does_have_ik) {
                // add the joints to the pose
                p.add(joints);
//                    ROS_INFO("Solution found");
            }
            // add the pose to the sphere
            sphere.add(p);
        }
        //  add the sphere to the master_ik
        data_ik.add(sphere);
//        bar.update();
    }

    // get time
    ros::Time end = ros::Time::now();
    ros::Duration duration = end - begin;
    ROS_INFO("Total time taken: %f", duration.toSec());
    // Write the data to json
    data_ik.write_data("/home/will/master_ik_data.json");
    ROS_INFO("fini !");

}