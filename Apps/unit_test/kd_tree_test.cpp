//
// Created by yunfan on 2021/6/4.
//

/*
 *  This file is a part of FOV Planner
 *
 *  该文件主要测试用于RRT的简易kd-tree
 *
 *
 * */

#include "nanoflann_interface/nanoflann_map.hpp"
#include "nanoflann_interface/msg_utils.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;


const int laserCloudWidth = 200;
const int laserCloudHeight = 200;
const int laserCloudDepth = 200;
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
nanoflann_map<PointType>::Ptr localmap;
bool tree_has_init = false;

void LocalCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (tree_has_init) {
        return;
    }
    localmap->AddPointsFromPC2(msg);
    tree_has_init = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fan_grader");
    ros::NodeHandle nh("~");
    ros::Subscriber local_cloud_sub_ = nh.subscribe("/local_cloud/body", 1, LocalCloudCallback);
    ros::Publisher nn_pt_pub = nh.advertise<sensor_msgs::PointCloud2>("/nn_pts",1);
    localmap.reset(new nanoflann_map<PointType>(featsFromMap,laserCloudDepth,laserCloudWidth,laserCloudHeight));
    localmap->SetCubeLen(0.1);
    localmap->SetResolutionLen(0.1);
    ros::Duration(0.1).sleep();
    ros::AsyncSpinner spinner(0);
    spinner.start();

    vector<Vec3> nn_pts;
    vector<float> nn_dis;
    Vec3 nn_pt; float nn_d;
    while(ros::ok()){
        if(!tree_has_init){
            continue;
        }
        {
            TimeConsuming t__("nn_search",1e5);
            int cnt=1e5;
            while(cnt--)
                localmap->GetKNNPoints(Vec3(5,0,0),1,nn_pts,nn_dis);
        }
//        localmap->GetKNNPoints(Vec3(5,0,0),1,nn_pts,nn_dis);
        pcl::PointCloud<pcl::PointXYZ> cloud_;
        for (size_t i = 0; i < nn_pts.size(); i++) {
                Vec3 position = nn_pts[i];
                pcl::PointXYZ p_;
                p_.x = position.x();
                p_.y = position.y();
                p_.z = position.z();
                cloud_.points.push_back(p_);
        }
        sensor_msgs::PointCloud2 globalMap_pcd;
        pcl::toROSMsg(cloud_, globalMap_pcd);
        globalMap_pcd.header.frame_id = "body";
        globalMap_pcd.header.stamp = ros::Time::now();
        nn_pt_pub.publish(globalMap_pcd);
        ros::Duration(0.1).sleep();
    }
    ros::waitForShutdown();
    return 0;
}
