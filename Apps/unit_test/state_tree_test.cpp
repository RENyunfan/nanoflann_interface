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

#include "nanoflann_interface/state_kdtree.hpp"
#include "nanoflann_interface/msg_utils.hpp"
#include <ros/ros.h>
using namespace std;
using namespace Eigen;

StateKdTree::Ptr kd_tree_ptr;

int main(int argc, char **argv) {
    ros::init(argc, argv, "fan_grader");
    ros::NodeHandle nh("~");

    int reserve_size = 6000;
    kd_tree_ptr.reset(new StateKdTree(reserve_size));

    TreeNodePtr new_pt(new TreeNode);
    new_pt->position = Vec3(1,2,1);
    new_pt->name="No.1";
    kd_tree_ptr->AddStatePoint(new_pt);

    TreeNodePtr new_pt2(new TreeNode);
    new_pt2->position = Vec3(1,2,2);
    new_pt2->name="No.2";
    kd_tree_ptr->AddStatePoint(new_pt2);

    TreeNodePtr search_pt(new TreeNode);
    search_pt->position = Vec3(1,2,3);
    search_pt->name="Search";
    vector<TreeNodePtr> out;
    out = kd_tree_ptr->RadiusSearch(5,search_pt);
    cout<<out.size()<<endl;
    for(int i = 0 ; i < out.size() ; i++){
        printf("Knn seach namr: %s \n", out[i]->name.c_str());
    }

    kd_tree_ptr->DeletePoint(out[0]);

    out = kd_tree_ptr->RadiusSearch(5,search_pt);
    cout<<out.size()<<endl;
    for(int i = 0 ; i < out.size() ; i++){
        printf("Knn seach namr: %s \n", out[i]->name.c_str());
    }

    kd_tree_ptr->AddStatePoint(new_pt);
    out = kd_tree_ptr->RadiusSearch(5,search_pt);
    cout<<out.size()<<endl;
    for(int i = 0 ; i < out.size() ; i++){
        printf("Knn seach namr: %s \n", out[i]->name.c_str());
    }

    kd_tree_ptr.reset(new StateKdTree);
    cout<<"--------------------------------"<<endl;

    TreeNodePtr new_pt3(new TreeNode);
    new_pt3->position = Vec3(1,2,1);
    new_pt3->name="No.3";
    kd_tree_ptr->AddStatePoint(new_pt3);

    TreeNodePtr new_pt4(new TreeNode);
    new_pt4->position = Vec3(1,2,2);
    new_pt4->name="No.4";
    kd_tree_ptr->AddStatePoint(new_pt4);


    search_pt->position = Vec3(1,2,3);
    search_pt->name="Search";

    out = kd_tree_ptr->RadiusSearch(8,search_pt);
    cout<<out.size()<<endl;
    for(int i = 0 ; i < out.size() ; i++){
        printf("Knn seach namr: %s \n", out[i]->name.c_str());
    }

    kd_tree_ptr->DeletePoint(out[0]);

    out = kd_tree_ptr->RadiusSearch(5,search_pt);
    cout<<out.size()<<endl;
    for(int i = 0 ; i < out.size() ; i++){
        printf("Knn seach namr: %s \n", out[i]->name.c_str());
    }

    ros::Duration(0.1).sleep();
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();
    return 0;
}
