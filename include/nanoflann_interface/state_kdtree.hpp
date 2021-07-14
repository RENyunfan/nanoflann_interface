//
// Created by yunfan on 2021/7/11.
//

#ifndef STATE_KDTREE_HPP
#define STATE_KDTREE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include "nanoflann.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "memory"
#include "cstring"

using namespace std;
using namespace nanoflann;

typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 3, 1> Vec3;

struct TreeNode {
    Vec3 position;
    double cost;
    Vec3 nn_pt;
    double nn_dist;
    double nn_angle;
    size_t idx_on_tree;
    string name;
    // Piece
    // Trajectory
    typedef shared_ptr<TreeNode> Ptr;
};
typedef TreeNode::Ptr TreeNodePtr;

class StateVector{
public:
    StateVector(){}
    StateVector(int reserve_size){
        points.reserve(reserve_size);
    }
    typedef shared_ptr<StateVector> Ptr;

    vector<TreeNodePtr> points;

    inline void emplace_back(TreeNodePtr pt_in){
        points.emplace_back(pt_in);
    }
    inline void push_back(TreeNodePtr pt_in){
        points.push_back(pt_in);
    }
};
//typedef vector<TreeNodePtr> StateVector;

struct NanoTreeAdaptor {
    StateVector::Ptr pts;
    int valid_tree_num;
    NanoTreeAdaptor(){
        pts.reset(new StateVector);
    }
    NanoTreeAdaptor(int reserve_size){
        pts.reset(new StateVector(reserve_size));
    }
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts->points.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts->points[idx]->position.x();
        else if (dim == 1) return pts->points[idx]->position.y();
        else return pts->points[idx]->position.z();
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template<class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }

};

class StateKdTree {
private:
    NanoTreeAdaptor state_tree;
    int cur_idx = 0;
    int bound_x_, bound_y_, bound_z_;
    double search_radius_, resolution_;
    typedef KDTreeSingleIndexDynamicAdaptor<L2_Simple_Adaptor<float, NanoTreeAdaptor>, NanoTreeAdaptor, 3> nanoflann_kdtree;
    nanoflann_kdtree kdtree;
public:
//    StateKdTree(StateVector dataset, int bound_x, int bound_y, int bound_z)
//            : state_tree(dataset), bound_x_(bound_x), bound_y_(bound_y), bound_z_(bound_z),
//              kdtree(3 /*dim*/, state_tree, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)) {};
    StateKdTree():kdtree(3 /*dim*/, state_tree, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)) {};
    StateKdTree(int reserve_size):state_tree(reserve_size),kdtree(3 /*dim*/, state_tree, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)) {};
    ~StateKdTree() {};
    typedef shared_ptr<StateKdTree> Ptr;
    inline void AddStatePoint(TreeNodePtr new_pts) {
        new_pts->idx_on_tree = cur_idx;
        cur_idx+=1;
        InsertToMap(new_pts);
    }

    inline void reset(){
        cur_idx = 0;
        state_tree.pts->points.clear();
    }

    inline void DeletePoint(const size_t & idx) {
        kdtree.removePoint(idx);
    }

    inline void DeletePoint(const TreeNodePtr & node_in) {
        kdtree.removePoint(node_in->idx_on_tree);
    }

    inline vector<TreeNodePtr> KnnSearch(int K, TreeNodePtr search_pt) {
        size_t ret_index[K];
        float nn_dists[K];
        float query_pt[3];
        float out_dist_sqr[K];
        query_pt[0] = search_pt->position.x();
        query_pt[1] = search_pt->position.y();
        query_pt[2] = search_pt->position.z();
        nanoflann::KNNResultSet<float> resultSet(K);
        resultSet.init(ret_index, out_dist_sqr);
        kdtree.findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
        vector<TreeNodePtr> out_vec;

        for (int i = 0; i < K; i++) {
            out_vec.push_back(state_tree.pts->points[ret_index[i]]);
        }
        return out_vec;
    }

    inline vector<TreeNodePtr> RadiusSearch(float search_radius, TreeNodePtr search_pt) {
        std::vector<std::pair<size_t, float>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;
        float center[3], min_dist;
        center[0] = search_pt->position.x();
        center[1] = search_pt->position.y();
        center[2] = search_pt->position.z();
        vector<TreeNodePtr> out_vec;
        RadiusResultSet<float, size_t> resultSet(search_radius, ret_matches);
        kdtree.findNeighbors(resultSet, center, params);
        for (int i = 0; i < ret_matches.size(); i++) {
            out_vec.push_back(state_tree.pts->points[ret_matches[i].first]);
        }
        return out_vec;
    }


private:
    inline void InsertToMap(TreeNodePtr point) {
        size_t N;
        state_tree.pts->push_back(point);
        N = state_tree.kdtree_get_point_count();
        printf("kd_tree num: %d\n",N);
        kdtree.addPoints(N - 1, N - 1);
    }


};

#endif //SRC_NANOFLANN_INTERFACE_H
