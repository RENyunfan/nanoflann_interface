//
// Created by yunfan on 2021/6/5.
//

#ifndef SRC_NANOFLANN_INTERFACE_H
#define SRC_NANOFLANN_INTERFACE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include "nanoflann.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "memory"
using namespace std;
using namespace nanoflann;

typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 3, 1> Vec3;

struct TreeNode{
    Vec3 position;
    int valid_tree_num;
    TreeNode(int cur_number){
        valid_tree_num = cur_number;
    }
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return valid_tree_num; }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return position.x();
        else if (dim == 1) return position.y();
        else return position.z();
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template<class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }

};

class NanoKdTree{
public:
    NanoKdTree(){};
    ~NanoKdTree(){};


private:

};

#endif //SRC_NANOFLANN_INTERFACE_H
