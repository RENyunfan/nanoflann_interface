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



class NanoKdTree{
public:
    NanoKdTree(){};
    ~NanoKdTree(){};


private:

};

#endif //SRC_NANOFLANN_INTERFACE_H
