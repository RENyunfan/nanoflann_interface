# Nanoflann-interface


> Please contact me if you have any questions
>
> Email: renyunfan@outlook.com

In this project, we provide two [nanoflann](https://github.com/jlblancoc/nanoflann)-based KD-Tree structure for motion planning and point cloud processing algorithms. This package is just a example package, you only need to include a few header files to apply them to your own projects.

* `nanoflann.hpp`: Nanoflann's source code.
* `pointcloud_kdtree.hpp`: A point cloud type interface for PCL and ROS.
* `state_kdtree.hpp`: A user-customizable KD-Tree data structure.

# 1 PC-KD-Tree

The typical application of `PcKdTree`  is storage environment point cloud. It contained point cloud down-sampling, adds points, deletes points, nearest neighbor search, and other APIs.

First you need to set the environmental resolution and the map size.

```cpp
const int laserCloudWidth = 200;
const int laserCloudHeight = 200;
const int laserCloudDepth = 200;
```

Then define the kd-tree with shared_ptr

```cpp
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PcKdTree<PointType>::Ptr localmap;
localmap.reset(new PcKdTree<PointType>(featsFromMap,laserCloudDepth,laserCloudWidth,laserCloudHeight));
```



# 2 StateKdTree

This class is new for Node state search using in RRT like algorithm.

First define your datastructure in `state_kdtree.hpp`

```cpp
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
```



# 1 Simple useage

1）首先包含头文件（需要拷贝两个文件）

* nanoflann_interface/nanoflann_map.hpp
* nanoflann_interface/nanoflann.hpp

```c++
#include "nanoflann_interface/nanoflann_map.hpp"
```

2）创建共享指针

```c++

nanoflann_map<PointType>::Ptr localmap;
```

3）分配内存并初始化，参数意义分别如下，涉及到距离单位都是m

* Creat 点云尺寸，表示总共有多少个cube
  * laserCloudWidth
  * laserCloudHeight
  * laserCloudDepth
* 点云cube的边长
  * SetCubeLen
  * 所以地图大小等于cubelen * cloudsize
* 点云的分辨率
  * SetResolutionLen

```cpp
const int laserCloudWidth = 200;
const int laserCloudHeight = 200;
const int laserCloudDepth = 200;
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
localmap.reset(new nanoflann_map<PointType>(featsFromMap,laserCloudDepth,laserCloudWidth,laserCloudHeight));
localmap->SetCubeLen(0.1);
localmap->SetResolutionLen(0.1);
```

4）加入点云

```cpp
// 该方法为直接从sensor_msgs加入到点云地图
void LocalCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (tree_has_init) {
        return;
    }
    localmap->AddPointsFromPC2(msg);
    tree_has_init = true;
}
```

5）最近邻搜索

```cpp
/*
 * 1) KNN搜索
 *	[搜索点，搜索个数，搜索点输出vector，搜索点距离vector] 
 *	其中后两个参数为引用
 */

inline void GetKNNPoints(Vec3 pt_in, int num,vector<Vec3>& nn_points, vector<float>& nn_dis)

/*
 * 2) 最近邻搜索
 *	 [搜索点，近邻点，近邻距离] 
 *	 其中后两个参数为引用
 */    
     inline void GetKNNPoints(Vec3 pt_in, int num,vector<Vec3>& nn_points, vector<float>& nn_dis)
    
  /*
 * 3) 最近点位置
 *	 [搜索点] 
 */ 
    inline Vec3 GetNNPoint(Vec3 pt_in);

  /*
 * 4) 最近点距离
 *	 [搜索点] 
 */ 
  inline float GetNNDis(Vec3 pt_in)
```

# 2 Acknowledge

感谢[Ecstasy-EC](https://github.com/Ecstasy-EC)提供的代码模板