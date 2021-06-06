# Nanoflann-interface

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

感谢EC提供的代码模板[Ecstasy-EC](https://github.com/Ecstasy-EC)