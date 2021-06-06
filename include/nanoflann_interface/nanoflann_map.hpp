
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
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
template<typename T>
struct nanoflann_PointCloud {
    typename pcl::PointCloud<T>::Ptr pts;

    nanoflann_PointCloud(typename pcl::PointCloud<T>::Ptr clouds) {
        pts = clouds;
    }

    // nanoflann_PointCloud (pcl::PointCloud<T>::Ptr clouds){
    //     pts->clear;
    //     pts = clouds;
    // }
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts->points.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts->points[idx].x;
        else if (dim == 1) return pts->points[idx].y;
        else return pts->points[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template<class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }

};


template<typename T>
class nanoflann_map {
private:
    nanoflann_PointCloud<T> cloud;
    vector<bool> PointIsOnTree;
    vector<bool> DownsampleDeleteFromTree;
    vector<int> *PointIndices;
    typedef KDTreeSingleIndexDynamicAdaptor<L2_Simple_Adaptor<float, nanoflann_PointCloud<T>>, nanoflann_PointCloud<T>, 3> nanoflann_kdtree;
    nanoflann_kdtree kdtree;
    typedef vector<T, Eigen::aligned_allocator<T>> PointVec;
    int Depth, Width, Height;
    float resolution = 0.5;
    float search_radius, Len;

    bool check(float center[3], T point) {
        return (fabs(center[0] - point.x) <= resolution / 2 && fabs(center[1] - point.y) <= resolution / 2 &&
                fabs(center[2] - point.z) <= resolution / 2);
    }

    float calc_dist(float center[3], T point) {
        return ((center[0] - point.x) * (center[0] - point.x) + (center[1] - point.y) * (center[1] - point.y) +
                (center[2] - point.z) * (center[2] - point.z));
    }

    int GetCubeIndex(T point) {
        int i, j, k;
        i = floor((point.x + Len * Depth / 2.0 + Len / 2.0) / Len);
        j = floor((point.y + Len * Width / 2.0 + Len / 2.0) / Len);
        k = floor((point.z + Len * Height / 2.0 + Len / 2.0) / Len);\
            return (i + Width * j + Width * Height * k);
    }

    void InsertToMap(T point) {
        size_t N;
        int cube_idx = GetCubeIndex(point);
        cloud.pts->points.push_back(point);
        PointIsOnTree.push_back(true);
        DownsampleDeleteFromTree.push_back(false);
        N = cloud.kdtree_get_point_count();
        PointIndices[cube_idx].push_back(N - 1);
        kdtree.addPoints(N - 1, N - 1);
    }

public:
    typedef shared_ptr<nanoflann_map<T>> Ptr;

//    nanoflann_map();

    nanoflann_map(typename pcl::PointCloud<T>::Ptr dataset, int laserCloudDepth, int laserCloudWidth,
                  int laserCloudHeight) : cloud(dataset),
                                          kdtree(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)) {
        search_radius = 1.73205080757f * (resolution / 2.0);
        Depth = laserCloudDepth;
        Width = laserCloudWidth;
        Height = laserCloudHeight;
        PointIndices = new vector<int>[laserCloudDepth * laserCloudWidth * laserCloudHeight];
    }

    ~nanoflann_map() {
        cloud.pts->points.clear();
    }
//
//    inline void init(typename pcl::PointCloud<T>::Ptr dataset, int laserCloudDepth,
//                     int laserCloudWidth, int laserCloudHeight) {
//        cloud = dataset;
//        search_radius = 1.73205080757f * (resolution / 2.0);
//        Depth = laserCloudDepth;
//        Width = laserCloudWidth;
//        Height = laserCloudHeight;
//        PointIndices = new vector<int>[laserCloudDepth * laserCloudWidth * laserCloudHeight];
//    }

    bool empty() {
        return (cloud.pts->points.empty());
    }

    int size() {
        int s = kdtree.TreeSize();
        return (s);
    }

    void SetCubeLen(float CubeLength) {
        Len = CubeLength;
    }

    void SetResolutionLen(float r) {
        resolution = r;
    }

    inline void AddPointsFromPC2(const sensor_msgs::PointCloud2ConstPtr & msg){
        typename pcl::PointCloud<T>::Ptr cloud_in(new typename pcl::PointCloud<T>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud_in);
//        cout<<"Input cloud: "<<cloud_in->size()<<endl;
        AddPoints(cloud_in);

    }

    void AddPoints(typename pcl::PointCloud<T>::Ptr newpts) {
        std::vector<std::pair<size_t, float>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;
        float center[3], min_dist;
        bool flag = false;
        size_t best_idx;
        for (int i = 0; i < newpts->points.size(); i++) {
            // Downsample + Add Points;
            ret_matches.clear();
            center[0] = floor(newpts->points[i].x / resolution) * resolution + resolution / 2.0f;
            center[1] = floor(newpts->points[i].y / resolution) * resolution + resolution / 2.0f;
            center[2] = floor(newpts->points[i].z / resolution) * resolution + resolution / 2.0f;
            // printf("Original Point: (%f,%f,%f)\n",newpts->points[i].x,newpts->points[i].y,newpts->points[i].z);
            // printf("Downsample Box Center: (%f,%f,%f)\n",center[0],center[1],center[2]);
            RadiusResultSet<float, size_t> resultSet(search_radius, ret_matches);
            kdtree.findNeighbors(resultSet, center, params);
            if (ret_matches.size() == 0) InsertToMap(newpts->points[i]); //
            else {
                // printf("ret_matches size: %d\n",int(ret_matches.size()));
                best_idx = -1;
                min_dist = INFINITY;
                for (int j = 0; j < ret_matches.size(); j++) {
                    if (ret_matches[j].second < min_dist && check(center, cloud.pts->points[ret_matches[j].first])) {
                        best_idx = ret_matches[j].first;
                        min_dist = ret_matches[j].second;
                    }
                    // T match_pts = cloud.pts->points[ret_matches[j].first];
                    // float match_dist = ret_matches[j].second;
                    // printf("Match Points %d: (%f,%f,%f),dist is %f\n",j,match_pts.x,match_pts.y,match_pts.z,match_dist);
                }
                if (best_idx != -1) {
                    for (int j = 0; j < ret_matches.size(); j++) {
                        if (ret_matches[j].first != best_idx &&
                            check(center, cloud.pts->points[ret_matches[j].first])) {
                            kdtree.removePoint(ret_matches[j].first);
                            PointIsOnTree[ret_matches[j].first] = false;
                            DownsampleDeleteFromTree[ret_matches[j].first] = true;
                        }
                    }
                    float dist_a = calc_dist(center, cloud.pts->points[best_idx]);
                    float dist_b = calc_dist(center, newpts->points[i]);
                    // printf("Dist Cmp: old - %f, new - %f\n",dist_a, dist_b);
                    if (dist_b < dist_a) {
                        kdtree.removePoint(best_idx);
                        PointIsOnTree[best_idx] = false;
                        DownsampleDeleteFromTree[best_idx] = true;
                        InsertToMap(newpts->points[i]);
                        // printf("Downsample Replaced;\n");
                    }
                } else {
                    InsertToMap(newpts->points[i]);
                }
            }

        }
        return;
    }

    void DeletePoints(vector<size_t> indices) {
        for (int i = 0; i < indices.size(); i++) {
            kdtree.removePoint(indices[i]);
        }
    }

    void DeleteCubeFromMap(int cube_idx) {
        int i, j, point_idx;
        for (j = 0; j < PointIndices[cube_idx].size(); j++) {
            point_idx = PointIndices[cube_idx][j];
            if (!PointIsOnTree[point_idx]) continue;
            kdtree.removePoint(point_idx);
            PointIsOnTree[point_idx] = false;
        }
    }

    void ReaddCubeToMap(int cube_idx) {
        int i, j, point_idx;
        for (j = 0; j < PointIndices[cube_idx].size(); j++) {
            point_idx = PointIndices[cube_idx][j];
            if (PointIsOnTree[point_idx] || DownsampleDeleteFromTree[point_idx]) continue;
            kdtree.addPoints(point_idx, point_idx);
            PointIsOnTree[point_idx] = true;
        }
    }

    void NearestSearch(T target, size_t k, PointVec &points_near, vector<float> &dist_square) {
        size_t ret_index[k];
        float out_dist_sqr[k];
        float query_pt[3];
        query_pt[0] = target.x;
        query_pt[1] = target.y;
        query_pt[2] = target.z;
        nanoflann::KNNResultSet<float> resultSet(k);
        resultSet.init(ret_index, out_dist_sqr);
        kdtree.findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
        points_near.clear();
        dist_square.clear();
        for (int i = 0; i < k; i++) {
            points_near.push_back(cloud.pts->points[ret_index[i]]);
            dist_square.push_back(out_dist_sqr[i]);
        }
    }
    inline void GetNNPoint(Vec3 pt_in, Vec3& pt_out, float &dis){
        size_t ret_index[1] ;
        float out_dist_sqr[1];
        float query_pt[3];
        query_pt[0] = pt_in.x();
        query_pt[1] = pt_in.y();
        query_pt[2] = pt_in.z();
        nanoflann::KNNResultSet<float> resultSet(1);
        resultSet.init(ret_index, out_dist_sqr);
        kdtree.findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
        pt_out.x() = cloud.pts->points[ret_index[0]].x;
        pt_out.y() = cloud.pts->points[ret_index[0]].y;
        pt_out.z() = cloud.pts->points[ret_index[0]].z;
        dis = out_dist_sqr[0];
    }
    inline void GetKNNPoints(Vec3 pt_in, int num,vector<Vec3>& nn_points, vector<float>& nn_dis){

        size_t ret_index[num];
        float out_dist_sqr[num];
        float query_pt[3];
        query_pt[0] = pt_in.x();
        query_pt[1] = pt_in.y();
        query_pt[2] = pt_in.z();
        nanoflann::KNNResultSet<float> resultSet(num);
        resultSet.init(ret_index, out_dist_sqr);
        kdtree.findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
        nn_points.clear();
        nn_dis.clear();
        Vec3 cur_pts;
        for (int i = 0; i < num; i++) {

            cur_pts.x() = cloud.pts->points[ret_index[i]].x;
            cur_pts.y() = cloud.pts->points[ret_index[i]].y;
            cur_pts.z() = cloud.pts->points[ret_index[i]].z;
            nn_points.push_back(cur_pts);
            nn_dis.push_back(out_dist_sqr[i]);
        }
//
//        T tar_;
//        tar_.x = pt_in.x();tar_.y = pt_in.y();tar_.z = pt_in.z();
//        PointVec nn_pts;
//        NearestSearch(tar_,num,nn_pts,nn_dis);
//        nn_points.clear();
//        for(size_t i = 0 ; i <  nn_pts.size() ; i ++){
//            Vec3 cur_pos;
//            cur_pos.x() =  nn_pts[i].x;
//            cur_pos.y() =  nn_pts[i].y;
//            cur_pos.z() =  nn_pts[i].z;
//            nn_points.push_back(cur_pos);
//        }
    }

    inline Vec3 GetNNPoint(Vec3 pt_in){
        Vec3 pt_out;float dis;
        GetNNPoint(pt_in,pt_out,dis);
        return pt_out;
    }
  {
        Vec3 pt_out;float dis;  inline float GetNNDis(Vec3 pt_in)
        GetNNPoint(pt_in,pt_out,dis);
        return dis;
    }
};