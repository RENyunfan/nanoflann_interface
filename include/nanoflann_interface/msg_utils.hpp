
#ifndef _MSG_UTILS_
#define _MSG_UTILS_
#include "Eigen/Eigen"
#include "cstring"
#include "tf/tf.h"
#include "chrono"
using namespace std;
typedef Eigen::Matrix3d Mat33;
typedef Eigen::Vector3d Vec3;

class MsgUtils {
public:
    static Mat33 poseToEigenRotation(geometry_msgs::Pose msg) {
        Mat33 R_eig;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.orientation, quat);
        tf::Matrix3x3 R = tf::Matrix3x3(quat);
        R_eig << R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2];
        return R_eig;
    }

    static Vec3 poseToEigenVec(geometry_msgs::Pose pose) {
        return Vec3(pose.position.x, pose.position.y, pose.position.z);
    }

    static double deg2rad(double deg){
        return deg*3.141592/180.0;
    }
};
class TimeConsuming{
public:
    TimeConsuming();
    TimeConsuming(string msg, int repeat_time){
        repeat_time_ =repeat_time;
        msg_ = msg;
        tc_start = std::chrono::high_resolution_clock::now();
    }
    TimeConsuming(string msg){
        msg_ = msg;
        repeat_time_ = 1;
        tc_start = std::chrono::high_resolution_clock::now();
    }
    ~TimeConsuming(){
        if(!has_shown){
            tc_end = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
//            ROS_WARN("%s time consuming %lf us.",msg_.c_str(),(double)(end_t - start_t).toNSec()/ 1e3);
            printf("%s time consuming \033[32m %lf us\033[0m\n",msg_.c_str(),(double)dt*1e6/repeat_time_);
        }

    }
    void start(){
        tc_start = std::chrono::high_resolution_clock::now();
    }

    void stop(){
        has_shown=true;
        tc_end = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
//            ROS_WARN("%s time consuming %lf us.",msg_.c_str(),(double)(end_t - start_t).toNSec()/ 1e3);
        printf("%s time consuming \033[32m %lf us\033[0m\n",msg_.c_str(),(double)dt*1e6/repeat_time_);
    }

private:
    std::chrono::high_resolution_clock::time_point tc_start, tc_end;
    string msg_;
    int repeat_time_;
    bool has_shown = false;
};

#endif //_MSG_UTILS_