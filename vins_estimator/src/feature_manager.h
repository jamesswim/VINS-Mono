#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include <std_msgs/Float64MultiArray.h>  
#include <limits>
#include <std_msgs/Header.h>

//指的是每帧基本的数据：特征点[x,y,z,u,v,vx,vy]和td：IMU与cam同步时间差
class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        // _point 每帧的特征点[x,y,z,u,v,vx,vy]和td IMU和cam同步时间差 
        point.x() = _point(0);  // 归一化坐标 
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3); // 像素坐标
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;  // IMU和相机的同步时间差
    }
    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z; // 特征点的深度
    bool is_used;
    double parallax;
    MatrixXd A; //变换矩阵
    VectorXd b;
    double dep_gradient;
};

//指的是某feature_id下的所有FeaturePerFrame，即能观测到该ID特征点的所有的帧。
//常用feature_id和观测第一帧start_frame、最后一帧endFrame()
class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;  //首次被观测到时，该帧的索引
    vector<FeaturePerFrame> feature_per_frame;  ////所有观测到该特征的图像帧的信息，包括图像坐标、特征点的跟踪速度、空间坐标等属性

    int used_num; //该特征出现的次数
    bool is_outlier;  //是否是外点
    bool is_margin; // 是否边缘化
    double estimated_depth; //逆深度  int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame(); // 返回最后一次观测到这个特征点的帧数ID
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();

    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td, ros::Publisher* parallax_pub, const std_msgs::Header& header);
    void debugShow();
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth(const VectorXd &x);
    VectorXd getDepthVector();
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();
    list<FeaturePerId> feature;
    int last_track_num;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};

#endif