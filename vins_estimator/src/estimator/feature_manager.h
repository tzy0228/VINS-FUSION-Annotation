/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

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
#include "../utility/tic_toc.h"

class FeaturePerFrame//一个特征点的属性
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
      //特征点归一化坐标
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        // 特征点投影到该帧相机坐标下的坐标
        uv.x() = _point(3);
        uv.y() = _point(4);
        // 特征点的速度
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 

        cur_td = td;//imu-camera的不同步时的相差时间
        is_stereo = false;
    }
    // 右目相机的参数，和上面一样
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;
    }
    double cur_td;

    Vector3d point, pointRight;//特征点归一化坐标
    Vector2d uv, uvRight;//特征点映射到该帧上的图像坐标
    Vector2d velocity, velocityRight;//特征点的跟踪速度
    bool is_stereo;
};

class FeaturePerId//管理一个特征点
{
  public:
    const int feature_id;//特征点id
    int start_frame;//第一次出现该特征点的帧号
    vector<FeaturePerFrame> feature_per_frame; // 该id对应的特征点在每个帧中的属性
    
    int used_num;//出现的次数
    double estimated_depth;//逆深度
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;该特征点的状态，是否被三角化


    // 构造函数，参数初始化
    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();//返回最后一个观测到这个特征点的图像帧ID
};

class FeatureManager//管理所有的特征点
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);
    void clearState();
    int getFeatureCount();//窗口中被跟踪的特征点数量
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);//特征点进入时检查视差，是否为关键帧
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);//前后两帧之间匹配特征点3D坐标
    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);//设置特征点逆深度
    void removeFailures();
    void clearDepth();
    VectorXd getDepthVector();
    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);//特征点三角化求深度（SVD分解）
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);

    // 边缘化最老帧时，处理特征点保存的帧号，将起始帧是最老帧的特征点的深度值进行转移                            
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();//边缘化最老帧，直接将特征点保存的帧号前移
    void removeFront(int frame_count);//边缘化次新帧，对特征点在次新帧的信息移除
    void removeOutlier(set<int> &outlierIndex);	//移除外点

    list<FeaturePerId> feature;//存储每一个特征点，及其对应的每帧的属性
    
    int last_track_num;// 被跟踪的个数
    double last_average_parallax;
    int new_feature_num;//新增特征点个数
    int long_track_num;//追踪到上一帧的特征点数目

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);//计算某个特征点it_per_id在次新帧和次次新帧的视差ans
    const Matrix3d *Rs;
    Matrix3d ric[2];//左右目ric（cam to imu的旋转）
};

#endif