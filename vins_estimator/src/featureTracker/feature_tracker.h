/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);// 判断跟踪的特征点是否在图像边界内部

// 去除无法跟踪的特征点
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
    FeatureTracker();//构造函数
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());    //使用稀疏光流法进行追踪函数,包含了单目双目的处理情况
    void setMask();
    void readIntrinsicParameter(const vector<string> &calib_file);    //读取相机的内参
    void showUndistortion(const string &name);
    void rejectWithF();//使用FM矩阵进行点的筛选,但是具体好像没有使用,使用了反追踪光流去筛选点
    void undistortedPoints();    //对获得点集去畸变
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    //计算特征点的速度,具体可以参考SLAM14讲的稀疏光流法
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    //绘制特征点,有一部分是点的数量决定颜色
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);    //设置预测点
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);    //两点的欧式距离
    void removeOutliers(set<int> &removePtsIds);    //去除外点
    cv::Mat getTrackImage();    // 返回的画好特征的图像
    bool inBorder(const cv::Point2f &pt);

    int row, col; //图像宽和高
    cv::Mat imTrack; 
    cv::Mat mask;   // 图像掩码
    cv::Mat fisheye_mask; // 鱼眼相机mash,用于去除边缘的噪点
    cv::Mat prev_img, cur_img;    // prev_img是上一次发布的帧的图像数据 // cur_img是光流跟踪的前一帧的图像数据
    vector<cv::Point2f> n_pts;    // 每一帧中新提取的特征点
    vector<cv::Point2f> predict_pts;    // 预测的特征点
    vector<cv::Point2f> predict_pts_debug;    // 预测的特征点的debug
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;    // 前帧点,当前帧特征点,和当前右图点
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;    // 去畸变的前帧点,当前帧点,和当前右图点
    vector<cv::Point2f> pts_velocity, right_pts_velocity;    // 特征点的速度,
    vector<int> ids, ids_right;    // 特征点的id
    vector<int> track_cnt;    // 追踪到点的数量
     
    // 这几个哈希表在去畸变的时候用到了,但是没有找到填充数据的地方,是空映射有点疑惑
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;

    vector<camodocal::CameraPtr> m_camera;    // 相机参数和工具
    double cur_time;// 当前帧时间
    double prev_time;    // 前一帧时间
    bool stereo_cam;// 是否双目的标志位
    int n_id;    // 帧的id
    bool hasPrediction; // 是否有预测点信息的标志位
};
