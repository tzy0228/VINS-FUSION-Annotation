/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"


class Estimator
{
  public:
    Estimator(); // 构造函数
    ~Estimator(); // 析构函数

    void setParameter();//开启了滑动窗口估计的一个新线程

    // interface接口函数
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);//初始化初始位姿
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);//输入IMU数据
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);//输入特征
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());//输入图像
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);//处理IMU数据，对IMU进行预积分；
    //    处理相机数据；
    // 1. 基于特征点的视差来判断当前帧是否属于关键帧；
    // 2. 判断相机到IMU的外参是否有校正，若无则用手眼标定法进行标定,具体在CalibrationExRotation里，此处只标定旋转矩阵，未标定平移矩阵，原因是系统对旋转矩阵较敏感，系统易因为小幅度的角度偏差而崩溃；
    // 3. 判断是否有进行初始化;若已完成初始化，则调用optimization( )，用ceres_solver对滑窗进行非线性优化的求解，优化项主要有四项：边缘化残差、 imu残差、相机重投影残差以及相机与Imu间同步时间差的残差项。否则进行相应的初始化过程。
    // 4. 本函数中包含一个failureDetection()函数,用于判断系统在一定条件下是否崩溃，比如非线性求解器中的解有大跳动，求解出相机IMU的外参矩阵或IMU偏移等等，系统挂掉就清空状态，重新初始化。
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);

    // 处理测量值；处理各buffer里的数据，当featureBuf不等于空时，开始进行以下处理（为什么是featureBuf，因为当有图像buffer数据的时候，才会有featuretracker.push(make_pair(t,featureFrame))，
    // 即有图像数据后，程序才发给跟踪器叫他产生feature，因此当featureBuf不等于空，所有的buffer，包括imu,图像，都不为空）：
    void processMeasurements();
    
    void changeSensorType(int use_imu, int use_stereo);//改变传感器类型，用于确定是否使用IMU，使用单目相机还是双目相机
    
    // internal

    // 清空估计器状态. 主要需要做的操作有：清空buff缓冲区；设置时间、初始化位姿；把滑动窗口中的数据都清空（删除预积分指针）；清空tic、ric；特征管理器。
    void clearState();
   
    // 初始化结构，SFM利用SFM进行初始化；视觉的结构初始化，首先得到纯视觉的,所有图像在IMU坐标系下的,一个初始化结果,也就是RT然后进行视觉imu对其,陀螺仪偏执估计等等如果IMU的方差小于0.25，则输出IMU激励过小（返回失败）
    bool initialStructure();
   
    // 视觉初始化对齐  :1. 求取scale尺度  2. 把所有图像帧都设置为关键帧  3.三角化
    bool visualInitialAlign();
    
    // 相对位姿;查找到与最新帧中包含足够的对应点对和视差的关键帧
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    
    // 滑动窗口函数
    void slideWindow();
    
    // 滑到倒数第二帧,作用主要是删除特征点
    void slideWindowNew();
    
    // 滑掉最老的那一帧,,作用主要是删除特征点
    void slideWindowOld();
    
    // 后端优化函数 基于滑动窗口的紧耦合的非线性优化，残差项的构造和求解
    void optimization();
    
    // 把滑动窗口向量转为double数据；主要包括：1. 位置；2、姿态；3、IMU速度4、加速度偏差；5、角速度偏差；
    void vector2double();
    
    void double2vector();//double转向量

    // 系统失败检测；发生以下情况则认定为失败:1. 检测数目<2;2. IMU的加速度偏差>2.5；3. IMU的角速度偏差大于1.0；4. 最新帧与上一帧的平移量P的模>5；5. 最新帧与上一帧z值变化>1;
    bool failureDetection();
    
    // 对imu的时间进行判断，将队列里的imu数据放入到accVector和gyrVector中，完成之后返回true
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    
    void getPoseInWorldFrame(Eigen::Matrix4d &T);//获得世界坐标系下的位置
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);//获得世界坐标系下的位置
    
    void predictPtsInNextFrame();//对特征跟踪的预测

    void outliersRejection(set<int> &removeIndex);//外点移除函数
    
    // 计算重投影误差；    输入：第i与第j帧之间的像素点位置，第i帧像素点的深度，相机模型转换矩阵；用head提取前两个元素以计算残差；
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    
    // 更新估计器的最新状态,  主要是利用滑动窗口中的最近帧的数据来更新最新的time,P,Q,V,Ba,Bg,acc_0,gyr_0；tmp_accBuf,tmp_gyrBuf,acc,gyr;再次调用IMU的数据快速预测状态；
    void updateLatestStates();
    
    // 快速预测IMU；利用IMU数据更新Latest_P，Latest_V，Latest_acc_0,Latest_gyr_0;
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    
    bool IMUAvailable(double t);// 判断输入的时间t时候的imu是否可用 

    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);//初始第一个imu位姿:就是求一个姿态角,然后把航向角设为0


    enum SolverFlag// VINS系统的两种状态：
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,// 还未成功初始化
        MARGIN_SECOND_NEW = 1 // 已成功初始化，正处于紧耦合优化状态
    };

    std::mutex mProcess;//进程互斥信号
    std::mutex mBuf;//缓存空间互斥信号
    std::mutex mPropagate;//传播的互斥访问信号
    queue<pair<double, Eigen::Vector3d>> accBuf;//加速度计数据缓冲区
    queue<pair<double, Eigen::Vector3d>> gyrBuf;//陀螺仪数据缓冲区
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;//特征点（7维）数据缓冲区
    double prevTime, curTime;//上一帧时间戳,当前帧时间戳
    bool openExEstimation;

    std::thread trackThread;//追踪线程
    std::thread processThread;//处理线程

    // 特征追踪器 用来对原始图像进行畸变校正，特征点采集，光流跟踪
    FeatureTracker featureTracker;

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;//重力加速度在各个方向上的分量；最开始由参数设置函数确定，在初始化过程中还要进行后续优化

    Matrix3d ric[2];
    Vector3d tic[2];

    // 滑动窗口中的数据；位置、速度、旋转矩阵、加速度偏差、角速度偏差
    Vector3d        Ps[(WINDOW_SIZE + 1)];
    Vector3d        Vs[(WINDOW_SIZE + 1)];
    Matrix3d        Rs[(WINDOW_SIZE + 1)];
    Vector3d        Bas[(WINDOW_SIZE + 1)];
    Vector3d        Bgs[(WINDOW_SIZE + 1)];
    // double td;时间变化量

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];//窗口内所有帧的时间戳

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];//滑动窗口里边存放的imu预积分
    Vector3d acc_0, gyr_0;//IMU的加速度，陀螺仪初始值

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    bool initFirstPoseFlag;
    bool initThreadFlag;
};
