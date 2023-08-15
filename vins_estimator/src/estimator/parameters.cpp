/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;//特征点估计深度的默认值
double MIN_PARALLAX;//yaml里的 关键帧选择阈值（像素）
double ACC_N, ACC_W;//yaml里的acc_n，acc_w
double GYR_N, GYR_W;//yaml里的gyr_n,gyr_w

std::vector<Eigen::Matrix3d> RIC;//cam to imu 的旋转
std::vector<Eigen::Vector3d> TIC;//cam to imu 的平移

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;//具体的IMU 和 相机数据之间的延迟 
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;//单帧图像最大特征点数目
int MIN_DIST;//两个特征点之间的最短像素距离
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

// 读取参数，通过roslaunch文件的参数服务器获得
void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    // 检查文件地址是否为空
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    //通过opencv的yaml文件接口来读取文件 
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;//左目相机话题
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;//右目相机话题
    MAX_CNT = fsSettings["max_cnt"];//单帧图像最大特征点数目
    MIN_DIST = fsSettings["min_dist"];//两个特征点之间的最短像素距离
    F_THRESHOLD = fsSettings["F_threshold"];//阈值（像素）
    SHOW_TRACK = fsSettings["show_track"];//将跟踪图像发布为topic
    FLOW_BACK = fsSettings["flow_back"];//执行正向和反向光流以提高特征跟踪精度

    MULTIPLE_THREAD = fsSettings["multiple_thread"];//使用多线程

    USE_IMU = fsSettings["imu"];//使用imu
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;//imu话题
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        
        //yaml里的acc_n，acc_w，gyr_n，gyr_w，g_norm
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];//单次优化最大求解时间
    NUM_ITERATIONS = fsSettings["max_num_iterations"];//单次优化最大迭代次数
    MIN_PARALLAX = fsSettings["keyframe_parallax"];//根据视差确定关键帧
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();


    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];//是否有准确的估计
    //需要在线标定估计
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        //将外参 cam0 to imu的外参赋值给Ric和Tic
        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;  //cam0 to body
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];//相机数量
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)//至少有一个相机
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;//读取左目相机参数信息
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    // 如果是双目
    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;//读取右目相机参数信息
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        //和上面一样，将外参 cam1 to imu的外参赋值给Ric和Tic
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;//特征点估计深度的默认值
    BIAS_ACC_THRESHOLD = 0.1;//加速度计常值偏差的阈值 0.1
    BIAS_GYR_THRESHOLD = 0.1;//陀螺仪常值偏差的阈值 0.1

    TD = fsSettings["td"];//具体的IMU 和 相机数据之间的延迟 
    ESTIMATE_TD = fsSettings["estimate_td"];//在线估计 IMU 和 相机数据之间的延迟 
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];//图像的高
    COL = fsSettings["image_width"];//图像的宽
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    // 如果不使用imu，和imu相关的时间差评估状态位 置0
    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    fsSettings.release();
}
