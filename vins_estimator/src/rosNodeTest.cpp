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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;//实例化 estimator对象

queue<sensor_msgs::ImuConstPtr> imu_buf;//存储imu的buf
queue<sensor_msgs::PointCloudConstPtr> feature_buf;//存储特征点的buf
queue<sensor_msgs::ImageConstPtr> img0_buf;//存储左目的buf
queue<sensor_msgs::ImageConstPtr> img1_buf;//存储右目的buf
std::mutex m_buf;

//左目图像回调函数
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);//将图像消息送入img0_buf中
    m_buf.unlock();
}

//右目图像回调函数
void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);//将图像消息送入img1_buf中
    m_buf.unlock();
}

// 将ros消息转成opencv的cv::Mat格式
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// 主处理函数extract images with same timestamp from two topics
// 光流追踪入口
void sync_process()
{
    while(1)
    {
        // 双目模式
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            // 双目的话做一个简单的时间同步
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance时间同步阈值为0.003秒
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }

                // 时间同步做完后，将消息拿出来转成opencv的格式
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());//获得左目图像：cv::Mat类型的image0
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());//获得右目图像：cv::Mat类型的image1
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();

            // 图像不为0，即可进行光流追踪
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }

        // 单目模式
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

// IMU回调函数：从IMU话题信息中提取数据保存到acc向量和gyr向量中，然后把{t,acc,gyr}保存到估计器中。
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    // 得到加速度
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    // 得到角速度
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;

    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);//两个作用：一个是送入imu数据到buffer中，另一个是输出高频里程计
    return;
}

// 将前端信息送入buff
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

// Vins复位
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

// 模式切换
void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

// 模式切换
void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

// 主函数入口
int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "vins_estimator");//许ROS通过命令行进行名称重映射，指定节点名称的地方
    ros::NodeHandle n("~");//创建节点句柄-实际上将执行节点的初始化
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // 确定输入参数个数为2
    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1]; //将输入的配置文件的路径赋给config_file
    printf("config_file: %s\n", argv[1]); //打印config_file

    readParameters(config_file);//读取参数，设置状态估计器参数
    estimator.setParameter();// 设置参数  ，它读取了每一个相机到IMU坐标系的旋转/平移外参数和非线性优化的重投影误差部分的信息矩阵。

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    //发布用于  RVIZ 显示 和 pose_graph_node.cpp 接收 的Topic，本模块具体发布的内容详见输入输出
    //这个函数定义在utility/visualization.cpp里面：void registerPub(ros::NodeHandle &n)
    registerPub(n);

    //创建消息订阅者，订阅IMU、feature、restart、match_points的topic，执行各自回调函数
    ros::Subscriber sub_imu;

// ros::Subscriber subscribe (const std::string &topic, uint32_t queue_size, void(*fp)(M), const TransportHints &transport_hints=TransportHints())
// 第一个参数是订阅话题的名称；
// 第二个参数是订阅队列的长度；（如果收到的消息都没来得及处理，那么新消息入队，旧消息就会出队）；
// 第三个参数是回调函数的指针，指向回调函数来处理接收到的消息！
// 第四个参数：似乎与延迟有关系，暂时不关心。（该成员函数有13重载）


    // 如果使用imu，才会调用imu话题
    if(USE_IMU)
    {
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }

    //这一部分接收的是feature_tracker_node发布的在cur帧的所有特征点的信息
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    //接受左目消息，进入回调函数
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1;

    // 如果是双目模式才会订阅另一个相机的topic
    if(STEREO)
    {
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    }

    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);//重定位的回调函数

    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);//模式切换
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);//模式切换

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}
