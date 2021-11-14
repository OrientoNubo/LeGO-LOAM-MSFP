#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;


// 我们提出了LeGO-LOAM，它是一种轻量级和地面优化的激光雷达里程计和建图方法，用于实时估计地面车辆的六自由度姿态。LeGO-LOAM是轻量级的，因为它可以在低功耗嵌入式系统上实现实时姿态估计。LeGO-LOAM经过地面优化，因为它在分割和优化步骤中利用了地面的约束。我们首先应用点云分割来滤除噪声，并进行特征提取，以获得独特的平面和边缘特征。然后，采用两步Levenberg-Marquardt优化方法，使用平面和边缘特征来解决连续扫描中六个自由度变换的不同分量。我们使用地面车辆从可变地形环境中收集的数据集，比较LeGO-LOAM与最先进的LOAM方法的性能，结果表明LeGO-LOAM在减少计算开销的情况下实现了相似或更好的精度。为了消除由漂移引起的姿态估计误差，我们还将LeGO-LOAM集成到SLAM框架中，并用KITTI数据集进行了测试。

// error: Failed to find match for field ‘intensity’  



typedef pcl::PointXYZI  PointType;

// extern const string pointCloudTopic = "/velodyne_points";
extern const string pointCloudTopic = "/velodyne_points";

extern const string imuTopic = "/imu/data";

// Save pcd 保存路径
extern const string fileDirectory = "/home/scale/catkin_ws/";

// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
extern const bool useCloudRing = false; // if true, ang_res_y and ang_bottom are not used


// LiDAR 参数说明
// extern const int N_SCAN = 线数;
// extern const int Horizon_SCAN = 水平分辨率(这意味着一行scan最多有几个点);
// extern const float ang_res_x = 水平角分辨率;
// extern const float ang_res_y = 垂直视场角(垂直最大探测角度);
// extern const float ang_bottom = 垂直视场角偏置(0度上下偏置分配); 竖直方向上起始角度是负角度
// extern const int groundScanInd = 地线分配; 以多少个扫描圈来表示地面

// VLP-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 2.0;
// extern const float ang_bottom = 15.0+0.1;
// extern const int groundScanInd = 6;

// HDL-32E
extern const int N_SCAN = 32;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 360.0/float(Horizon_SCAN);
extern const float ang_res_y = 41.33/float(N_SCAN-1);
extern const float ang_bottom = 30.67;
extern const int groundScanInd = 20;

// HDL-64E
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 0.427;
// extern const float ang_bottom = 24.9;
// extern const int groundScanInd = 60;

// VLS-128
// extern const int N_SCAN = 128;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 0.3;
// extern const float ang_bottom = 25.0;
// extern const int groundScanInd = 10;


// Ouster users may need to uncomment line 159 in imageProjection.cpp
// cloudHeader.stamp = ros::Time::now();
// Usage of Ouster imu data is not fully supported yet (LeGO-LOAM needs 9-DOF IMU), please just publish point cloud data

// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

// Ouster OS1-128 
// extern const int N_SCAN = 128;
// extern const int Horizon_SCAN = 2048;             
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);  
// extern const float ang_res_y = 45.0/float(N_SCAN-1);         
// extern const float ang_bottom = 22.5+0.1;                    
// extern const int groundScanInd = 7;                            



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern const bool loopClosureEnableFlag = true;        // 回环检测 Flag
extern const double mappingProcessInterval = 0.1;       // 0.3

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;

extern const float sensorMinimumRange = 1.0;            // 最小探知范围
extern const float sensorMountAngle = 0.0;
extern const float segmentTheta_Hard = 50/180.0*M_PI;  // 苛刻集 角度
extern const float segmentTheta = 50.0/180.0*M_PI;      // decrese this value may improve accuracy 60c

extern const int segmentValidPointNum = 5;              // 检查上下左右连续5个点做为分割的特征依据
extern const int segmentValidLineNum = 3;               // 检查上下左右连续3线做为分割的特征依据
extern const int segmentValidPointNum_Hard = 5;     //10  8
extern const int segmentValidLineNum_Hard = 3;      //5  4
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;    // 水平角分辨率（转弧度）
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;    // 垂直角分辨率（转弧度）


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;


// Mapping Params
extern const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
extern const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)

// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
extern const int   historyKeyframeSearchNum = 25; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment

extern const float globalMapVisualizationSearchRadius = 500.0; // key frames with in n meters will be visualized


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has "ring" channel
    定义了一种包含ring_id的点云数据类型，velodyne的激光雷达是有这个数据的，其它厂商的不一定有;
    区别于laser_id，ring_id是指从最下面的激光线依次向上递增的线号（0-15 for VLP-16）
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D                     // 该点类型有4个元素
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // 确保new操作符对齐操作
} EIGEN_ALIGN16;                        // 强制SSE对齐

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;       // PointTypePose指的是具备姿态角的特定点

#endif
