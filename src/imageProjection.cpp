// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above groundScanInd notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.


// 总体流程：订阅点云数据回调处理->点云转换到pcl预处理->截取一帧激光数据->投影映射到图像->地面移除->点云分割->发布点云数据->重置参数;



#include "utility.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

class ImageProjection{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;           // 雷达直接传出的点云
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud;              // 投影后的点云 projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud;          // 整体的点云 其中的intensity保存的是深度值 same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;            // 只包含地面点云
    pcl::PointCloud<PointType>::Ptr segmentedCloud;         // 分割后的部分，含地面点云
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;     // 分割后的部分的几何信息，不含地面点云
    pcl::PointCloud<PointType>::Ptr outlierCloud;           // 在分割时出现的异常

    PointType nanPoint; // fill in fullCloud at each iteration

    char szSaveName[1024];    //序号size
    int SaveNameNum = 1;    //序号

    cv::Mat rangeMat;   // range matrix for range image  距離圖像的距離矩陣
    cv::Mat rangeMatTemp;   // rangeMat 的转存
    cv::Mat rangeMatDiff;
    cv::Mat rangeMat2st;
    cv::Mat labelMat;   // label matrix for segmentaiton marking  標籤矩陣
    cv::Mat labelMat_Copy;      // 单纯用来Max判断，可以为了性能修改
    cv::Mat labelMat_Hard;      // 苛刻集
    cv::Mat groundMat;  // ground matrix for ground cloud marking  地面矩陣

    cv::Mat MatTemp;
    
    int labelCount;
    int labelCount_Hard;

    float startOrientation;
    float endOrientation;

    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process


    uint16_t *allPushedIndX; // array for tracking points of a segmented object 用於跟踪分割對象點的數組
    uint16_t *allPushedIndY;



    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed 用於分割的廣度優先搜索過程的數組
    uint16_t *queueIndY;


    int Count_full = 0;

    int N_num_ground_full = 0;
    int N_num_usessd_full = 0;
    int H_num_usessd_full = 0;
    int C_num_usessd_full = 0;




public:
    ImageProjection():
        nh("~"){
        
        // 仅订阅：pointCloudTopic ，其定义在 utility.h 中
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this);   // callback

        // 
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        // 
        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);


        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;    // 无效点的标志

        allocateMemory();
        resetParameters();
    }

    // 分配內存
    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);


        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];


        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];





    }

    // 重置参数
    void resetParameters(){
        laserCloudIn->clear();      
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        // rangeMatTemp = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));     // rangeMatTemp
        // rangeMatDiff = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));     // rangeMatTemp

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        rangeMat2st = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelMat_Copy = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelMat_Hard = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));

        MatTemp = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));

        labelCount = 1;
        labelCount_Hard = 1;


        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~ImageProjection(){}        // 解構

    // 复制点云
    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

        // have "ring" channel in the cloud
        if (useCloudRing == true){
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }  
        }
    }
    
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // 1. Convert ros message to pcl point cloud
        // 1. ros消息格式的点云转化为PCL格式的点云
        // 将ROS定义的PointCloud保存成PCL的PointCloud，因为前者方便借助ROS进行通信，后者方便处理，同时去除了nan，是为了后面的计算中不会出现各种异常情况；
        copyPointCloud(laserCloudMsg); //使用pcl库函数

        // 2. Start and end angle of a scan
        // 2. 一帧点云的起始点与结束点之间的夹角
        // 多线激光雷达第一个点和最后一个点并不是严格的360°，这里计算出起至角度后保存在自定义的cloud_msgs::cloud_info 类型的成员变量segMsg中，请注意，这里的segMsg很重要，它保存了当前帧的一些重要信息，包括起至角度，每个线的起至序号，及成员变量fullCloud中每个点的状态
        findStartEndAngle(); //1.一圈数据的角度差，使用atan2计算; 2.注意计算结果的范围合理性

        // 3. Range image projection
        // 3. 把一帧3D点云映射到一张带有深度值的平面图
        // 把无序点云以角度展开成图像的形式，计算所在行列和深度，其中不同线对应不同行，不同航向角代表不同列，这里以x轴的负方向开始逆时针列序列号逐渐递增，即图像中的从左到右。以Mat图像保存深度，并以单值索引在pcl::PointCloud fullCloud/fullInfoCloud中依次保存点的三维坐标，所在行列（fullCloud）和深度（fullInfoCloud）等。这里的PointCloud与第1步直接拷贝出来的，主要的不同之处在于，根据计算出来的行列，重新组织了点在PointCloud中的顺序，例如，激光雷达本身可能是先列后行的方向输出的点云，或者是livox雷达那种非重复扫描出来的结果，这里都会被重新组织成先行后列的顺序保存。
        projectPointCloud(); //将激光雷达数据投影成一个16x1800（依雷达角分辨率而定）的点云阵列


        // rangeMatRemoval();  // 效果太差(已弃用)
        // SaveNameNum++;

        // 4. Mark ground points
        // 4. 标记出地面上的点云
        // 在贴近地面的几个线中提取地面点,并在groundMat中标记1即地面，labelMat中标记-1，即nan或地面点 ，不会参与后面的分割。
        groundRemoval(); //根据上下两线之间点的位置计算两线之间俯仰角判断，小于10度则为地面点
        

        labelHandler();


        // 5. Point cloud segmentation
        // 5. 点云分割
        // 这个函数主要完成了两个任务，一是通过广度优先搜索，从非地面点中找出所有连成片的入射角比较小的patch上的点，并在labelMat标注patch的编号（从1开始）；二是把所有地面点和刚分割出来的patch上的点合并保存在segmentedCloud中，这也是该node需要传递给下一个node进行特征提取和匹配的点云，并在segMsg中对应位置处保存每个点的属性（比如该点是不是地面，深度，属于第几列等）。
        cloudSegmentation(); //首先对点云进行聚类标记，根据标签进行对应点云块存储;

        // 6. Publish all clouds
        // 6. 发布分割后的各种点云
        // 发布，包括该帧的segMsg，完整点云fullCloud/fullInfoCloud（步骤3），地面点云（步骤4），发从非地面提取出来的点和降采样的地面点（步骤5），外点（步骤5,实际为比较小的patch上的点）。
        publishCloud(); // 发布各类点云数据

        // 7. Reset parameters for next iteration
        // 7. 重置参数，准备处理下一帧点云
        resetParameters();
    }


    // void rangeMatRemoval(){

    //     // rangeMatTemp = rangeMat;
    //     string SaveName;
    //     float rangeMatDiffMax = 0.0;

    //     // path name
    //     // sprintf(szSaveName, "/home/scale/catkin_ws/rangeviewDiff%03d.png", SaveNameNum);
    //     sprintf(szSaveName, "/home/scale/catkin_ws/rangeviewDiff%03d.xml", SaveNameNum);
    //     SaveName.assign( szSaveName );
    //     cout << SaveName << "";
    //     if(SaveNameNum%10 == 0){
    //         cout << endl;
    //     }


    //     // 计算 rangeMat 和 rangeMatTemp 的差值
    //     for(size_t x = 0; x < N_SCAN; ++x){
    //         for(size_t y = 0; y < Horizon_SCAN; ++y){
    //             // rangeMatDiff.at<float>(x, y) = fabs(rangeMat.at<float>(x, y) 
    //             //                                             - rangeMatTemp.at<float>(x, y));
    //             rangeMatDiff.at<float>(x, y) = rangeMat.at<float>(x, y);
                
    //             // 最大值
    //             // if(rangeMatDiff.at<float>(x, y) > rangeMatDiffMax){
    //             //     rangeMatDiffMax = rangeMatDiff.at<float>(x, y);
    //             // }
    //             // cout << rangeMatDiff.at<float>(x, y) << "" << endl;
    //         }
    //     }

    //     // 归一化 0～255
    //     // for(size_t x = 0; x < N_SCAN; ++x){
    //     //     for(size_t y = 0; y < Horizon_SCAN; ++y){
    //     //         rangeMatDiff.at<float>(x, y) = round((255-0) * (rangeMatDiff.at<float>(x, y)-0) / (rangeMatDiffMax-0) + 0);
    //     //     }
    //     // }
        

    //     // cv::FileStorage fs("rangeMatDiff.xml", FileStorage::WRITE);
    //     FileStorage fs;
    //     fs.open(SaveName, FileStorage::WRITE);
	//     fs << "rangeMatDiff" << rangeMatDiff;
	//     fs.release();

    //     // imwrite(szSaveName, rangeMatDiff);   // 保存每一帧
    // }


    void findStartEndAngle(){

        // 激光一圈一圈的转，一圈是360°，原以为一帧点云数据也是标准的360°的内的点云，但是仔细想想不一定，一帧点云的多少可以人为规定是多少的，比如我把0~377°的内点云作为一帧数据。每个厂家的激光雷达经过ros驱动包后，我认为一帧点云的起始点与终点夹角在360°左右吧。不是一圈标准360°内的点云也没关系了，不影响后面构图什么。还有，激光雷达一般有自己的坐标系，该激光雷达产生的点云数据都是参考本机的坐标系。激光雷达起始点一般都不在标准的x轴那里，一圈点云数据也不是标准的一圈，有一圈多的，也没有一圈的，有的2D激光就只有270°的点云
        // 计算这个的原因？ 找到雷达旋转的开始的角度和结束的角度

        // start and end orientation of this cloud
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        // 计算方式？ 用atan2()求反正弦函数求角度，注意atan2()求出来的角度是-π ~ +π 范围，这些角度都是相对于X轴来讲的
        // 为什么有负号？ 因为velodyne雷达顺时针旋转，所以前面加负号吧

        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        // 末尾扫描线求出角度后要加上2π，体现转一圈的角度

        // https://www.freesion.com/article/4533590429/
        // -π^ -- 0 -- ^π* -- 2π -- *3π     ^:起始点可能位置   *:结束点可能位置
        // 起始点 - 结束点 > 3π 时，说明起始点在 -π 右边，结束点在 3π 左边，相差几乎 4π，就需要修正
        // 起始点 - 结束点 <  π 时，说明起始点在  π 左边，结束点在  π 右边，两个点相差几乎 就在 π 附近，也需要修正
        // 修正的目的：让起始点和结束点相差在 2π 附近，即一圈。这两种特殊情况的处理，来保证 e-s 的值始终在 2π 范围附近
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
        // 如果使用的 lego-loam 推荐的数据集 ，最后求出的 segMsg.orientationDiff 大概在 6.5 ~ 6.6 弧度之间，就是说扫描起始点与终点相差大概 377°
    }

    // 将3D point cloud投影映射到2D range image
    void projectPointCloud(){
        // range image projection

        // verticalAngle：算的是当前点的垂直角度，以velodyne16为例，范围在(-15°，+15°)之间
        // rowIdn：将垂直的度数范围(0,30)除以16线的分辨率(也就是2°一线),得到每一条线的ring号

        float verticalAngle, horizonAngle, range, rangexy;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            // find the row and column index in the iamge for this point
            // 计算行索引 rowIdn
            if (useCloudRing == true){
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else{
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;  // 确定行索引
            }
            // error
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // 计算列索引 columnIdn
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            
            //round是四舍五入取偶
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;  // 确定列索引
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
            // error
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            // 计算距离 range
            // range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y);
            rangexy = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y);      // only xy
            // range = rangexy;

            // error
            if (range < sensorMinimumRange)
                continue;
            
            rangeMat.at<float>(rowIdn, columnIdn) = range;
            // rangeMatTemp.at<float>(rowIdn, columnIdn) = rangexy;

            // 其字面意思是每个点的强度值，实际上，整数部分代表行号，小数部分代表列号，相当于这里记录了这个点的2D位置。 后续会通过intensity来恢复这个点。
            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;         // 设定 0-1800*16 范围的索引
            fullCloud->points[index] = thisPoint;               // fullCloud 里根据 0-1800*16 的索引范围来保存每个点；
            
            // the corresponding range of a point is saved as "intensity"
            // fullInfoCloud 和fullCloud 没什么两样，就是fullInfoCloud的intensity被存成了range 
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range;      
        }
    }

    // 通过在16*1800的下半面，也就是8*1800的部分，通过计算每个点和其相同列的上一个点之间的向量，判断该向量和地面的夹角
    // 小于一定的阈值则认为是平面，在groundMat中标记为1； 同时，在labelmat中，该点被标记为-1.
    // 但这里其实 地面点范围 是由 groundScanInd 限定的 
    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;
                
                // error
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                
                // handle
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                // 垂直方向相邻两点俯仰角小于10度就判定为地面点;相邻扫描圈 sensorMountAngle set 0
                if (abs(angle - sensorMountAngle) <= 10){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }
    }


    void labelHandler(){
        labelMat_Hard = labelMat.clone();

        // 常规集
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (labelMat.at<int>(i,j) == 0){
                    labelComponents(i, j);
                }
            }
        }

        // 苛刻集
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (labelMat_Hard.at<int>(i,j) == 0){
                    labelComponentsForHard(i, j);
                }
            }
        }

    }


    // 這個function調用了labelComponents(i, j)進行四鄰域搜索判斷，給每個點上標簽也是labelComponents判斷的，cloudSegmentation除了labelComponents之外，就只有“地面點下采樣、把三個Mat存到segMsg裏”
    void cloudSegmentation(){





        // count  function start //
        int N_num_ground = 0;
        int N_num_usessd = 0;
        int N_num_abound = 0;

        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (labelMat.at<int>(i, j) == 999999){
                    N_num_abound += 1;
                }
                else if(labelMat.at<int>(i, j) == -1){
                    N_num_ground += 1;
                }
                else{
                    N_num_usessd +=1;
                }
            }
        }

        










        
        // 做剔除的 function 需要放在这里！！！
        labelMat_Copy = labelMat.clone();             // ! 直接用 = 賦值的話，就是傳引用
        // cout << "labelMat_Copy start..." << endl;
        for(size_t i = 0; i < N_SCAN; ++i){
            for(size_t j = 0; j < Horizon_SCAN; ++j){
                if(labelMat_Copy.at<int>(i, j) == 999999){
                    labelMat_Copy.at<int>(i, j) = 0;

                    // cout << "check 01" << endl;
                    // cout << labelMat_Copy.at<int>(i, j) << endl;
                }
            }
            // cout << "labelMat_Copy over..." << endl;
        }
        

        // 取 labelMat 的最大最小值（除999999），确定最大标签
        double Mat_max, Mat_min;
        cv::minMaxLoc(labelMat_Copy, &Mat_min, &Mat_max, 0, 0);
        int labelMat_NumMax = (int)Mat_max;
        // cout << labelMat_NumMax << " ; " << Mat_max << " ; " << Mat_min << endl;


        // 查点、Flag处理
        for(int labelMat_Num = 1; labelMat_Num < (labelMat_NumMax + 1); ++labelMat_Num){

            bool labelMatFlag = false;

            // 查找是否存在可靠点
            for(size_t i = 0; i < N_SCAN; ++i){
                for(size_t j = 0; j < Horizon_SCAN; ++j){
                    if(labelMat.at<int>(i, j) == labelMat_Num){
                        // labelMatFlag = true;
                        
                        // only for test
                        // int labelMat_Num_Choose = labelMat_Num % 5;
                        // if(labelMat_Num_Choose == 0){
                        //     labelMatFlag = true;
                        // }

                        // int labelMat_Num_Choose = 3;
                        // if(labelMat_Hard.at<int>(i, j) == labelMat_Num_Choose){
                        //     labelMatFlag = true;
                            // cout << "check 01" << endl;
                        // }else{
                        //     // cout << "check 04" << endl;
                        // }


                        if((labelMat_Hard.at<int>(i, j) > 0) && (labelMat_Hard.at<int>(i, j) < 999999)){
                            labelMatFlag = true;
                            // cout << "可以用哦！" << endl;
                        }

                    }
                }
            }

            // labelMat_Hard里如果一个点都没有的话，labelMat就处理为 异常点
            if(labelMatFlag == false){
                for(size_t i = 0; i < N_SCAN; ++i){
                    for(size_t j = 0; j < Horizon_SCAN; ++j){
                        if(labelMat.at<int>(i, j) == labelMat_Num){
                            labelMat.at<int>(i, j) = 999999;
                            // cout << "删点啦！删点啦！" << endl;
                            // cout << "check 02" << endl;
                        }
                        // cout << "check 03" << endl;
                    }
                }
            }
        }





        // 至此 筛选操作完成






        
        // count  function start //
        int H_num_ground = 0;
        int H_num_usessd = 0;
        int H_num_abound = 0;

        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (labelMat_Hard.at<int>(i, j) == 999999){
                    H_num_abound += 1;
                }
                else if(labelMat_Hard.at<int>(i, j) == -1){
                    H_num_ground += 1;
                }
                else{
                    H_num_usessd +=1;
                }
            }
        }



        // count  function start //
        int C_num_ground = 0;
        int C_num_usessd = 0;
        int C_num_abound = 0;

        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (labelMat.at<int>(i, j) == 999999){
                    C_num_abound += 1;
                }
                else if(labelMat.at<int>(i, j) == -1){
                    C_num_ground += 1;
                }
                else{
                    C_num_usessd +=1;
                }
            }
        }

        // cout << "C_num_ground: " << C_num_ground << "\n" << "C_num_abound: " << C_num_abound << "\n" << "C_num_usessd: " << C_num_usessd << endl;
        // cout << "------------------" << endl;

        cout << "G: " << N_num_ground << "\n" << "N: " << N_num_usessd << "\n" << "H: " << H_num_usessd << "\n" << "C: " << C_num_usessd << endl;
        cout << "------------------" << endl;


        Count_full = Count_full + 1;
        N_num_ground_full = N_num_ground_full + N_num_ground;
        N_num_usessd_full = N_num_usessd_full + N_num_usessd;
        H_num_usessd_full = H_num_usessd_full + H_num_usessd;
        C_num_usessd_full = C_num_usessd_full + C_num_usessd;

        ros::Time beginning2(1317617735.038322);

        cout << "Time_Now: " << (ros::Time::now() - beginning2) << endl;
        cout << "Count_full: " << Count_full << endl;
        cout << "N_num_ground_full: " << N_num_ground_full << endl;
        cout << "N_num_usessd_full: " << N_num_usessd_full << endl;
        cout << "H_num_usessd_full: " << H_num_usessd_full << endl;
        cout << "C_num_usessd_full: " << C_num_usessd_full << endl;
        


        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i) {

            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;  //4~19

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                //如果是被认可的特征点或者是地面点，就可以纳入被分割点云
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    
                    //离群点或异常点的处理
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }

                    //大多數地麵點被跳過
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){
                        //地面点云每隔5个点纳入被分割点云
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }

                    // mark ground points so they will not be considered as edge features later 標記地麵點，以便它們以後不會被視為邊緣特徵
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);

                    // mark the points' column index for marking occlusion later 標記點的列索引以便稍後標記遮擋
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;

                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);

                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);

                    // size of seg cloud
                    ++sizeOfSegCloud;



                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }
        
        // extract segmented cloud for visualization 如果有节点订阅SegmentedCloudPure，那么把点云数据保存到segmentedCloudPure中去
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    // 局部特征检测聚类、平面点与边缘点 BFS
    // 聚类，超过30个点聚为一类，labelCount++;小于30超过5,统计垂直方向聚类点数，超过3个也标记为一类;若都不满足，则赋值999999表示需要舍弃的聚类点。
    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;  

        bool lineCountFlag[N_SCAN] = {false};  //聚类后竖直方向跨越的数量

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;


        allPushedIndX[0] = row;     // 用於跟踪分割對象點的數組 -> row
        allPushedIndY[0] = col;


        int allPushedIndSize = 1;

        
        //queueSize 指的是在特征处理时还未处理好的点的数量，
        //因此该while循环是在尝试检测该特定点的周围的点的几何特征
        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];

            --queueSize;
            ++queueStartInd;

            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;


            // Loop through all the neighboring grids of popped grid
            //上下左右四个邻域点
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;

                // 判斷
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                
                // 左右互博 
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                    
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                // 比较得出较大深度与较小深度, 该特定点与某邻点的深度
                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                //该迭代器的first是0则是水平方向上的邻点，否则是竖直方向上的，设置相应的角分辨率
                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));     // 平面聚类，角度越大两点越趋向于平面
                // cout << "angle: " << angle << endl;
                // 角度大于设定聚类角阈值，说明趋于同一平面
                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;      // 用於分割的廣度優先搜索過程的數組 -> thisIndX
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                    
                }

            }
        }


        //////////////////////////////////////////////////////////////////////////////
        ////                                普通集                                 ////
        //////////////////////////////////////////////////////////////////////////////
        // 如果聚类超过30个点，直接标记为一个可用聚类
        // check if this segment is valid  如果聚类超过30个点，直接标记为一个可用聚类
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;

        else if (allPushedIndSize >= segmentValidPointNum){
            // 如果聚类点数小于30大于等于5
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                // 统计竖直方向上的聚类点数,竖直方向上超过3个也将它标记为有效聚类
                feasibleSegment = true;            
        }

        // error
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++labelCount;
            // cout << "labelCount: " << labelCount << endl;
        }else{ // segment is invalid, mark these points
            // cout << "allPushedIndSize: " << allPushedIndSize << endl;
            for (size_t i = 0; i < allPushedIndSize; ++i){

                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }

    }


    void labelComponentsForHard(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;  

        bool lineCountFlag[N_SCAN] = {false};  //聚类后竖直方向跨越的数量

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;


        allPushedIndX[0] = row;     // 用於跟踪分割對象點的數組 -> row
        allPushedIndY[0] = col;


        int allPushedIndSize = 1;

        
        //queueSize 指的是在特征处理时还未处理好的点的数量，
        //因此该while循环是在尝试检测该特定点的周围的点的几何特征
        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];

            --queueSize;
            ++queueStartInd;

            // Mark popped point
            labelMat_Hard.at<int>(fromIndX, fromIndY) = labelCount_Hard;

            // Loop through all the neighboring grids of popped grid
            //上下左右四个邻域点
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;

                // 判斷
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                
                // 左右互博 
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                    
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat_Hard.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                // 比较得出较大深度与较小深度, 该特定点与某邻点的深度
                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                //该迭代器的first是0则是水平方向上的邻点，否则是竖直方向上的，设置相应的角分辨率
                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));     // 平面聚类，角度越大两点越趋向于平面
                // cout << "angle: " << angle << endl;

                if (angle > segmentTheta_Hard){

                    queueIndX[queueEndInd] = thisIndX;       
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat_Hard.at<int>(thisIndX, thisIndY) = labelCount_Hard;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                    
                }

            }
        }

        //////////////////////////////////////////////////////////////////////////////
        ////                                苛刻集                                 ////
        //////////////////////////////////////////////////////////////////////////////
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;

        else if (allPushedIndSize >= segmentValidPointNum_Hard){
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum_Hard)
                feasibleSegment = true;            
        }

        // error
        if (feasibleSegment == true){
            ++labelCount_Hard;
        }else{
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat_Hard.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }


    
    void publishCloud(){
        // 1. Publish Seg Cloud Info
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);
        
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }

        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }

        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }

        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};




int main(int argc, char** argv){
    ros::init(argc, argv, "lego_loam");
    ImageProjection IP;
    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");
    ros::spin();
    return 0;
}
