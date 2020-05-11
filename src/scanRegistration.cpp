// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Sunggoo Jung       sunggoo@etri.re.kr

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
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


#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

const double PI = 3.1415926;
const double rad2deg = 180.0 / PI;
const double deg2rad = PI / 180.0;
const double eps = 0.00000000000000001;
const double lidarScanPeriod = 0.1;

double initTime;
double timeStart;
double timeLasted;
bool systemInited = false;

double timeScanCur = 0;
double timeScanLast = 0;
double scanAngle_rad = 0;

int laserRotDir = 1;
int skipFrameNum = 2;
int skipFrameCount = 0;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLessExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());

sensor_msgs::PointCloud2 laserCloudExtreCur2;
sensor_msgs::PointCloud2 laserCloudLast2;

ros::Publisher* pubLaserCloudExtreCurPointer;
ros::Publisher* pubLaserCloudLastPointer;

int cloudSortInd[1200];
int cloudNeighborPicked[1200];

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 50;
bool imuInited = false;

float imuRollStart, imuPitchStart, imuYawStart;
float imuRollCur, imuPitchCur, imuYawCur;
float imuVeloXStart, imuVeloYStart, imuVeloZStart;
float imuShiftXStart, imuShiftYStart, imuShiftZStart;
float imuVeloXCur, imuVeloYCur, imuVeloZCur;
float imuShiftXCur, imuShiftYCur, imuShiftZCur;
float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};
float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};
float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};
float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

//double imuAccuRoll = 0;
//double imuAccuPitch = 0;
double imuAccuYaw = 0;

void jointStateHandler(const sensor_msgs::JointState jointStateMsg){

    scanAngle_rad = jointStateMsg.position[0];
}

void ShiftToStartIMU(){

  float x1 = imuShiftFromStartXCur * cos(imuYawStart) + imuShiftFromStartYCur * sin(imuYawStart);
  float y1 = imuShiftFromStartYCur * cos(imuYawStart) - imuShiftFromStartXCur * sin(imuYawStart);
  float z1 = imuShiftFromStartZCur;

  float x2 = x1*cos(imuPitchStart) - z1*sin(imuPitchStart);
  float y2 = y1;
  float z2 = x1*sin(imuPitchStart) + z1*cos(imuPitchStart);
  

  imuShiftFromStartXCur = x2;
  imuShiftFromStartYCur = y2*cos(imuRollStart) + z2*sin(imuRollStart);
  imuShiftFromStartZCur = z2*cos(imuRollStart) - y2*sin(imuRollStart);  
}

void VeloToStartIMU(){

  float x1 = sin(imuYawStart) * imuVeloFromStartYCur + cos(imuYawStart) * imuVeloFromStartXCur;
  float y1 = cos(imuYawStart) * imuVeloFromStartYCur - sin(imuYawStart) * imuVeloFromStartXCur;
  float z1 = imuVeloFromStartZCur;
  
  float x2 =  x1*cos(imuPitchStart) - z1*sin(imuPitchStart);
  float y2 = y1;
  float z2 = x1*sin(imuPitchStart) + z1*cos(imuPitchStart);  

  imuVeloFromStartXCur = x2;
  imuVeloFromStartYCur = y2*cos(imuRollStart) + z2*sin(imuRollStart);
  imuVeloFromStartZCur = z2*cos(imuRollStart) - y2*sin(imuRollStart);  
}

void TransformToStartIMU(pcl::PointXYZHSV *p){

  float x1 = p->x;
  float y1 = cos(imuRollCur) * p->y - sin(imuRollCur) * p->z;
  float z1 = sin(imuRollCur) * p->y + cos(imuRollCur) * p->z;  
  
  float x2 = sin(imuPitchCur) * z1 + cos(imuPitchCur) * x1;
  float y2 = y1;
  float z2 = cos(imuPitchCur) * z1 - sin(imuPitchCur) * x1;
  
  float x3 = -sin(imuYawCur) * y2 + cos(imuYawCur) * x2;
  float y3 = cos(imuYawCur) * y2 + sin(imuYawCur) * x2;
  float z3 = z2;  

  float x4 = sin(imuYawStart) * y3 + cos(imuYawStart) * x3;
  float y4 = cos(imuYawStart) * y3 - sin(imuYawStart) * x3;
  float z4 = z3;  

  float x5 = -sin(imuPitchStart) * z4 + cos(imuPitchStart) * x4;
  float y5 = y4;
  float z5 = cos(imuPitchStart) * z4 + sin(imuPitchStart) * x4;  

  p->x = x5 + imuShiftFromStartXCur;
  p->y = cos(imuRollStart) * y5 + sin(imuRollStart) * z5 + imuShiftFromStartYCur;
  p->z = -sin(imuRollStart) * y5 + cos(imuRollStart) * z5 + imuShiftFromStartZCur;  
}

//The AccumulateIMUShift() function is mainly used to obtain the displacement and velocity of the
//IMU in the global coordinate system corresponding to each frame of IMU data.
void AccumulateIMUShift(){

  //obtain euler angle and angular cceleration from IMUHandler()
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

/*
  //rotate the acceleration value of the current moment around the ZXY fixed axis
  //by (roll, pitch, yaw) angle to conver to the world.

  //rotate around the z axis (yaw)
  float x1 = -sin(yaw) * accY + cos(yaw) * accX;
  float y1 = cos(yaw) * accY + sin(yaw) * accX;
  float z1 = accZ;

  //rotate around the x axis (roll)
  float x2 = x1;
  float y2 = cos(roll) * y1 - sin(roll) * z1;
  float z2 = sin(roll) * y1 + cos(roll) * z1;

  //rotate around the y axis (pitch)
  accX = sin(pitch) * z2 + cos(pitch) * x2;
  accY = y2;
  accZ = cos(pitch) * z2 - sin(pitch) * x2;
*/
  //Previous IMU point
  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  //The time elapsed from the previous point to the current point->IMU measurement period(0.02, 50Hz)
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];

  //It is required that the frequency of imu is at least higher than that of lidar
  //, such imu information is used, and subsequent correction is meaningful.
  if (timeDiff < lidarScanPeriod) {

    //Find the displacement and speed of each imu time point, 
    //between two points is regarded as uniformly accelerated linear motion
    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
                              + accZ * timeDiff * timeDiff / 2;    

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;    
  }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2){

  //Wait IMU before processing the pointcloud data.
  if (!systemInited) {
      initTime = laserCloudIn2->header.stamp.toSec();
      imuPointerFront = (imuPointerLast + 1) % imuQueLength;
      systemInited = true;
    }

  //Filter pointcloud data to filter out invalid points
  timeScanLast = timeScanCur;
  timeScanCur = laserCloudIn2->header.stamp.toSec();
  timeLasted = timeScanCur - initTime;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);
  int cloudInSize = laserCloudIn->points.size();

  int cloudSize = 0;
  pcl::PointXYZHSV laserPointIn;
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>());
  for (int i = 0; i < cloudInSize; i++) {
    laserPointIn.x = laserCloudIn->points[i].x;
    laserPointIn.y = laserCloudIn->points[i].y;
    laserPointIn.z = laserCloudIn->points[i].z;
    laserPointIn.h = timeLasted;
    laserPointIn.v = 0;

    if (!(fabs(laserPointIn.x) < 0.5 && fabs(laserPointIn.y) < 0.5 && fabs(laserPointIn.z) < 0.5)) {
      laserCloud->push_back(laserPointIn);
      cloudSortInd[cloudSize] = cloudSize;
      cloudNeighborPicked[cloudSize] = 0;
      cloudSize++;
    }
  }

  pcl::PointXYZ laserPointFirst = laserCloudIn->points[0];
  pcl::PointXYZ laserPointLast = laserCloudIn->points[cloudInSize - 1];

  float rangeFirst = sqrt(laserPointFirst.x * laserPointFirst.x + laserPointFirst.y * laserPointFirst.y
                 + laserPointFirst.z * laserPointFirst.z);
  laserPointFirst.x /= rangeFirst;
  laserPointFirst.y /= rangeFirst;
  laserPointFirst.z /= rangeFirst;

  float rangeLast = sqrt(laserPointLast.x * laserPointLast.x + laserPointLast.y * laserPointLast.y
                 + laserPointLast.z * laserPointLast.z);
  laserPointLast.x /= rangeLast;
  laserPointLast.y /= rangeLast;
  laserPointLast.z /= rangeLast;

  float laserAngle = atan2(laserPointLast.y -laserPointFirst.y, laserPointLast.z -laserPointFirst.z)*180.0/PI;
  //ROS_INFO("%.5f, x: %.5f, y: %.5f, z: %.5f", laserAngle, laserPointLast.x, laserPointLast.y, laserPointLast.z);
  bool newSweep = false;
  if (laserAngle * laserRotDir < 0 && timeLasted - timeStart > 0.7) {
    laserRotDir *= -1;
    newSweep = true;
    //ROS_INFO("check");
  }

  if (newSweep) {
    timeStart = timeScanLast - initTime;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    imuTrans->points[0].x = imuRollStart;
    imuTrans->points[0].y = imuPitchStart;
    imuTrans->points[0].z = imuYawStart;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = imuRollCur;
    imuTrans->points[1].y = imuPitchCur;
    imuTrans->points[1].z = imuYawCur;
    imuTrans->points[1].v = 11;

    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    *laserCloudExtreCur += *laserCloudLessExtreCur;
    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudLast2);
    laserCloudLast2.header.stamp = ros::Time().fromSec(timeScanLast);
    laserCloudLast2.header.frame_id = "/camera";
    laserCloudExtreCur->clear();
    laserCloudLessExtreCur->clear();
    imuTrans->clear();

    imuRollStart = imuRollCur;
    imuPitchStart = imuPitchCur;
    imuYawStart = imuYawCur;

    imuVeloXStart = imuVeloXCur;
    imuVeloYStart = imuVeloYCur;
    imuVeloZStart = imuVeloZCur;

    imuShiftXStart = imuShiftXCur;
    imuShiftYStart = imuShiftYCur;
    imuShiftZStart = imuShiftZCur;
  }

  imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;
  imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
  imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;
  if (imuPointerLast >= 0) {
    while (imuPointerFront != imuPointerLast) {
      if (timeScanCur < imuTime[imuPointerFront]) {
        break;
      }
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

    if (timeScanCur > imuTime[imuPointerFront]) {
      imuRollCur = imuRoll[imuPointerFront];
      imuPitchCur = imuPitch[imuPointerFront];
      imuYawCur = imuYaw[imuPointerFront];

      imuVeloXCur = imuVeloX[imuPointerFront];
      imuVeloYCur = imuVeloY[imuPointerFront];
      imuVeloZCur = imuVeloZ[imuPointerFront];

      imuShiftXCur = imuShiftX[imuPointerFront];
      imuShiftYCur = imuShiftY[imuPointerFront];
      imuShiftZCur = imuShiftZ[imuPointerFront];
    } else {
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
      float ratioFront = (timeScanCur - imuTime[imuPointerBack]) 
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      float ratioBack = (imuTime[imuPointerFront] - timeScanCur) 
                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
      } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
      } else {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
      }

      imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
      imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
      imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

      imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
      imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
      imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
    }
  }

  if (!imuInited) {
    imuRollStart = imuRollCur;
    imuPitchStart = imuPitchCur;
    imuYawStart = imuYawCur;

    imuVeloXStart = imuVeloXCur;
    imuVeloYStart = imuVeloYCur;
    imuVeloZStart = imuVeloZCur;

    imuShiftXStart = imuShiftXCur;
    imuShiftYStart = imuShiftYCur;
    imuShiftZStart = imuShiftZCur;

    imuInited = true;
  }

  imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * (timeLasted - timeStart);
  imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * (timeLasted - timeStart);
  imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * (timeLasted - timeStart);

  //ShiftToStartIMU();

  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

  //VeloToStartIMU();
  /*
  for (int i = 0; i < cloudSize; i++) {
    TransformToStartIMU(&laserCloud->points[i]);
  }
*/
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x 
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x 
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x 
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y 
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y 
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y 
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z 
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z 
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z 
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;
    
    laserCloud->points[i].s = diffX * diffX + diffY * diffY + diffZ * diffZ;
  }
  
  for (int i = 5; i < cloudSize - 6; i++) {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    if (diff > 0.05) {

      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);

      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }

    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());

  int startPoints[4] = {5, 6 + int((cloudSize - 10) / 4.0), 
                        6 + int((cloudSize - 10) / 2.0), 6 + int(3 * (cloudSize - 10) / 4.0)};
  int endPoints[4] = {5 + int((cloudSize - 10) / 4.0), 5 + int((cloudSize - 10) / 2.0), 
                      5 + int(3 * (cloudSize - 10) / 4.0), cloudSize - 6};

  for (int i = 0; i < 4; i++) {
    int sp = startPoints[i];
    int ep = endPoints[i];

    for (int j = sp + 1; j <= ep; j++) {
      for (int k = j; k >= sp + 1; k--) {
        if (laserCloud->points[cloudSortInd[k]].s < laserCloud->points[cloudSortInd[k - 1]].s) {
          int temp = cloudSortInd[k - 1];
          cloudSortInd[k - 1] = cloudSortInd[k];
          cloudSortInd[k] = temp;
        }
      }
    }

    int largestPickedNum = 0;
    for (int j = ep; j >= sp; j--) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s > 0.1 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {
        
        largestPickedNum++;
        if (largestPickedNum <= 2) {
          laserCloud->points[cloudSortInd[j]].v = 2;
          cornerPointsSharp->push_back(laserCloud->points[cloudSortInd[j]]);
        } else if (largestPickedNum <= 20) {
          laserCloud->points[cloudSortInd[j]].v = 1;
          cornerPointsLessSharp->push_back(laserCloud->points[cloudSortInd[j]]);
        } else {
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;
        for (int k = 1; k <= 5; k++) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k - 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k - 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
        for (int k = -1; k >= -5; k--) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k + 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k + 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
      }
    }

    int smallestPickedNum = 0;
    for (int j = sp; j <= ep; j++) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s < 0.1 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {

        laserCloud->points[cloudSortInd[j]].v = -1;
        surfPointsFlat->push_back(laserCloud->points[cloudSortInd[j]]);

        smallestPickedNum++;
        if (smallestPickedNum >= 4) {
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;
        for (int k = 1; k <= 5; k++) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k - 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k - 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
        for (int k = -1; k >= -5; k--) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k + 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k + 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
      }
    }
  }

  for (int i = 0; i < cloudSize; i++) {
    if (laserCloud->points[i].v == 0) {
      surfPointsLessFlat->push_back(laserCloud->points[i]);
    }
  }

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
  downSizeFilter.setInputCloud(surfPointsLessFlat);
  downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
  downSizeFilter.filter(*surfPointsLessFlatDS);

  *laserCloudExtreCur += *cornerPointsSharp;
  *laserCloudExtreCur += *surfPointsFlat;
  *laserCloudLessExtreCur += *cornerPointsLessSharp;
  *laserCloudLessExtreCur += *surfPointsLessFlatDS;

  laserCloudIn->clear();
  laserCloud->clear();
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();
  surfPointsLessFlatDS->clear();

  if (skipFrameCount >= skipFrameNum) {
    skipFrameCount = 0;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    imuTrans->points[0].x = imuRollStart;
    imuTrans->points[0].y = imuPitchStart;
    imuTrans->points[0].z = imuYawStart;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = imuRollCur;
    imuTrans->points[1].y = imuPitchCur;
    imuTrans->points[1].z = imuYawCur;
    imuTrans->points[1].v = 11;

    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    sensor_msgs::PointCloud2 laserCloudExtreCur2;
    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudExtreCur2);
    laserCloudExtreCur2.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudExtreCur2.header.frame_id = "/camera";
    pubLaserCloudExtreCurPointer->publish(laserCloudExtreCur2);
    imuTrans->clear();

    pubLaserCloudLastPointer->publish(laserCloudLast2);

    //ROS_INFO ("%d %d", laserCloudLast2.width, laserCloudExtreCur2.width);
  }
  skipFrameCount++;
}


//SUNGGOO: IMU is used to remove the error(motion distortion) caused by the non-uniform
//velocity (acceleration and deceleration) part of the laser sensor during the movement.
//LOAM is based on the assumption of uniform motion, but the actual movement of the laser
//sensor is definitely not uniform. Therefore the IMU is used to remove the error caused
//by the non-uniform motion part to meet the assumption of uniform motion.
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){

  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  int imuPointerBack = imuPointerLast;
  imuPointerLast = (imuPointerLast + 1) % imuQueLength;
  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];

  if (timeDiff < 0.1) {

    //imuAccuRoll += timeDiff * imuIn->angular_velocity.x;
    //imuAccuPitch += timeDiff * imuIn->angular_velocity.y;
    //imuAccuYaw += timeDiff * imuIn->angular_velocity.z;
    float accX = imuIn-> linear_acceleration . x + sin (pitch) * 9.81 ;
    float accY = imuIn-> linear_acceleration . y - sin (roll) * cos (pitch) * 9.81 ;    
    float accZ = imuIn-> linear_acceleration . z - cos (roll) * cos (pitch) * 9.81 ;    

    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
    imuYaw[imuPointerLast] = yaw;
    //imuRoll[imuPointerLast] = imuAccuRoll;
    //imuPitch[imuPointerLast] = -imuAccuPitch;
    //imuYaw[imuPointerLast] = -imuAccuYaw;

    imuAccX[imuPointerLast] = accX;//-imuIn->linear_acceleration.y;
    imuAccY[imuPointerLast] = accY;//-imuIn->linear_acceleration.z - 9.81;
    imuAccZ[imuPointerLast] = accZ;//imuIn->linear_acceleration.x;

    AccumulateIMUShift();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  ros::Subscriber subJointState = nh.subscribe<sensor_msgs::JointState>
                                ("spin_hokuyo/joint_states", 100, jointStateHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> 
  //                                ("/sync_scan_cloud_filtered", 2, laserCloudHandler);
                                  ("/hokuyo_points", 2, laserCloudHandler);
  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> 
  //                         ("microstrain/imu", 5, imuHandler);
                           ("mavros/imu/data", 5, imuHandler);

  ros::Publisher pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2> 
                                         ("/laser_cloud_extre_cur", 2);

  ros::Publisher pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2> 
                                     ("/laser_cloud_last", 2);

  pubLaserCloudExtreCurPointer = &pubLaserCloudExtreCur;
  pubLaserCloudLastPointer = &pubLaserCloudLast;

  ros::spin();

  return 0;
}
