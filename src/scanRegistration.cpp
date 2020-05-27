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

//for ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

//for PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
double MINIMUM_RANGE = 0.1;
string LASER_TOPIC = "sync_scan_cloud_filtered";
string IMU_TOPIC = "microstrain/imu";

ros::Publisher pubLaserCloudSweepCur;
ros::Publisher pubLaserCloudLast;
sensor_msgs::PointCloud2 laserCloudSweepCur2;
sensor_msgs::PointCloud2 laserCloudLast2;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSweepCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLessSweepCur(new pcl::PointCloud<pcl::PointXYZHSV>());
const int systemDelay = 0; 

double scanAngle_rad = 0;
float cloudCurvature[1200];

int cloudSortInd[1200];
int cloudNeighborPicked[1200];
int cloudLabel[1200];

int systemInitCount = 0;
int laserRotDir = 1;
int skipFrameNum = 2;
int skipFrameCount = 0;

bool systemInited = false;

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

void jointStateHandler(const sensor_msgs::JointState jointStateMsg){

    scanAngle_rad = jointStateMsg.position[0];
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

	bool newSweep = false;

	if(!systemInited){
	    
	    systemInitCount++;
	    if (systemInitCount >= systemDelay)
	    {
	        systemInited = true;
	    }
	    else{
	    	ROS_WARN("system closed");
	        return;
        }
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
	int cloudInSize = laserCloudIn->points.size();

	int cloudSize = 0;
	pcl::PointXYZHSV laserPointIn;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>());

  	for (int i = 0; i < cloudInSize; i++) {
	    laserPointIn.x = laserCloudIn->points[i].x;
	    laserPointIn.y = laserCloudIn->points[i].y;
	    laserPointIn.z = laserCloudIn->points[i].z;
	    laserPointIn.h = 0;//timeLasted;
	    laserPointIn.v = 0;

	    if (!(fabs(laserPointIn.x) < MINIMUM_RANGE && fabs(laserPointIn.y) < MINIMUM_RANGE && fabs(laserPointIn.z) < MINIMUM_RANGE)) {
	   
	      	laserCloud->push_back(laserPointIn);            
	      	cloudSortInd[cloudSize] = cloudSize;      
	      	cloudNeighborPicked[cloudSize] = 0;
	      	cloudSize++;
    	}
  	}
  	pcl::PointXYZ laserPointFirst = laserCloudIn->points[0];
  	pcl::PointXYZ laserPointLast = laserCloudIn->points[cloudInSize - 1];

  	//range of firstPoint
  	float rangeFirst = sqrt(laserPointFirst.x * laserPointFirst.x + laserPointFirst.y * laserPointFirst.y + laserPointFirst.z * laserPointFirst.z);
  	laserPointFirst.x /= rangeFirst;
  	laserPointFirst.y /= rangeFirst;
  	laserPointFirst.z /= rangeFirst;
  	//range of lastPoint
  	float rangeLast = sqrt(laserPointLast.x * laserPointLast.x + laserPointLast.y * laserPointLast.y + laserPointLast.z * laserPointLast.z);
  	laserPointLast.x /= rangeLast;
  	laserPointLast.y /= rangeLast;
  	laserPointLast.z /= rangeLast;
  	//calculate the angle of 2D laser plane
  	float laserAngle = atan2(laserPointLast.y - laserPointFirst.y, laserPointLast.z - laserPointFirst.z)*180.0/3.141592;

  	if (laserAngle * laserRotDir < 0){

    	laserRotDir *= -1;
    	newSweep = true;
  	}

  	if (newSweep) {
    	
    	*laserCloudSweepCur += *laserCloudLessSweepCur; //(lessFlat, lessSharp)
    	
	    pcl::toROSMsg(*laserCloudSweepCur, laserCloudLast2);
    	laserCloudLast2.header.stamp = laserCloudMsg->header.stamp;
    	laserCloudLast2.header.frame_id = "/camera";
	    
	    //sweep이 끝나면 clear
    	laserCloudSweepCur->clear();
	    laserCloudLessSweepCur->clear();
  	}

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
    	
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudLabel[i] = 0;

  	}  

  	//A point i can be selected only if
  	//1. S does not form a surface patch that is roughly parallel to the laser beams
  	//2. no point in S that is disconnected from i by a gap in the direction of the laser beam
  	//3. no point in S that is at the same time closer to the lidar then point i.
  	for (int i = 5; i < cloudSize - 6; i++) {

    	//coplanar geometric relationship
    	float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    	float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    	float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    	float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;    	
    	//sqrt(diff) = 0.223
    	//두 포인트 사이 거리가 22.3cm 이상일 때, curvature threshold filtering
    	if (diff > 0.05) {
      		
      		float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);
        	
        	float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
        	//for paper fig4(b)
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
        	} 
        	else {
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
    	//for paper fig4(a)
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

  	//divide a scan into 4 sub-region 
  	int startPoints[4] = {5, 6 + int((cloudSize - 10) / 4.0), 6 + int((cloudSize - 10) / 2.0), 6 + int(3 * (cloudSize - 10) / 4.0)};
  	int endPoints[4] = {5 + int((cloudSize - 10) / 4.0), 5 + int((cloudSize - 10) / 2.0), 5 + int(3 * (cloudSize - 10) / 4.0), cloudSize - 6};

  	for (int i = 0; i < 4; i++) {

	    int sp = startPoints[i];
	    int ep = endPoints[i];

	    //curvature 값이 작은 순서대로 sort
        std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);

    	int largestPickedNum = 0;
    	for (int j = ep; j >= sp; j--) {
    		
    		int ind = cloudSortInd[j]; 
    		//cloudNeighborPicked=0 이고 곡률이 0.1이상이며, 0.3~30사이에 있을 때
	      	if (cloudNeighborPicked[ind] == 0 && 
	      		cloudCurvature[ind] > 0.1 &&
	          	(fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
	          	fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
	          	fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && fabs(laserCloud->points[cloudSortInd[j]].z) < 30){
        		largestPickedNum++;
        		//Edge는 최대 2개
		        if (largestPickedNum <= 2) {
		          	cloudLabel[ind] = 2;
		          	cornerPointsSharp->push_back(laserCloud->points[ind]);
		          	cornerPointsLessSharp->push_back(laserCloud->points[ind]);
		        } 
		        else if (largestPickedNum <= 20) {
		          	cloudLabel[ind] = 1;
		          	cornerPointsLessSharp->push_back(laserCloud->points[ind]);
		        } 
		        else {
		          	break;
		        }

        		cloudNeighborPicked[ind] = 1;
		        
		        for (int k = 1; k <= 5; k++) {
		          	float diffX = laserCloud->points[ind + k].x - laserCloud->points[ind + k - 1].x;
		          	float diffY = laserCloud->points[ind + k].y - laserCloud->points[ind + k - 1].y;
		          	float diffZ = laserCloud->points[ind + k].z - laserCloud->points[ind + k - 1].z;
		          	if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
		            	break;
		          	}

          			cloudNeighborPicked[ind + k] = 1;
        		}
        		for (int k = -1; k >= -5; k--) {
			        float diffX = laserCloud->points[ind + k].x - laserCloud->points[ind + k + 1].x;
			        float diffY = laserCloud->points[ind + k].y - laserCloud->points[ind + k + 1].y;
			        float diffZ = laserCloud->points[ind + k].z - laserCloud->points[ind + k + 1].z;
			        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
			        	break;
			        }
          			cloudNeighborPicked[ind + k] = 1;
        		}
      		}
    	}

    	int smallestPickedNum = 0;
	    for (int j = sp; j <= ep; j++) {
	      	
	    	int ind = cloudSortInd[j];
	      	if (cloudNeighborPicked[ind] == 0 &&
	          cloudCurvature[ind] < 0.1 &&
	          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
	          fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
	          fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
	          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
	          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
	          fabs(laserCloud->points[cloudSortInd[j]].z) < 30){

	        	cloudLabel[ind] = -1;
	        	surfPointsFlat->push_back(laserCloud->points[ind]);
	        	smallestPickedNum++;
	        	//Planar는 최대 4개
	        	if (smallestPickedNum >= 4) {
	          		break;
	        	}

	        	cloudNeighborPicked[ind] = 1;
	        	for (int k = 1; k <= 5; k++) {
	          		float diffX = laserCloud->points[ind + k].x - laserCloud->points[ind + k - 1].x;
	          		float diffY = laserCloud->points[ind + k].y - laserCloud->points[ind + k - 1].y;
	          		float diffZ = laserCloud->points[ind + k].z - laserCloud->points[ind + k - 1].z;
	          		if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
	            		break;
	          		}

	          		cloudNeighborPicked[ind + k] = 1;
	        	}
	        	for (int k = -1; k >= -5; k--) {
	          		float diffX = laserCloud->points[ind + k].x - laserCloud->points[ind + k + 1].x;
	          		float diffY = laserCloud->points[ind + k].y - laserCloud->points[ind + k + 1].y;
	          		float diffZ = laserCloud->points[ind + k].z - laserCloud->points[ind + k + 1].z;
	          		if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
	            		break;
	          		}

		          	cloudNeighborPicked[ind + k] = 1;
	        	}
	      	}
	    }  	

	  	for (int i = sp; i <= ep; i++) {

		    if (cloudLabel[i] <= 0) {
		      surfPointsLessFlat->push_back(laserCloud->points[i]);
		    }
	  	}
	}

  	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
  	pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
  	downSizeFilter.setInputCloud(surfPointsLessFlat);
  	downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
  	downSizeFilter.filter(*surfPointsLessFlatDS);

  	*laserCloudSweepCur += *cornerPointsSharp;
  	*laserCloudSweepCur += *surfPointsFlat;
  	*laserCloudLessSweepCur += *cornerPointsLessSharp;
  	*laserCloudLessSweepCur += *surfPointsLessFlatDS;

  	laserCloudIn->clear();
  	laserCloud->clear();
  	cornerPointsSharp->clear();
  	cornerPointsLessSharp->clear();
  	surfPointsFlat->clear();
  	surfPointsLessFlat->clear();
  	surfPointsLessFlatDS->clear();

    //2개 frame skip
  	if (skipFrameCount >= skipFrameNum) {

    	skipFrameCount = 0;
    
      	sensor_msgs::PointCloud2 laserCloudSweepCur2;
      	pcl::toROSMsg(*laserCloudSweepCur, laserCloudSweepCur2);
      	laserCloudSweepCur2.header.stamp = laserCloudMsg->header.stamp;
      	laserCloudSweepCur2.header.frame_id = "/camera";
      	pubLaserCloudSweepCur.publish(laserCloudSweepCur2);
      	pubLaserCloudLast.publish(laserCloudLast2);
  	}
  	skipFrameCount++;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){
    
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    /*
    //int imuPointerBack = imuPointerLast;  //previous imuPointer
    //imuPointerLast = (imuPointerLast + 1) % imuQueLength; //imuPoint (0~49)

    //latest subscribed imuTime
    //imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
    //imu Hz for mavros Sim timeDiff: 0.02 [50Hz]
    //for microstran Imu timeDiff: 0.01 [100Hz]
    //double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack]; 

    if (timeDiff < 0.1) {

      //imuAccuRoll += timeDiff * imuIn->angular_velocity.x;
      //imuAccuPitch += timeDiff * imuIn->angular_velocity.y;
      //imuAccuYaw += timeDiff * imuIn->angular_velocity.z;

      //imuRoll[imuPointerLast] = roll;
      //imuPitch[imuPointerLast] = -pitch;
      //imuYaw[imuPointerLast] = -yaw;
      //imuRoll[imuPointerLast] = imuAccuRoll;
      //imuPitch[imuPointerLast] = -imuAccuPitch;
      //imuYaw[imuPointerLast] = -imuAccuYaw;

      //imuAccX[imuPointerLast] = -imuIn->linear_acceleration.y;
      //imuAccY[imuPointerLast] = -imuIn->linear_acceleration.z - 9.81;
      //imuAccZ[imuPointerLast] = imuIn->linear_acceleration.x;

      //AccumulateIMUShift();
    }*/
}

int main(int argc, char** argv){

  	ros::init(argc, argv, "laserscan_matcher");
  	ros::NodeHandle nh;

	nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);
  	nh.param<string>("laser_topic", LASER_TOPIC, "sync_scan_cloud_filtered");
  	nh.param<string>("imu_topic", IMU_TOPIC, "microstrain/imu");

  	ros::Subscriber subJointState = nh.subscribe<sensor_msgs::JointState>("spin_hokuyo/joint_states", 100, jointStateHandler);  
  	ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(LASER_TOPIC, 2, laserCloudHandler);
	ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>(IMU_TOPIC, 5, imuHandler);
  	pubLaserCloudSweepCur = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_Sweep_cur", 2);
  	pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_last", 2);


  	ros::spin();
  	return 0; 
}
