#ifndef CTELLO_ORB_SUPPORT_H
#define CTELLO_ORB_SUPPORT_H

#include<System.h>
#include<Converter.h>
#include<KeyFrame.h>
#include "opencv2/imgcodecs.hpp"
#include <pthread.h>
#include <string>
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <boost/bind/bind.hpp>

#include "System.h"

/************* GLOBALS ***********/
float PI = static_cast<float>(M_PI);
bool writeImages = true, killFrameUpdate=false;
VideoCapture globalCapture;
Mat globalFrame;
pthread_mutex_t frameLocker;
std::vector<ORB_SLAM2::MapPoint*> allMapPoints;
std::vector<ORB_SLAM2::MapPoint*> currMapPoints;
std::string basefilename = "pointData";
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000"};

const bool verboseDrone = true; //this is if you need the drone commands/replys for debug

/************* OLD ORBSLAM *************/
#define ROWS 720
#define COLS 960
#define COLORS 3
#define NotLocalized 6
#define Localized 7
void send_signal(int signal);
int get_signal(std::string fileName);
void save_points();
void save_curr_points();

void *UpdateFrame(void *arg);

void saveCurrentPosition(cv::Mat Tcw);
void saveMap( int fileNumber = 0);
void saveCurrentMap();
bool updateSLAM(cv::Mat &currentFrame_, cv::Mat &Tcw_,
                 ORB_SLAM2::System * SLAM_);

class AnalyzedFrame
{
protected:

   bool isWall, isGoodFrame, initialized,closedLoop;
   float maxFloorDist, minFloorDist, minNonFloorDist, kValueOnFloor, avgDist,scale;
   cv::Mat rotatedAveragePoint, averageXYZ, gapRotated, minRotated,
           maxRotated, selfPose, minWallPoint, minFloorPoint, maxFloorPoint, maxFloorPointRotated, rotatedCovariance;
   int numOfPoints, numOfPointsLowerThanDrone, numOfPointsHeigherThanDrone;
   unsigned long int frameID;
   ORB_SLAM2::KeyFrame* currentKeyFrame;
   set<ORB_SLAM2::MapPoint*> allPoints;//,pointsAbove,pointsBelow;

   //CV2 only knows how to multiply double scalar and double matrix. If your data structure is float you might get garbage through implicit conversions
   cv::Mat FloatMatScalarMult(cv::Mat Matrix, float scalar);

public:

   AnalyzedFrame (ORB_SLAM2::System *SLAM, float scale_, float selfPoseYSign);
   AnalyzedFrame() : isWall(false), isGoodFrame(false), initialized(false), closedLoop(false),
                       maxFloorDist(0.0), minFloorDist(1e10), minNonFloorDist(1e10), kValueOnFloor(0.0), avgDist(0.0), scale(1.0),
                        numOfPoints(0), numOfPointsLowerThanDrone(0), numOfPointsHeigherThanDrone(0),frameID(0), currentKeyFrame(nullptr){
       rotatedAveragePoint = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
       averageXYZ = (Mat_<float>(3,1) << 0, 0, 0);
       gapRotated = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
       minRotated = (Mat_<float>(3,1) << 1e10, 1e10, 1e10);
       maxRotated = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
       selfPose   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
       minWallPoint   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
       minFloorPoint   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
       maxFloorPoint   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
       maxFloorPointRotated   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
       rotatedCovariance = (Mat_<float>(3,3)<< 0.0,0.0,0.0,  0.0,0.0,0.0, 0.0,0.0,0.0);

   }
   ~AnalyzedFrame(){}

   cv::Mat ComputePoseVec();
   float FloorDist(cv::Mat point1, cv::Mat point2){
       return std::sqrt(std::pow(point1.at<float>(0) - point2.at<float>(0),2.0f) + std::pow(point1.at<float>(2) - point2.at<float>(2),2.0f) );
   }

   float K_function(std::vector<cv::Mat> points, float area, float search_radius);
   bool ArePointsScattered(std::vector<cv::Mat> points, float area, float search_radius, float epsilon);
   //static cv::Mat XZYtoXYZ(cv::Mat vector);

   bool GetIsWall(){return isWall;}
   void SetIsWall(bool value) {isWall = value;}
   bool GetIsGoodFrame(){return isGoodFrame;}
   bool GetInitialized(){return initialized;}
   bool GetClosedLoop(){return closedLoop;}
   float GetMaxFloorDist(){return maxFloorDist;}
   float GetMinFloorDist(){return minFloorDist;}
   float GetMinKValueOnFloor(){return kValueOnFloor;}
   float GetMinNonFloorDist(){return minNonFloorDist;}
   float GetAverageDistance(){return avgDist;}
   cv::Mat GetAverageXYZPoint(){return averageXYZ;}
   cv::Mat GetRotatedAveragePoint(){return rotatedAveragePoint;}
   cv::Mat GetRotatedCovariance(){return rotatedCovariance;}
   cv::Mat GetSelfPose(){return selfPose;}
   cv::Mat GetMaxFloorPoint(){return maxFloorPoint;}
   cv::Mat GetMaxmaxFloorPointRotated(){return maxFloorPointRotated;}
   cv::Mat GetMinFloorPoint(){return minFloorPoint;}
   cv::Mat GetMinWallPoint(){return minWallPoint;}
   int GetNumOfPoints(){return numOfPoints;}
   int GetNumOfPointsLowerThanDrone(){return numOfPointsLowerThanDrone;}
   int GetNumOfPointsHeigherThanDrone(){return numOfPointsHeigherThanDrone;}
   int GetFrameID(){return frameID;}

   void saveFramePoints(string filename);
};

#endif // CTELLO_ORB_SUPPORT_H
