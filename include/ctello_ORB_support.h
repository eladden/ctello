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
bool writeImages = true, killFrameUpdate=false;
VideoCapture globalCapture;
Mat globalFrame;
pthread_mutex_t frameLocker;
std::vector<ORB_SLAM2::MapPoint*> allMapPoints;
std::vector<ORB_SLAM2::MapPoint*> currMapPoints;
std::string basefilename = "pointData";

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

class AnalyzedFrame
{
protected:
   bool isWall{false}, isGoodFrame{false}, initialized{false};
   float maxFloorDist{0.0}, minFloorDist{1e10}, minNonFloorDist{1e10}, kValueOnFloor{0.0},scale{1.0};
   cv::Mat rotatedAveragePoint = (Mat_<float>(3,1) << 0.0, 0.0, 0.0),
           averageXYZ = (Mat_<float>(3,1) << 0, 0, 0),
           gapRotated = (Mat_<float>(3,1) << 0.0, 0.0, 0.0),
           minRotated = (Mat_<float>(3,1) << 1e10, 1e10, 1e10),
           maxRotated = (Mat_<float>(3,1) << 0.0, 0.0, 0.0),
           rotatedCovariance = (Mat_<float>(3,3)<< 0.0,0.0,0.0,  0.0,0.0,0.0, 0.0,0.0,0.0);
   int numOfPoints{0}, numOfPointsLowerThanDrone{0};
   unsigned long int frameID{0};
   ORB_SLAM2::KeyFrame* currentKeyFrame;

public:

   AnalyzedFrame (ORB_SLAM2::System *SLAM, float scale_);
   ~AnalyzedFrame(){};

   cv::Mat ComputePoseVec();
   float FloorDist(cv::Mat point1, cv::Mat point2){
       return std::sqrt(std::pow(point1.at<float>(0) - point2.at<float>(0),2) + std::pow(point1.at<float>(1) - point2.at<float>(1),2) );
   }
   //We are swapping the values in position 1 and 2 in the vector so that the vector would be in XYZ.
   cv::Mat XZYtoXYZ(cv::Mat vector);

   bool GetIsWall(){return isWall;}
   bool GetIsGoodFrame(){return isGoodFrame;}
   bool GetInitialized(){return initialized;}
   float GetMaxFloorDist(){return maxFloorDist;}
   float GetMinFloorDist(){return minFloorDist;}
   float GetMinKValueOnFloor(){return kValueOnFloor;}
   float GetMinNonFloorDist(){return minNonFloorDist;}
   cv::Vec<float,3> GetRotatedAveragePoint(){return rotatedAveragePoint;}
   cv::Matx<float,3,3> GetRotatedCovariance(){return rotatedCovariance;}
   int GetNumOfPoints(){return numOfPoints;}
   int GetnumOfPointsLowerThanDrone(){return numOfPointsLowerThanDrone;}
   int GetFrameID(){return frameID;}
};

#endif // CTELLO_ORB_SUPPORT_H
