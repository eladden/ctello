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

/************* GLOBALS ***********/
bool writeImages = true, killFrameUpdate=false;
VideoCapture globalCapture;
Mat globalFrame;
pthread_mutex_t frameLocker;
std::vector<ORB_SLAM2::MapPoint*> allMapPoints;
std::vector<ORB_SLAM2::MapPoint*> currMapPoints;
std::string basefilename = "pointData";

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

#endif // CTELLO_ORB_SUPPORT_H
