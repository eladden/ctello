//  CTello is a C++ library to interact with the DJI Ryze Tello Drone
//  Copyright (C) 2020 Carlos Perez-Lopez
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>
//
//  You can contact the author via carlospzlz@gmail.com

#include <iostream>
#include <cstdlib>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<System.h>
#include<Converter.h>
#include<KeyFrame.h>
//#include<sys/socket.h>	//socket
//#include<arpa/inet.h>	//inet_addr
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <csignal>
#include <math.h>
#include <string>
//#include <thread>
#include <pthread.h>
#include <vector>
#include <unordered_map>

#include "ctello.h"
#include "opencv2/imgcodecs.hpp"

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000"};

using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;
/************* GLOBALS ***********/
bool verboseDrone = true, writeImages = true;
VideoCapture globalCapture;
Mat globalFrame;
pthread_mutex_t frameLocker;
std::vector<ORB_SLAM2::MapPoint*> allMapPoints;
std::vector<ORB_SLAM2::MapPoint*> currMapPoints;

/************* PATHS *************/
//const std::string slam_to_tello_file = "/tmp/slam_to_tello.txt";
//const std::string slam_to_tello_file_current_map = "/tmp/slam_to_tello_file_current_map.txt";
std::string basefilename = "pointData";
//const char RequsetSavePoints= '1';
//const char PointsSaved = '2';
//const char CheckSlamStatus = '3';
//const char Exit = '5';
//char Localized = '0';
//int closeProgram = 0;
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

void *UpdateFrame(void *arg)
{
    for(int i=0;;++i)
    {
        Mat tempFrame;
        try {
            globalCapture >> tempFrame;
            cv::resize(tempFrame, tempFrame, cv::Size(960, 720));
            if (writeImages && !tempFrame.empty()){
                cv::imwrite(basefilename + std::to_string(i) + ".jpg",tempFrame);
            }
        } catch (const std::exception& e) {
            std::cout << "Capture error: " << e.what();
        }

        pthread_mutex_lock(&frameLocker);
        globalFrame = tempFrame;
        pthread_mutex_unlock(&frameLocker);
    }
}

void saveCurrentPosition(cv::Mat Tcw){
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    std::ofstream currentPostionFile;
    currentPostionFile.open("tello_last_location.csv",std::ofstream::out | std::ofstream::trunc);
    currentPostionFile << twc.at<float>(0)  << "," << twc.at<float>(1)  << "," << twc.at<float>(2) << "," << q[0]
    << "," << q[1]<< "," << q[2]<< "," << q[3] << std::endl;
    currentPostionFile.close();
}

void saveCurrentMap(){
    std::ofstream currentPointData;
    currentPointData.open(basefilename + ".csv");
    for(auto p : currMapPoints) {
        Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
        currentPointData << v.x() << "," << v.y() << "," << v.z() << std::endl;
    }

    currentPointData.close();
}

void saveMap( int fileNumber=0 ) {

    std::string dirname = basefilename + "Dir";
    std::string mkdircommand = "mkdir " + dirname;
    system(mkdircommand.c_str());
    mkdircommand = "mkdir " + dirname + "/Frames";
    system(mkdircommand.c_str());
    std::ofstream pointData;
    std::vector<int> savedFrames;
    pointData.open(dirname + "//" + basefilename + std::to_string(fileNumber) + ".csv");
    for(auto p : allMapPoints) {
        if (p != NULL)
        {
            auto frame = p->GetReferenceKeyFrame();
            int frameId = frame->mnFrameId;
            cv::Mat Tcw = frame->GetPose();
            auto point = p->GetWorldPos();
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            auto q = ORB_SLAM2::Converter::toQuaternion(Rwc);
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<< "," << q[0]
            << "," << q[1]<< "," << q[2]<< "," << q[3] << "," << frameId <<
            ","<< twc.at<float>(0)  << "," << twc.at<float>(1)  << "," << twc.at<float>(2) << std::endl;

            if (writeImages &&
                    std::find(savedFrames.begin(), savedFrames.end(), frameId) == savedFrames.end())
            {
                savedFrames.push_back(frameId);
                const string command = "mv " + basefilename + std::to_string(frameId) + ".jpg ./" + dirname + "/Frames";
                system(command.c_str());
            }

        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;
    system("rm *.jpg");
}

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cerr << std::endl << "Usage: ./tello_SLAM path_to_vocabulary path_to_settings" << std::endl;
        std::cout << "Example command: ./tello_SLAM ../../Vocabulary/ORBvoc.txt tello.yaml" << std::endl;
        return -2;
    }
    std::cout << "Starting SLAM...";
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    std::cout << "SLAM  on!" << std::endl;

    std::cout << "Starting Tello..." << std::endl;
    Tello tello{};
    if (!tello.Bind())
    {
        return 0;
    }
    std::cout << "Bound Tello, starting stream...";

    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()))
        ;
    std::cout << "Stream  on!" << std::endl;


    std::cout << "Starting video...";
    globalCapture.open(TELLO_STREAM_URL,CAP_FFMPEG);

    pthread_mutex_init(&frameLocker,nullptr);
    pthread_t UpdThread;
    pthread_create(&UpdThread, nullptr, UpdateFrame, nullptr);
    std::cout << "Video started" << std::endl;

    std::array<std::string, 3> initialize_commands{"takeoff", "up 25", "down 20"};
    std::array<std::string, 3> scan_commands{"cw 25", "up 25", "down 25"};
    std::array<std::string, 3> end_commands{"land", "battery?","streamoff"};
    std::array<std::string, 3> lost_commands{"ccw 20","up 25", "down 25"};
    //unsigned scan_times = 20, total_commands = initialize_commands.size()+end_commands.size()+scan_times*scan_commands.size();
    unsigned index{0}, lostIdx{0}, endIdx{0};
    float initialAngle=0, currentAngle=0, previousAngle=0, sumAngle=0;
    bool busy{false},gotNewFrame{false},done{false};
    while (true)
    {
        // See surrounding.

        //capture >> frame;
        Mat currentFrame;
        ORB_SLAM2::KeyFrame* currentKeyFrame;// = SLAM.GetMap()->GetAllKeyFrames().back();
        double currentFrameMilisecondPos;
        pthread_mutex_lock(&frameLocker);
        currentFrame = globalFrame;
        currentFrameMilisecondPos = globalCapture.get(cv::CAP_PROP_POS_MSEC);
        pthread_mutex_unlock(&frameLocker);
        if(currentFrame.empty()){
//            std::cout<< "recieved empty frame" << std::endl;
            gotNewFrame = false;
        }
        else
            gotNewFrame = true;
        if (gotNewFrame){
            try {
                SLAM.TrackMonocular(currentFrame,currentFrameMilisecondPos);
            } catch (const std::exception& e) {
                cout << "ORBSLAM error: " << e.what();
            }
        }
        if (const auto response = tello.ReceiveResponse())
        {
            if (verboseDrone)
                std::cout << "Tello: " << *response << std::endl;
            busy = false;
        }

        //get the rotation of the current frame (we could use quaternions, but I'd rather use the rotation mat. I f* hate quaternions dude
        Mat Rwc;
        if (gotNewFrame && SLAM.GetTrackingState() == ORB_SLAM2::Tracking::OK) {
            currentKeyFrame = SLAM.GetMap()->GetAllKeyFrames().back();
            Rwc = currentKeyFrame->GetPose();
            //std::cout << "current rotation = " << std::endl << " "  << Rwc << std::endl << std::endl;
            //std::cout << "-sin(beta) = " << Rwc.at<float>(2,0);// << std::endl;
            //Rwc = Rz(a)Ry(b)Rx(g) = [c(a)c(b) c(a)s(b)s(g)-s(a)c(g) c(a)s(b)c(g)+s(a)s(g)
            //                         s(a)c(b) s(a)s(b)s(g)+c(a)c(g) s(a)s(b)c(g)-c(a)s(g)
            //                         -s(b)          c(b)s(g)              c(b)c(g)]
            //So to get the rotation angle alpha around the y axis we can use Rwc(0,1) = -s(b) to compute beta
            float beta = asin(-Rwc.at<float>(2,0)); //This is the angle of rotation around the y axis (beta)
            currentAngle = beta;
        }
        // Act
        if (!busy)// && (index < total_commands))
        {
            std::string command;
            if (index < initialize_commands.size()){
                command = initialize_commands[index];
                initialAngle = currentAngle;
                std::cout << "initial angle: " << initialAngle << std::endl;
            }
            else if (sumAngle <= 6.0f &&//index < initialize_commands.size() + scan_times*scan_commands.size() &&//
                     SLAM.GetTrackingState() == ORB_SLAM2::Tracking::OK &&
                     lostIdx%lost_commands.size() == 0 &&
                     endIdx == 0 && !done){
                command = scan_commands[(index-initialize_commands.size())%scan_commands.size()];
                if (currentAngle >= previousAngle){
                    sumAngle += currentAngle-previousAngle;
                }
                else{
                    sumAngle += previousAngle - currentAngle;
                }
                std::cout << "current angle: " << sumAngle << std::endl;
            }
            else if ((SLAM.GetTrackingState() == ORB_SLAM2::Tracking::LOST || (lostIdx)%lost_commands.size() != 0) &&
                     endIdx == 0 && !done){
                command = lost_commands[(lostIdx)%lost_commands.size()];
                lostIdx++;
                //std::cout << "current angle: " << currentAngle << std::endl;
            }
            else{
                command = end_commands[endIdx];
                endIdx++;
                if (endIdx == end_commands.size())
                        done=true;
            }
            ++index;
            previousAngle = currentAngle;
            tello.SendCommand(command);
            if(verboseDrone){
                std::cout << "Command: " << command << std::endl;
            }
            busy = true;
        }

        if (done){
            std::cout << "Done!" << std::endl;
            break;
        }
    }
    pthread_cancel(UpdThread);
    globalCapture.release();
    allMapPoints = SLAM.GetMap()->GetAllMapPoints();
    if (allMapPoints.size() > 0)
    {
        saveMap(0);
    }

    SLAM.Shutdown();
}

