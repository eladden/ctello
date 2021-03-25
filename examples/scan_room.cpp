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
#include <vector>
#include <unordered_map>
#include "ctello_ORB_support.h"
#include "ctello.h"


const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000"};

using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;

const bool verboseDrone = true;

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
            else if (sumAngle <= 1.0f &&//index < initialize_commands.size() + scan_times*scan_commands.size() &&//
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
                if (endIdx == end_commands.size()){
                    done=true;
                }
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
            killFrameUpdate = true;
            break;
        }
    }
    pthread_join(UpdThread,nullptr);
    globalCapture.release();
    allMapPoints = SLAM.GetMap()->GetAllMapPoints();
    if (allMapPoints.size() > 0)
    {
        saveMap(0);
    }

    SLAM.Shutdown();
}

