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
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <csignal>
#include <math.h>
#include <string>
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

bool updateSLAM(cv::Mat &currentFrame_, cv::Mat &Tcw_,
                 ORB_SLAM2::System * SLAM_){

    double currentFrameMilisecondPos;
    bool gotNewFrame = false;
    pthread_mutex_lock(&frameLocker);
    currentFrame_ = globalFrame;
    currentFrameMilisecondPos = globalCapture.get(cv::CAP_PROP_POS_MSEC);
    pthread_mutex_unlock(&frameLocker);

    if(currentFrame_.empty()){
        gotNewFrame = false;
    }
    else{
        gotNewFrame = true;
    }
    if (gotNewFrame){
        try {
            Tcw_ = SLAM_->TrackMonocular(currentFrame_,currentFrameMilisecondPos);
        } catch (const std::exception& e) {
            cout << "ORBSLAM error: " << e.what();
        }
    }
    return gotNewFrame;
}

int main(int argc, char **argv)
{
    writeImages = false; //TODO add options for this as well
    if (!verboseDrone)
        std::setvbuf(stderr,nullptr,_IOFBF,1024); // This gets rid of ffmpeg error messages and buffers theme to oblivion
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
    if (!tello.Bind(LOCAL_CLIENT_COMMAND_PORT,""))
    {
        return 0;
    }
    std::cout << "Bound Tello, starting stream...";

    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()))
        ;
    std::cout << "Stream  on!" << std::endl;


    std::cout << "Starting video...";

    // The main loop sleeps every time you wait for the drone's reply. So it's best if the frame update is done in a seperate frame
    globalCapture.open(TELLO_STREAM_URL,CAP_FFMPEG);

    pthread_mutex_init(&frameLocker,nullptr);
    pthread_t UpdThread;
    pthread_create(&UpdThread, nullptr, UpdateFrame, nullptr);
    std::cout << "Video started" << std::endl;

    //constants for the loop

    std::array<std::string, 5> initialize_commands{"takeoff", "up 25", "down 25", "up 60", "down 60"};
    std::array<std::string, 2> lost_commands{"up 25", "down 25"};
    std::array<std::string, 2> scale_commands{"up 65", "down 65"};
    //std::array<std::string, 3> test_commands{"cw 25", "up 25","down 25"};
    //std::array<std::string, 2> end_commands{"land", "streamoff"};

    unsigned index{0}, endIdx{0}, lostIdx{0}, busyIdx{0};
    const unsigned repetitions{6};
    double previousHeightSLAM{0}, currentHeightSLAM{0}, previousHeightDrone{0}, currentHeightDrone{0};//, initialHeightDrone{0}, initialHeightSLAM{0};
    double /*scaleFromInit{0},*/ scaleFromPrev{0};
    bool busy{false},gotNewFrame{false},done{false};
    cv::Mat currentFrame;
    cv::Mat Tcw;

    // scaling loop
    while (!done)
    {
        if (const std::optional<string> response = tello.ReceiveResponse())
        {
            std::string respStr = response.value();
            if (respStr.find("OK") != std::string::npos ||
                    respStr.find("ok") != std::string::npos)
            {
                busy = false;
                busyIdx = 0;
            }else if (respStr.find("ERROR") != std::string::npos ||
                         respStr.find("error") != std::string::npos)
            {
                busyIdx = 1000;
            }

            else
                busyIdx++;
            //TODO: add other possible tello outputs here like can't find IMU and whatnot
            if (verboseDrone)
                std::cout << "Tello: " << respStr << std::endl;
        }

        // See surrounding.
        gotNewFrame = updateSLAM(currentFrame,Tcw,&SLAM);

        // Act
        if (!busy && !done)// && (index < total_commands))
        {
            //Get the position of the current frame (we could use quaternions, but I'd rather use the rotation mat. I f* hate quaternions dude

            if (gotNewFrame && SLAM.GetTrackingState() == ORB_SLAM2::Tracking::OK) {

                tello.SendCommand("height?");
                std::optional<string> response = tello.ReceiveResponse();

                while (!done)
                {
                    gotNewFrame = updateSLAM(currentFrame,Tcw,&SLAM);

                    if (response){
                        std::string respStr = response.value();

                        if (respStr.find("ERROR") != std::string::npos ||
                            respStr.find("error") != std::string::npos){
                            tello.SendCommand("land");
                            std::cerr << "got an error from drone" << std::endl;
                            abort();
                        }

                        currentHeightDrone = std::stod(respStr) * 10;

                        break;
                    }
                    else{
                        response = tello.ReceiveResponse();
                    }

                }
                //cv::Mat Tcw = currentKeyFrame->GetPose(); //Position homogenious mat.
                cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); //Rotation in world coordinates
                cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation in world coordinates

                currentHeightSLAM = -static_cast<double>(twc.at<float>(2)); // see? simple!
                 //the height is given in dm, multiply to convert to cm
                if (verboseDrone){
                    //std::cout << "Rwc = " << std::endl << " " << Rwc << std::endl << std::endl;
                    std::cout << "Current height measured by drone: " << currentHeightDrone << "cm" << std::endl;
                    std::cout << "Current height estimated by SLAM: " << currentHeightSLAM << std::endl;
                }
            }
            std::string command;
            if (index < initialize_commands.size()){
                command = initialize_commands[index++];
            }
            else if (index < initialize_commands.size() + scale_commands.size()*repetitions &&
                     SLAM.GetTrackingState() == ORB_SLAM2::Tracking::OK &&
                     lostIdx%lost_commands.size() == 0 && endIdx == 0 && !done){
//                if (index == initialize_commands.size()){
//                    SLAM.ActivateLocalizationMode();
//                }

                command = scale_commands[(index - initialize_commands.size())%scale_commands.size()];

                scaleFromPrev += std::sqrt(std::pow(currentHeightSLAM - previousHeightSLAM,2))/
                        std::sqrt(std::pow(currentHeightDrone - previousHeightDrone,2));
                //std::cout << "added " << index - initialize_commands.size() << std::endl;
                index++;

            }
            else if ((SLAM.GetTrackingState() == ORB_SLAM2::Tracking::LOST || (lostIdx)%lost_commands.size() != 0) &&
                     endIdx == 0 && !done){
                command = lost_commands[(lostIdx)%lost_commands.size()];
                lostIdx++;
            }
            else{
                //command = end_commands[endIdx++];
                //if (endIdx == end_commands.size()){
                    //scaleFromInit = scaleFromInit/(scale_commands.size()*repetitions);
                    scaleFromPrev = scaleFromPrev/(scale_commands.size()*repetitions);
                    done=true;
                    std::cout << "Done!" << std::endl;
                    break;
                //}
            }

            previousHeightSLAM = currentHeightSLAM;
            previousHeightDrone = currentHeightDrone;
            if (!done){
                tello.SendCommand(command);
                if(verboseDrone){
                    std::cout << "Command: " << command << std::endl;
                }
                busy = true;

            }
        }

//        if (done){
//            while (!(tello.ReceiveResponse()))
//                ;
//            std::cout << "Done!" << std::endl;
//            killFrameUpdate = true; //Shut down the frame update thread main loop
//            break;
//        }
        if (busyIdx >= 100){
            std::cout << "Something went wrong!" << std::endl;
            tello.SendCommandWithResponse("land");
            break;
        }
    }//End of scaling loop


    std::cout << "tello bat: " << tello.GetBatteryStatus() << std::endl;
    float scale = static_cast<float>(scaleFromPrev = 1/scaleFromPrev);
    std::cout << "The scale from the previous height: " << scale<< std::endl;

    allMapPoints = SLAM.GetMap()->GetAllMapPoints();
    if (allMapPoints.size() > 0)
    {
        saveMap(0);
    }

    ///////// Now we have a scale, lets see if we can get distances
    float wallDist{1e10};
    if (verboseDrone)
        std::cout << "starting forward loop..." << std::endl;
    tello.SendCommand("forward 25");
    if(verboseDrone){
        std::cout << "Command: " << "forward 25" << std::endl;
    }
    while(SLAM.GetTrackingState()!=ORB_SLAM2::Tracking::LOST && wallDist > 70){


        gotNewFrame = updateSLAM(currentFrame,Tcw,&SLAM);
        if (gotNewFrame){
            AnalyzedFrame analyzedFrame(&SLAM,scale);
            wallDist = analyzedFrame.GetMinNonFloorDist();
        }


        if (const auto response = tello.ReceiveResponse())
        {
            if (verboseDrone){
                std::cout << "Tello: " << *response << std::endl;
                std::cout << "WallDist: " << wallDist<< std::endl;
            }
            tello.SendCommand("forward 25");
            if(verboseDrone){
                std::cout << "Command: " << "forward 25" << std::endl;
            }
        }
    }
    if (verboseDrone)
        std::cout << "WallDist at land: " << wallDist<< std::endl;
    tello.SendCommandWithResponse("land");
    killFrameUpdate = true;
    pthread_join(UpdThread,nullptr);
    globalCapture.release();
    std::cout << "tello bat: " << tello.GetBatteryStatus() << std::endl;

    //SLAM.SaveKeyFrameTrajectoryTUM("traject.csv");

    SLAM.Shutdown();
}

