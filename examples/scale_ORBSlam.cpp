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

const bool verboseDrone = false; //this is if you need the drone commands/replys for debug

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

    // The main loop sleeps every time you wait for the drone's reply. So it's best if the frame update is done in a seperate frame
    globalCapture.open(TELLO_STREAM_URL,CAP_FFMPEG);

    pthread_mutex_init(&frameLocker,nullptr);
    pthread_t UpdThread;
    pthread_create(&UpdThread, nullptr, UpdateFrame, nullptr);
    std::cout << "Video started" << std::endl;

    //constants for the loop

    std::array<std::string, 3> initialize_commands{"takeoff", "up 25", "down 25"};
    std::array<std::string, 2> lost_commands{"up 25", "down 25"};
    std::array<std::string, 4> scale_commands{"up 50", "down 50", "up 100", "down 100"};
    //std::array<std::string, 3> test_commands{"cw 25", "up 25","down 25"};
    std::array<std::string, 3> end_commands{"land", "battery?","streamoff"};

    unsigned index{0}, endIdx{0}, lostIdx{0};
    double previousHeightSLAM{0}, currentHeightSLAM{0}, previousHeightDrone{0}, currentHeightDrone{0}, initialHeightDrone{0}, initialHeightSLAM{0};
    double scaleFromInit{0}, scaleFromPrev{0};
    bool busy{false},gotNewFrame{false},done{false};

    // main loop
    while (true)
    {
        // See surrounding.

        Mat currentFrame;
        ORB_SLAM2::KeyFrame* currentKeyFrame;
        double currentFrameMilisecondPos;

        pthread_mutex_lock(&frameLocker);
        currentFrame = globalFrame;
        currentFrameMilisecondPos = globalCapture.get(cv::CAP_PROP_POS_MSEC);
        pthread_mutex_unlock(&frameLocker);

        if(currentFrame.empty()){
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

        std::cout << "The drone height is : " << tello.GetHeight() << std::endl;

        //Get the position of the current frame (we could use quaternions, but I'd rather use the rotation mat. I f* hate quaternions dude
        Mat twc;
        if (gotNewFrame && SLAM.GetTrackingState() == ORB_SLAM2::Tracking::OK) {
            currentKeyFrame = SLAM.GetMap()->GetAllKeyFrames().back();
            twc = currentKeyFrame->GetTranslation();
            currentHeightSLAM = twc.at<double>(0,1);
            currentHeightDrone = tello.GetHeight() * 10; //the height is given in dm, multiply to convert to cm
        }
        // Act
        if (!busy)// && (index < total_commands))
        {
            std::string command;
            if (index < initialize_commands.size()){
                command = initialize_commands[index++];
                if (index == initialize_commands.size()){
                    initialHeightDrone = currentHeightDrone;
                    initialHeightSLAM  = currentHeightSLAM;

                    std::cout << "Height measured by drone: " << initialHeightSLAM << "cm";
                    std::cout << "Height estimated by SLAM: " << initialHeightSLAM<< "cm";
                }
            }
            else if (index < initialize_commands.size() + scale_commands.size() &&
                     SLAM.GetTrackingState() == ORB_SLAM2::Tracking::OK &&
                     lostIdx%lost_commands.size() !=0){
                command = scale_commands[index++];
                scaleFromInit += std::sqrt(std::pow(currentHeightSLAM,2) - std::pow(initialHeightSLAM,2))/
                        std::sqrt(std::pow(currentHeightDrone,2) - std::pow(initialHeightDrone,2));
                scaleFromPrev += std::sqrt(std::pow(currentHeightSLAM,2) - std::pow(previousHeightSLAM,2))/
                        std::sqrt(std::pow(currentHeightDrone,2) - std::pow(previousHeightDrone,2));

            }
            else if ((SLAM.GetTrackingState() == ORB_SLAM2::Tracking::LOST || (lostIdx)%lost_commands.size() != 0) &&
                     endIdx == 0 && !done){
                command = lost_commands[(lostIdx)%lost_commands.size()];
                lostIdx++;
            }
            else{
                scaleFromInit = scaleFromInit/(index-initialize_commands.size());
                scaleFromPrev = scaleFromPrev/(index-initialize_commands.size());
                command = end_commands[endIdx++];
                if (endIdx == end_commands.size()){
                    done=true;
                }
            }

            previousHeightSLAM = currentHeightSLAM;
            previousHeightDrone = currentHeightDrone;
            tello.SendCommand(command);
            if(verboseDrone){
                std::cout << "Command: " << command << std::endl;
            }
            busy = true;
        }

        if (done){
            std::cout << "Done!" << std::endl;
            killFrameUpdate = true; //Shut down the frame update thread main loop
            break;
        }
    }
    pthread_join(UpdThread,nullptr);
    globalCapture.release();

    std::cout << "The scale from the initial height: " << scaleFromInit << "cm";
    std::cout << "The scale from the previous height: " << scaleFromPrev<< "cm";

    SLAM.Shutdown();
}

