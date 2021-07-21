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
//#include "ctello.h"
#include "ctello_control.h"

using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;

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


    ORBDrone Drone(argv[1],argv[2]);
    ORBDrone::DroneState droneState = ORBDrone::DroneState::Lost;
    cv::Mat currentFrame, Tcw, currentPosition, previousPosition;
    const bool cw = false;//When in lab set to false
    float turneAngle = 20, keepDistanceFromWall = 130.0f, maxAdvanceDist = 400.0f,wallDist{1e10},movedDist{0.0};
    int step = 40;
    bool needToFindWall = false;

    if (verboseDrone) std::cout<< "getting scale...";
    Drone.SetScale();
    if (verboseDrone) std::cout<< "Done ! scale is: " << Drone.GetScale() << std::endl;
    int i=0, iterations=3,getBackIterations = 3;

    AnalyzedFrame analyzedFrame_(Drone.GetSLAM(),Drone.GetScale(),Drone.GetSelfPoseYSign());
    previousPosition = analyzedFrame_.GetSelfPose();
    if (verboseDrone)
        cout << "current postition: " << std::endl << previousPosition <<  std::endl;
    Drone.SetCurrentAnalyzedFrame(analyzedFrame_);


    //This is a Odesius-like method, go until you're close to the wall, then turn then go forward again.
    while(true){
        //This is the advance loop, you start by goin straight until a wall, turn, go straight, and so on, until you turn about 90deg
        std::cout << "Scanning..." << std::endl;

        while (true){
            AnalyzedFrame analyzedFrame(Drone.GetSLAM(),Drone.GetScale(),Drone.GetSelfPoseYSign());
            if (analyzedFrame.GetClosedLoop()){
                if (verboseDrone) std::cout<< "Closed Loop Detected, rescaling...";
                Drone.SetScale();
                if (verboseDrone) std::cout<< "Done ! scale is: " << Drone.GetScale() << std::endl;
            }
            droneState = Drone.AdvanceForward(keepDistanceFromWall, step, maxAdvanceDist);

            currentPosition = Drone.GetCurrentAnalyzedFrame().GetSelfPose();
            movedDist = sqrt((previousPosition.at<float>(0) - currentPosition.at<float>(0)) * (previousPosition.at<float>(0) - currentPosition.at<float>(0)) +
                             (previousPosition.at<float>(1) - currentPosition.at<float>(1)) * (previousPosition.at<float>(1) - currentPosition.at<float>(1)) +
                             (previousPosition.at<float>(2) - currentPosition.at<float>(2)) * (previousPosition.at<float>(2) - currentPosition.at<float>(2))
                             );
            previousPosition = currentPosition;
            if(verboseDrone)
                cout << "I moved " << movedDist << std::endl;

            needToFindWall = (i >= iterations) && (movedDist > 25.0f);
            if(needToFindWall)
                break;

            if (droneState == ORBDrone::DroneState::TooCloseToWall){
                std::cout << "saving as PointData" << std::to_string(i) << std::endl;
                Drone.GetCurrentAnalyzedFrame().saveFramePoints("PointData" + std::to_string(i));
                Drone.SetCurrentWallDist(1e10);
                i++;
                droneState = Drone.Turn(cw,turneAngle);
            }
            else if (droneState == ORBDrone::DroneState::Lost){
                Drone.Turn(!cw,25);
                //Drone.Turn(cw,25);
            }/*else{//This should happen when the state is not too close and not lost, i.e., still ok, we went the maxadvanced
                //continue;
                needToFindWall = true;
            }*/

        }
        i = 0;
        if (needToFindWall)
            if (verboseDrone) std::cout << "Getting back to wall..." << std::endl;
        while (needToFindWall && i < getBackIterations && droneState != ORBDrone::DroneState::Lost){//(droneState != ORBDrone::DroneState::TooCloseToWall && i < getBackIterations){
            droneState = Drone.Turn(!cw,turneAngle);
            ++i;
            // the state in the turn does not check closeness so we need to check distance from wall manually
            while (!updateSLAM(currentFrame,Tcw,Drone.GetSLAM()));
            AnalyzedFrame analyzedFrame(Drone.GetSLAM(),Drone.GetScale(), Drone.GetSelfPoseYSign());
            wallDist = analyzedFrame.GetMinNonFloorDist();
            Drone.SetCurrentAnalyzedFrame(analyzedFrame);

            if (wallDist > 1e8f){
                if (verboseDrone){
                    std::cout << "Distance too high, did I lose the SLAM?";
                }
                Drone.Turn(!cw,25);
                Drone.Turn(cw,25);
                continue;
            }else if (wallDist <= keepDistanceFromWall && wallDist > 1e-5f){
                if (verboseDrone){
                    std::cout<< "I'm at Wall";
                    std::cout << "WallDist: " << wallDist << std::endl;
                }
                Drone.SetCurrentWallDist(wallDist);
                droneState = ORBDrone::DroneState::TooCloseToWall;
                break;
            }else{
                if (verboseDrone){
                    std::cout<< "I'm NOT at Wall";
                    std::cout << "WallDist: " << wallDist << std::endl;
                }
                droneState = ORBDrone::DroneState::OK;
                droneState = Drone.AdvanceForward(keepDistanceFromWall, step, maxAdvanceDist);
            }

        }
        needToFindWall = false;
        i = 0;
        //if (droneState != ORBDrone::DroneState::TooCloseToWall)

    }
    Drone.SendCommand("land");

    allMapPoints = Drone.GetSLAM()->GetMap()->GetAllMapPoints();
    if (allMapPoints.size() > 0)
    {
        saveMap(1);
    }

}
