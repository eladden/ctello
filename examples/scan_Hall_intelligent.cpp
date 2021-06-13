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
    ORBDrone::DroneState droneState;
    const bool cw = false; //When in lab set to false

    if (verboseDrone) std::cout<< "getting scale...";
    Drone.SetScale();
    if (verboseDrone) std::cout<< "Done ! scale is: " << Drone.GetScale() << std::endl;
    int i=0, iterations=5;

    while (i < iterations){
        droneState = Drone.SeekFloor(cw,380.0f);
        //TODO if not wall then what
        if (droneState == ORBDrone::DroneState::Seeking){
            std::cout<< "could not find a good direction to go to, trying again"<<std::endl;
            i++;
        }
        if (droneState == ORBDrone::DroneState::OK){//This means that the drone is currently not looking at a wall and is facing the far point
            droneState = Drone.AdvanceForward(100.0f, 37); //advance, while making sure this really is not a wall
        }
        else if (droneState == ORBDrone::DroneState::Lost){
            Drone.Scan(!cw,25);
            Drone.Scan(cw,25);
        }

    }
    Drone.SendCommand("land");

    allMapPoints = Drone.GetSLAM()->GetMap()->GetAllMapPoints();
    if (allMapPoints.size() > 0)
    {
        saveMap(1);
    }

}
