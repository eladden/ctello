#ifndef CTELLO_CONTROL_H
#define CTELLO_CONTROL_H

#include "ctello.h"
#include "ctello_ORB_support.h"
#include <chrono>

class ORBDrone
{
    ORB_SLAM2::System *SLAM;
    ctello::Tello *Drone;
    AnalyzedFrame currentAnalyzedFrame;
    pthread_t UpdThread;
    float scale;
    bool writeImages,initialized;
    std::chrono::time_point<std::chrono::system_clock> actionSent;


public:

    enum DroneState{
        OK=1,
        Lost=-1,
        TooCloseToWall=0
    };

    ORBDrone(char* vocPath, char* settingsPath);
    ~ORBDrone();

    DroneState Scan(bool cw, float maxAngle);
    DroneState SeekFloor(bool we, maxAngle);
    DroneState AdvanceForward(float distFromWall);
    bool IsAWall_Opt(AnalyzedFrame FrameInfo);
    bool IsAWall_Tree(AnalyzedFrame FrameInfo);

    void ReScanLostSLAM();
    void ScanHall();
    bool SetScale();

    AnalyzedFrame GetCurrentAnalyzedFrame(){return currentAnalyzedFrame;}
    ORB_SLAM2::System* GetSLAM(){return SLAM;}
    ctello::Tello* GetDrone(){return Drone;}
    float GetScale(){return scale;}
    bool GetWriteImages(){return writeImages;}
    void SetWriteImages(bool value) {writeImages = value;}

    std::optional<string> GotDroneResponse();
    void SendCommand(std::string command);




};

#endif // CTELLO_CONTROL_H
