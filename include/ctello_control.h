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
    float scale, currentWallDist,selfPoseYSign;
    bool writeImages,initialized;
    std::chrono::time_point<std::chrono::system_clock> actionSent;
    std::string analyzedFramesFilename;
    std::ofstream analyzedFramesFile;


public:

    enum DroneState{
        Seeking=2,
        OK=1,
        Lost=-1,
        TooCloseToWall=0
    };

    ORBDrone(char* vocPath, char* settingsPath);
    ~ORBDrone();

    //These are parameters optimized
    void SetOptParams(float epsilon_for_Ripples, float search_radius1_param, float search_radius2_param, float floatthreshold, float min_num_of_points);

    DroneState Turn(bool cw, float maxAngle, bool analyzeAndSave);
    DroneState Turn(bool cw, float maxAngle){return Turn(cw,maxAngle,false);};
    DroneState SeekFloor(bool cw, float maxAngle);
    DroneState AdvanceForward(float distFromWall, int step, float maxDist);
    //bool IsAWall_Opt(AnalyzedFrame FrameInfo);
    bool IsAWall(AnalyzedFrame FrameInfo);


    void ReScanLostSLAM();
    void ScanHall();
    bool SetScale();

    AnalyzedFrame GetCurrentAnalyzedFrame(){return currentAnalyzedFrame;}
    void SetCurrentAnalyzedFrame(AnalyzedFrame value){currentAnalyzedFrame = value;}
    ORB_SLAM2::System* GetSLAM(){return SLAM;}
    ctello::Tello* GetDrone(){return Drone;}
    float GetScale(){return scale;}
    float GetSelfPoseYSign(){return selfPoseYSign;}
    bool GetWriteImages(){return writeImages;}
    void SetWriteImages(bool value) {writeImages = value;}
    float GetCurrentWallDist(){return currentWallDist;}
    void SetCurrentWallDist(float value) {currentWallDist = value;}
    std::string GetAnalyzedFramesFilename(){return analyzedFramesFilename;};
    void SetAnalyzedFramesFilename(std::string Filename){analyzedFramesFilename = Filename;};

    void OpenAnalyzedFrameFile();
    std::ofstream *GetAnalyzedFramesFile(){return &analyzedFramesFile;};

    std::optional<string> GotDroneResponse();
    void SendCommand(std::string command);



private:
    float ComputeMSD(float number);
};

#endif // CTELLO_CONTROL_H
