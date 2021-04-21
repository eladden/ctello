#include "include/ctello_control.h"

ORBDrone::ORBDrone(char *vocPath, char *settingsPath){
    scale = 0.0;
    initialized = false;
    writeImages = false; //TODO add options for this as well
    if (!verboseDrone)
        std::setvbuf(stderr,nullptr,_IOFBF,1024); // This gets rid of ffmpeg error messages and buffers theme to oblivion

    std::cout << "Starting SLAM...";
    SLAM = new ORB_SLAM2::System (vocPath,settingsPath,ORB_SLAM2::System::MONOCULAR,true);
    std::cout << "SLAM  on!" << std::endl;

    std::cout << "Starting Tello..." << std::endl;
    Drone = new ctello::Tello;
    if (!Drone->Bind(LOCAL_CLIENT_COMMAND_PORT,""))
    {
        cerr <<"could not bind with tello"<<std::endl;
        abort();
    }
    std::cout << "Bound Tello, starting stream...";

    Drone->SendCommand("streamon");
    while (!(Drone->ReceiveResponse()))
        ;
    std::cout << "Stream  on!" << std::endl;


    std::cout << "Starting video...";

    // The main loop sleeps every time you wait for the drone's reply. So it's best if the frame update is done in a seperate frame
    globalCapture.open(TELLO_STREAM_URL,CAP_FFMPEG);

    pthread_mutex_init(&frameLocker,nullptr);
    pthread_create(&UpdThread, nullptr, UpdateFrame, nullptr);
    std::cout << "Video started" << std::endl;

    std::array<std::string, 6> initialize_commands{"takeoff","left 25", "right 50", "left 25", "up 25", "down 25"};

    unsigned index{0};
    bool busy{false},done{false};
    cv::Mat currentFrame,Tcw;
    std::string command = initialize_commands[index];

    while(!done){

        //See
        updateSLAM(currentFrame,Tcw,SLAM);
        if (SLAM->GetTrackingState() == ORB_SLAM2::Tracking::OK)
        {
            done = true;
            initialized = true;
            break;
        }
        //Act
        if ((!busy && !done)){
            if (index < initialize_commands.size()){
                command = initialize_commands[index++];
                busy = true;
            }
            else{
                std::cerr << "Failed to initialize SLAM" << std::endl;
                Drone->SendCommand("land");
                abort();
            }

            if (!done){
                /*Drone->SendCommand(command);
                if(verboseDrone)
                   std::cout << "Command: " << command << std::endl;*/
                SendCommand(command);
            }
        }//end if not busy


        //Did we get a reply?
        busy = !GotDroneResponse().has_value();

    }//end while loop
}

ORBDrone::DroneState ORBDrone::Scan(bool cw,float maxAngleDegrees){
    float maxAngle = maxAngleDegrees*PI/180.0f;
    const unsigned int command_list_size = 3;
    std::array<std::string, command_list_size> cw_commands{"cw 20", "up 25", "down 25"};
    std::array<std::string, command_list_size> ccw_commands{"ccw 20","up 25", "down 25"};

    unsigned index{0}, lostIdx{0};
    float currentAngle=0, previousAngle=0, sumAngle=0;
    bool busy{false},gotNewFrame{false},done{false};
    cv::Mat currentFrame,Tcw;
    ORB_SLAM2::KeyFrame* currentKeyFrame;
    std::string command;

    // main loop
    while (true)
    {
        // See surrounding.

        gotNewFrame = updateSLAM(currentFrame,Tcw,SLAM);

        //if (GotDroneResponse()){
        busy = !GotDroneResponse().has_value();
        //}

        //Get the rotation of the current frame (we could use quaternions, but I'd rather use the rotation mat. I f* hate quaternions dude
        Mat Rwc;
        if (gotNewFrame && SLAM->GetTrackingState() == ORB_SLAM2::Tracking::OK) {
            currentKeyFrame = SLAM->GetMap()->GetAllKeyFrames().back();
            Rwc = currentKeyFrame->GetRotation();
            //Rwc = Rz(a)Ry(b)Rx(g) = [c(a)c(b) c(a)s(b)s(g)-s(a)c(g) c(a)s(b)c(g)+s(a)s(g)
            //                         s(a)c(b) s(a)s(b)s(g)+c(a)c(g) s(a)s(b)c(g)-c(a)s(g)
            //                         -s(b)          c(b)s(g)              c(b)c(g)]
            //So to get the rotation angle beta around the y axis we can use Rwc(0,1) = -s(b)
            float beta = asin(-Rwc.at<float>(2,0)); //This is the angle of rotation around the y axis (beta)
            currentAngle = beta; // see? simple!
        }
        // Act
        if (!busy)// && (index < total_commands))
        {
            //The angle we get is beta = [-pi,pi]. It starts at 0, then goes up or down,
            // but eventually we get to either pi or -pi then we have a jump.
            // To see how much the drone rotated, we measure the difference from the previous state in sumAngle
            if (sumAngle <= maxAngle && // 2pi turn (drone usually does more, but that's ok I guess)
                     SLAM->GetTrackingState() == ORB_SLAM2::Tracking::OK &&
                     !done &&
                     (lostIdx)%command_list_size == 0){
                if (cw)
                    command = cw_commands[index%command_list_size];
                else command = ccw_commands[index%command_list_size];
                if (currentAngle >= previousAngle){
                    sumAngle += currentAngle-previousAngle;
                }
                else{
                    sumAngle += previousAngle - currentAngle;
                }
                if (verboseDrone)
                    std::cout << "current angle: " << sumAngle << std::endl;
            }
            else if ((SLAM->GetTrackingState() == ORB_SLAM2::Tracking::LOST || (lostIdx)%command_list_size != 0) && !done){
                if (cw)
                    command = ccw_commands[(lostIdx)%command_list_size];
                else command = cw_commands[(lostIdx)%command_list_size];
                lostIdx++;
            }
            else{
                done = true;
            }
            ++index;
            previousAngle = currentAngle;
            /*Drone->SendCommand(command);
            if(verboseDrone){
                std::cout << "Command: " << command << std::endl;
            }*/
            SendCommand(command);
            busy = true;
        }

        if (done){
            std::cout << "Done!" << std::endl;
            //killFrameUpdate = true; //Shut down the frame update thread main loop
            return ORBDrone::DroneState::OK;
        }
    }
}

void ORBDrone::SendCommand(std::string command){
    actionSent = std::chrono::system_clock::now();
    Drone->SendCommand(command);
    if(verboseDrone){
        std::cout << "Command: " << command << std::endl;
    }
}

std::optional<string> ORBDrone::GotDroneResponse(){

    std::optional<string> response;
    std::chrono::duration<double> diff = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - actionSent);

    if (diff.count() > 0.5){
        response = Drone->ReceiveResponse();
    }
    else{
        return response;
    }
    if (diff.count() > 10){
        std::cout << "Didn't get response in a long while (diff =" << diff.count() << ")" << std::endl;
        Drone->SendCommand("battery?");
        actionSent = std::chrono::system_clock::now();
    }
    if (response)
    {
        std::string respStr = response.value();
        if (respStr.find("OK") != std::string::npos ||
                respStr.find("ok") != std::string::npos)
        {
            if (verboseDrone)
                std::cout << "Tello: " << respStr << std::endl;
            return response;
        }else if (respStr.find("ERROR") != std::string::npos ||
                     respStr.find("error") != std::string::npos)
        {
            Drone->SendCommand("land");
            std::cerr << "Landing because of error: " << response.value() << std::endl;
            abort();
        }

        else
        {
           if (verboseDrone)
               std::cout << "Tello: " << respStr << std::endl;
            return response;
        }

    }
    return response;
}

bool ORBDrone::SetScale(){
    bool debug = true;
    scale = 0;
    std::array<std::string, 2> lost_commands{"up 25", "down 25"};
    std::array<std::string, 2> scale_commands{"up 70", "down 70"};


    unsigned index{0}, endIdx{0}, lostIdx{0}, scaleCounter{0};
    const unsigned repetitions{3};
    float previousHeightSLAM{0}, currentHeightSLAM{0}, previousHeightDrone{0}, currentHeightDrone{0};//, initialHeightDrone{0}, initialHeightSLAM{0};
    //float scaleFromInit{0}, scale{0};
    bool busy{false},gotNewFrame{false},done{false};
    cv::Mat currentFrame;
    cv::Mat Tcw;

    // scaling loop
    while (!done)
    {

        //if (GotDroneResponse()){
        busy = !GotDroneResponse().has_value();
        //}
        // See surrounding.
        gotNewFrame = updateSLAM(currentFrame,Tcw,SLAM);

        // Act
        if (!busy && !done)// && (index < total_commands))
        {
            //Get the position of the current frame (we could use quaternions, but I'd rather use the rotation mat. I f* hate quaternions dude

            if (gotNewFrame && SLAM->GetTrackingState() == ORB_SLAM2::Tracking::OK) {

                Drone->SendCommand("height?");
                if (debug)
                    std::cout << "Sent height quary" << std::endl;

                std::optional<string> response = GotDroneResponse();

                while (!done)
                {
                    gotNewFrame = updateSLAM(currentFrame,Tcw,SLAM);

                    if (response){
                        std::string respStr = response.value();

                        if (respStr.find("ERROR") != std::string::npos ||
                            respStr.find("error") != std::string::npos){
                            Drone->SendCommand("land");
                            std::cerr << "got an error from drone" << std::endl;
                            abort();
                        }

                        // the 10 is because Tello measures in dm. The minus because we're working with ORB_SLAM coordinates
                        currentHeightDrone = -std::stof(respStr) * 10;

                        break;
                    }
                    else{
                        response = GotDroneResponse();
                    }

                }
                //cv::Mat Tcw = currentKeyFrame->GetPose(); //Position homogenious mat.
                cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); //Rotation in world coordinates
                cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation in world coordinates

                currentHeightSLAM = twc.at<float>(1); // see? simple!
                if (verboseDrone){
                    std::cout << "Current height measured by drone: " << currentHeightDrone << "cm" << std::endl;
                    std::cout << "Current height estimated by SLAM: " << currentHeightSLAM << std::endl;
                    std::cout << "Previous height measured by drone: " << previousHeightDrone << "cm" << std::endl;
                    std::cout << "Previous height estimated by SLAM: " << previousHeightSLAM << std::endl;
                }
            }
            std::string command;
            if (index < scale_commands.size()*repetitions &&
                     SLAM->GetTrackingState() == ORB_SLAM2::Tracking::OK &&
                     lostIdx%lost_commands.size() == 0 && endIdx == 0 && !done){

                command = scale_commands[index%scale_commands.size()];

                if (index > 1){ //The initial number 0 is meaningless
                    scale += std::abs(currentHeightSLAM - previousHeightSLAM)/
                        std::abs(currentHeightDrone - previousHeightDrone);
                    scaleCounter++;
                }
                index++;
                if (verboseDrone){
                    std::cout << "scale sum: " << scale << std::endl;
                    std::cout << "scale: " << scale/(scaleCounter) << std::endl;
                }

            }
            else if ((SLAM->GetTrackingState() == ORB_SLAM2::Tracking::LOST || (lostIdx)%lost_commands.size() != 0) &&
                     endIdx == 0 && !done){
                command = lost_commands[(lostIdx)%lost_commands.size()];
                lostIdx++;
            }
            else{
                    scale = scaleCounter/scale;
                    done=true;
                    return true;
            }

            //if (std::abs(currentHeightSLAM - previousHeightSLAM) < 1e-5f)
            previousHeightSLAM = currentHeightSLAM;
            previousHeightDrone = currentHeightDrone;
            if (!done){
//                Drone->SendCommand(command);
//                if(verboseDrone){
//                    std::cout << "Command: " << command << std::endl;
//                }
                SendCommand(command);
                busy = true;

            }
        }
    }//End of scaling loop
    return false;
}

ORBDrone::DroneState ORBDrone::AdvanceForward(float distFromWall){
    float wallDist{1e10};
    int index = 0;
    if (verboseDrone)
        std::cout << "starting forward loop..." << std::endl;
    //std::array<std::string, 3> commands{"left 25", "right 25","forward 25"};
    //int idx = 0;
//    Drone->SendCommand("forward 25");
//    if(verboseDrone){
//        std::cout << "Command: " << "forward 25" << std::endl;
//    }
    SendCommand("forward 25");

    cv::Mat currentFrame, Tcw;

    float averageDist_{wallDist};
    bool allOK{true},gotNewFrame{false};
    while (allOK){

        if (SLAM->GetTrackingState()==ORB_SLAM2::Tracking::LOST){
            std::cout<< "Lost ORB SLAM" << std::endl;
            return DroneState::Lost;
        }
        // The 1e-5 is to prevent from a case where no frame is found and
        // you get wall distance 0.0 which makes no sense.
        if (wallDist <= distFromWall && wallDist > 1e-5f){
            std::cout<< "Too close to wall" << std::endl;
            std::cout << "WallDist: " << wallDist << std::endl;
            return DroneState::TooCloseToWall;
        }

        gotNewFrame = updateSLAM(currentFrame,Tcw,SLAM);

        if (gotNewFrame){
            AnalyzedFrame analyzedFrame(SLAM,scale);
            averageDist_ += analyzedFrame.GetMinNonFloorDist();
            index++;
        }

        if (const auto response = GotDroneResponse())
        {
            wallDist = averageDist_/index;
            averageDist_ = {0.0};
            index = 0;
            if (verboseDrone){
                std::cout << "WallDist: " << wallDist << std::endl;
            }

            SendCommand("forward 25");

        }
    }
    return DroneState::OK;
}

ORBDrone::~ORBDrone(){
        killFrameUpdate = true;
        pthread_join(UpdThread,nullptr);
        globalCapture.release();
        Drone->SendCommand("land");
        std::cout << "tello bat: " << Drone->GetBatteryStatus() << std::endl;

        SLAM->Shutdown();
}
