#include "include/ctello_control.h"

ORBDrone::~ORBDrone(){
        killFrameUpdate = true;
        pthread_join(UpdThread,nullptr);
        globalCapture.release();
        Drone->SendCommand("land");
        std::cout << "tello bat: " << Drone->GetBatteryStatus() << std::endl;

        SLAM->Shutdown();
}

ORBDrone::ORBDrone(char *vocPath, char *settingsPath){
    scale = 0.0;
    currentWallDist = 1e10f;
    bool debug = false;
    int buffer = 0;
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

    std::array<std::string, 5> initialize_commands{"left 25", "right 50", "left 25", "up 25", "down 25"};

    unsigned index{0};
    int const iterations{5};
    bool busy{false},done{false};
    cv::Mat currentFrame,Tcw;
    std::string command = "takeoff";
    updateSLAM(currentFrame,Tcw,SLAM);
    SendCommand(command);
     while(!GotDroneResponse().has_value())
         updateSLAM(currentFrame,Tcw,SLAM);

    while(!done){

        //See
        if (debug)
            std::cout << "seeing..";
        updateSLAM(currentFrame,Tcw,SLAM);
        buffer++;
        if (SLAM->GetTrackingState() == ORB_SLAM2::Tracking::OK)
        {
            done = true;
            if (index%initialize_commands.size() == 4){ //if you just went up - go down...
                command = initialize_commands[index++%initialize_commands.size()];
                SendCommand(command);
                while(!GotDroneResponse().has_value())
                    updateSLAM(currentFrame,Tcw,SLAM);
            }
            initialized = true;
            break;
        }
        //Act
        if (debug)
            std::cout << "acting..";
        if ((!busy && !done && buffer > 5)){
            buffer = 0;
//            if (index%initialize_commands.size() == 0){
//                SLAM->Reset();
//            }
            if (index < initialize_commands.size()*iterations){
                command = initialize_commands[index++%initialize_commands.size()];
                if (debug)
                    std::cout << "set command to.." << command << ". ";
                busy = true;
            }
            else{
                std::cerr << "Failed to initialize SLAM" << std::endl;
                Drone->SendCommand("land");
                abort();
            }

            if (!done){
                SendCommand(command);
            }
        }//end if not busy


        //Did we get a reply?
        if (debug)
            std::cout << "checking reply." << std::endl;
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
                if (index%(command_list_size) == 2){
                    command = cw_commands[index%command_list_size];
                }else
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
float ORBDrone::ComputeMSD(float number){
    number = abs(number);
    float MSD{1};

    while (std::trunc(number*MSD) < 1){
        MSD *= 10;
    }
    return 1/abs(MSD);
}

bool ORBDrone::SetScale(){
    bool debug = true;
    //bool gotDiff = false;
    scale = 0;
    std::array<std::string, 2> lost_commands{"up 25", "down 25"};
    std::array<std::string, 2> scale_commands{"up 75", "down 75"};


    unsigned index{0}, endIdx{0}, lostIdx{0}, scaleCounter{0};
    const unsigned repetitions{5};
    float previousHeightSLAM{0}, currentHeightSLAM{0}, previousHeightDrone{0}, currentHeightDrone{0};//, initialHeightDrone{0}, initialHeightSLAM{0};
    float MSD{100};
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
                        currentHeightDrone = std::stof(respStr) * 10;

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
            }//end if got frame
            std::string command;
            if (//index < scale_commands.size()*repetitions &&
                    scaleCounter < repetitions &&
                     SLAM->GetTrackingState() == ORB_SLAM2::Tracking::OK &&
                     lostIdx%lost_commands.size() == 0 && endIdx == 0 && !done){

                command = scale_commands[index++%scale_commands.size()];
                //index++;
                if (verboseDrone){
                    std::cout << "scale sum: " << scale << std::endl;
                    std::cout << "scale counter: " << scaleCounter << std::endl;
                    std::cout << "scale: " << scale/scaleCounter << std::endl;
                }

            }
            else if ((SLAM->GetTrackingState() == ORB_SLAM2::Tracking::LOST || (lostIdx)%lost_commands.size() != 0) &&
                     endIdx == 0 && !done){
                command = lost_commands[(lostIdx)%lost_commands.size()];
                lostIdx++;
            }
            else{
                    if (index%scale_commands.size() == 1) { //don't finish after up
                        std::cout << "the index is: " << index << std::endl;
                        std::cout << "the command is: <<" << scale_commands[index%scale_commands.size()] << std::endl;
                        command = scale_commands[index%scale_commands.size()];
                        SendCommand(command);
                        while(!GotDroneResponse().has_value())
                            updateSLAM(currentFrame,Tcw,SLAM);
                    }
                    scale = scale/scaleCounter;
                    done=true;
                    return true;
            }

            if (verboseDrone)
                std::cout << "The index is: " << index << std::endl;

            if (index <= 1){//First time this is running
                MSD = ComputeMSD(currentHeightSLAM);
            }
            if (verboseDrone){
                std::cout << "The most significant digit is:" << MSD << std::endl;
                std::cout << "SLAM diff:" << std::abs(currentHeightSLAM - previousHeightSLAM) << std::endl;
                std::cout << "Drone diff:" << std::abs(currentHeightDrone - previousHeightDrone) << std::endl;
            }
            if (std::abs(currentHeightSLAM - previousHeightSLAM) > MSD*1e-2f &&
                    std::abs(currentHeightDrone - previousHeightDrone) > 1e-2f){
                if (index > 1){
                    if (verboseDrone){
                        std::cout << "Updating scale value" << std::endl;
                    }
                    scale += std::abs(currentHeightDrone - previousHeightDrone)/std::abs(currentHeightSLAM - previousHeightSLAM);
                    scaleCounter++;
                }
                if (verboseDrone){
                    std::cout << "Updating previous values" << std::endl;
                }
                previousHeightSLAM = currentHeightSLAM;
                previousHeightDrone = currentHeightDrone;
            }
//            else
//                gotDiff = false;
            if (!done){
                SendCommand(command);
                busy = true;

            }
        }
    }//End of scaling loop
    return false;
}

ORBDrone::DroneState ORBDrone::AdvanceForward(float distFromWall, int step=50){
    float wallDist = currentWallDist;
    std::string stepString = to_string(step), command;
    //int index = 0;
    if (verboseDrone)
        std::cout << "starting forward loop..." << std::endl;

    SendCommand("forward " + stepString);

    cv::Mat currentFrame, Tcw;

    //float averageDist_{wallDist};
    bool allOK{true},gotNewFrame{false};
    while (allOK){

        if (SLAM->GetTrackingState()==ORB_SLAM2::Tracking::LOST){
            std::cout<< "Lost ORB SLAM" << std::endl;
            return DroneState::Lost;
        }
        // The 1e-5 is to prevent from a case where no frame is found and
        // you get wall distance 0.0 which makes no sense.
        if (wallDist <= distFromWall && wallDist > 1e-5f){
            std::cout<< "Too close to wall, ";
            std::cout << "WallDist: " << wallDist << std::endl;
            currentWallDist = wallDist;
            return DroneState::TooCloseToWall;
        }

        gotNewFrame = updateSLAM(currentFrame,Tcw,SLAM);

        if (const auto response = GotDroneResponse())
        {
            while (!gotNewFrame){
                updateSLAM(currentFrame,Tcw,SLAM);
            }
            AnalyzedFrame analyzedFrame(SLAM,scale);
            wallDist = analyzedFrame.GetMinNonFloorDist();
            currentAnalyzedFrame = analyzedFrame;
            //wallDist = averageDist_/index;
            //averageDist_ = {0.0};
            //index = 0;
            if (verboseDrone){
                std::cout << "WallDist: " << wallDist << std::endl;
                std::cout << "number of points not under drone: " << analyzedFrame.GetNumOfPoints() - analyzedFrame.GetNumOfPointsLowerThanDrone() << std::endl;
                std::cout << "min wall point: " << std::endl << analyzedFrame.GetMinWallPoint() << std::endl;
                std::cout << "current position: " << std::endl << analyzedFrame.GetSelfPose() << std::endl;
            }

            SendCommand("forward " + stepString);

        }
    }
    currentWallDist = wallDist;
    return DroneState::OK;
}

bool ORBDrone::IsAWall(AnalyzedFrame FrameInfo){
    //These values are taken from the descision tree learned from aprox 10 360 scans
    //TODO have a function that updates the numbers
    if (FrameInfo.GetRotatedCovariance().at<float>(0,0) <= 775.5f*scale*scale){
        if(verboseDrone){
            cout << "RotatedCovarience_11 = " << FrameInfo.GetRotatedCovariance().at<float>(0,0) << " <= 0.081f*scale^2 = " <<  0.081f*scale*scale << std::endl;
        }
        if (FrameInfo.GetMinFloorDist() <= 0.05f*scale){
            if(verboseDrone){
                cout << "I've determined it's NOT a wall: MinFloorDist = " << FrameInfo.GetMinFloorDist() << " <= 0.05 * scale = " << scale*0.05f << std::endl;
            }
            return false;
        }
        else{
            if(verboseDrone){
                cout << "I've determined it IS a wall: MinFloorDist = " << FrameInfo.GetMinFloorDist() << " > 0.05 * scale = " << scale*0.05f << std::endl;
            }
            return true;
        }
    }
    else{
        if(verboseDrone){
            cout << "RotatedCovarience_11 = " << FrameInfo.GetRotatedCovariance().at<float>(0,0) << " > 0.081f*scale^2 = " <<  0.081f*scale*scale << std::endl;
        }
        if (-FrameInfo.GetRotatedAveragePoint().at<float>(2) <= 0.057f*scale){
            if(verboseDrone){
                cout << "I've determined it's NOT a wall: RotatedAverage_z = " << FrameInfo.GetRotatedAveragePoint().at<float>(2) << " <= 0.057f*scale = " << 0.057f*scale << std::endl;
            }
            return false;
        }
        else{
            if(verboseDrone){
                cout << "I've determined it IS a wall: RotatedAverage_z = " << FrameInfo.GetRotatedAveragePoint().at<float>(2) << " > 0.057f*scale = " << 0.057f*scale << std::endl;
            }
            return true;
        }
    }
}

ORBDrone::DroneState ORBDrone::SeekFloor(bool cw, float maxAngle){
    maxAngle = maxAngle*PI/180.0f;
    const unsigned int command_list_size = 3;
    std::array<std::string, command_list_size> cw_commands{"cw 20", "up 25", "down 25"};
    std::array<std::string, command_list_size> ccw_commands{"ccw 20","up 25", "down 25"};

    unsigned index{0}, lostIdx{0};
    float currentAngle=0, previousAngle=0, sumAngle=0;
    bool busy{false},gotNewFrame{false},done{false};
    cv::Mat currentFrame,Tcw;
    ORB_SLAM2::KeyFrame* currentKeyFrame;
    std::string command;


    ///////////////////////////TURN THIS TO SEEKING FLOOR
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

                //Get the numbers
                AnalyzedFrame analyzedFrame(SLAM,scale);
                currentAnalyzedFrame = analyzedFrame;
                if (!IsAWall(currentAnalyzedFrame)){
                    if (verboseDrone)
                        std::cout << "Not Wall!"<< std::endl;
                    //turn to face the max point
                    float x,z,angle;
                    x = analyzedFrame.GetMaxmaxFloorPointRotated().at<float>(0);
                    if (verboseDrone)
                        cout << "x = " << x;
                    z = analyzedFrame.GetMaxmaxFloorPointRotated().at<float>(2);
                    if (verboseDrone)
                        cout << " z = " << z;
                    angle = abs(x)/abs(z);
                    if (verboseDrone)
                        cout << " angle = " << angle << std::endl;
                    if (x <= 0){
                        command = "ccw " + std::to_string(angle);
                    }
                    else{
                        command = "cw " + std::to_string(angle);
                    }
                    if(verboseDrone)
                        std::cout << "turning to face point" << std::endl;
                    SendCommand(command);
                    while (!GotDroneResponse().has_value())
                        ;
                    return ORBDrone::DroneState::OK;
                }

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
            SendCommand(command);
            busy = true;
        }

        if (done){
            std::cout << "Done!" << std::endl;
            return ORBDrone::DroneState::Seeking;
        }
    }//end main loop
    //return ORBDrone::DroneState::Seeking;
}
