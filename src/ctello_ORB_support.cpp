#include "include/ctello_ORB_support.h"

void *UpdateFrame(void *arg)
{
    for(int i=0;;++i)
    {
        Mat tempFrame;
        try {
            globalCapture >> tempFrame;
            cv::resize(tempFrame, tempFrame, cv::Size(960, 720));
            if (writeImages && !tempFrame.empty()){
                cv::imwrite(basefilename + std::to_string(i) + ".jpg", tempFrame);
            }
        } catch (const std::exception& e) {
            std::cout << "Capture error: " << e.what();
        }

        pthread_mutex_lock(&frameLocker);
        globalFrame = tempFrame;
        pthread_mutex_unlock(&frameLocker);

        if (killFrameUpdate){
            break;
        }
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

void saveMap( int fileNumber) {

    std::string dirname = basefilename + "Dir";
    std::string mkdircommand = "mkdir " + dirname;
    system(mkdircommand.c_str());
    mkdircommand = "mkdir " + dirname + "/Frames";
    system(mkdircommand.c_str());
    std::ofstream pointData;
    std::vector<int> savedFrames;
    pointData.open(dirname + "//" + basefilename + std::to_string(fileNumber) + ".csv");
    for(auto p : allMapPoints) {
        if (p != nullptr)
        {
            auto frame = p->GetReferenceKeyFrame();
            int frameId = static_cast<int>(frame->mnFrameId);
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

void saveCurrentMap(){
    std::ofstream currentPointData;
    currentPointData.open(basefilename + ".csv");
    for(auto p : currMapPoints) {
        Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
        currentPointData << v.x() << "," << v.y() << "," << v.z() << std::endl;
    }

    currentPointData.close();
}

cv::Mat AnalyzedFrame::ComputePoseVec(){

    cv::Mat Tcw = currentKeyFrame->GetPose(); //Position homogenious mat.
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); //Rotation in world coordinates
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation in world coordinates

    return twc;
}

cv::Mat AnalyzedFrame::XZYtoXYZ(cv::Mat vector){

    cv::Mat toReturn = vector;
    toReturn.at<float>(1) += toReturn.at<float>(2); //Zpos = Z +Y
    toReturn.at<float>(2)  = toReturn.at<float>(1) - vector.at<float>(2); //Ypos = Zpos - Y = Z + Y - Y = Z   So now we have Z in Ypos and Z+Y in Zpos
    toReturn.at<float>(1) -= toReturn.at<float>(2); //Zpos = Zpos - Ypos = Z + Y - Z = Y   So they are now swapped

    return toReturn;
}

AnalyzedFrame::AnalyzedFrame(ORB_SLAM2::System *SLAM,float scale_){

    scale = scale_;
    std::vector<cv::Mat> pointsOnFloor;
    std::vector<cv::Mat> pointsAboveFloor;
    cv::Mat minPoint, maxPoint;

//    frameID = (SLAM->GetMap()->GetMaxKFid());

//    vector<ORB_SLAM2::KeyFrame*>::iterator findFrame = std::find_if(SLAM->GetMap()->GetAllKeyFrames().begin(), SLAM->GetMap()->GetAllKeyFrames().end(),
//                                     [this](ORB_SLAM2::KeyFrame* pKF) {return pKF->mnId == frameID;});

//    unsigned long int frameIDtest = currentKeyFrame->mnFrameId;
//    if (frameID != frameIDtest) //really really should not happen now
//    {
//        cerr << "We have a problem getting the correct info. The max frameID is " << frameIDtest << " and the current frameID is " << frameID;
//        abort();
//    }

    currentKeyFrame = SLAM->GetMap()->GetLastKeyFrame(); // not using maxId because frames can be deleted. This is the last added frame.

    set<ORB_SLAM2::MapPoint*> points = currentKeyFrame->GetMapPoints();

    numOfPoints = static_cast<int>(points.size());

    cv::Mat selfPose = ComputePoseVec();

    for (set<ORB_SLAM2::MapPoint*>::iterator point_itr = points.begin(); point_itr != points.end();  ++point_itr){
        cv::Mat point =  scale * XZYtoXYZ((*(point_itr))->GetWorldPos());
        averageXYZ += point;
        float floordist = FloorDist(point,selfPose);

        if (point.at<float>(2) < selfPose.at<float>(2)){
            numOfPointsLowerThanDrone++;
            pointsOnFloor.push_back(point);
        }else{
            pointsAboveFloor.push_back(point);
            if (floordist < minNonFloorDist){
                minNonFloorDist = floordist;
            }
        }

        if (floordist < minFloorDist){
            minPoint = point;
            minFloorDist = floordist;
        }
        if (floordist > maxFloorDist){
            maxPoint = point;
            maxFloorDist = floordist;
        }
    }//end of first run over points

    averageXYZ = averageXYZ/numOfPoints;

    if (numOfPoints > 1){
        // Create unit vectors of the new
        cv::Mat newCoord = cv::Mat_<float>(3,3);
        newCoord.row(0) = (selfPose - maxPoint)/maxFloorDist;
        newCoord.row(0).col(2) = 0.0;
        newCoord.row(1).col(0) = - newCoord.row(0).col(1);
        newCoord.row(1).col(1) = - newCoord.row(0).col(0);
        newCoord.row(1).col(2) = 0.0;
        newCoord.row(2).col(0) = 0.0;
        newCoord.row(2).col(1) = 0.0;
        newCoord.row(2).col(2) = 1.0;

        //now we have unit vectors: x pointing from self position to max point, z up and y right hand
        //we rotate all the points to this coordinate system:
        rotatedAveragePoint = newCoord * averageXYZ;

        for (set<ORB_SLAM2::MapPoint*>::iterator point_itr = points.begin(); point_itr != points.end();  ++point_itr){
            cv::Mat rotatedPoint =  newCoord * XZYtoXYZ((*(point_itr))->GetWorldPos())*scale;


            if (rotatedPoint.at<float>(0) > maxRotated.at<float>(0))
                maxRotated.at<float>(0) = rotatedPoint.at<float>(0);
            if (rotatedPoint.at<float>(1) > maxRotated.at<float>(1))
                maxRotated.at<float>(1) = rotatedPoint.at<float>(1);
            if (rotatedPoint.at<float>(2) > maxRotated.at<float>(2))
                maxRotated.at<float>(2) = rotatedPoint.at<float>(2);

            if (rotatedPoint.at<float>(0) < minRotated.at<float>(0))
                minRotated.at<float>(0) = rotatedPoint.at<float>(0);
            if (rotatedPoint.at<float>(1) < minRotated.at<float>(1))
                minRotated.at<float>(1) = rotatedPoint.at<float>(1);
            if (rotatedPoint.at<float>(2) < minRotated.at<float>(2))
                minRotated.at<float>(2) = rotatedPoint.at<float>(2);
            float xdiff = (rotatedPoint.at<float>(0) - rotatedAveragePoint.at<float>(0)),
                  ydiff = (rotatedPoint.at<float>(1) - rotatedAveragePoint.at<float>(1)),
                  zdiff = (rotatedPoint.at<float>(2) - rotatedAveragePoint.at<float>(2));


            try {
                rotatedCovariance.row(0) += xdiff * (rotatedPoint - rotatedAveragePoint).t();
                rotatedCovariance.row(1) += ydiff * (rotatedPoint - rotatedAveragePoint).t();
                rotatedCovariance.row(2) += zdiff * (rotatedPoint - rotatedAveragePoint).t();
            }
            catch (const std::exception& err) {
                std::cout << "We got a problem here: " << err.what() << std::endl;
                std::cout << "rotatedAveragePoint = " << std::endl << " "  << rotatedAveragePoint << std::endl << std::endl;
                std::cout << "rotatedPoint = " << std::endl << " "  << rotatedPoint << std::endl << std::endl;
                std::cout << "rotatedCovariance.row(0) = " << std::endl << " "  << rotatedCovariance.row(0) << std::endl << std::endl;
                std::cout << "rotatedCovariance.row(1) = " << std::endl << " "  << rotatedCovariance.row(1) << std::endl << std::endl;
                std::cout << "rotatedCovariance.row(2) = " << std::endl << " "  << rotatedCovariance.row(2) << std::endl << std::endl;
                std::cout << "xdiff = "  << xdiff << std::endl;
                std::cout << "ydiff = "  << ydiff << std::endl;
                std::cout << "zdiff = "  << zdiff << std::endl;

                abort();
            }

        }//end for floor points

        gapRotated = maxRotated - minRotated;
        rotatedCovariance = rotatedCovariance/(numOfPoints -1);
    }//end If have any points on floor
    else isGoodFrame = false;

}
