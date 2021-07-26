#include "include/ctello_ORB_support.h"

void *UpdateFrame(void *arg)
{
    for(int i=0;;++i)
    {
        Mat tempFrame;
        try {
            globalCapture >> tempFrame;
            cv::resize(tempFrame, tempFrame, cv::Size(960, 720));
//            if (writeImages && !tempFrame.empty()){
//                cv::imwrite(dirname + "//"  + basefilename + std::to_string(i) + ".jpg", tempFrame);
//            }
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

    return nullptr;
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
    std::string mkdircommand = "mkdir " + dirname;
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
                const string command = "mv " + dirname + "//" + basefilename + std::to_string(frameId) + ".jpg ./" + dirname + "/Frames";
                system(command.c_str());
            }

        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;
    const string command = "rm ./" + dirname + "//" + "*.jpg";
    system(command.c_str());
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

void AnalyzedFrame::saveFramePoints(std::string filename){


    std::ofstream currentPointData;
    currentPointData.open(filename + ".csv");
    for(auto p : allPoints) {
        auto frame = p->GetReferenceKeyFrame();
        int frameId = static_cast<int>(frame->mnFrameId);
        cv::Mat Tcw = frame->GetPose();
        auto point = p->GetWorldPos();
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
        auto q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
        currentPointData << v.x() << "," << v.y() << "," << v.z()<< "," << q[0]
        << "," << q[1]<< "," << q[2]<< "," << q[3] << "," << frameId <<
        ","<< twc.at<float>(0)  << "," << twc.at<float>(1)  << "," << twc.at<float>(2) << std::endl;
    }

    currentPointData.close();
}

cv::Mat AnalyzedFrame::ComputePoseVec(){

    cv::Mat Tcw = currentKeyFrame->GetPose(); //Position homogenious mat.
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); //Rotation in world coordinates
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation in world coordinates

    //twc = XZYtoXYZ(twc);

    return twc;
}


cv::Mat AnalyzedFrame::FloatMatScalarMult(cv::Mat Matrix, float scalar){
    if (Matrix.empty())
        return Matrix;
    cv::Mat tempMat;
    Matrix.convertTo(tempMat,CV_64F); //convert to double
    tempMat = tempMat * static_cast<double>(scalar);
    tempMat.convertTo(tempMat,CV_32F);

    return tempMat;
}

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
            unsigned long frameid = SLAM_->GetTracker()->mCurrentFrame.mnId;
            if (writeImages){
                cv::imwrite(dirname + "//"  + basefilename + std::to_string(frameid) + ".jpg", currentFrame_);
            }
        } catch (const std::exception& e) {
            cout << "ORBSLAM error: " << e.what();
        }
    }
    return gotNewFrame;
}
//we don't need this we'll work with the upside down RHS coordinates ORB_SLAM is using.
/*cv::Mat AnalyzedFrame::XZYtoXYZ(cv::Mat vector){

    cv::Mat toReturn=vector;

    toReturn.at<float>(1) += toReturn.at<float>(2); //Zpos = Z +Y
    toReturn.at<float>(2)  = toReturn.at<float>(1) - vector.at<float>(2); //Ypos = Zpos - Y = Z + Y - Y = Z   So now we have Z in Ypos and Z+Y in Zpos
    toReturn.at<float>(1) -= toReturn.at<float>(2); //Zpos = Zpos - Ypos = Z + Y - Z = Y   So they are now swapped

    //toReturn = -toReturn;

    return toReturn;
}*/

float AnalyzedFrame::K_function(std::vector<cv::Mat> points, float area, float search_radius){
    unsigned long numOfPoints = points.size();
    int pointsInRadius{0};
    float dij;

    if (numOfPoints == 0 || area < 1e-5f)
        return std::numeric_limits<double>::infinity();
    float Lamb = numOfPoints/area;

    for(std::vector<cv::Mat>::iterator points_itr1 = points.begin(); points_itr1 != points.end()-1; ++points_itr1){
        for (std::vector<cv::Mat>::iterator points_itr2 = points_itr1 +1; points_itr2 != points.end(); ++points_itr2){
            dij = FloorDist(*points_itr1,*points_itr2);
            if (dij < search_radius)
                ++pointsInRadius;
        }
    }

    return pointsInRadius/(Lamb * numOfPoints);//This is the K value
}

bool AnalyzedFrame::ArePointsScattered(std::vector<cv::Mat> points, float area, float search_radius, float epsilon){
    float K = K_function(points,area,search_radius);
    if (K < (PI * std::pow(search_radius ,2.0f)*(1.0f+epsilon)) ){
        return true;
    }
    else {
        return false;
    }
}

AnalyzedFrame::AnalyzedFrame(ORB_SLAM2::System *SLAM,float scale_, float selfPoseYSign):
    isWall(false), isGoodFrame(false), initialized(false), closedLoop(false),
                           maxFloorDist(0.0), minFloorDist(1e10), minNonFloorDist(1e10), kValueOnFloor(0.0), avgDist(0.0), scale(1.0),
                            numOfPoints(0), numOfPointsLowerThanDrone(0),numOfPointsHeigherThanDrone(0), frameID(0), currentKeyFrame(nullptr)
{
    rotatedAveragePoint = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    averageXYZ = (Mat_<float>(3,1) << 0, 0, 0);
    gapRotated = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    minRotated = (Mat_<float>(3,1) << 1e10, 1e10, 1e10);
    maxRotated = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    selfPose   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    minWallPoint   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    minFloorPoint   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    maxFloorPoint   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    maxFloorPointRotated   = (Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    rotatedCovariance = (Mat_<float>(3,3)<< 0.0,0.0,0.0,  0.0,0.0,0.0, 0.0,0.0,0.0);
    float area,search_radius,wallMin1{1e10},wallMin2{1e10},wallMin3{1e10};
    scale = scale_;
    std::vector<cv::Mat> pointsOnFloor;
    std::vector<cv::Mat> pointsAboveFloor;
    //cv::Mat minPoint, maxPoint;

    currentKeyFrame = SLAM->GetMap()->GetLastKeyFrame(); // not using maxId because frames can be deleted. This is the last added frame.

    set<ORB_SLAM2::MapPoint*> points = currentKeyFrame->GetMapPoints();
    allPoints = points;

    numOfPoints = static_cast<int>(points.size());

    cv::Mat selfPoseUnscaled = ComputePoseVec();
    selfPose = FloatMatScalarMult(selfPoseUnscaled,scale);


    for (set<ORB_SLAM2::MapPoint*>::iterator point_itr = points.begin(); point_itr != points.end();  ++point_itr){
        cv::Mat point,unscaledPoint =  (*(point_itr))->GetWorldPos();
        if ((*(point_itr))->mnCorrectedByKF !=0 || (*(point_itr))->mnLoopPointForKF != 0 || (*(point_itr))->mnCorrectedReference !=0){
            closedLoop = true;
        }
        point = FloatMatScalarMult(unscaledPoint,scale);
        averageXYZ += point;
        float floordist = FloorDist(point,selfPose);
//        float selfPoseYSign{1};
//        if (selfPose.at<float>(1) < 0.0f)
//            selfPoseYSign = -1;

        //Y is pointing to the floor so if greater than Ydrone you are lower than drone.
        if (point.at<float>(1)*selfPoseYSign < selfPose.at<float>(1)*selfPoseYSign - 5.0f){
            numOfPointsLowerThanDrone++;
            pointsOnFloor.push_back(point);

            if (floordist < minFloorDist){
                minFloorPoint = point;
                minFloorDist = floordist;
            }
            if (floordist > maxFloorDist){
                maxFloorPoint = point;
                maxFloorDist = floordist;
            }
        }else{
            pointsAboveFloor.push_back(point);
            numOfPointsHeigherThanDrone++;
            if (point.at<float>(1)*selfPoseYSign > selfPose.at<float>(1)*selfPoseYSign && //- 5.0f &&
                    point.at<float>(1)*selfPoseYSign < selfPose.at<float>(1)*selfPoseYSign + 25.0f){
                bool updatedMin{false};
                if (floordist < wallMin1){
                    wallMin1 = floordist;
                    minWallPoint = point;
                    updatedMin = true;
                }
                else if (floordist < wallMin2){
                    wallMin2 = floordist;
                    updatedMin = true;
                }
                else if (floordist < wallMin3){
                    wallMin3 = floordist;
                    updatedMin = true;
                }
                if (updatedMin){
                    minNonFloorDist = (wallMin1 + wallMin2 + wallMin3)/3;
                }

            }
        }


    }//end of first run over points

    averageXYZ = averageXYZ/numOfPoints;
    avgDist = FloorDist(averageXYZ,selfPose);

    if (numOfPoints > 1){
        // Create unit vectors of the new
        cv::Mat newCoord = cv::Mat_<float>(3,3);
        newCoord.row(0) = FloatMatScalarMult(maxFloorPoint - selfPose,1/maxFloorDist);
        newCoord.row(0).col(1) = 0.0;

        newCoord.row(2).col(0) = - newCoord.row(0).col(2);
        newCoord.row(2).col(2) = - newCoord.row(0).col(0);
        newCoord.row(2).col(1) = 0.0;

        newCoord.row(1).col(0) = 0.0;
        newCoord.row(1).col(1) = 1.0;
        newCoord.row(1).col(2) = 0.0;

        //now we have unit vectors: x pointing from self position to max point, z up and y right hand
        //we rotate all the points to this coordinate system:
        rotatedAveragePoint = newCoord * (averageXYZ - selfPose);
        maxFloorPointRotated = newCoord * (maxFloorPoint - selfPose);
        minFloorPointRotated = newCoord * (minFloorPoint - selfPose);

        for (vector<cv::Mat>::iterator point_itr = pointsOnFloor.begin(); point_itr != pointsOnFloor.end();  ++point_itr){
            cv::Mat rotatedPoint =  FloatMatScalarMult(newCoord * (*point_itr - selfPose),scale);

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
                rotatedCovariance.row(0) += FloatMatScalarMult((rotatedPoint - rotatedAveragePoint).t(),xdiff);
                rotatedCovariance.row(1) += FloatMatScalarMult((rotatedPoint - rotatedAveragePoint).t(),ydiff);
                rotatedCovariance.row(2) += FloatMatScalarMult((rotatedPoint - rotatedAveragePoint).t(),zdiff);
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

        //TODO compute K value

        gapRotated = maxRotated - minRotated;
        rotatedCovariance = rotatedCovariance/(numOfPoints -1);

        area = PI * (gapRotated.at<float>(0)/2) * (gapRotated.at<float>(2)/2);
        search_radius = ((std::sqrt(rotatedCovariance.at<float>(0,0)) + std::sqrt(rotatedCovariance.at<float>(2,2)))/2.0f) * 0.1f;

        kValueOnFloor = K_function(pointsOnFloor,area,search_radius);


    }//end If have any points on floor
    else isGoodFrame = false;

}
