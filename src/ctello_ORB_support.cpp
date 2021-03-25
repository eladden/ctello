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
        if (p != NULL)
        {
            auto frame = p->GetReferenceKeyFrame();
            int frameId = frame->mnFrameId;
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
