// by Jianhua Li 
// email:jianhuali@csail.mit.edu
// this function is used for gelsight calibration

//standard includes
#include <iostream>
#include <string>

#include "CGelSightGrabImg.h"
#include "CGelSightHeightMapCalibration.h"

using namespace std;

// the camera ID
const int CameraID = 1;
const std::string SaveDataFolderName = "..//CameraData"; 

int main(int argc, char** argv)
{
    CGelSightGrabImg m_GelSightGrabImg;
    
    bool bSaveData;
    cv::Mat localInitBluredImg, localCurrColorImg;
    // the folder to save data
    std::string m_SaveImgPreName;
    //===================================================================================
    m_GelSightGrabImg.SetSaveDataFolderName(SaveDataFolderName);
    m_GelSightGrabImg.GetSaveDataPreFileName(m_SaveImgPreName);
    if(m_GelSightGrabImg.ConnectCamera(CameraID) < 0){
        return -1;
    }
    m_GelSightGrabImg.SetCameraInitFrame(localInitBluredImg);

    //==================================================================================
    CGelSightHeightMapCalibration m_GelSightHeightMapCalibration;
    m_GelSightHeightMapCalibration.SetFileNameAndInitBluredImg(m_SaveImgPreName, localInitBluredImg);

    // image threshold
    cv::namedWindow("BallPositionInCurrImg", 1);

    int iImgCircleCenterXAddValue = 100;
    cv::createTrackbar("CircleX", "BallPositionInCurrImg", &iImgCircleCenterXAddValue, 200);
    int iImgCircleCenterYAddValue = 100;
    cv::createTrackbar("CircleY", "BallPositionInCurrImg", &iImgCircleCenterYAddValue, 200);

    int iImgCircleRadiusAddValue = 100;
    cv::createTrackbar("CirRadius", "BallPositionInCurrImg", &iImgCircleRadiusAddValue, 200);

    std::cout << "Get the calibration frames" << std::endl;

    // find circles
    int ImgCalibNum = 0;
    while (ImgCalibNum < CALIB_RAWIMGNUM){
        if(m_GelSightGrabImg.CameraCaptureCurrFrame() < 0){
            continue;
        }
        m_GelSightGrabImg.GetCameraCurrFrame(localCurrColorImg, bSaveData);

        bool local_state = m_GelSightHeightMapCalibration.FindBallPositionInCurrImgForCalibration(localCurrColorImg);
        int CurrKey = cv::waitKey(3);
        // stop capturing by pressing ESC
        if (((CurrKey == 'C') || (CurrKey == 'c')) && (local_state == true)){
            do{
                m_GelSightHeightMapCalibration.ManualSetBallPositionInCurrImgForCalibration(
                    (iImgCircleCenterXAddValue - 100), 
                    (iImgCircleCenterYAddValue - 100),
                    (iImgCircleRadiusAddValue - 100));
                CurrKey = cv::waitKey(3);
            } while ((CurrKey != 'M') && (CurrKey != 'm'));

            m_GelSightHeightMapCalibration.GetLookupTableFromBall(ImgCalibNum);

            cv::setTrackbarPos("CircleX", "BallPositionInCurrImg", 100);
            cv::setTrackbarPos("CircleY", "BallPositionInCurrImg", 100);
            cv::setTrackbarPos("CirRadius", "BallPositionInCurrImg", 100);

            ImgCalibNum++;
            std::cout << "ImgNum = " << ImgCalibNum << std::endl;
        }else if((CurrKey == 27) || (CurrKey == 'Q') || (CurrKey == 'q')){
            break;
        }
    }
    if (ImgCalibNum < CALIB_RAWIMGNUM){
        return -1;
    }
    
    m_GelSightHeightMapCalibration.SmoothLookuptable();
    m_GelSightHeightMapCalibration.SaveLookupTableDataToFile();

	return 1;
}