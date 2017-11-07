//standard includes
#include <iostream>
#include <string>

#include "CGelSightHeightMap.h"
#include "CGelSightGrabImg.h"
#include "CGelSightMarkerMotion.h"

using namespace std;

// the camera ID
const int CameraID = 1;
const std::string SaveDataFolderName = "../CameraData"; 
const std::string GelSightCalibData_FileName = "../CameraData/2107_11_7_15_33_25_CalibLookupTab.xml"; 

int main(int argc, char** argv)
{
    // the folder to save data
    std::string m_SaveImgPreName;

    CGelSightGrabImg m_GelSightGrabImg;
    CGelSightHeightMap m_GelSightHeightMap;
    CGelSightMarkerMotion m_GelSightMarkerMotion;

    cv::Mat localInitBluredImg, localCurrColorImg;
    bool bSaveData;

    //===================================================================================
    m_GelSightGrabImg.SetSaveDataFolderName(SaveDataFolderName);
    m_GelSightGrabImg.GetSaveDataPreFileName(m_SaveImgPreName);
    m_GelSightGrabImg.ConnectCamera(CameraID);
    m_GelSightGrabImg.SetCameraInitFrame(localInitBluredImg);

    //==================================================================================
    if(m_GelSightHeightMap.LoadCalibrationData(GelSightCalibData_FileName) == false){
        return -1;
    }
    m_GelSightHeightMap.SetFileNameAndInitBluredImg(m_SaveImgPreName, localInitBluredImg);
    m_GelSightMarkerMotion.SetFileNameAndInitBluredImg(m_SaveImgPreName, localInitBluredImg);

    //==================================================================================
    // set the threshold for motion flow
    for (;;){
        if(m_GelSightGrabImg.CameraCaptureCurrFrame() < 0){
            continue;
        }
        m_GelSightGrabImg.GetCameraCurrFrame(localCurrColorImg, bSaveData);

        m_GelSightMarkerMotion.SetMarkerMotionInitFrame(localCurrColorImg);
        int CurrKey = cv::waitKey(1);
        if((CurrKey == 'F') || (CurrKey == 'f')){
            break;
        }
    }

    //==================================================================================
	for (;;){
        double TempSartTime = (double)cv::getTickCount();
        if(m_GelSightGrabImg.CameraCaptureCurrFrame() < 0){
            continue;
        }
        m_GelSightGrabImg.GetCameraCurrFrame(localCurrColorImg, bSaveData);
        
        m_GelSightHeightMap.GetCurrHeightMap(localCurrColorImg, bSaveData);
        m_GelSightMarkerMotion.GetCurrMotionFlow(localCurrColorImg);

        //The function returns the number of ticks per second. That is, the following code computes 
        // the execution time in seconds:
        double TempTimeCost = ((double)cv::getTickCount() - TempSartTime) / cv::getTickFrequency();
        TempTimeCost = TempTimeCost * 1000.0;
        std::cout << "TempTimeCost = " << TempTimeCost << " ms"<< std::endl;

        int CurrKey = cv::waitKey(1);
        if((CurrKey == 27) || (CurrKey == 'Q') || (CurrKey == 'q')){
            break;
        }
    }

	return 1;
}