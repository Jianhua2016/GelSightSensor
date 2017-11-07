// #pragma once

// by Jianhua Li 
// email:jianhuali@csail.mit.edu

#ifndef CGELSIGHTGRABIMG_H_
#define CGELSIGHTGRABIMG_H_

//standard includes
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <ctime>
#include <cmath>

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

// for creating a folder
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

using namespace std;
using namespace cv;

// the image threshold for bulring image
const float INITIMG_BLUR_THRESHOLD = 5.0;
const cv::Size INITIMG_GAUSS_BLUR_SIZE = cv::Size(21, 21);

const cv::Size RAWIMG_SET_SIZE = cv::Size(640, 480);
const int RAWIMG_SET_CHANNELS = 3;
const int INITIMG_NUM = 10;

class CGelSightGrabImg
{
public:
    CGelSightGrabImg();
    ~CGelSightGrabImg();

    // set the data file pre name
    void SetSaveDataFolderName(const std::string InSaveDataFolderName);

    //set camera exposure
    void SetCameraExposure(const int CurrCameraExposure);

    //connect camera
    int ConnectCamera(const int InCameraID);

    //get camera initial frame
    int SetCameraInitFrame(cv::Mat &outInitBluredImg);

    //get camera current frame
    int CameraCaptureCurrFrame();

    //get camera current frame
    void GetCameraCurrFrame(cv::Mat &outCurrImg, bool &bSaveData);

    void GetSaveDataPreFileName(std::string &OutSaveDataPreFileName);

private:
    void GetInitBluredImg(const cv::Mat in_rawimg_uint8, cv::Mat &out_bluredimg_float);

private:
    cv::VideoCapture m_CameraCap;

    cv::Mat m_CurrCapColorImg;
    cv::Mat m_InitColorImg;
    cv::Mat m_CurrShowImage;

    std::string m_ConstSaveDataFolderName;
    std::string m_ConstSaveDataPreFileName;

    bool m_bInitImgIsCaptured;
    bool m_bSaveImg;
    int m_CurrSaveImgCount;
    int m_CurrShowImgCount;

    int m_GrabFailTime;
};

#endif // CGELSIGHTGRABIMG_H_