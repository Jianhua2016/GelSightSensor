// #pragma once

#ifndef CGELSIGHTMARKERMOTION_H_
#define CGELSIGHTMARKERMOTION_H_

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

using namespace std;
using namespace cv;

// in the debug mode
// #define MARKERMOTION_DEBUG_MODE

const float MARKERMOTION_IMGSCALE = (float)(0.5);

const cv::Size MARKERMOTION_RAWIMG_SIZE = cv::Size(640, 480);
const int MARKERMOTION_BORDER_SIZE = 20;

const cv::Size MARKERMOTION_RESIZEDIMG_SIZE = cv::Size((int)(MARKERMOTION_IMGSCALE * MARKERMOTION_RAWIMG_SIZE.width), 
	(int)(MARKERMOTION_IMGSCALE * MARKERMOTION_RAWIMG_SIZE.height));
const int MARKERMOTION_RESIZEDBORDER_SIZE = (int)(MARKERMOTION_BORDER_SIZE * MARKERMOTION_IMGSCALE);

const uchar MARKERMOTION_MARKERTHRES_LOW = (uchar)(80);
const uchar MARKERMOTION_MARKERTHRES_HIGH = (uchar)(120);
const unsigned int MARKERMOTION_MARKERNUM_INCOL = 30;
const unsigned int MARKERMOTION_MARKERNUM = 400;
const unsigned int MARKERMOTION_MARKER_MINDIST = 6;
// the min continous black pixel in col
const unsigned int MARKERMOTION_PIXEL_MINNUM_INCOL = 3;

const uchar MARKERMOTION_CONTACTTHRES_HIGH = (uchar)(20);
const uchar MARKERMOTION_CONTACTTHRES_LOW = (uchar)(10);

const float MARKERMOTION_IMGPLOT_RATIO = (float)(5.0);
const int MARKERMOTION_IMGPLOT_GRID = 20;

const int MXA_INITIMG_NUM = 10;

const float MARKER_MINMOTIONGRAD = 1.0f;
const float MARKER_MAXMOTIONGRAD = 5.0f;
const float MARKER_INITIMG_RESET_GRAD = 1.5f;
const float MARKER_MOTIONMASK_THRES = 3.0f;
const int MARKER_INITIMG_RESET_MAXPOINTNUM = 9000;

const float MARKER_DEG2RAD = CV_PI/180.0f;

class CGelSightMarkerMotion
{
public:
	CGelSightMarkerMotion();
	~CGelSightMarkerMotion();

	void GetCurrMotionFlow(const cv::Mat inCurrColorImg);
	void SetMarkerMotionInitFrame(const cv::Mat inCurrColorImg);

	void DispColorMap();

	// set parameters for 3d reconstructon
	void SetFileNameAndInitBluredImg(const std::string InSaveDataPreFileName, const cv::Mat inInitBluredColorImg);

private:
	void DetectBlackMarkerPosition(const cv::Mat inRawColorImg, const bool bInitImg, cv::Mat &outMarkerMaskImg);

	void DetectBlackMarkerMotionWithOpticalFlow(const cv::Mat inMotionFlow, const bool bInitImg);

	void DetectBlackMarkerMotionByTracking(const bool bInitImg);

	void ShowMarkerMotion(const cv::Mat inRawColorImg, const bool bInitImg, cv::Mat &outMarkerMotionImg);

	void DetectContactMapMask(const cv::Mat inCurrColorImg, const cv::Mat inInitBluredColorImg, cv::Mat &outContactMapMaskImg);


private:
	std::string m_SaveDataPreFileName;

	cv::Mat m_InitResizedBluredColorImg;
	cv::Mat m_InitResizedGrayImg;

	cv::Mat m_CurrResizedColorImg;
	cv::Mat m_CurrResizedGrayImg;

	// the black marker initial position
	float m_InitMarkerCenter[MARKERMOTION_MARKERNUM][2];
	// the black marker connecting position
	float m_InitMarkerCenterNext[MARKERMOTION_MARKERNUM][2];
	// the black marker latest position
	float m_InitMarkerCenterLatest[MARKERMOTION_MARKERNUM][2];

	int m_InitMarkerSize[MARKERMOTION_MARKERNUM];

	float m_CurrMarkerCenter[MARKERMOTION_MARKERNUM][2];
	int m_CurrMarkerSize[MARKERMOTION_MARKERNUM];

	int m_InitMarkerNum;
	int m_CurrMarkerNum;
};

#endif // CGELSIGHTMARKERMOTION_H_