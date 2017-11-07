// #pragma once

// by Jianhua Li 
// email:jianhuali@csail.mit.edu

#ifndef CGELSIGHTHEIGHTMAPCALIBRATION_H_
#define CGELSIGHTHEIGHTMAPCALIBRATION_H_

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

// #define LOAD_USB_GEL

const int CALIB_LOOKUPTAB_BINNUM = 60;
const int CALIB_LOOKUPTAB_MAXBINNUM = CALIB_LOOKUPTAB_BINNUM - 1;

// Size_ (_Tp _width, _Tp _height)
const cv::Size CALIB_HEIGHTMAP_RAWIMG_SIZE = cv::Size(640, 480);

// the board size to detect the raw image
const int CALIB_HEIGHTMAP_RAWIMG_BORDER = 20;

const uchar CALIB_MARKERTHRES_LOW = (uchar)(80);
const uchar CALIB_MARKERTHRES_HIGH = (uchar)(120);

const uchar CALIB_CONTACTTHRES_HIGH = (uchar)(20);
const uchar CALIB_CONTACTTHRES_LOW = (uchar)(10);

// // normal gel
const float CALIB_Pixel2mm_ratio = (float)(0.035);
// ball radius in mm
// small ball
const float CALIB_BALL_ACTUALRADIUS_mm = (float)(3.97/2.0);

const float CALIB_BALL_ACTUALRADIUS_pixel = CALIB_BALL_ACTUALRADIUS_mm/CALIB_Pixel2mm_ratio;

const int CALIB_UNIFORMIMG_RATIO = (int)(200);
// the min number of calibration images
const int CALIB_RAWIMGNUM = 15;

class CGelSightHeightMapCalibration
{
public:
	CGelSightHeightMapCalibration();
	~CGelSightHeightMapCalibration();

	// set parameters for 3d reconstructon
	void SetFileNameAndInitBluredImg(const std::string inSaveDataPreFileName, const cv::Mat inInitBluredColorImg);
	
	void SaveReconstructionDataToFile(const int FrameIndex, const float Pixel2mmRatio);

	int SaveLookupTableDataToFile();
	
	void FindBallPositionInCurrImgForCalibration(const cv::Mat inCurrColorImg);

	void ManualSetBallPositionInCurrImgForCalibration(const int inCircleAddX, const int inCircleAddY, 
		const int inCircleAddRadius);

	void GetLookupTableFromBall(const int FrameIndex);

	void SmoothLookuptable();

private:

	// get the weight of initial image
	void GetInitFrameWeightRatio(const cv::Mat inRawImg, cv::Mat &outImgWeight);

	void DetectBlackMarkerMask(const cv::Mat inCurrColorImg, const int border_size, cv::Mat &outMarkerMaskImg);

	void DetectContactMapMask(const cv::Mat inCurrColorImg, const cv::Mat inInitBluredColorImg, 
		const int border_size, cv::Mat &outContactMapMaskImg);

	void DetectContactMapMaskWithoutBlackMarker(const cv::Mat inCurrColorImg, const cv::Mat inInitBluredColorImg, 
		const int border_size, cv::Mat &outMaskImg);

	void UpdateLookuptableFromBall(const cv::Mat inCurrColorImg, const cv::Mat inInitWeightFloat,
		const cv::Point2f inCircleCenterPixel, const float inCircleRadiusPixel, const float inBallActualRadiusPixel, 
		const int border_size);

	void GetCurrUniformImg(const cv::Mat RawInCurrColorImgFloat, const int ImgUniformScale, cv::Mat &OutUniformColorImg);

	void DetectCircleInColorImg(const cv::Mat inCurrColorImg, const cv::Mat inInitColorImg, 
		cv::Point2f &outPredCircleCenter, float &outPredCircleRadius);

private:
	cv::Mat m_LookupTabCountMap;
	cv::Mat m_LookupTabGradMapX;
	cv::Mat m_LookupTabGradMapY;

	std::string m_SaveDataPreFileName;

	cv::Mat m_InitBluredColorImg;
	cv::Mat m_InitBluredColorImgWeightFloat;

	cv::Mat m_CurrColorImg;

	cv::Point2f m_PredCircleCenter;
	float m_PredCircleRadius;

	cv::Point2f m_ActualCircleCenter;
	float m_ActualCircleRadius;

#ifdef HEIGHTMAP_CALIB_DEBUG_MODE
	cv::Mat m_UniformCurrColorImg;
	cv::Mat m_ShowCurrColorDiffImg;
#endif
};

#endif // CGELSIGHTHEIGHTMAPCALIBRATION_H_