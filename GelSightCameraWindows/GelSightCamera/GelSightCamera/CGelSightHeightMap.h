// #pragma once
// by Jianhua Li 
// email:jianhuali@csail.mit.edu

#ifndef CGELSIGHTHEIGHTMAP_H_
#define CGELSIGHTHEIGHTMAP_H_

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
#include "COpenCVPoissonSolver.h"

#pragma comment(lib, "opencv_world320.lib")

using namespace std;
using namespace cv;

// show grad images
// #define HEIGHTMAP_SHOWGRAD

const int LOOKUPTAB_BINNUM = 60;
const int LOOKUPTAB_MAXBINNUM = LOOKUPTAB_BINNUM - 1 ;

// the ratio to resize the raw image
const float HEIGHTMAP_RESIZERATIO = (float)(0.5);

// Size_ (_Tp _width, _Tp _height)
const cv::Size HEIGHTMAP_RAWIMG_SIZE = cv::Size(640, 480);
const cv::Size HEIGHTMAP_RESIZEDIMG_SIZE = cv::Size((int)(HEIGHTMAP_RESIZERATIO * HEIGHTMAP_RAWIMG_SIZE.width), 
	(int)(HEIGHTMAP_RESIZERATIO * HEIGHTMAP_RAWIMG_SIZE.height));
const int HEIGHTMAP_RAWIMG_BORDER_SIZE = 16;

const int HEIGHTMAP_GRADIMG_BORDER_SIZE = (int)(HEIGHTMAP_RESIZERATIO * HEIGHTMAP_RAWIMG_BORDER_SIZE);
const int HEIGHTMAP_GRADIMG_HEIGHT = HEIGHTMAP_RESIZEDIMG_SIZE.height - 2 * HEIGHTMAP_GRADIMG_BORDER_SIZE;
const int HEIGHTMAP_GRADIMG_WIDTH = HEIGHTMAP_RESIZEDIMG_SIZE.width - 2 * HEIGHTMAP_GRADIMG_BORDER_SIZE;
const cv::Size HEIGHTMAP_GRADIMG_SIZE = cv::Size(HEIGHTMAP_GRADIMG_WIDTH, HEIGHTMAP_GRADIMG_HEIGHT);
const int HEIGHTMAP_POINTNUM = HEIGHTMAP_GRADIMG_HEIGHT * HEIGHTMAP_GRADIMG_WIDTH;
// Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
const cv::Rect HEIGHTMAP_GRADIMG_RECT = cv::Rect(HEIGHTMAP_GRADIMG_BORDER_SIZE, HEIGHTMAP_GRADIMG_BORDER_SIZE,
	HEIGHTMAP_GRADIMG_WIDTH, HEIGHTMAP_GRADIMG_HEIGHT);

const int HEIGHTMAP_GRADIMG_CENTER_POSX = HEIGHTMAP_GRADIMG_WIDTH/2;
const int HEIGHTMAP_GRADIMG_CENTER_POSY = HEIGHTMAP_GRADIMG_HEIGHT/2;

const uchar HEIGHTMAP_MARKERTHRES_LOW = (uchar)(80);
const uchar HEIGHTMAP_MARKERTHRES_HIGH = (uchar)(120);

const uchar HEIGHTMAP_CONTACTTHRES_HIGH = (uchar)(20);
const uchar HEIGHTMAP_CONTACTTHRES_LOW = (uchar)(10);

const float HEIGHTMAP_MAXHEIGHT_RATIO = (float)(0.5);
const float HEIGHTMAP_MAXGRAD_RATIO = (float)(1.0/(0.7 * CV_PI));
const float HEIGHTMAP_MINGRAD = (float)(0.01);

const int HEIGHTMAP_UNIFORMIMG_RATIO = (int)(200);

const float RAD2DEGREE_RATIO = (float)(180.0/CV_PI);
const float DEGREE2RAD_RATIO = (float)(CV_PI/180.0);

// jet map
const int JET_COLORMAP_BINNUM = 64;
const int JET_COLORMAP_MAXNUM = JET_COLORMAP_BINNUM - 1;
const uchar JET_COLORMAP[JET_COLORMAP_BINNUM][3] = {
	{0, 0, 143},
	{0, 0, 159},
	{0, 0, 175},
	{0, 0, 191},
	{0, 0, 207},
	{0, 0, 223},
	{0, 0, 239},
	{0, 0, 255},
	{0, 16, 255},
	{0, 32, 255},
	{0, 48, 255},
	{0, 64, 255},
	{0, 80, 255},
	{0, 96, 255},
	{0, 112, 255},
	{0, 128, 255},
	{0, 143, 255},
	{0, 159, 255},
	{0, 175, 255},
	{0, 191, 255},
	{0, 207, 255},
	{0, 223, 255},
	{0, 239, 255},
	{0, 255, 255},
	{16, 255, 239},
	{32, 255, 223},
	{48, 255, 207},
	{64, 255, 191},
	{80, 255, 175},
	{96, 255, 159},
	{112, 255, 143},
	{128, 255, 128},
	{143, 255, 112},
	{159, 255, 96},
	{175, 255, 80},
	{191, 255, 64},
	{207, 255, 48},
	{223, 255, 32},
	{239, 255, 16},
	{255, 255, 0},
	{255, 239, 0},
	{255, 223, 0},
	{255, 207, 0},
	{255, 191, 0},
	{255, 175, 0},
	{255, 159, 0},
	{255, 143, 0},
	{255, 128, 0},
	{255, 112, 0},
	{255, 96, 0},
	{255, 80, 0},
	{255, 64, 0},
	{255, 48, 0},
	{255, 32, 0},
	{255, 16, 0},
	{255, 0, 0},
	{239, 0, 0},
	{223, 0, 0},
	{207, 0, 0},
	{191, 0, 0},
	{175, 0, 0},
	{159, 0, 0},
	{143, 0, 0},
	{128, 0, 0}};

class CGelSightHeightMap
{
public:
	CGelSightHeightMap();
	~CGelSightHeightMap();

	// set parameters for 3d reconstructon
	void SetFileNameAndInitBluredImg(const std::string InSaveDataPreFileName, const cv::Mat inInitBluredColorImg);

	void GetCurrHeightMap(cv::Mat inCurrColorImg, bool bSaveData);
	bool LoadCalibrationData(const std::string InCalibrationFileName);

private:

	// get the weight of initial image
	void GetInitFrameWeightRatio(const cv::Mat inRawImg, cv::Mat &outImgWeight);

	void ShowGradImg(const cv::Mat inGradX, const cv::Mat inGradY);

	void ShowHeightMap(const cv::Mat inHeightImg);

	void DetectBlackMarkerMask(const cv::Mat inCurrColorImg, const int border_size, cv::Mat &outMarkerMaskImg);

	void DetectContactMapMask(const cv::Mat inCurrColorImg, const cv::Mat inInitBluredColorImg, 
		const int border_size, cv::Mat &outContactMapMaskImg);

	void DetectContactMapMaskWithoutBlackMarker(const cv::Mat inCurrColorImg, const cv::Mat inInitBluredColorImg, 
		const int border_size, cv::Mat &outMaskImg);

	void GetCurrImgGradFromLookupTable(const cv::Mat InCurrImg, const cv::Mat RawInitWeightFloat,
		const cv::Mat inMaskImg, cv::Mat &ImgGradX, cv::Mat &ImgGradY, cv::Mat &outImgGradMap);

	void GetCurrUniformImg(const cv::Mat InCurrColorImg, const int ImgUniformScale, cv::Mat &OutUniformColorImg);

	void SaveCurrHeightMapToFiles(const int FrameIndex);

private:

	COpenCVPoissonSolver m_PoissonSolver;

	std::string m_SaveDataPreFileName;

	cv::Mat m_ResizedInitBluredColorImg;
	cv::Mat m_ResizedInitBluredColorImgWeightFloat;

	cv::Mat m_LookupTabGradMapX;
	cv::Mat m_LookupTabGradMapY;

	cv::Mat m_CurrResizedColorImg;

	cv::Mat m_CurrHeightMap;
	cv::Mat m_CurrHeightGrayMap;
	cv::Mat m_CurrHeightColorMap;

	int m_HeightMapFrameSaveIndex;

	float HEIGHTMAP_Pixel2mm_RATIO;
	float HEIGHTMAP_ACTUAL_Pixel2mm_RATIO;

};
#endif // CGELSIGHTHEIGHTMAP_H_