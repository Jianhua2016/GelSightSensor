// #pragma once
#ifndef COPENCVPOISSONSOLVER_H_
#define COPENCVPOISSONSOLVER_H_

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

#pragma comment(lib, "opencv_world320.lib")

using namespace std;
using namespace cv;

// dst size for fft
const cv::Size HEIGHTMAP_OPENCV_DST_IMG_SIZE = cv::Size(319, 239);

class COpenCVPoissonSolver
{
public:
	COpenCVPoissonSolver();
	~COpenCVPoissonSolver();
	void OpenCV_DST_PossionSolver(const cv::Mat InRawImgGradx, const cv::Mat InRawImgGrady, cv::Mat &OutImgDirect);
private:
	void init_OpenCV_DST_Eigen_Values();
	void get_img_DST_with_OpenCV(const cv::Mat inImg, cv::Mat &outImgDST);
	void get_img_iDST_with_OpenCV(const cv::Mat inImg, cv::Mat &outImgDST);    
private:
	cv::Mat m_OpenCV_DST_eigen_values;
};

#endif // COPENCVPOISSONSOLVER_H_