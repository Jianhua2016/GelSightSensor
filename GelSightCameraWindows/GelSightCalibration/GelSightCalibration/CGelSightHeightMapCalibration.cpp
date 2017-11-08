#include "CGelSightHeightMapCalibration.h"

CGelSightHeightMapCalibration::CGelSightHeightMapCalibration()
{
	m_SaveDataPreFileName = "";

	// initial the lookup table data
	const int localGradHeight = CALIB_LOOKUPTAB_BINNUM * CALIB_LOOKUPTAB_BINNUM;
	m_LookupTabCountMap = cv::Mat::zeros(cv::Size(CALIB_LOOKUPTAB_BINNUM, localGradHeight), CV_32SC1);
	m_LookupTabGradMapX = cv::Mat::zeros(cv::Size(CALIB_LOOKUPTAB_BINNUM, localGradHeight), CV_32FC1);
	m_LookupTabGradMapY = cv::Mat::zeros(cv::Size(CALIB_LOOKUPTAB_BINNUM, localGradHeight), CV_32FC1);
}


CGelSightHeightMapCalibration::~CGelSightHeightMapCalibration()
{

}

void CGelSightHeightMapCalibration::SetFileNameAndInitBluredImg(const std::string inSaveDataPreFileName, 
	const cv::Mat inInitBluredColorImg)
{
	CV_Assert((inInitBluredColorImg.type() == CV_8UC3) && (inInitBluredColorImg.size() == CALIB_HEIGHTMAP_RAWIMG_SIZE));

	m_SaveDataPreFileName = inSaveDataPreFileName;
	m_InitBluredColorImg = inInitBluredColorImg.clone();

	// get the weight for init image
	GetInitFrameWeightRatio(inInitBluredColorImg, m_InitBluredColorImgWeightFloat);
}

// get the weight of initial image
void CGelSightHeightMapCalibration::GetInitFrameWeightRatio(const cv::Mat inRawImg, cv::Mat &outImgWeight)
{
	// make sure the input image with the correct type
	CV_Assert((inRawImg.type() == CV_8UC3) && (inRawImg.size() == CALIB_HEIGHTMAP_RAWIMG_SIZE));

	// init the result
	outImgWeight = cv::Mat::zeros(inRawImg.size(), CV_32FC3);
	cv::Mat localRawImg= inRawImg.clone();

	// get the raw image size
	const int ImgWidth = inRawImg.cols;
	const int ImgHeight = inRawImg.rows;
	const float localRatio = (float)(1.0);
	const uchar local_thres = (uchar)(0);

	cv::Vec3b* localRawImg_ptr;
	cv::Vec3f* OutImgWeight_ptr;
	for (int iRow = 0; iRow < ImgHeight; iRow++){
		localRawImg_ptr = localRawImg.ptr<cv::Vec3b>(iRow);
		OutImgWeight_ptr = outImgWeight.ptr<cv::Vec3f>(iRow);
		for (int iCol = 0; iCol < ImgWidth; iCol++){
			for(int iChanel = 0; iChanel < 3; iChanel++){
				if((*localRawImg_ptr)[iChanel] > local_thres){
					(*OutImgWeight_ptr)[iChanel] = localRatio / (float)((*localRawImg_ptr)[iChanel]);
				}
			}
			
			// move to the next position
			localRawImg_ptr++;
			OutImgWeight_ptr++;
		}
	}
}

int CGelSightHeightMapCalibration::SaveLookupTableDataToFile(){
	// save the lookup table date
	const std::string localSaveFileNameLookupTab = m_SaveDataPreFileName + "_CalibLookupTab.xml";
	std::cout << "SaveFileNameLookupTab: " << localSaveFileNameLookupTab << std::endl;

	// ====================================================================================== 
	FileStorage LookupTabGra_fs(localSaveFileNameLookupTab, FileStorage::WRITE);
	LookupTabGra_fs << "CALIB_LOOKUPTAB_BINNUM" << CALIB_LOOKUPTAB_BINNUM;
	LookupTabGra_fs << "CALIB_Pixel2mm_ratio" << CALIB_Pixel2mm_ratio;
	LookupTabGra_fs << "CALIB_BALL_ACTUALRADIUS_mm" << CALIB_BALL_ACTUALRADIUS_mm;
	// cv::Mat
	LookupTabGra_fs << "LookupTabGradMapX" << m_LookupTabGradMapX;  
	LookupTabGra_fs << "LookupTabGradMapY" << m_LookupTabGradMapY; 
	// explicit close                                    
	LookupTabGra_fs.release();                                     

	std::cout << "Successfull to write data to file" << std::endl;

	return 1;
}

bool CGelSightHeightMapCalibration::FindBallPositionInCurrImgForCalibration(const cv::Mat inCurrColorImg)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inCurrColorImg.size() == CALIB_HEIGHTMAP_RAWIMG_SIZE));
	m_CurrColorImg = inCurrColorImg;

	const int localFontFace = cv::FONT_HERSHEY_SIMPLEX;
	const double localFontScale = 1.0;
	const int localTextThickness = 2;
	std::string localShowText = "Press C or c to capture calib image";
	const cv::Point localTextOrg(10, 50);

	DetectCircleInColorImg(inCurrColorImg, m_InitBluredColorImg, m_PredCircleCenter, m_PredCircleRadius);

	// show the result
	cv::Mat localShowImg = inCurrColorImg.clone();
	cv::putText(localShowImg, localShowText, localTextOrg, localFontFace, localFontScale, cv::Scalar(0, 0, 255),
		localTextThickness, 8);

	if ((m_PredCircleRadius > 0.0) && (m_PredCircleCenter.x > 0.0) && (m_PredCircleCenter.y > 0.0)){
		cv::circle(localShowImg, m_PredCircleCenter, (int)(m_PredCircleRadius), CV_RGB(255, 0, 0), 1, 8, 0);
		cv::imshow("BallPositionInCurrImg", localShowImg);
		return true;
	}
	else {
		cv::imshow("BallPositionInCurrImg", localShowImg);
		return false;
	}
}

void CGelSightHeightMapCalibration::ManualSetBallPositionInCurrImgForCalibration(const int inCircleAddX, 
	const int inCircleAddY, const int inCircleAddRadius)
{
	m_ActualCircleCenter.x = m_PredCircleCenter.x + (float)(inCircleAddX);
	m_ActualCircleCenter.y = m_PredCircleCenter.y + (float)(inCircleAddY);
	m_ActualCircleRadius = m_PredCircleRadius + (float)(inCircleAddRadius);
	m_ActualCircleRadius = std::min(std::max(1.0f, m_ActualCircleRadius), (CALIB_BALL_ACTUALRADIUS_pixel - 0.5f));

	const int localFontFace = cv::FONT_HERSHEY_SIMPLEX;
	const double localFontScale = 1.0;
	const int localTextThickness = 2;
	std::string localShowText = "Press M or m to set calib image";
	const cv::Point localTextOrg(10, 50);

	cv::Mat localShowImg = m_CurrColorImg.clone();
	cv::putText(localShowImg, localShowText, localTextOrg, localFontFace, localFontScale, 
		cv::Scalar(0, 0, 255), localTextThickness, 8);

	cv::circle(localShowImg, m_ActualCircleCenter, (int)(m_ActualCircleRadius), CV_RGB(0, 255, 0), 1, 8, 0);
	cv::imshow("BallPositionInCurrImg", localShowImg);
}

void CGelSightHeightMapCalibration::GetLookupTableFromBall(const int FrameIndex)
{
	std::ostringstream localSaveDataFilename;
	localSaveDataFilename << m_SaveDataPreFileName << "_calibimg_" << std::setfill('0') 
		<< std::setw(6) << FrameIndex << ".png";
	cv::imwrite(localSaveDataFilename.str(), m_CurrColorImg);

	std::ostringstream localSaveResultFilename;
	localSaveResultFilename << m_SaveDataPreFileName << "_calibimg_" << std::setfill('0') 
		<< std::setw(6) << FrameIndex << "_result.txt";

	const std::string outresult_txtfilename = localSaveResultFilename.str();
	std::ofstream localwritefile(outresult_txtfilename.c_str());
	if (localwritefile.is_open() == false) {
		std::cout << "Unable to open file" << std::endl;
		return;
	}
	localwritefile << "circlex(pixel) = " << std::setprecision(8) <<  m_ActualCircleCenter.x <<std::endl;
	localwritefile << "circley(pixel) = " << std::setprecision(8) <<  m_ActualCircleCenter.y <<std::endl;
	localwritefile << "radius(pixel) = " << std::setprecision(8) <<  m_ActualCircleRadius <<std::endl;
	localwritefile.close();

	UpdateLookuptableFromBall(m_CurrColorImg, m_InitBluredColorImgWeightFloat, m_ActualCircleCenter, 
		m_ActualCircleRadius, CALIB_BALL_ACTUALRADIUS_pixel, CALIB_HEIGHTMAP_RAWIMG_BORDER);
}

void CGelSightHeightMapCalibration::UpdateLookuptableFromBall(const cv::Mat inCurrColorImg, 
	const cv::Mat inInitWeightFloat, const cv::Point2f inCircleCenterPixel, const float inCircleRadiusPixel, 
	const float inBallActualRadiusPixel, const int border_size)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inInitWeightFloat.type() == CV_32FC3)
		&& (inCurrColorImg.size() == inInitWeightFloat.size()) && (inCurrColorImg.size() == CALIB_HEIGHTMAP_RAWIMG_SIZE));

	cv::Mat localCurrFloatImg;
	inCurrColorImg.convertTo(localCurrFloatImg, CV_32FC3, 1.0, 0.0);

	cv::Mat localMarkerMask;
	DetectBlackMarkerMask(inCurrColorImg, CALIB_HEIGHTMAP_RAWIMG_BORDER, localMarkerMask);

	cv::Mat localInitWeightFloat = inInitWeightFloat.clone();
	const int ImgWidth = inCurrColorImg.cols;
	const int ImgHeight = inCurrColorImg.rows;

	const int localImg_ROI_row_start = border_size;
	const int localImg_ROI_row_end = ImgHeight - border_size;
	const int localImg_ROI_col_start = border_size;
	const int localImg_ROI_col_end = ImgWidth - border_size;

	std::cout << "inCircleCenterPixel = " << inCircleCenterPixel << std::endl;
	std::cout << "inCircleRadiusPixel = " << inCircleRadiusPixel << std::endl;

	int iRowStart = (int)(inCircleCenterPixel.y - inCircleRadiusPixel);
	int iRowEnd = (int)(inCircleCenterPixel.y + inCircleRadiusPixel);
	iRowStart = std::min(std::max(iRowStart, localImg_ROI_row_start), localImg_ROI_row_end);
	iRowEnd = std::min(std::max(iRowEnd, localImg_ROI_row_start), localImg_ROI_row_end);

	int iColStart = (int)(inCircleCenterPixel.x - inCircleRadiusPixel);
	int iColEnd = (int)(inCircleCenterPixel.x + inCircleRadiusPixel);
	iColStart = std::min(std::max(iColStart, localImg_ROI_col_start), localImg_ROI_col_end);
	iColEnd = std::min(std::max(iColEnd, localImg_ROI_col_start), localImg_ROI_col_end);

	const float localBallActualRadiusPixel_2 = inBallActualRadiusPixel * inBallActualRadiusPixel;
	const float localCircleRadiusPixel_2 = inCircleRadiusPixel * inCircleRadiusPixel;

	cv::Vec3f* localCurrFloatImg_ptr;
	cv::Vec3f* localInitWeightFloat_ptr;
	uchar* localMarkerMask_ptr;
	for (int iRow = iRowStart; iRow < iRowEnd; iRow++){
		localCurrFloatImg_ptr = localCurrFloatImg.ptr<cv::Vec3f>(iRow);
		localInitWeightFloat_ptr = localInitWeightFloat.ptr<cv::Vec3f>(iRow);
		localMarkerMask_ptr = localMarkerMask.ptr<uchar>(iRow);
		for (int iCol = 0; iCol < iColEnd; iCol++){
			// remove the black markers
			if ((iCol >= iColStart) && (*localMarkerMask_ptr > 0)){
				const float localGradGx = -((float)(iCol) - inCircleCenterPixel.x);
				const float localGradGy = -((float)(iRow) - inCircleCenterPixel.y);
				const float local_radius_2 = localGradGx * localGradGx + localGradGy * localGradGy;

				// make sure in the inner part of the circle
				if(local_radius_2 < (localCircleRadiusPixel_2 - 0.09f)){
					int localIntValue[3];
					for (int i = 0; i < 3; i++){
						// localCurrFloatImg_ptr/init is in [0.2, 1.4] to [0, 60]
						float localFloatValue = (*localCurrFloatImg_ptr)[i] * (*localInitWeightFloat_ptr)[i];
						localFloatValue = (float)((localFloatValue - 0.2f) * 50.0f);

						localIntValue[i] = (int)(std::floor(localFloatValue + 0.5f));
						localIntValue[i] = std::min(std::max(0, localIntValue[i]), CALIB_LOOKUPTAB_MAXBINNUM);
					}

					const float localBallZ = std::sqrt(localBallActualRadiusPixel_2 - local_radius_2);
					const float localGradAngleX = std::atan2(localGradGx, localBallZ);
					const float localGradAngleY = std::atan2(localGradGy, localBallZ);

					const int loca_row_no = localIntValue[0] * CALIB_LOOKUPTAB_BINNUM + localIntValue[1];
					const int loca_col_no = localIntValue[2];

					// in OpenCV, it is in [b g r]
					const float localFloatT1 = (float)(m_LookupTabCountMap.at<int>(loca_row_no, loca_col_no));
					const float localRatio = 1.0f / (1.0f + localFloatT1);

					m_LookupTabGradMapX.at<float>(loca_row_no, loca_col_no) =
						(m_LookupTabGradMapX.at<float>(loca_row_no, loca_col_no) * localFloatT1 + localGradAngleX) * localRatio;

					m_LookupTabGradMapY.at<float>(loca_row_no, loca_col_no) =
						(m_LookupTabGradMapY.at<float>(loca_row_no, loca_col_no) * localFloatT1 + localGradAngleY) * localRatio;

					m_LookupTabCountMap.at<int>(loca_row_no, loca_col_no)++;
				}
			}
			localCurrFloatImg_ptr++;
			localInitWeightFloat_ptr++;
			localMarkerMask_ptr++;
		}
	}
}

void CGelSightHeightMapCalibration::SmoothLookuptable()
{
	const int localGradHeight = CALIB_LOOKUPTAB_BINNUM * CALIB_LOOKUPTAB_BINNUM;
	// get the valid point number
	int TotalValidPtNum = 0;
	for (int i = 0; i < localGradHeight; i++){
		for (int j = 0; j < CALIB_LOOKUPTAB_BINNUM; j++){
			if (m_LookupTabCountMap.at<int>(i, j) > 0){
				TotalValidPtNum++;
			}
		}
	}
	std::cout << "TotalValidPtNum = " << TotalValidPtNum << std::endl;

	int** ValidPoint;
	ValidPoint = new int*[TotalValidPtNum];
	for (int i = 0; i < TotalValidPtNum; i++){
		ValidPoint[i] = new int[3];
	}

	// get the valid point position
	int ValidPtNum = 0;
	for (int i = 0; i < localGradHeight; i++){
		for (int j = 0; j < CALIB_LOOKUPTAB_BINNUM; j++){
			if (m_LookupTabCountMap.at<int>(i, j) > 0){
				ValidPoint[ValidPtNum][0] = i/CALIB_LOOKUPTAB_BINNUM;
				ValidPoint[ValidPtNum][1] = i%CALIB_LOOKUPTAB_BINNUM;
				ValidPoint[ValidPtNum][2] = j;
				ValidPtNum++;
			}
		}
	}

	const int localSetMaxValue = CALIB_LOOKUPTAB_BINNUM * CALIB_LOOKUPTAB_BINNUM * CALIB_LOOKUPTAB_BINNUM;
	const int localSetMaxValidDist = (int)(0.0225f * CALIB_LOOKUPTAB_BINNUM * CALIB_LOOKUPTAB_BINNUM);

	// get the valid point position
	int local_update_point_num = 0;
	for (int i = 0; i < localGradHeight; i++){
		const int local_i = i/CALIB_LOOKUPTAB_BINNUM;
		const int local_j = i%CALIB_LOOKUPTAB_BINNUM;

		for (int k = 0; k < CALIB_LOOKUPTAB_BINNUM; k++){
			if (m_LookupTabCountMap.at<int>(i, k) == 0){
				int localMinValue = localSetMaxValue;
				int localPos[3];
				localPos[0] = 0;
				localPos[1] = 0;
				localPos[2] = 0;
				for (int l = 0; l < TotalValidPtNum; l++){
					int localValue = (ValidPoint[l][0] - local_i) * (ValidPoint[l][0] - local_i)
						+ (ValidPoint[l][1] - local_j) * (ValidPoint[l][1] - local_j)
						+ (ValidPoint[l][2] - k) * (ValidPoint[l][2] - k);

					if (localValue < localMinValue){
						localMinValue = localValue;
						localPos[0] = ValidPoint[l][0];
						localPos[1] = ValidPoint[l][1];
						localPos[2] = ValidPoint[l][2];
					}
				}

				// update
				if (localMinValue < localSetMaxValidDist){
					const int loca_row_no = localPos[0] * CALIB_LOOKUPTAB_BINNUM + localPos[1];
					const int loca_col_no = localPos[2];

					m_LookupTabGradMapX.at<float>(i, k) = m_LookupTabGradMapX.at<float>(loca_row_no, loca_col_no);
					m_LookupTabGradMapY.at<float>(i, k) = m_LookupTabGradMapY.at<float>(loca_row_no, loca_col_no);
					local_update_point_num++;
				}
			}
		}
	}
	std::cout << "local_update_point_num = " << local_update_point_num << std::endl;

	// release the data
	for (int i = 0; i < TotalValidPtNum; i++){
		delete[] ValidPoint[i];
	}
	delete[] ValidPoint;
}

void CGelSightHeightMapCalibration::GetCurrUniformImg(const cv::Mat inCurrColorImg, const int ImgUniformScale, 
	cv::Mat &OutUniformColorImg)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inCurrColorImg.size() == CALIB_HEIGHTMAP_RAWIMG_SIZE));

	// set the output
	OutUniformColorImg = cv::Mat::zeros(inCurrColorImg.size(), CV_8UC3);

	const int ImgWidth = inCurrColorImg.cols;
	const int ImgHeight = inCurrColorImg.rows;

	cv::Mat localCurrColorImgFloat;
	inCurrColorImg.convertTo(localCurrColorImgFloat, CV_32FC3, 1.0, 0.0);
	cv::Mat localInitColorImgWeightFloat = m_InitBluredColorImgWeightFloat.clone();

	cv::Vec3f* localCurrColorImgFloat_ptr;
	cv::Vec3f* localInitColorImgWeightFloat_ptr;
	cv::Vec3b* OutUniformColorImg_ptr;
	for (int iRow = 0; iRow < ImgHeight; iRow++){
		localCurrColorImgFloat_ptr = localCurrColorImgFloat.ptr<cv::Vec3f>(iRow);
		localInitColorImgWeightFloat_ptr = localInitColorImgWeightFloat.ptr<cv::Vec3f>(iRow);
		OutUniformColorImg_ptr = OutUniformColorImg.ptr<cv::Vec3b>(iRow);

		for (int iCol = 0; iCol < ImgWidth; iCol++){
			for (int i = 0; i < 3; i++){
				float localValue = (*localCurrColorImgFloat_ptr)[i] * (*localInitColorImgWeightFloat_ptr)[i];
				(*OutUniformColorImg_ptr)[i] = cv::saturate_cast<uchar>(ImgUniformScale * localValue);
			}

			// move to the next point
			localCurrColorImgFloat_ptr++;
			localInitColorImgWeightFloat_ptr++;
			OutUniformColorImg_ptr++;
		}
	}
}

void CGelSightHeightMapCalibration::DetectBlackMarkerMask(const cv::Mat inCurrColorImg, const int border_size, 
	cv::Mat &outMarkerMaskImg)
{
	CV_Assert(inCurrColorImg.type() == CV_8UC3);

	// init image
	outMarkerMaskImg = cv::Mat(inCurrColorImg.size(), CV_8UC1, (uchar)(255));
	cv::Mat localCurrColorImg = inCurrColorImg.clone();
	
	const int ImgHeight = inCurrColorImg.rows;
	const int ImgWidth = inCurrColorImg.cols;
	const int localImg_ROI_row_start = border_size;
	const int localImg_ROI_row_end = ImgHeight - border_size;
	const int localImg_ROI_col_start = border_size;
	const int localImg_ROI_col_end = ImgWidth - border_size;

	//=================================================================================================================================
	// find the black part
	cv::Vec3b* localCurrColorImg_ptr;
	uchar* outMarkerMaskImg_ptr;
	for (int iRow = localImg_ROI_row_start; iRow < localImg_ROI_row_end; iRow++){
		localCurrColorImg_ptr = localCurrColorImg.ptr<cv::Vec3b>(iRow);
		outMarkerMaskImg_ptr = outMarkerMaskImg.ptr<uchar>(iRow);

		for (int iCol = 0; iCol < localImg_ROI_col_end; iCol++){
			if(iCol >= localImg_ROI_col_start){
				uchar localValue[3];
				localValue[0] = (*localCurrColorImg_ptr)[0];
				localValue[1] = (*localCurrColorImg_ptr)[1];
				localValue[2] = (*localCurrColorImg_ptr)[2];

				const uchar localMaxValue = std::max(localValue[0], std::max(localValue[1], localValue[2]));
				const uchar localMinValue = std::min(localValue[0], std::min(localValue[1], localValue[2]));
				
				if ((localMinValue < CALIB_MARKERTHRES_LOW) && (localMaxValue < CALIB_MARKERTHRES_HIGH)){
					(*outMarkerMaskImg_ptr) = (uchar)(0);
				}
			}
			localCurrColorImg_ptr++;
			outMarkerMaskImg_ptr++;
		}
	}
}

void CGelSightHeightMapCalibration::DetectContactMapMask(const cv::Mat inCurrColorImg, const cv::Mat inInitBluredColorImg, 
	const int border_size, cv::Mat &outContactMapMaskImg)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inInitBluredColorImg.type() == CV_8UC3)
		&& (inCurrColorImg.size() == inInitBluredColorImg.size()));

	// init image
	outContactMapMaskImg = cv::Mat::zeros(inCurrColorImg.size(), CV_8UC1);
	cv::Mat localCurrColorImg = inCurrColorImg.clone();
	cv::Mat localInitBluredColorImg = inInitBluredColorImg.clone();
	
	const int ImgHeight = inCurrColorImg.rows;
	const int ImgWidth = inCurrColorImg.cols;
	const int localImg_ROI_row_start = border_size;
	const int localImg_ROI_row_end = ImgHeight - border_size;
	const int localImg_ROI_col_start = border_size;
	const int localImg_ROI_col_end = ImgWidth - border_size;

	//=================================================================================================================================
	// find the contact part
	cv::Vec3b* localCurrColorImg_ptr;
	cv::Vec3b* localInitBluredColorImg_ptr;
	uchar* outContactMapMaskImg_ptr;

	for (int iRow = localImg_ROI_row_start; iRow < localImg_ROI_row_end; iRow++){
		localCurrColorImg_ptr = localCurrColorImg.ptr<cv::Vec3b>(iRow);
		localInitBluredColorImg_ptr = localInitBluredColorImg.ptr<cv::Vec3b>(iRow);
		outContactMapMaskImg_ptr = outContactMapMaskImg.ptr<uchar>(iRow);

		for (int iCol = 0; iCol < localImg_ROI_col_end; iCol++){
			if(iCol >= localImg_ROI_col_start){
				uchar localHighValue[3];
				uchar localLowValue[3];
				for(int local_i = 0; local_i < 3; local_i++){
					if((*localCurrColorImg_ptr)[local_i] >= (*localInitBluredColorImg_ptr)[local_i]){
						localHighValue[local_i] = (*localCurrColorImg_ptr)[local_i] - (*localInitBluredColorImg_ptr)[local_i];
						localLowValue[local_i] = (uchar)(0);
					}else{
						localHighValue[local_i] = (uchar)(0);
						localLowValue[local_i] = (*localInitBluredColorImg_ptr)[local_i] - (*localCurrColorImg_ptr)[local_i];
					}
				}
				const uchar localMaxHighValue = std::max(localHighValue[0], std::max(localHighValue[1], localHighValue[2]));
				const uchar localMinLowValue = std::max(localLowValue[0], std::max(localLowValue[1], localLowValue[2]));
				
				if ((localMaxHighValue >= CALIB_CONTACTTHRES_HIGH) && (localMinLowValue >= CALIB_CONTACTTHRES_LOW)){
					(*outContactMapMaskImg_ptr) = (uchar)(255);
				}
			}

			localCurrColorImg_ptr++;
			localInitBluredColorImg_ptr++;
			outContactMapMaskImg_ptr++;
		}
	}
}

void CGelSightHeightMapCalibration::DetectContactMapMaskWithoutBlackMarker(const cv::Mat inCurrColorImg, 
	const cv::Mat inInitBluredColorImg, const int border_size, cv::Mat &outMaskImg)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inInitBluredColorImg.type() == CV_8UC3)
		&& (inCurrColorImg.size() == inInitBluredColorImg.size()));

	// init image
	outMaskImg = cv::Mat::zeros(inCurrColorImg.size(), CV_8UC1);
	cv::Mat localCurrColorImg = inCurrColorImg.clone();
	cv::Mat localInitBluredColorImg = inInitBluredColorImg.clone();
	
	const int ImgHeight = inCurrColorImg.rows;
	const int ImgWidth = inCurrColorImg.cols;
	const int localImg_ROI_row_start = border_size;
	const int localImg_ROI_row_end = ImgHeight - border_size;
	const int localImg_ROI_col_start = border_size;
	const int localImg_ROI_col_end = ImgWidth - border_size;

	//======================================================================================================
	// find the contact part
	cv::Vec3b* localCurrColorImg_ptr;
	cv::Vec3b* localInitBluredColorImg_ptr;
	uchar* outMaskImg_ptr;

	for (int iRow = localImg_ROI_row_start; iRow < localImg_ROI_row_end; iRow++){
		localCurrColorImg_ptr = localCurrColorImg.ptr<cv::Vec3b>(iRow);
		localInitBluredColorImg_ptr = localInitBluredColorImg.ptr<cv::Vec3b>(iRow);
		outMaskImg_ptr = outMaskImg.ptr<uchar>(iRow);

		for (int iCol = 0; iCol < localImg_ROI_col_end; iCol++){
			if(iCol >= localImg_ROI_col_start){
				uchar localValue[3];
				localValue[0] = (*localCurrColorImg_ptr)[0];
				localValue[1] = (*localCurrColorImg_ptr)[1];
				localValue[2] = (*localCurrColorImg_ptr)[2];

				const uchar localMaxValue = std::max(localValue[0], std::max(localValue[1], localValue[2]));
				const uchar localMinValue = std::min(localValue[0], std::min(localValue[1], localValue[2]));

				if ((localMinValue < CALIB_MARKERTHRES_LOW) && (localMaxValue < CALIB_MARKERTHRES_HIGH)){
					(*outMaskImg_ptr) = (uchar)(0);
				}else{
					uchar localHighValue[3];
					uchar localLowValue[3];
					for(int local_i = 0; local_i < 3; local_i++){
						if((*localCurrColorImg_ptr)[local_i] >= (*localInitBluredColorImg_ptr)[local_i]){
							localHighValue[local_i] = (*localCurrColorImg_ptr)[local_i] - (*localInitBluredColorImg_ptr)[local_i];
							localLowValue[local_i] = (uchar)(0);
						}else{
							localHighValue[local_i] = (uchar)(0);
							localLowValue[local_i] = (*localInitBluredColorImg_ptr)[local_i] - (*localCurrColorImg_ptr)[local_i];
						}
					}
					const uchar localMaxHighValue = std::max(localHighValue[0], std::max(localHighValue[1], localHighValue[2]));
					const uchar localMinLowValue = std::max(localLowValue[0], std::max(localLowValue[1], localLowValue[2]));

					if ((localMaxHighValue >= CALIB_CONTACTTHRES_HIGH) && (localMinLowValue >= CALIB_CONTACTTHRES_LOW)){
						(*outMaskImg_ptr) = (uchar)(255);
					}

				}
			}

			localCurrColorImg_ptr++;
			localInitBluredColorImg_ptr++;
			outMaskImg_ptr++;
		}
	}
}

void CGelSightHeightMapCalibration::DetectCircleInColorImg(const cv::Mat inCurrColorImg, const cv::Mat inInitColorImg, 
	cv::Point2f &outPredCircleCenter, float &outPredCircleRadius)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inInitColorImg.type() == CV_8UC3)
		&& (inCurrColorImg.size() == inInitColorImg.size()));

	// init the value
	cv::Point2f localCircleCenter = cv::Point2f(0.0f, 0.0f);
	float localCircleRadius = 0.0f;
	outPredCircleCenter = localCircleCenter;
	outPredCircleRadius = localCircleRadius;

	cv::Mat localMaskImg, localNonZeroImg;
	DetectContactMapMaskWithoutBlackMarker(inCurrColorImg, m_InitBluredColorImg, CALIB_HEIGHTMAP_RAWIMG_BORDER, 
		localMaskImg);
	// cv::imshow("MaskImage", localMaskImg);

	const int erosion_size = 6;
	const cv::Mat localElement = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		cv::Point(erosion_size, erosion_size));

	cv::erode(localMaskImg, localMaskImg, localElement);
	cv::dilate(localMaskImg, localMaskImg, localElement);
	cv::findNonZero(localMaskImg, localNonZeroImg);

	//std::cout << "The Center is "  << localNonZeroImg.size()<< std::endl;
	if (localNonZeroImg.rows < 1000){
		// std::cerr << "There are not enough points" << std::endl;
		return;
	}
	// cv::imshow("MaskImageAfterFilter", localMaskImg);
	cv::minEnclosingCircle(localNonZeroImg, localCircleCenter, localCircleRadius);

	outPredCircleCenter = localCircleCenter;
	outPredCircleRadius = localCircleRadius - (float)(10.0);
	// std::cout << "The Center is " << outPredCircleCenter << " and Radius is " << outPredCircleRadius << std::endl;
}
