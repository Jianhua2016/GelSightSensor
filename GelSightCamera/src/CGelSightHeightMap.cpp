#include "CGelSightHeightMap.h"

CGelSightHeightMap::CGelSightHeightMap()
{
	m_SaveDataPreFileName = "";
	m_HeightMapFrameSaveIndex = 0;

	// m_PoissonSolver.init_DST_Eigen_Values();	
	// CV_Assert((LookupTabWidth == LOOKUPTAB_BINNUM) && (LookupTabHeight == (LOOKUPTAB_BINNUM * LOOKUPTAB_BINNUM)));
}

CGelSightHeightMap::~CGelSightHeightMap()
{

}

bool CGelSightHeightMap::LoadCalibrationData(const std::string InCalibrationFileName){
	std::cout << std::endl << "Loading gelsight calbration data" << std::endl;

	FileStorage LookupTab_fs;
	LookupTab_fs.open(InCalibrationFileName, FileStorage::READ);
	int CALIB_LOOKUPTAB_BINNUM;
	CALIB_LOOKUPTAB_BINNUM = (int)(LookupTab_fs["CALIB_LOOKUPTAB_BINNUM"]);

	if (!LookupTab_fs.isOpened()){
		std::cerr << "Failed to open " << InCalibrationFileName << std::endl;
		return false;
	}

	HEIGHTMAP_Pixel2mm_RATIO = (float)(LookupTab_fs["CALIB_Pixel2mm_ratio"]);
	float local_ball_radius = (float)(LookupTab_fs["CALIB_BALL_ACTUALRADIUS_mm"]);
	HEIGHTMAP_ACTUAL_Pixel2mm_RATIO = HEIGHTMAP_Pixel2mm_RATIO/HEIGHTMAP_RESIZERATIO;
	CV_Assert(CALIB_LOOKUPTAB_BINNUM == LOOKUPTAB_BINNUM);
	CV_Assert(HEIGHTMAP_Pixel2mm_RATIO > 1e-5);

	// Read cv::Mat
	LookupTab_fs["LookupTabGradMapX"] >> m_LookupTabGradMapX;   
	LookupTab_fs["LookupTabGradMapY"] >> m_LookupTabGradMapY;  

	cv::Size local_size1 = m_LookupTabGradMapX.size();
	CV_Assert(local_size1 == cv::Size(LOOKUPTAB_BINNUM, (LOOKUPTAB_BINNUM*LOOKUPTAB_BINNUM)));
	cv::Size local_size2 = m_LookupTabGradMapY.size();
	CV_Assert(local_size2 == cv::Size(LOOKUPTAB_BINNUM, (LOOKUPTAB_BINNUM*LOOKUPTAB_BINNUM)));

	return true;
}

void CGelSightHeightMap::SetFileNameAndInitBluredImg(const std::string InSaveDataPreFileName, 
	const cv::Mat inInitBluredColorImg)
{
	CV_Assert((inInitBluredColorImg.type() == CV_8UC3) && (inInitBluredColorImg.size() == HEIGHTMAP_RAWIMG_SIZE));

	m_SaveDataPreFileName = InSaveDataPreFileName;
	cv::resize(inInitBluredColorImg, m_ResizedInitBluredColorImg, HEIGHTMAP_RESIZEDIMG_SIZE, 0, 0, INTER_CUBIC);

	// get the weight for init image
	GetInitFrameWeightRatio(m_ResizedInitBluredColorImg, m_ResizedInitBluredColorImgWeightFloat);

}

// get the weight of initial image
void CGelSightHeightMap::GetInitFrameWeightRatio(const cv::Mat inRawImg, cv::Mat &outImgWeight)
{
	// make sure the input image with the correct type
	CV_Assert((inRawImg.type() == CV_8UC3) && (inRawImg.size() == HEIGHTMAP_RESIZEDIMG_SIZE));

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
	for (int iRow = 0; iRow < ImgHeight; iRow++)
	{
		localRawImg_ptr = localRawImg.ptr<cv::Vec3b>(iRow);
		OutImgWeight_ptr = outImgWeight.ptr<cv::Vec3f>(iRow);
		for (int iCol = 0; iCol < ImgWidth; iCol++){
			if((*localRawImg_ptr)[0] > local_thres){
				(*OutImgWeight_ptr)[0] = localRatio / (float)((*localRawImg_ptr)[0]);
			}

			if((*localRawImg_ptr)[1] > local_thres){
				(*OutImgWeight_ptr)[1] = localRatio / (float)((*localRawImg_ptr)[1]);
			}

			if((*localRawImg_ptr)[2] > local_thres){
				(*OutImgWeight_ptr)[2] = localRatio / (float)((*localRawImg_ptr)[2]);
			}

			// move to the next position
			localRawImg_ptr++;
			OutImgWeight_ptr++;
		}
	}
}

void CGelSightHeightMap::GetCurrHeightMap(cv::Mat inCurrColorImg, bool bSaveData)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inCurrColorImg.size() == HEIGHTMAP_RAWIMG_SIZE));

	cv::Mat ImgGradX, ImgGradY, ImgGradMap;
	cv::Mat localMaskImg;

	cv::resize(inCurrColorImg, m_CurrResizedColorImg, HEIGHTMAP_RESIZEDIMG_SIZE, 0, 0, INTER_CUBIC);

	DetectContactMapMaskWithoutBlackMarker(m_CurrResizedColorImg, m_ResizedInitBluredColorImg, 
		HEIGHTMAP_GRADIMG_BORDER_SIZE, localMaskImg);
	GetCurrImgGradFromLookupTable(m_CurrResizedColorImg, m_ResizedInitBluredColorImgWeightFloat, localMaskImg, 
		ImgGradX, ImgGradY, ImgGradMap);

	// cv::imshow("ImgGradMap", ImgGradMap);

#ifdef HEIGHTMAP_SHOWGRAD
	ShowGradImg(ImgGradX, ImgGradY);
#endif
	
	// get the result
	// m_PoissonSolver.FastPossionSolver(ImgGradX, ImgGradY, m_CurrHeightMap);
	m_PoissonSolver.OpenCV_DST_PossionSolver(ImgGradX, ImgGradY, m_CurrHeightMap);
	m_CurrHeightMap = cv::max(m_CurrHeightMap, (float)(0.0));
	ShowHeightMap(m_CurrHeightMap);

	if(bSaveData){
		SaveCurrHeightMapToFiles(m_HeightMapFrameSaveIndex);
		m_HeightMapFrameSaveIndex++;
	}
}

void CGelSightHeightMap::ShowGradImg(const cv::Mat inGradX, const cv::Mat inGradY)
{
	CV_Assert((inGradX.type() == CV_32FC1) && (inGradX.type() == inGradY.type())
		&& (inGradX.size() == inGradY.size()) && (inGradX.size() == HEIGHTMAP_GRADIMG_SIZE));

	const int localImgRows = inGradX.rows;
	const int localImgCols = inGradX.cols;

	const int localSetImgDist = 20;
	const int locaSetImgHeight = 2 * localImgRows + localSetImgDist;
	const int locaSetImgWidth = 2 * localImgCols + localSetImgDist;
	cv::Mat localGradShowImg = cv::Mat::zeros(cv::Size(locaSetImgWidth, locaSetImgHeight), CV_8UC3);

	// Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
	cv::Mat localGradXColorImg = localGradShowImg(cv::Rect(0,0,localImgCols, localImgRows));
	cv::Mat localGradYColorImg = localGradShowImg(cv::Rect((localImgCols + localSetImgDist),0,localImgCols, localImgRows));
	cv::Mat localGradMagColorImg = localGradShowImg(cv::Rect(0,(localImgRows + localSetImgDist),localImgCols, localImgRows));
	cv::Mat localGradAngleColorImg = localGradShowImg(
		cv::Rect((localImgCols + localSetImgDist),(localImgRows + localSetImgDist),localImgCols, localImgRows));

	cv::Vec3b* localGradXColorImg_ptr;
	cv::Vec3b* localGradYColorImg_ptr;
	cv::Vec3b* localGradMagColorImg_ptr;
	cv::Vec3b* localGradAngleColorImg_ptr;

	cv::Mat localGradX = inGradX.clone();
	cv::Mat localGradY = inGradY.clone();
	float* localGradX_ptr;
	float* localGradY_ptr;

	const float localAngleRatio = float(JET_COLORMAP_BINNUM) / (float)(360.0);
	for (int i = 0; i < localImgRows; i++){
		localGradX_ptr = localGradX.ptr<float>(i);
		localGradY_ptr = localGradY.ptr<float>(i);

		localGradXColorImg_ptr = localGradXColorImg.ptr<cv::Vec3b>(i);
		localGradYColorImg_ptr = localGradYColorImg.ptr<cv::Vec3b>(i);
		localGradMagColorImg_ptr = localGradMagColorImg.ptr<cv::Vec3b>(i);
		localGradAngleColorImg_ptr = localGradAngleColorImg.ptr<cv::Vec3b>(i);

		for (int j = 0; j < localImgCols; j++){
			const float localGx = *localGradX_ptr;
			const float localGy = *localGradY_ptr;
			const float localMag = std::sqrt(localGx * localGx + localGy * localGy);

			int local_index_gx = std::floor((HEIGHTMAP_MAXGRAD_RATIO * std::tan(localGx) + 0.5) * JET_COLORMAP_BINNUM);
			local_index_gx = std::min(JET_COLORMAP_MAXNUM, std::max(0, local_index_gx));
			(*localGradXColorImg_ptr)[0] = JET_COLORMAP[local_index_gx][0];
			(*localGradXColorImg_ptr)[1] = JET_COLORMAP[local_index_gx][1];
			(*localGradXColorImg_ptr)[2] = JET_COLORMAP[local_index_gx][2];

			int local_index_gy = std::floor((HEIGHTMAP_MAXGRAD_RATIO * std::tan(localGy) + 0.5) * JET_COLORMAP_BINNUM);
			local_index_gy = std::min(JET_COLORMAP_MAXNUM, std::max(0, local_index_gy));
			(*localGradYColorImg_ptr)[0] = JET_COLORMAP[local_index_gy][0];
			(*localGradYColorImg_ptr)[1] = JET_COLORMAP[local_index_gy][1];
			(*localGradYColorImg_ptr)[2] = JET_COLORMAP[local_index_gy][2];

			int local_index_mag = std::floor((HEIGHTMAP_MAXGRAD_RATIO * std::tan(localMag) + 0.5) * JET_COLORMAP_BINNUM);
			local_index_mag = std::min(JET_COLORMAP_MAXNUM, std::max(0, local_index_mag));
			(*localGradMagColorImg_ptr)[0] = JET_COLORMAP[local_index_mag][0];
			(*localGradMagColorImg_ptr)[1] = JET_COLORMAP[local_index_mag][1];
			(*localGradMagColorImg_ptr)[2] = JET_COLORMAP[local_index_mag][2];

			// Calculates the angle of a 2D vector in degrees.
			// The function fastAtan2 calculates the full - range angle of an input 2D vector.
			// The angle is measured in degrees and varies from 0 to 360 degrees.The accuracy is about 0.3 degrees.
			// C++: float fastAtan2(float y, float x)
			if(localMag > HEIGHTMAP_MINGRAD){
				int local_index_ang = (int)(cv::fastAtan2(localGx, localGy) * localAngleRatio);
				local_index_ang = std::min(JET_COLORMAP_MAXNUM, std::max(0, local_index_ang));
				(*localGradAngleColorImg_ptr)[0] = JET_COLORMAP[local_index_ang][0];
				(*localGradAngleColorImg_ptr)[1] = JET_COLORMAP[local_index_ang][1];
				(*localGradAngleColorImg_ptr)[2] = JET_COLORMAP[local_index_ang][2];
			}

			localGradXColorImg_ptr++;
			localGradYColorImg_ptr++;
			localGradMagColorImg_ptr++;
			localGradAngleColorImg_ptr++;

			localGradX_ptr++;
			localGradY_ptr++;
		}
	}

	cv::imshow("GradShowImg", localGradShowImg);
}

void CGelSightHeightMap::ShowHeightMap(const cv::Mat inHeightMap)
{
	CV_Assert((inHeightMap.type() == CV_32FC1) && (inHeightMap.size() == HEIGHTMAP_GRADIMG_SIZE));
	const int localImgRows = inHeightMap.rows;
	const int localImgCols = inHeightMap.cols;

	cv::Mat localHeightColorImg = cv::Mat::zeros(cv::Size(localImgCols, localImgRows), CV_8UC3);
	cv::Mat localHeightGrayImg = cv::Mat::zeros(cv::Size(localImgCols, localImgRows), CV_8UC1);

	cv::Vec3b* localHeightColorImg_ptr;
	uchar* localHeightGrayImg_ptr;

	cv::Mat localHeightMap = inHeightMap.clone();
	float* localHeightMap_ptr;

	for (int i = 0; i < localImgRows; i++){
		localHeightMap_ptr = localHeightMap.ptr<float>(i);
		localHeightColorImg_ptr = localHeightColorImg.ptr<cv::Vec3b>(i);
		localHeightGrayImg_ptr = localHeightGrayImg.ptr<uchar>(i);

		for (int j = 0; j < localImgCols; j++){
			float localValue = HEIGHTMAP_ACTUAL_Pixel2mm_RATIO * (*localHeightMap_ptr);
			int local_color_index = std::floor((HEIGHTMAP_MAXHEIGHT_RATIO * localValue) * JET_COLORMAP_BINNUM);
			local_color_index = std::min(JET_COLORMAP_MAXNUM, std::max(0, local_color_index));

			(*localHeightColorImg_ptr)[0] = JET_COLORMAP[local_color_index][2];
			(*localHeightColorImg_ptr)[1] = JET_COLORMAP[local_color_index][1];
			(*localHeightColorImg_ptr)[2] = JET_COLORMAP[local_color_index][0];

			*localHeightGrayImg_ptr = cv::saturate_cast<uchar>(HEIGHTMAP_MAXHEIGHT_RATIO * localValue * 256);

			localHeightColorImg_ptr++;
			localHeightGrayImg_ptr++;

			localHeightMap_ptr++;
		}
	}

	cv::imshow("HeightColorImg", localHeightColorImg);
	cv::imshow("HeightGrayImg", localHeightGrayImg);

	m_CurrHeightGrayMap = localHeightGrayImg.clone();
	m_CurrHeightColorMap = localHeightColorImg.clone();
}

void CGelSightHeightMap::GetCurrImgGradFromLookupTable(const cv::Mat inCurrColorImg, const cv::Mat inInitWeightFloat,
	const cv::Mat inMaskImg, cv::Mat &outImgGradX, cv::Mat &outImgGradY, cv::Mat &outImgGradMap)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3)
		&& (inInitWeightFloat.type() == CV_32FC3)
		&& (inMaskImg.type() == CV_8UC1)
		&& (inCurrColorImg.size() == inInitWeightFloat.size())
		&& (inCurrColorImg.size() == inMaskImg.size())
		&& (inCurrColorImg.size() == HEIGHTMAP_RESIZEDIMG_SIZE));

	// Init the result
	//  C++: static MatExpr Mat::zeros(int rows, int cols, int type)
	outImgGradX = cv::Mat::zeros(HEIGHTMAP_GRADIMG_SIZE, CV_32FC1);
	outImgGradY = cv::Mat::zeros(HEIGHTMAP_GRADIMG_SIZE, CV_32FC1);
	cv::Mat localGradHSVMap = cv::Mat::zeros(HEIGHTMAP_GRADIMG_SIZE, CV_8UC3);

	// Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
	cv::Mat localMaskImg = inMaskImg(HEIGHTMAP_GRADIMG_RECT);
	cv::Mat localInitWeightFloat = inInitWeightFloat(HEIGHTMAP_GRADIMG_RECT);
	cv::Mat localCurrImg = inCurrColorImg(HEIGHTMAP_GRADIMG_RECT);

	cv::Mat localCurrImgFloat;
	localCurrImg.convertTo(localCurrImgFloat, CV_32FC3, 1.0, 0.0);

	const int ImgWidth = localCurrImg.cols;
	const int ImgHeight = localCurrImg.rows;

	cv::Vec3f* localCurrImgFloat_ptr;
	cv::Vec3f* localInitWeightFloat_ptr;
	uchar* localMaskImg_ptr;

	float* outImgGradX_ptr;
	float* outImgGradY_ptr;
	cv::Vec3b* localGradHSVMap_ptr;

	// hue value 
	const float localAngleRatio = 180.0f/360.0f;
    // s and v value
    const float localMagRatio = 255.0f/1.0f;

	for (int iRow = 0; iRow < ImgHeight; iRow++){

		localCurrImgFloat_ptr = localCurrImgFloat.ptr<cv::Vec3f>(iRow);
		localInitWeightFloat_ptr = localInitWeightFloat.ptr<cv::Vec3f>(iRow);
		localMaskImg_ptr = localMaskImg.ptr<uchar>(iRow);

		outImgGradX_ptr = outImgGradX.ptr<float>(iRow);
		outImgGradY_ptr = outImgGradY.ptr<float>(iRow);
		localGradHSVMap_ptr = localGradHSVMap.ptr<cv::Vec3b>(iRow);

		for (int iCol = 0; iCol < ImgWidth; iCol++){
			if((*localMaskImg_ptr) > 0){
				int localIntValue[3];
				for (int i = 0; i < 3; i++){
					// localCurrFloatImg_ptr/init is in [0.2, 1.4] to [0, 60]
					float localFloatValue = (*localCurrImgFloat_ptr)[i] * (*localInitWeightFloat_ptr)[i];
					localFloatValue = (float)((localFloatValue - 0.2) * 50.0);

					localIntValue[i] = std::floor(localFloatValue + 0.5);
					localIntValue[i] = std::min(std::max(0, localIntValue[i]), LOOKUPTAB_MAXBINNUM);
				}

				const int loca_row_no = localIntValue[0] * LOOKUPTAB_BINNUM + localIntValue[1];
				const int loca_col_no = localIntValue[2];

				// in OpenCV, it is in [b g r]
				// *outImgGradX_ptr = std::tan(LookupTabGradx[loca_row_no][loca_col_no]);
				// *outImgGradY_ptr = std::tan(LookupTabGrady[loca_row_no][loca_col_no]);
				*outImgGradX_ptr = std::tan(m_LookupTabGradMapX.at<float>(loca_row_no,loca_col_no));
				*outImgGradY_ptr = std::tan(m_LookupTabGradMapY.at<float>(loca_row_no,loca_col_no));


				const float localMag = std::sqrt((*outImgGradX_ptr) * (*outImgGradX_ptr) 
					+ (*outImgGradY_ptr) * (*outImgGradY_ptr));
                int localMagIndex = std::floor(localMag * localMagRatio);
                localMagIndex = std::min(std::max(0, localMagIndex), 255);

                const float localAngle = cv::fastAtan2((*outImgGradY_ptr), (*outImgGradX_ptr));
                int localAngleIndex = std::floor(localAngle * localAngleRatio);
                localAngleIndex = std::min(std::max(0, localAngleIndex), 255);

                (*localGradHSVMap_ptr)[0] = (uchar)(localAngleIndex);
                (*localGradHSVMap_ptr)[1] = (uchar)(localMagIndex);
                (*localGradHSVMap_ptr)[2] = (uchar)(localMagIndex);
		
			}
			localCurrImgFloat_ptr++;
			localInitWeightFloat_ptr++;
			localMaskImg_ptr++;

			outImgGradX_ptr++;
			outImgGradY_ptr++;
			localGradHSVMap_ptr++;
		}
	}

	cv::cvtColor(localGradHSVMap, outImgGradMap, CV_HSV2BGR);
}

void CGelSightHeightMap::SaveCurrHeightMapToFiles(int FrameIndex)
{
	std::ostringstream localOutStream;
	localOutStream << std::setfill('0') << std::setw(8) << FrameIndex;

	const std::string SaveImg3DGrayFilenameStr = m_SaveDataPreFileName + "_GrayMap_"  + localOutStream.str() + ".png";
	const std::string SaveImg3DColorFilenameStr = m_SaveDataPreFileName + "_ColorMap_"  + localOutStream.str() + ".png";

	// std::cout << "SaveImg3DGrayFilenameStr " << SaveImg3DGrayFilenameStr << std::endl;
	std::cout << "SaveImg3DColorFilenameStr " << SaveImg3DColorFilenameStr << std::endl;

	cv::imwrite(SaveImg3DGrayFilenameStr, m_CurrHeightGrayMap);
	cv::imwrite(SaveImg3DColorFilenameStr, m_CurrHeightColorMap);
}

void CGelSightHeightMap::GetCurrUniformImg(const cv::Mat InCurrColorImg, const int ImgUniformScale, 
	cv::Mat &OutUniformColorImg)
{
	CV_Assert((InCurrColorImg.type() == CV_8UC3) && (InCurrColorImg.size() == HEIGHTMAP_RESIZEDIMG_SIZE));

	// set the output
	OutUniformColorImg = cv::Mat::zeros(InCurrColorImg.size(), CV_8UC3);

	const int ImgWidth = InCurrColorImg.cols;
	const int ImgHeight = InCurrColorImg.rows;

	cv::Mat localCurrColorImgFloat;
	InCurrColorImg.convertTo(localCurrColorImgFloat, CV_32FC3, 1.0, 0.0);
	cv::Mat localInitColorImgWeightFloat = m_ResizedInitBluredColorImgWeightFloat.clone();

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


void CGelSightHeightMap::DetectBlackMarkerMask(const cv::Mat inCurrColorImg, const int border_size, cv::Mat &outMarkerMaskImg)
{
	CV_Assert(inCurrColorImg.type() == CV_8UC3);

	// init image
	outMarkerMaskImg = cv::Mat(inCurrColorImg.size(), CV_8UC1, (uchar)(255));
	cv::Mat localCurrColorImg = inCurrColorImg.clone();
	
	const int ImgHeight = inCurrColorImg.rows;
	const int ImgWidth = inCurrColorImg.cols;
	const int LocalImg_ROI_row_start = border_size;
	const int LocalImg_ROI_row_end = ImgHeight - border_size;
	const int LocalImg_ROI_col_start = border_size;
	const int LocalImg_ROI_col_end = ImgWidth - border_size;

	//=================================================================================================================================
	// find the black part
	cv::Vec3b* localCurrColorImg_ptr;
	uchar* outMarkerMaskImg_ptr;
	for (int iRow = LocalImg_ROI_row_start; iRow < LocalImg_ROI_row_end; iRow++){

		localCurrColorImg_ptr = localCurrColorImg.ptr<cv::Vec3b>(iRow);
		outMarkerMaskImg_ptr = outMarkerMaskImg.ptr<uchar>(iRow);
		for (int iCol = 0; iCol < LocalImg_ROI_col_end; iCol++){
			if(iCol >= LocalImg_ROI_col_start){
				uchar LocalValue[3];
				LocalValue[0] = (*localCurrColorImg_ptr)[0];
				LocalValue[1] = (*localCurrColorImg_ptr)[1];
				LocalValue[2] = (*localCurrColorImg_ptr)[2];

				const uchar LocalMaxValue = std::max(LocalValue[0], std::max(LocalValue[1], LocalValue[2]));
				const uchar LocalMinValue = std::min(LocalValue[0], std::min(LocalValue[1], LocalValue[2]));
				
				if ((LocalMinValue < HEIGHTMAP_MARKERTHRES_LOW) && (LocalMaxValue < HEIGHTMAP_MARKERTHRES_HIGH)){
					(*outMarkerMaskImg_ptr) = (uchar)(0);
				}
			}
			localCurrColorImg_ptr++;
			outMarkerMaskImg_ptr++;
		}
	}
}

void CGelSightHeightMap::DetectContactMapMask(const cv::Mat inCurrColorImg, const cv::Mat inInitBluredColorImg, 
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
	const int LocalImg_ROI_row_start = border_size;
	const int LocalImg_ROI_row_end = ImgHeight - border_size;
	const int LocalImg_ROI_col_start = border_size;
	const int LocalImg_ROI_col_end = ImgWidth - border_size;

	//=================================================================================================================================
	// find the contact part
	cv::Vec3b* localCurrColorImg_ptr;
	cv::Vec3b* localInitBluredColorImg_ptr;
	uchar* outContactMapMaskImg_ptr;

	for (int iRow = LocalImg_ROI_row_start; iRow < LocalImg_ROI_row_end; iRow++){

		localCurrColorImg_ptr = localCurrColorImg.ptr<cv::Vec3b>(iRow);
		localInitBluredColorImg_ptr = localInitBluredColorImg.ptr<cv::Vec3b>(iRow);
		outContactMapMaskImg_ptr = outContactMapMaskImg.ptr<uchar>(iRow);
		for (int iCol = 0; iCol < LocalImg_ROI_col_end; iCol++){
			if(iCol >= LocalImg_ROI_col_start){
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
				const uchar LocalMaxHighValue = std::max(localHighValue[0], std::max(localHighValue[1], localHighValue[2]));
				const uchar LocalMinLowValue = std::max(localLowValue[0], std::max(localLowValue[1], localLowValue[2]));
				
				if ((LocalMaxHighValue >= HEIGHTMAP_CONTACTTHRES_HIGH) && (LocalMinLowValue >= HEIGHTMAP_CONTACTTHRES_LOW)){
					(*outContactMapMaskImg_ptr) = (uchar)(255);
				}
			}

			localCurrColorImg_ptr++;
			localInitBluredColorImg_ptr++;
			outContactMapMaskImg_ptr++;
		}
	}
}

void CGelSightHeightMap::DetectContactMapMaskWithoutBlackMarker(const cv::Mat inCurrColorImg, const cv::Mat inInitBluredColorImg, 
	const int border_size, cv::Mat &outMaskImg)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inInitBluredColorImg.type() == CV_8UC3)
		&& (inCurrColorImg.size() == inInitBluredColorImg.size()));

	// init image
	outMaskImg = cv::Mat::zeros(inCurrColorImg.size(), CV_8UC1);
	cv::Mat localCurrColorImg = inCurrColorImg.clone();
	cv::Mat localInitBluredColorImg = inInitBluredColorImg.clone();
	
	const int ImgHeight = inCurrColorImg.rows;
	const int ImgWidth = inCurrColorImg.cols;
	const int LocalImg_ROI_row_start = border_size;
	const int LocalImg_ROI_row_end = ImgHeight - border_size;
	const int LocalImg_ROI_col_start = border_size;
	const int LocalImg_ROI_col_end = ImgWidth - border_size;

	//=================================================================================================================================
	// find the contact part
	cv::Vec3b* localCurrColorImg_ptr;
	cv::Vec3b* localInitBluredColorImg_ptr;
	uchar* outMaskImg_ptr;

	for (int iRow = LocalImg_ROI_row_start; iRow < LocalImg_ROI_row_end; iRow++){

		localCurrColorImg_ptr = localCurrColorImg.ptr<cv::Vec3b>(iRow);
		localInitBluredColorImg_ptr = localInitBluredColorImg.ptr<cv::Vec3b>(iRow);
		outMaskImg_ptr = outMaskImg.ptr<uchar>(iRow);
		for (int iCol = 0; iCol < LocalImg_ROI_col_end; iCol++){
			if(iCol >= LocalImg_ROI_col_start){
				uchar LocalValue[3];
				LocalValue[0] = (*localCurrColorImg_ptr)[0];
				LocalValue[1] = (*localCurrColorImg_ptr)[1];
				LocalValue[2] = (*localCurrColorImg_ptr)[2];

				const uchar LocalMaxValue = std::max(LocalValue[0], std::max(LocalValue[1], LocalValue[2]));
				const uchar LocalMinValue = std::min(LocalValue[0], std::min(LocalValue[1], LocalValue[2]));

				if ((LocalMinValue < HEIGHTMAP_MARKERTHRES_LOW) && (LocalMaxValue < HEIGHTMAP_MARKERTHRES_HIGH)){
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
					const uchar LocalMaxHighValue = std::max(localHighValue[0], std::max(localHighValue[1], localHighValue[2]));
					const uchar LocalMinLowValue = std::max(localLowValue[0], std::max(localLowValue[1], localLowValue[2]));

					if ((LocalMaxHighValue >= HEIGHTMAP_CONTACTTHRES_HIGH) && (LocalMinLowValue >= HEIGHTMAP_CONTACTTHRES_LOW)){
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