#include "CGelSightMarkerMotion.h"


CGelSightMarkerMotion::CGelSightMarkerMotion()
{
#ifdef MARKERMOTION_DEBUG_MODE	
	cv::namedWindow("LocalMaskImg", 0);
	cv::namedWindow("DebugOutFlowImg",0);
	cv::namedWindow("MarkerMotionContactMap",0);

#endif
	cv::namedWindow("MarkerMotionFlow",0);
	
	m_InitMarkerNum = 0;
	m_CurrMarkerNum = 0;
}

CGelSightMarkerMotion::~CGelSightMarkerMotion()
{

}

void CGelSightMarkerMotion::DispColorMap()
{
	const float local_radius = (float)(0.5 * MARKERMOTION_RAWIMG_SIZE.height);

	// hue value 
	const float localAngleRatio = (float)(180.0)/(float)(360.0);
    // s and v value
    const float localMagRatio = (float)(255.0)/local_radius;

    cv::Mat localHSV_Img = cv::Mat::zeros(MARKERMOTION_RAWIMG_SIZE, CV_8UC3);

   	const float localCenterX = (float)(0.5 * MARKERMOTION_RAWIMG_SIZE.width);
   	const float localCenterY = (float)(0.5 * MARKERMOTION_RAWIMG_SIZE.height);
   	const cv::Point2f localCenter = cv::Point2f(localCenterX, localCenterY);

    for (int iRow = 0; iRow < MARKERMOTION_RAWIMG_SIZE.height; iRow++)
    {
        for (int iCol = 0; iCol < MARKERMOTION_RAWIMG_SIZE.width; iCol++)
        {
        	// Point_(_Tp _x, _Tp _y);
            const cv::Point2f localPt = cv::Point2f(iCol, iRow) - localCenter;
            const float localMag = std::sqrt(localPt.x * localPt.x + localPt.y * localPt.y);

            if( localMag < local_radius)
            {
                int localMagIndex = std::floor(localMag * localMagRatio);
                localMagIndex = std::min(std::max(0, localMagIndex), 255);

                const float localAngle = cv::fastAtan2(localPt.y, localPt.x);
                int localAngleIndex = std::floor(localAngle * localAngleRatio);
                localAngleIndex = std::min(std::max(0, localAngleIndex), 255);

                localHSV_Img.at<Vec3b>(iRow, iCol)[0] = (uchar)(localAngleIndex);
                localHSV_Img.at<Vec3b>(iRow, iCol)[1] = (uchar)(localMagIndex);
                localHSV_Img.at<Vec3b>(iRow, iCol)[2] = (uchar)(localMagIndex);
            }
        }
    }
    cv::Mat localBGR_Img;
    cv::cvtColor(localHSV_Img, localBGR_Img, CV_HSV2BGR);
    // cv::imshow("MarkerMotion HSV", localHSV_Img);
    cv::imshow("MarkerMotion BGR", localBGR_Img);
}

void CGelSightMarkerMotion::SetFileNameAndInitBluredImg(const std::string InSaveDataPreFileName, const cv::Mat inInitBluredColorImg)
{
	CV_Assert((inInitBluredColorImg.type() == CV_8UC3) && (inInitBluredColorImg.size() == MARKERMOTION_RAWIMG_SIZE));

	m_SaveDataPreFileName = InSaveDataPreFileName;
	cv::resize(inInitBluredColorImg, m_InitResizedBluredColorImg, MARKERMOTION_RESIZEDIMG_SIZE, 0, 0, INTER_CUBIC);
}


void CGelSightMarkerMotion::SetMarkerMotionInitFrame(const cv::Mat inCurrColorImg)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inCurrColorImg.size() == MARKERMOTION_RAWIMG_SIZE));

	cv::resize(inCurrColorImg, m_CurrResizedColorImg, MARKERMOTION_RESIZEDIMG_SIZE, 0, 0, INTER_CUBIC);

	cv::Mat localMarkerMaskImg;
	DetectBlackMarkerPosition(m_CurrResizedColorImg, true, localMarkerMaskImg);
	// cv::imshow("MarkerMotionMaskImg", localMarkerMaskImg);
	
	cv::Mat localMarkerMotionImg;
	ShowMarkerMotion(m_CurrResizedColorImg, true, localMarkerMotionImg);
	cv::imshow("MarkerMotionFlow", localMarkerMotionImg);

}

void CGelSightMarkerMotion::GetCurrMotionFlow(const cv::Mat inCurrColorImg)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inCurrColorImg.size() == MARKERMOTION_RAWIMG_SIZE));
	cv::resize(inCurrColorImg, m_CurrResizedColorImg, MARKERMOTION_RESIZEDIMG_SIZE, 0, 0, INTER_CUBIC);

	//=====================================================================================
	cv::Mat localMarkerMaskImg;
	DetectBlackMarkerPosition(m_CurrResizedColorImg, false, localMarkerMaskImg);
	// cv::imshow("MarkerMotionMaskImg", localMarkerMaskImg);

	DetectBlackMarkerMotionByTracking(false);

	cv::Mat localMarkerMotionImg;
	ShowMarkerMotion(m_CurrResizedColorImg, false, localMarkerMotionImg);
	cv::imshow("MarkerMotionFlow", localMarkerMotionImg);

	// DetectMarkerMotionState(m_CurrResizedColorImg);

	// cv::cvtColor(m_CurrResizedColorImg, m_CurrResizedGrayImg, CV_BGR2GRAY);
	// cv::blur(localGrayImg, m_CurrResizedGrayImg, Size(3, 3));

}

void CGelSightMarkerMotion::DetectBlackMarkerPosition(const cv::Mat inRawColorImg, const bool bInitImg, cv::Mat &outMarkerMaskImg)
{
	CV_Assert((inRawColorImg.type() == CV_8UC3) && (inRawColorImg.size() == MARKERMOTION_RESIZEDIMG_SIZE));

	// init image
	const int ImgHeight = inRawColorImg.rows;
	const int ImgWidth = inRawColorImg.cols;
	const int LocalImg_ROI_row_start = MARKERMOTION_RESIZEDBORDER_SIZE;
	const int LocalImg_ROI_row_end = ImgHeight - MARKERMOTION_RESIZEDBORDER_SIZE;
	const int LocalImg_ROI_col_start = MARKERMOTION_RESIZEDBORDER_SIZE;
	const int LocalImg_ROI_col_end = ImgWidth - MARKERMOTION_RESIZEDBORDER_SIZE;

	cv::Mat localRawColorImg = inRawColorImg.clone();
	//=================================================================================================================================
	int local_maker_pos[ImgHeight][MARKERMOTION_MARKERNUM_INCOL][4];
	int local_maker_num_inrow[ImgHeight];

	outMarkerMaskImg = cv::Mat::zeros(cv::Size(ImgWidth, ImgHeight), CV_8UC1);
	uchar* outMarkerMaskImg_ptr;

	//=================================================================================================================================
	// find the black part
	cv::Vec3b* localRawColorImg_ptr;
	for (int iRow = 0; iRow < ImgHeight; iRow++)
	{
		local_maker_num_inrow[iRow] = 0;
		localRawColorImg_ptr = localRawColorImg.ptr<cv::Vec3b>(iRow);
		outMarkerMaskImg_ptr = outMarkerMaskImg.ptr<uchar>(iRow);

		int local_index = 0;
		int local_length = 0;
		int local_pos_sum = 0;
		bool local_premarker_valid = false;
		bool local_currmarker_valid = false;

		for (int iCol = 0; iCol < ImgWidth; iCol++)
		{
			uchar LocalValue[3];
			LocalValue[0] = (*localRawColorImg_ptr)[0];
			LocalValue[1] = (*localRawColorImg_ptr)[1];
			LocalValue[2] = (*localRawColorImg_ptr)[2];

			uchar LocalMaxValue = std::max(LocalValue[0], std::max(LocalValue[1], LocalValue[2]));
			uchar LocalMinValue = std::min(LocalValue[0], std::min(LocalValue[1], LocalValue[2]));
			
			if ((LocalMinValue < MARKERMOTION_MARKERTHRES_LOW) && (LocalMaxValue < MARKERMOTION_MARKERTHRES_HIGH))
			{
				(*outMarkerMaskImg_ptr) = (uchar)(255);

				local_length++;
				local_pos_sum += iCol;
				local_currmarker_valid = true;
			}
			else
			{
				local_currmarker_valid = false;
			}

			if(iCol == (ImgWidth - 1))
			{
				local_currmarker_valid = false;
			}

			if((local_premarker_valid) && (local_currmarker_valid == false))
			{
				// add the valid length
				if(local_length > MARKERMOTION_PIXEL_MINNUM_INCOL)
				{
					local_maker_pos[iRow][local_index][0] = iRow;
					local_maker_pos[iRow][local_index][1] = iCol - (local_length/2);
					local_maker_pos[iRow][local_index][2] = local_length;
					local_maker_pos[iRow][local_index][3] = local_pos_sum;

#ifdef MARKERMOTION_DEBUG_MODE
					std::cout << local_maker_pos[iRow][local_index][0] << "\t" 
						<< local_maker_pos[iRow][local_index][1] << "\t"
						<< local_maker_pos[iRow][local_index][2] << "\t"
						<< local_maker_pos[iRow][local_index][3] << "\t";
#endif
					local_index++;
				}
				local_length = 0;
				local_pos_sum = 0;
			}

			local_premarker_valid = local_currmarker_valid;
			localRawColorImg_ptr++;
			outMarkerMaskImg_ptr++;
		}

		// fill other
		for(int local_i = local_index; local_i < MARKERMOTION_MARKERNUM_INCOL; local_i++)
		{
			local_maker_pos[iRow][local_index][0] = iRow;
			local_maker_pos[iRow][local_index][1] = 0;
			local_maker_pos[iRow][local_index][2] = 0;
			local_maker_pos[iRow][local_index][3] = 0;
		}
		// valid continous black pixel in col
		local_maker_num_inrow[iRow] = local_index;

#ifdef MARKERMOTION_DEBUG_MODE
		std::cout << std::endl << std::endl;
#endif
	}

	int local_maker_index = 0;
	int local_sum_list_x[MARKERMOTION_MARKERNUM];
	int local_sum_list_y[MARKERMOTION_MARKERNUM];
	int local_sum_list_num[MARKERMOTION_MARKERNUM];
	float local_MarkerCenter[MARKERMOTION_MARKERNUM][2];
	int local_MarkerSize[MARKERMOTION_MARKERNUM];

	for (int iRow = 0; iRow < ImgHeight; iRow++)
	{
		// std::cout << "iRow = " << iRow << std::endl;
		for(int local_i = 0; local_i < local_maker_num_inrow[iRow]; local_i++)
		{
			if(local_maker_pos[iRow][local_i][2] == 0)
			{
				continue;
			}
			int local_radius1 = (local_maker_pos[iRow][local_i][2] + 3)/2;
			int local_sum_x = local_maker_pos[iRow][local_i][3];
			int local_sum_y = local_maker_pos[iRow][local_i][2] * iRow;
			int local_sum_num = local_maker_pos[iRow][local_i][2];
			//===================================================================================
			// find next match in height
			for(int local_inner_row = (iRow + 1); local_inner_row < ImgHeight; local_inner_row++)
			{
				bool local_matched = false;
				for(int local_inner_i = 0; local_inner_i < local_maker_num_inrow[local_inner_row]; local_inner_i++)
				{
					int local_pos_diff_x = std::abs(local_maker_pos[iRow][local_i][1] - local_maker_pos[local_inner_row][local_inner_i][1]);
					int local_radius2 = (local_maker_pos[local_inner_row][local_inner_i][2] + 3)/2;
					if((local_pos_diff_x < local_radius1) && (local_pos_diff_x < local_radius2) 
						&& (local_maker_pos[local_inner_row][local_inner_i][2] > 0))
					{
						local_matched = true;

						local_sum_x += local_maker_pos[local_inner_row][local_inner_i][3];
						local_sum_y += local_maker_pos[local_inner_row][local_inner_i][2] * local_inner_row;
						local_sum_num += local_maker_pos[local_inner_row][local_inner_i][2];

						// set the candidate to be invalid
						local_maker_pos[local_inner_row][local_inner_i][2] = 0;
						local_maker_pos[local_inner_row][local_inner_i][3] = 0;
						break;
					}
				}
				if(local_matched == false)
				{
					break;
				}
			}

			//===================================================================================
			int loca_posx_int = local_sum_x/local_sum_num;
			int loca_posy_int = local_sum_y/local_sum_num;
			if( (loca_posx_int>LocalImg_ROI_col_start) && (loca_posx_int<LocalImg_ROI_col_end) &&
				(loca_posy_int>LocalImg_ROI_row_start) && (loca_posy_int<LocalImg_ROI_row_end))
			{
				//===================================================================================
				// remove close points
				bool local_near_pt = false;
				int local_near_pt_index = 0;
				for (int local_pt_index = 0; local_pt_index < local_maker_index; local_pt_index++)
				{
					int loca_set_posx = local_sum_list_x[local_pt_index]/local_sum_list_num[local_pt_index];
					int loca_set_posy = local_sum_list_y[local_pt_index]/local_sum_list_num[local_pt_index];

					int local_dist_x = std::abs(loca_set_posx - loca_posx_int);
					int local_dist_y = std::abs(loca_set_posy - loca_posy_int);
					if((local_dist_x < MARKERMOTION_MARKER_MINDIST) && (local_dist_y < MARKERMOTION_MARKER_MINDIST))
					{
						local_near_pt = true;
						local_near_pt_index = local_pt_index;
						break;
					}
				}

				// merge the near points
				if(local_near_pt)
				{
					local_sum_x += local_sum_list_x[local_near_pt_index];
					local_sum_y += local_sum_list_y[local_near_pt_index];
					local_sum_num += local_sum_list_num[local_near_pt_index];

					// add marker pos
					local_MarkerCenter[local_near_pt_index][0] = (float)(local_sum_x)/(float)local_sum_num;
					local_MarkerCenter[local_near_pt_index][1] = (float)(local_sum_y)/(float)local_sum_num;
					local_MarkerSize[local_near_pt_index] = local_sum_num;

					local_sum_list_x[local_near_pt_index] = local_sum_x;
					local_sum_list_y[local_near_pt_index] = local_sum_y;
					local_sum_list_num[local_near_pt_index] = local_sum_num;
				}
				else
				{
					// add marker pos
					local_MarkerCenter[local_maker_index][0] = (float)(local_sum_x)/(float)local_sum_num;
					local_MarkerCenter[local_maker_index][1] = (float)(local_sum_y)/(float)local_sum_num;
					local_MarkerSize[local_maker_index] = local_sum_num;

					local_sum_list_x[local_maker_index] = local_sum_x;
					local_sum_list_y[local_maker_index] = local_sum_y;
					local_sum_list_num[local_maker_index] = local_sum_num;

					local_maker_index++;
				}
			}
		}

	}
	// std::cout << "local_maker_index = " << local_maker_index << std::endl;

	if(bInitImg)
	{	
		m_InitMarkerNum = local_maker_index;
		for (int local_i = 0; local_i < m_InitMarkerNum; local_i++)
		{
			m_InitMarkerCenter[local_i][0] = local_MarkerCenter[local_i][0];
			m_InitMarkerCenter[local_i][1] = local_MarkerCenter[local_i][1];
			m_InitMarkerCenterNext[local_i][0] = local_MarkerCenter[local_i][0];
			m_InitMarkerCenterNext[local_i][1] = local_MarkerCenter[local_i][1];
			m_InitMarkerCenterLatest[local_i][0] = local_MarkerCenter[local_i][0];
			m_InitMarkerCenterLatest[local_i][1] = local_MarkerCenter[local_i][1];
			m_InitMarkerSize[local_i] = local_MarkerSize[local_i];
		}
	}
	else
	{
		// set curr marker position
		m_CurrMarkerNum = local_maker_index;
		for (int local_i = 0; local_i < m_CurrMarkerNum; local_i++)
		{
			m_CurrMarkerCenter[local_i][0] = local_MarkerCenter[local_i][0];
			m_CurrMarkerCenter[local_i][1] = local_MarkerCenter[local_i][1];
			m_CurrMarkerSize[local_i] = local_MarkerSize[local_i];
		}
	}
}

void CGelSightMarkerMotion::DetectBlackMarkerMotionWithOpticalFlow(const cv::Mat inMotionFlow, const bool bInitImg)
{
	CV_Assert((inMotionFlow.type() == CV_32FC2) && (inMotionFlow.size() == MARKERMOTION_RESIZEDIMG_SIZE));

	if(bInitImg){
		for (int local_i = 0; local_i < m_InitMarkerNum; local_i++){
			int local_x = (int)(m_InitMarkerCenter[local_i][0]);
			int local_y = (int)(m_InitMarkerCenter[local_i][1]);
			cv::Point2f local_flow = inMotionFlow.at<cv::Point2f>(local_y, local_x);

			m_InitMarkerCenter[local_i][2] = m_InitMarkerCenter[local_i][0] + local_flow.x;
			m_InitMarkerCenter[local_i][3] = m_InitMarkerCenter[local_i][1] + local_flow.y;
		}
	}else{
		for (int local_i = 0; local_i < m_CurrMarkerNum; local_i++){
			int local_x = (int)(m_CurrMarkerCenter[local_i][0]);
			int local_y = (int)(m_CurrMarkerCenter[local_i][1]);

			cv::Point2f local_flow = inMotionFlow.at<cv::Point2f>(local_y, local_x);
			// std::cout << "pos = " << cv::Point2i(local_x, local_y) << std::endl;
			// std::cout << "local_flow = " << local_flow << std::endl;

			m_CurrMarkerCenter[local_i][2] = m_CurrMarkerCenter[local_i][0] + local_flow.x;
			m_CurrMarkerCenter[local_i][3] = m_CurrMarkerCenter[local_i][1] + local_flow.y;
		}
	}
}

void CGelSightMarkerMotion::DetectBlackMarkerMotionByTracking(const bool bInitImg)
{
	if(bInitImg){
		for (int local_i = 0; local_i < m_InitMarkerNum; local_i++){
			m_InitMarkerCenterNext[local_i][0] = m_InitMarkerCenter[local_i][0];
			m_InitMarkerCenterNext[local_i][1] = m_InitMarkerCenter[local_i][1];

			m_InitMarkerCenterLatest[local_i][0] = m_InitMarkerCenter[local_i][0];
			m_InitMarkerCenterLatest[local_i][1] = m_InitMarkerCenter[local_i][1];
		}
	}else{
		int localCurrMatchedIndex[MARKERMOTION_MARKERNUM];
		for (int local_i = 0; local_i < m_CurrMarkerNum; local_i++){
			float localMin = 100000.0f;
			localCurrMatchedIndex[local_i] = -1;
			for (int local_j = 0; local_j < m_InitMarkerNum; local_j++){
				const float localDist0 = std::abs(m_CurrMarkerCenter[local_i][0] - m_InitMarkerCenterNext[local_j][0]);
				const float localDist1 = std::abs(m_CurrMarkerCenter[local_i][1] - m_InitMarkerCenterNext[local_j][1]);
				const float localDist2 = (float)(m_CurrMarkerSize[local_i] - m_InitMarkerSize[local_j]);
				const float localDist = (localDist0 + localDist1) * (std::abs(localDist2) + 100.0f);
				if(localDist < localMin){
					localMin = localDist;
					localCurrMatchedIndex[local_i] = local_j;
				}
			}
		}

		for (int local_i = 0; local_i < m_InitMarkerNum; local_i++){
			float localMin = 100000.0f;
			int localMinPos = -1;
			for (int local_j = 0; local_j < m_CurrMarkerNum; local_j++){
				const float localDist0 = std::abs(m_InitMarkerCenterNext[local_i][0] - m_CurrMarkerCenter[local_j][0]);
				const float localDist1 = std::abs(m_InitMarkerCenterNext[local_i][1] - m_CurrMarkerCenter[local_j][1]);
				const float localDist2 = (float)(m_InitMarkerSize[local_i] - m_CurrMarkerSize[local_j]);
				const float localDist = (localDist0 + localDist1) * (std::abs(localDist2) + 100.0f);
				if(localDist < localMin){
					localMin = localDist;
					localMinPos = local_j;
				}
			}
			if((float)(m_InitMarkerSize[local_i]) < (0.01f * localMin)){
				m_InitMarkerCenterLatest[local_i][0] = m_InitMarkerCenter[local_i][0];
				m_InitMarkerCenterLatest[local_i][1] = m_InitMarkerCenter[local_i][1];
			}else if((localMinPos >= 0) && (localMinPos < m_CurrMarkerNum) 
				&& (localCurrMatchedIndex[localMinPos] == local_i)){
				m_InitMarkerCenterLatest[local_i][0] = m_CurrMarkerCenter[localMinPos][0];
				m_InitMarkerCenterLatest[local_i][1] = m_CurrMarkerCenter[localMinPos][1];
			}else{
				m_InitMarkerCenterLatest[local_i][0] = m_InitMarkerCenter[local_i][0];
				m_InitMarkerCenterLatest[local_i][1] = m_InitMarkerCenter[local_i][1];
			}
		}

		for (int local_i = 0; local_i < m_InitMarkerNum; local_i++){
			m_InitMarkerCenterNext[local_i][0] = m_InitMarkerCenterLatest[local_i][0];
			m_InitMarkerCenterNext[local_i][1] = m_InitMarkerCenterLatest[local_i][1];
		}
	}
}

void CGelSightMarkerMotion::ShowMarkerMotion(const cv::Mat inRawColorImg, const bool bInitImg, cv::Mat &outMarkerMotionImg)
{
	CV_Assert((inRawColorImg.type() == CV_8UC3) && (inRawColorImg.size() == MARKERMOTION_RESIZEDIMG_SIZE));

	// show image
	outMarkerMotionImg = inRawColorImg.clone();

	if(bInitImg){	
		for (int local_i = 0; local_i < m_InitMarkerNum; local_i++){
			cv::circle(outMarkerMotionImg, cv::Point2f(m_InitMarkerCenter[local_i][0], m_InitMarkerCenter[local_i][1]), 
				3, cv::Scalar(255, 0, 0), -1, 8, 0);
		}

		const int LocalFontFace = cv::FONT_HERSHEY_SIMPLEX;
		const double LocalFontScale = 1.0;
	    const int LocalTextThickness = 2;
		std::string LocalShowText = "Press F or f to set";
    	const cv::Point LocalTextOrg1(10, 50);
		cv::putText(outMarkerMotionImg, LocalShowText, LocalTextOrg1, LocalFontFace, LocalFontScale, 
			cv::Scalar(0, 0, 255), LocalTextThickness, 8);
	}else{
		for (int local_i = 0; local_i < m_InitMarkerNum; local_i++){
			cv::circle(outMarkerMotionImg, cv::Point2f(m_InitMarkerCenter[local_i][0], m_InitMarkerCenter[local_i][1]), 
				3, cv::Scalar(255, 0, 0), -1, 8, 0);

			cv::circle(outMarkerMotionImg, cv::Point2f(m_InitMarkerCenterLatest[local_i][0], m_InitMarkerCenterLatest[local_i][1]), 
				3, cv::Scalar(0, 0, 255), -1, 8, 0);

			const cv::Point2f local_currpt = cv::Point2f(m_InitMarkerCenter[local_i][0], m_InitMarkerCenter[local_i][1]);
			cv::Point2f local_nextpt = cv::Point2f(m_InitMarkerCenterLatest[local_i][0], m_InitMarkerCenterLatest[local_i][1]);
			local_nextpt = local_currpt + MARKERMOTION_IMGPLOT_RATIO * (local_nextpt - local_currpt);

			cv::line(outMarkerMotionImg, local_currpt, local_nextpt, cv::Scalar(0, 255, 0), 2, 8, 0);
		}
	}
}

void CGelSightMarkerMotion::DetectContactMapMask(const cv::Mat inCurrColorImg, const cv::Mat inInitBluredColorImg, 
	cv::Mat &outContactMapMaskImg)
{
	CV_Assert((inCurrColorImg.type() == CV_8UC3) && (inInitBluredColorImg.type() == CV_8UC3)
		&& (inCurrColorImg.size() == MARKERMOTION_RESIZEDIMG_SIZE) && (inCurrColorImg.size() == inInitBluredColorImg.size()));

	// init image
	const int ImgHeight = inCurrColorImg.rows;
	const int ImgWidth = inCurrColorImg.cols;
	const int LocalImg_ROI_row_start = MARKERMOTION_RESIZEDBORDER_SIZE;
	const int LocalImg_ROI_row_end = ImgHeight - MARKERMOTION_RESIZEDBORDER_SIZE;
	const int LocalImg_ROI_col_start = MARKERMOTION_RESIZEDBORDER_SIZE;
	const int LocalImg_ROI_col_end = ImgWidth - MARKERMOTION_RESIZEDBORDER_SIZE;

	cv::Mat localCurrColorImg = inCurrColorImg.clone();
	cv::Mat localInitBluredColorImg = inInitBluredColorImg.clone();
	outContactMapMaskImg = cv::Mat::zeros(cv::Size(ImgWidth, ImgHeight), CV_8UC1);
	
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
				
				if ((LocalMaxHighValue >= MARKERMOTION_CONTACTTHRES_HIGH) 
					&& (LocalMinLowValue >= MARKERMOTION_CONTACTTHRES_LOW)){
					(*outContactMapMaskImg_ptr) = (uchar)(255);
				}
			}

			localCurrColorImg_ptr++;
			localInitBluredColorImg_ptr++;
			outContactMapMaskImg_ptr++;
		}
	}
}