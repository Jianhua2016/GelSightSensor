#include "CGelSightGrabImg.h"

CGelSightGrabImg::CGelSightGrabImg()
{
	m_ConstSaveDataPreFileName = "";
    m_ConstSaveDataFolderName = "";
    m_bInitImgIsCaptured = false;
    m_bSaveImg = false;
    m_CurrSaveImgCount = 0;
    m_CurrShowImgCount = 0;
    m_GrabFailTime = 0;
	iCameraExpo = 2;
    cv::namedWindow("CurrShowImg", 1);
	cv::createTrackbar("CameraExposure", "CurrShowImg", &iCameraExpo, 8);
}


CGelSightGrabImg::~CGelSightGrabImg()
{

}

int CGelSightGrabImg::ConnectCamera(const int InCameraID)
{
	if (!m_CameraCap.open(InCameraID)){
		std::cerr << "Fail to connect the camera with ID of " << InCameraID << std::endl;
		return -1;
	}else{
		std::cout << "Successful to connect the camera with ID of " << InCameraID << std::endl;
	}

	m_CameraCap.set(CV_CAP_PROP_FRAME_WIDTH, RAWIMG_SET_SIZE.width);
	m_CameraCap.set(CV_CAP_PROP_FRAME_HEIGHT, RAWIMG_SET_SIZE.height);
	// m_CameraCap.set(CV_CAP_PROP_AUTO_EXPOSURE, 0);

	std::cout << "CV_CAP_PROP_FRAME_WIDTH " << m_CameraCap.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
	std::cout << "CV_CAP_PROP_FRAME_HEIGHT " << m_CameraCap.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << "CV_CAP_PROP_AUTO_EXPOSURE " << m_CameraCap.get(CV_CAP_PROP_AUTO_EXPOSURE) << std::endl;

	std::cout << "CV_CAP_PROP_BRIGHTNESS " << m_CameraCap.get(CV_CAP_PROP_BRIGHTNESS) << std::endl;
	std::cout << "CV_CAP_PROP_CONTRAST " << m_CameraCap.get(CV_CAP_PROP_CONTRAST) << std::endl;
	std::cout << "CV_CAP_PROP_SATURATION " << m_CameraCap.get(CV_CAP_PROP_SATURATION) << std::endl;

	std::cout << "CV_CAP_PROP_HUE " << m_CameraCap.get(CV_CAP_PROP_HUE) << std::endl;
	std::cout << "CV_CAP_PROP_GAIN " << m_CameraCap.get(CV_CAP_PROP_GAIN) << std::endl;
	std::cout << "CV_CAP_PROP_EXPOSURE " << m_CameraCap.get(CV_CAP_PROP_EXPOSURE) << std::endl;

	return 1;
}

//set camera exposure
void CGelSightGrabImg::SetCameraExposure(const int CurrCameraExposure)
{
    double TempCameraCurrExposure = -1.0 * m_CameraCap.get(CV_CAP_PROP_EXPOSURE);
    if ((int)(TempCameraCurrExposure) != CurrCameraExposure){
        double TempCurrCameraExposure = (double)(-1.0 * CurrCameraExposure);
        m_CameraCap.set(CV_CAP_PROP_EXPOSURE, TempCurrCameraExposure);

        std::cout << "Current CV_CAP_PROP_EXPOSURE " << m_CameraCap.get(CV_CAP_PROP_EXPOSURE) << std::endl;
    }
}

//get camera current frame
int CGelSightGrabImg::CameraCaptureCurrFrame()
{
    std::string LocalDispText;
    std::string LocalShowText;
    const cv::Point LocalTextOrg1(10, 50);
    const cv::Point LocalTextOrg2(10, 100);

    const int LocalFontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double LocalFontScale = 1.0;
    const int LocalTextThickness = 2;

	// capture reference frame
	m_CameraCap >> m_CurrCapColorImg;
	if (m_CurrCapColorImg.empty()){
        m_GrabFailTime++;
		std::cerr << "Fail to get current frame" << std::endl;

        if(m_GrabFailTime > 500){
            std::exit(EXIT_FAILURE);
        }else{
            return -1;
        }
	}else{
        m_GrabFailTime--;
        m_GrabFailTime = std::max(m_GrabFailTime, 0);
        if(m_bInitImgIsCaptured){
            std::ostringstream LocalOutImgNo;
            if(m_bSaveImg){
                m_CurrSaveImgCount++;
                LocalOutImgNo << std::setfill('0') << std::setw(8) << m_CurrSaveImgCount;

                const std::string local_filename = m_ConstSaveDataPreFileName + "_img_" + LocalOutImgNo.str() + ".png";
                cv::imwrite(local_filename, m_CurrCapColorImg);

                m_CurrShowImage = m_CurrCapColorImg.clone();
                LocalShowText = "Save Image " + LocalOutImgNo.str();
                cv::putText(m_CurrShowImage, LocalShowText, LocalTextOrg1, LocalFontFace,
                    LocalFontScale, cv::Scalar(0, 0, 255), LocalTextThickness, 8);

                LocalDispText = "Press N or n to stop saving image";
                cv::putText(m_CurrShowImage, LocalDispText, LocalTextOrg2, LocalFontFace,
                    LocalFontScale, cv::Scalar(0, 0, 255), LocalTextThickness, 8);
            }else{
                m_CurrShowImgCount++;
                LocalOutImgNo << "Show Image " << std::setfill('0') << std::setw(8) << m_CurrShowImgCount;
                LocalShowText = LocalOutImgNo.str();

                m_CurrShowImage = m_CurrCapColorImg.clone();
                cv::putText(m_CurrShowImage, LocalShowText, LocalTextOrg1, LocalFontFace,
                    LocalFontScale, cv::Scalar(0, 0, 255), LocalTextThickness, 8);

                LocalDispText = "Press S or s to save image";
                cv::putText(m_CurrShowImage, LocalDispText, LocalTextOrg2, LocalFontFace,
                    LocalFontScale, cv::Scalar(0, 0, 255), LocalTextThickness, 8);
            }

            cv::imshow("CurrShowImg", m_CurrShowImage);
            //===========================================================================================
            int CurrKey = cv::waitKey(3);
            if ((CurrKey == 'S') || (CurrKey == 's')){
                m_bSaveImg = true;
            }else if ((CurrKey == 'N') || (CurrKey == 'n')){
                m_bSaveImg = false;
            }
        }else{
            m_CurrShowImage = m_CurrCapColorImg.clone();
            LocalShowText = "Press I or i to capture initial image";
            cv::putText(m_CurrShowImage, LocalShowText, LocalTextOrg1, LocalFontFace,
                LocalFontScale, cv::Scalar(0, 0, 255), LocalTextThickness, 8);

            cv::imshow("CurrShowImg", m_CurrShowImage);
			SetCameraExposure(iCameraExpo);
            //===========================================================================================
            int CurrKey = cv::waitKey(1);
            if ((CurrKey == 'I') || (CurrKey == 'i')){
                m_bInitImgIsCaptured = true;
            }
        }
		return 1;
	}
}

//get camera current frame
void CGelSightGrabImg::GetCameraCurrFrame(cv::Mat &outCurrImg, bool &bSaveData)
{
    outCurrImg = m_CurrCapColorImg.clone();
    bSaveData = m_bSaveImg;
}

//save camera initial frame
void CGelSightGrabImg::SetCameraInitFrame(cv::Mat &outInitBluredImg)
{
	//===================================================================================
    // get initial frame
    std::cout << "Set the initial frame" << std::endl;
    int local_initimg_num = 0;
    //====================================================================
    cv::Mat local_initimg_sum_float = cv::Mat::zeros(RAWIMG_SET_SIZE, CV_32FC3);

    for (;;){
        // capture reference frame
        if (CameraCaptureCurrFrame() < 0){
            continue;
        }

        if(m_bInitImgIsCaptured){
            if (CameraCaptureCurrFrame() > 0){
                cv::Mat local_rawimg_float;
                m_CurrCapColorImg.convertTo(local_rawimg_float, CV_32FC3, 1.0, 0.0);
                local_initimg_sum_float = local_initimg_sum_float + local_rawimg_float;

                std::ostringstream LocalSaveImgNo;
                LocalSaveImgNo << std::setfill('0') << std::setw(6) << local_initimg_num;

                const std::string local_initimg_filename = m_ConstSaveDataPreFileName + "_initimg_" 
                    + LocalSaveImgNo.str() + ".png";
                cv::imwrite(local_initimg_filename, m_CurrCapColorImg);

                local_initimg_num++;
            }

            std::cout << "local_initimg_num = " << local_initimg_num << std::endl;
            if(local_initimg_num >= INITIMG_NUM){
                std::cout << "Get the initial image" << std::endl;
                break;
            }
        }
    }
    CV_Assert(local_initimg_num == INITIMG_NUM);

    float localimg_num_float = (float)(INITIMG_NUM);
    localimg_num_float = (float)(1.0/localimg_num_float);
    local_initimg_sum_float = localimg_num_float * local_initimg_sum_float;
    cv::Mat local_initimg_sum_uint8, local_init_bluredimg_float, local_init_bluredimg_uint8;
    local_initimg_sum_float.convertTo(local_initimg_sum_uint8, CV_8UC3, 1.0, 0.0);

    GetInitBluredImg(local_initimg_sum_uint8, local_init_bluredimg_float);
    local_init_bluredimg_float.convertTo(local_init_bluredimg_uint8, CV_8UC3, 1.0, 0.0);

    const std::string local_initimg_filename = m_ConstSaveDataPreFileName + "_initbluredimg.png";
    cv::imwrite(local_initimg_filename, local_init_bluredimg_uint8);

    outInitBluredImg = local_init_bluredimg_uint8.clone();
}

void CGelSightGrabImg::SetSaveDataFolderName(const std::string InSaveDataFolderName)
{
	//====================================================================================================================
    // set the name for the saving images
	time_t t = time(NULL);
	tm* timePtr = localtime(&t);
	std::ostringstream local_time_stamp_out;
	local_time_stamp_out << (1990 + timePtr->tm_year) << "_"
			<< (timePtr->tm_mon + 1) << "_" << (timePtr->tm_mday) << "_"
			<< timePtr->tm_hour << "_" << timePtr->tm_min << "_" << timePtr->tm_sec;

	
	m_ConstSaveDataFolderName = InSaveDataFolderName + "//" + local_time_stamp_out.str() + "_data";
    std::cout << "The folder name to save images: " << m_ConstSaveDataFolderName << std::endl;

    // creat the folder
	if (_access(InSaveDataFolderName.c_str(), 0) == 0){
        std::cout << "SaveDataFolderName = " << InSaveDataFolderName << std::endl;
    }else{
		CreateDirectoryA(InSaveDataFolderName.c_str(), NULL);
    }
	CreateDirectoryA(m_ConstSaveDataFolderName.c_str(), NULL);
    m_ConstSaveDataPreFileName = m_ConstSaveDataFolderName + "//" + local_time_stamp_out.str();
}

void CGelSightGrabImg::GetSaveDataPreFileName(std::string &OutSaveDataPreFileName)
{
    OutSaveDataPreFileName = m_ConstSaveDataPreFileName;
}

void CGelSightGrabImg::GetInitBluredImg(const cv::Mat in_rawimg_uint8, cv::Mat &out_bluredimg_float)
{
    // make sure the input image with the correct type
    CV_Assert((in_rawimg_uint8.type() == CV_8UC3) && (in_rawimg_uint8.size() == RAWIMG_SET_SIZE));

    // init the result, the float type image
    out_bluredimg_float = cv::Mat::zeros(in_rawimg_uint8.size(), CV_32FC3);

    // get the image size
    const int LOCAL_IMAGE_WIDTH = in_rawimg_uint8.cols;
    const int LOCAL_IMAGE_HEIGHT = in_rawimg_uint8.rows;

    // the threshold to change the blur image
    const float LOCAL_RAW_RGBIMG_BLUR_DIFF_THRESHOLD = (float)(3.0 * INITIMG_BLUR_THRESHOLD);
    const float LOCAL_BLUREDIMG_RATIO = (float)(0.15);
    const float LOCAL_RAWIMG_RATIO = (float)(1.0) - LOCAL_BLUREDIMG_RATIO;

    cv::Mat rawimg_float;
    in_rawimg_uint8.convertTo(rawimg_float, CV_32FC3, 1.0, 0.0);
    cv::Mat raw_bluredimg_float;
    // the size to blur the image, const cv::Size GaussBlurSize = cv::Size(21, 21);
    cv::GaussianBlur(rawimg_float, raw_bluredimg_float, INITIMG_GAUSS_BLUR_SIZE, 0, 0);

    cv::Vec3f* rawimg_float_ptr;
    cv::Vec3f* raw_bluredimg_float_ptr;
    cv::Vec3f* out_bluredimg_float_ptr;
    for (int local_row_index = 0; local_row_index < LOCAL_IMAGE_HEIGHT; local_row_index++)
    {
        rawimg_float_ptr = rawimg_float.ptr<cv::Vec3f>(local_row_index);
        raw_bluredimg_float_ptr = raw_bluredimg_float.ptr<cv::Vec3f>(local_row_index);
        out_bluredimg_float_ptr = out_bluredimg_float.ptr<cv::Vec3f>(local_row_index);

        for (int local_col_index = 0; local_col_index < LOCAL_IMAGE_WIDTH; local_col_index++)
        {
            float LocalValue = (*raw_bluredimg_float_ptr)[0] - (*rawimg_float_ptr)[0]
                + (*raw_bluredimg_float_ptr)[1] - (*rawimg_float_ptr)[1]
                + (*raw_bluredimg_float_ptr)[2] - (*rawimg_float_ptr)[2];

            if (LocalValue > LOCAL_RAW_RGBIMG_BLUR_DIFF_THRESHOLD){
                (*out_bluredimg_float_ptr)[0] = (*raw_bluredimg_float_ptr)[0];
                (*out_bluredimg_float_ptr)[1] = (*raw_bluredimg_float_ptr)[1];
                (*out_bluredimg_float_ptr)[2] = (*raw_bluredimg_float_ptr)[2];
            }else{
                //(*out_bluredimg_float_ptr)[0] = (float)(0.15 * (*raw_bluredimg_float_ptr)[0] + 
                    //0.85 * (*rawimg_float_ptr)[0]);
                //(*out_bluredimg_float_ptr)[1] = (float)(0.15 * (*raw_bluredimg_float_ptr)[1] + 
                    //0.85 * (*rawimg_float_ptr)[1]);
                //(*out_bluredimg_float_ptr)[2] = (float)(0.15 * (*raw_bluredimg_float_ptr)[2] + 
                    //0.85 * (*rawimg_float_ptr)[2]);
                (*out_bluredimg_float_ptr)[0] = LOCAL_BLUREDIMG_RATIO * (*raw_bluredimg_float_ptr)[0] 
                    + LOCAL_RAWIMG_RATIO * (*rawimg_float_ptr)[0];
                (*out_bluredimg_float_ptr)[1] = LOCAL_BLUREDIMG_RATIO * (*raw_bluredimg_float_ptr)[1] 
                    + LOCAL_RAWIMG_RATIO * (*rawimg_float_ptr)[1];
                (*out_bluredimg_float_ptr)[2] = LOCAL_BLUREDIMG_RATIO * (*raw_bluredimg_float_ptr)[2] 
                    + LOCAL_RAWIMG_RATIO * (*rawimg_float_ptr)[2];
            }

            // move to the next point
            rawimg_float_ptr++;
            raw_bluredimg_float_ptr++;
            out_bluredimg_float_ptr++;
        }
    }
}