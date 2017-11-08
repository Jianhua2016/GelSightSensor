#include "COpenCVPoissonSolver.h"

COpenCVPoissonSolver::COpenCVPoissonSolver(){
	init_OpenCV_DST_Eigen_Values();
}

COpenCVPoissonSolver::~COpenCVPoissonSolver(){
}

void COpenCVPoissonSolver::init_OpenCV_DST_Eigen_Values(){

	const int ImgWidth = HEIGHTMAP_OPENCV_DST_IMG_SIZE.width;
	const int ImgHeight = HEIGHTMAP_OPENCV_DST_IMG_SIZE.height;
	m_OpenCV_DST_eigen_values = cv::Mat::zeros(HEIGHTMAP_OPENCV_DST_IMG_SIZE, CV_32FC1);

	float localDivPx = (float)(CV_PI)/(float)(ImgWidth + 1);
	float localDivPy = (float)(CV_PI)/(float)(ImgHeight + 1);

	//****************************************************************************************
	// get the value of the sin table
	float *local_ptr;
	for (int iRow = 0; iRow < ImgHeight; iRow++){

		local_ptr = m_OpenCV_DST_eigen_values.ptr<float>(iRow);
		for (int iCol = 0; iCol < ImgWidth; iCol++){
			//compute Eigen Values
			//[x,y] = meshgrid(1:xdim-2,1:ydim-2)
			//denom = (2*cos(pi*x/(xdim-1))-2) + (2*cos(pi*y/(ydim-1)) - 2)

			double localValue = 2.0 * std::cos(localDivPx*(iCol + 1)) - 2.0 + 
				2.0 * std::cos(localDivPy*(iRow + 1)) - 2.0;

			*local_ptr = 1.0f/(float)(localValue);
			local_ptr++;
		}
	}
}

void COpenCVPoissonSolver::get_img_DST_with_OpenCV(const cv::Mat inImg, cv::Mat &outImgDST){

	// init the result
	outImgDST = cv::Mat::zeros(inImg.size(), inImg.type());
	cv::Mat local_inImg = inImg.t();

	const int ImgWidth = local_inImg.cols;
	const int ImgHeight = local_inImg.rows;

	// Horizontal flipping of the image with the subsequent horizontal shift 
	// and absolute difference calculation to check for a vertical-axis symmetry (flipCode > 0).
	cv::Mat local_inImg_flip;
	cv::flip(local_inImg, local_inImg_flip, 1);
	local_inImg_flip = -1.0f * local_inImg_flip;

	cv::Mat local_DST_img = cv::Mat::zeros(cv::Size((2 * ImgWidth + 2), ImgHeight), inImg.type());
	cv::Mat local_DST_img_ROI_1 = local_DST_img(cv::Rect(1, 0, ImgWidth, ImgHeight));
	cv::Mat local_DST_img_ROI_2 = local_DST_img(cv::Rect((2+ImgWidth), 0, ImgWidth, ImgHeight));
	local_inImg.copyTo(local_DST_img_ROI_1);
	local_inImg_flip.copyTo(local_DST_img_ROI_2);

	// std::cout << "local_inImg = " << local_inImg << std::endl << std::endl;
	// std::cout << "local_DST_img = " << local_DST_img << std::endl << std::endl;

	cv::Mat local_planes[] = {Mat_<float>(local_DST_img), Mat::zeros(local_DST_img.size(), CV_32FC1)};
	cv::Mat local_complexI;
	// Add to the expanded another plane with zeros
	cv::merge(local_planes, 2, local_complexI);         

	// this way the result may fit in the source matrix
	cv::dft(local_complexI, local_complexI, DFT_ROWS);            

	// compute the magnitude and switch to logarithmic scale
	// => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
	// local_planes[0] = Re(DFT(I), local_planes[1] = Im(DFT(I))
	cv::split(local_complexI, local_planes);
	cv::Mat local_outImgDST = -0.5 * local_planes[1](cv::Rect(1, 0, ImgWidth, ImgHeight));
	outImgDST = local_outImgDST.t();
}

void COpenCVPoissonSolver::get_img_iDST_with_OpenCV(const cv::Mat inImg, cv::Mat &outImgDST){

	// init the result
	outImgDST = cv::Mat::zeros(inImg.size(), inImg.type());
	cv::Mat local_inImg = inImg.t();
	const int ImgWidth = local_inImg.cols;
	const int ImgHeight = local_inImg.rows;
	const float local_ratio = -1.0f/float(ImgWidth + 1);

	// Horizontal flipping of the image with the subsequent horizontal shift 
	// and absolute difference calculation to check for a vertical-axis symmetry (flipCode > 0).
	cv::Mat local_inImg_flip;
	cv::flip(local_inImg, local_inImg_flip, 1);
	local_inImg_flip = -1.0f * local_inImg_flip;

	cv::Mat local_DST_img = cv::Mat::zeros(cv::Size((2 * ImgWidth + 2), ImgHeight), inImg.type());
	cv::Mat local_DST_img_ROI_1 = local_DST_img(cv::Rect(1, 0, ImgWidth, ImgHeight));
	cv::Mat local_DST_img_ROI_2 = local_DST_img(cv::Rect((2+ImgWidth), 0, ImgWidth, ImgHeight));
	local_inImg.copyTo(local_DST_img_ROI_1);
	local_inImg_flip.copyTo(local_DST_img_ROI_2);


	cv::Mat local_planes[] = {Mat_<float>(local_DST_img), Mat::zeros(local_DST_img.size(), CV_32FC1)};
	cv::Mat local_complexI;
	// Add to the expanded another plane with zeros
	cv::merge(local_planes, 2, local_complexI);         

	// this way the result may fit in the source matrix
	cv::dft(local_complexI, local_complexI, DFT_ROWS);            

	// compute the magnitude and switch to logarithmic scale
	// => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
	// local_planes[0] = Re(DFT(I), local_planes[1] = Im(DFT(I))
	cv::split(local_complexI, local_planes);
	cv::Mat local_outImgDST = local_ratio * local_planes[1](cv::Rect(1, 0, ImgWidth, ImgHeight));
	outImgDST = local_outImgDST.t();
}

void COpenCVPoissonSolver::OpenCV_DST_PossionSolver(const cv::Mat InRawImgGradx, const cv::Mat InRawImgGrady, cv::Mat &OutImgDirect)
{
	// std::cout << "Fast_DST_PossionSolver" << std::endl;
	CV_Assert((InRawImgGradx.type() == CV_32FC1) && (InRawImgGradx.type() == CV_32FC1)
		&& (InRawImgGradx.size() == InRawImgGrady.size()));
	
	// init the result
	OutImgDirect = cv::Mat::zeros(InRawImgGradx.size(), InRawImgGradx.type());
	const int ImgWidth = InRawImgGradx.cols;
	const int ImgHeight = InRawImgGradx.rows;

	cv::Mat ImgGradx = InRawImgGradx.clone();
	cv::Mat ImgGrady = InRawImgGrady.clone();

	cv::Mat ImgGradxx = cv::Mat::zeros(ImgGradx.size(), ImgGradx.type());
	cv::Mat ImgGradyy = cv::Mat::zeros(ImgGrady.size(), ImgGrady.type());
	cv::Mat ImgGradAdd = cv::Mat::zeros(ImgGradx.size(), ImgGradx.type());
	// gxx
	float* localGx_ptr;
	float* localGxx_ptr;
	for (int iRow = 0; iRow < ImgHeight; iRow++){
		localGx_ptr = ImgGradx.ptr<float>(iRow);
		localGxx_ptr = ImgGradxx.ptr<float>(iRow);
		localGxx_ptr++;
		for (int iCol = 1; iCol < ImgWidth; iCol++){
			*localGxx_ptr = (*(localGx_ptr+1)) - (*localGx_ptr);
			localGx_ptr++;
			localGxx_ptr++;
		}
	}

	// gyy
	float* localGyPrev_ptr;
	float* localGy_ptr;
	float* localGyy_ptr;
	for (int iRow = 1; iRow < ImgHeight; iRow++){
		localGyPrev_ptr = ImgGrady.ptr<float>(iRow - 1);
		localGy_ptr = ImgGrady.ptr<float>(iRow);
		localGyy_ptr = ImgGradyy.ptr<float>(iRow);
		for (int iCol = 0; iCol < ImgWidth; iCol++){
			*localGyy_ptr = (*localGy_ptr) - (*localGyPrev_ptr);
			localGyPrev_ptr++;
			localGy_ptr++;
			localGyy_ptr++;
		}
	}
	ImgGradAdd = ImgGradxx + ImgGradyy;
	// std::cout << "step0 is finished" << std::endl;
	
	//****************************************************************************************
	//DST Sine Transform algo starts here
	//compute sine transform

	// get ROI
	cv::Mat local_DST_raw_img = cv::Mat::zeros(HEIGHTMAP_OPENCV_DST_IMG_SIZE, ImgGradx.type());
	
	float* ImgGradAdd_ptr;
	float* local_DST_raw_img_ptr;
	for (int iRow = 0; iRow < (ImgHeight-2); iRow++){
		// std::cout << "iRow = " << iRow << std::endl;

		ImgGradAdd_ptr = ImgGradAdd.ptr<float>(iRow+1);
		ImgGradAdd_ptr++;

		local_DST_raw_img_ptr = local_DST_raw_img.ptr<float>(iRow);
		for (int iCol = 0; iCol < (ImgWidth-2); iCol++){
			*local_DST_raw_img_ptr = *ImgGradAdd_ptr;
			local_DST_raw_img_ptr++;
			ImgGradAdd_ptr++;
		}
	}
	// std::cout << "step1 is finished" << std::endl;

	// compute sine transform
	// in 1D DST
	cv::Mat local_DST_raw_img_dst1, local_DST_raw_img_dst2;
	get_img_DST_with_OpenCV(local_DST_raw_img, local_DST_raw_img_dst1);
	get_img_DST_with_OpenCV(local_DST_raw_img_dst1.t(), local_DST_raw_img_dst2);
	// std::cout << "step2 is finished" << std::endl;

	// std::cout << "local_DST_raw_img = " << local_DST_raw_img << std::endl << std::endl;
	// std::cout << "local_DST_raw_img_dst1 = " << local_DST_raw_img_dst1 << std::endl << std::endl;
	// std::cout << "local_DST_raw_img_dst2 = " << local_DST_raw_img_dst2 << std::endl << std::endl;

	//****************************************************************************************
	cv::Mat local_dst_img = m_OpenCV_DST_eigen_values.mul(local_DST_raw_img_dst2.t());
	// std::cout << "step3 is finished" << std::endl;

	// ===================================================================================
	// Compute Inverse Sine Transform
	// in 1D iDST
	cv::Mat local_DST_raw_img_idst1, local_DST_raw_img_idst2;
	get_img_iDST_with_OpenCV(local_dst_img, local_DST_raw_img_idst1);
    get_img_iDST_with_OpenCV(local_DST_raw_img_idst1.t(), local_DST_raw_img_idst2);

	// std::cout << "local_dst_img = " << local_dst_img << std::endl << std::endl;
	// std::cout << "local_DST_raw_img_idst1 = " << local_DST_raw_img_idst1 << std::endl << std::endl;
	// std::cout << "local_DST_raw_img_idst2 = " << local_DST_raw_img_idst2 << std::endl << std::endl;

	cv::Mat local_out_img = local_DST_raw_img_idst2.t();
	float* local_out_img_ptr;
	float* OutImgDirect_ptr;
	for (int iRow = 0; iRow < (ImgHeight-2); iRow++){
		OutImgDirect_ptr = OutImgDirect.ptr<float>(iRow+1);
		OutImgDirect_ptr++;

		local_out_img_ptr = local_out_img.ptr<float>(iRow);
		for (int iCol = 0; iCol < (ImgWidth-2); iCol++){
			*OutImgDirect_ptr = *local_out_img_ptr;
			OutImgDirect_ptr++;
			local_out_img_ptr++;
		}
	}
}