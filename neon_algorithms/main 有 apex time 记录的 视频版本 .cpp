/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2016 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2/core/version.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <math.h>
#include <opencv/cv.h>
#include <iostream>
#include <opencv/highgui.h>
#include "opencv2/video/tracking.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <vector>
#include <pthread.h>
#include <thread>
#include <string>
#include<stdlib.h>
#include<algorithm>
#include <fstream>
#include <algorithm>
#include<numeric>

using namespace cv;
using namespace std;

Mat SourceImg;


//#include <apexcv_base_image_filters.h>
//#include "all_filter_graph_registrations.h"
//#include "apexcv_filter.h"

//#include <apexcv_base.h>


vector<double> Left_radius_of_Curve;
double Radius_of_Curve_left;
int Kalman_time_calcalate = 0;

int Left_warning_time_accumulate;
int Right_warning_time_accumulate;
int Warning_max_times = 10;//25

#define Kalman_update_time (10 * 60)
#define Lane_Latest_warning_width 0.3//(0.3*2)
#define Test_Lane_Width 3570//mm
#define Test_Pixels_Width 206.5//mm
#define Pixel_Unit_to_width (Test_Lane_Width / Test_Pixels_Width) //mm / pixel

#define Automobile_Test_Width 1825//1825 mm
#define Warning_width ((Automobile_Test_Width / 2.0 + Lane_Latest_warning_width) / Pixel_Unit_to_width)


#if 1
  //the LDW kalman class   predict six data
    namespace Kalman_kalpa_six
    {
        class KalmanFilter
        {
        public:
                KalmanFilter(vector<float> input_pt):
                    KF_(6, 6)
                    /*
                    KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F )
                    "dynamParams = 4": 4*1 vector of state (x, y, delta x, delta y)
                    "measureParams = 2": 2*1 vector of measurement (x, y)
                    */
                    {
                        measurement_ = Mat::zeros(6, 1, CV_32F);// (x, y, x2, y2)
                        KF_.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 0, 0, 0, //1  //**Latter 1: Larger, faster regression
                                                                     0, 1, 0, 0, 0, 0, // 2 //**Latter 1: Larger, faster regression
                                                                     0, 0, 1, 0, 0, 0, // 3
                                                                     0, 0, 0, 1, 0, 0, // 4
                                                                     0, 0, 0, 0, 1, 0,// 5
                                                                     0, 0, 0, 0, 0, 1// 6
                                                                     );
                        setIdentity(KF_.measurementMatrix, Scalar::all(1));
                        setIdentity(KF_.processNoiseCov, Scalar::all(1e-5));//**3: Larger, slower regression
                        setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-1));//1: Larger, quicker regression
                        setIdentity(KF_.errorCovPost, Scalar::all(1));

                        KF_.statePost = (Mat_<float>(6, 1) << input_pt[0], input_pt[1], input_pt[2], input_pt[3],
																input_pt[4], input_pt[5]);//Ensure beginner is default value
                    }
    #if 1
        bool run(vector<float>& input_pt, vector<float>& predict_pt, vector<float>& measure_pt)
        {
            Mat prediction = KF_.predict();

          #if 1
            predict_pt[0] = prediction.at<float>(0);
            predict_pt[1] = prediction.at<float>(1);
			predict_pt[2] = prediction.at<float>(2);
            predict_pt[3] = prediction.at<float>(3);
			predict_pt[4] = prediction.at<float>(4);
            predict_pt[5] = prediction.at<float>(5);



            //cout << "inside prediction is " << prediction.at<float>(0) << "  " << prediction.at<float>(1) << endl;
            //cout << "inside predict_pt is " << predict_pt[0] << "  " << predict_pt[1] << endl;
          #endif
            measurement_.at<float>(0, 0) = input_pt[0];
            measurement_.at<float>(1, 0) = input_pt[1];
			measurement_.at<float>(2, 0) = input_pt[2];
            measurement_.at<float>(3, 0) = input_pt[3];
			measurement_.at<float>(4, 0) = input_pt[4];
            measurement_.at<float>(5, 0) = input_pt[5];


            //measurement.at<float>(0) = (float)A[i][0];

            Mat measurement_correct = KF_.correct(measurement_);
            //Point2f measurement_pt = Point2f(measurement.at<float>(0),measurement.at<float>(1));

          #if 1
            measure_pt[0] = measurement_correct.at<float>(0);
            measure_pt[1] = measurement_correct.at<float>(1);
			measure_pt[2] = measurement_correct.at<float>(2);
            measure_pt[3] = measurement_correct.at<float>(3);
			measure_pt[4] = measurement_correct.at<float>(4);
            measure_pt[5] = measurement_correct.at<float>(5);

          #endif

            return true;
        }
      #endif

    private:
        Mat measurement_;
        cv::KalmanFilter KF_;//Differ from Kalman_example::KalmanFilter
        };

    }

#endif

void Remap_process(Mat& before_remap_image, Mat& map_x, Mat& map_y){
	Mat RGB_MAT;
	cvtColor(before_remap_image, RGB_MAT, COLOR_YUV2BGR_UYVY);
	remap(RGB_MAT, SourceImg, map_x, map_y, INTER_LINEAR);
	resize(SourceImg, SourceImg, Size(640, 400),0,0,INTER_LINEAR);
	return;
}


bool Perspective_process(Mat& image, Mat& Transform_Matrix, Mat& frame_Out_Perspective_convert_result);

void OpenMorphologyEx(Mat& Image_inputK, Mat& Image_outK);
void CloseMorphologyEx(Mat& Image_input, Mat& Image_out);

void Preprocess_bilateralFilter_threshold_APEX_CV(Mat& image, Mat& Image_threshold);
void Preprocess_bilateralFilter_threshold(Mat& image, Mat& Image_threshold);
void Preprocess_bilateralFilter_threshold_third(Mat& image, Mat& Image_threshold);
bool polynomial_curve_fit(vector<Point>& key_point, int n, Mat& A);

//Mat Curve_fit_and_calculate_radius_of_curvature_choices(Mat& Image_src_process, vector<float>& Left_kalmanx_display);
Mat Curve_fit_and_calculate_radius_of_curvature_choices(Mat& Image_src_process, vector<int>& Lane_flag);

//Mat Curve_fit_and_calculate_radius_of_curvature_choices(Mat& Image_src_process, vector<Point>& Last_Left_points_fitted_display, vector<Point>& Last_Right_points_fitted_display);

void Globel_Process_Ldw_Mat(Mat& Src_imge);
void PutText_Model(Mat& image, double radius_of_curve, double FPS_data);
void equalizeHist_Mat(Mat& image);
void Left_Warning_sign(Mat& img);
void Right_Warning_sign(Mat& img);

vector<float> Current_Fitting_X_Point_kalman(6);
vector<float> Predict_X_Point_kalman(6);
vector<float> Correct_X_Point_kalman(6);
vector<float> right_Current_Fitting_X_Point_kalman(6);
vector<float> right_Predict_X_Point_kalman(6);
vector<float> right_Correct_X_Point_kalman(6);

double Right_sub_Left_X_pixel_width;

vector<double> Apex_opencv_time(10);


///////////////////////////////////////////////////////
#ifndef __STANDALONE__
#include <signal.h>
#endif // #ifdef __STANDALONE__
#include <string.h>

#ifdef __STANDALONE__
#include "frame_output_dcu.h"
#define CHNL_CNT io::IO_DATA_CH2
#else // #ifdef __STANDALONE__
#include "frame_output_v234fb.h"
#define CHNL_CNT io::IO_DATA_CH2
#endif // else from #ifdef __STANDALONE__

#include "oal.h"
#include "vdb_log.h"
#include "sdi.hpp"

#include "ov10635_surround_c.h"

#include "vdb_log.h"
#include <common_helpers.h>

//**************ADDADDADDADDADDADDADDADDADDADDADDADD

#ifndef APEX2_EMULATE
#include "apex.h"
#else
#include "acf_lib.hpp"
#endif


//#include "apexcv_filter.h"
//#include "aapexcv_base_image_filters.h"

#include <apexcv_pro_canny.h> // ??? ????
#include <apexcv_base.h>
#include <common_helpers.h>
#include <string>
#include <apexcv_pro_resize.h>//apexcv_pro_resize.h
#include <apexcv_pro_histogram_equalization.h>



#ifdef INPUT_PATH_REL
#define INPUT_ROOT __FULL_DIR__ XSTR(INPUT_PATH_REL)/data/common/
#else
#define INPUT_ROOT "data/common/"
#endif
#ifdef OUTPUT_PATH_REL
#define OUTPUT_ROOT __FULL_DIR__ XSTR(OUTPUT_PATH_REL)/data/output/
#else
#define OUTPUT_ROOT "data/output/"
#endif


//**************ADDADDADDADDADDADDADDADDADDADDADDADD



//***************************************************************************
// constants
//***************************************************************************

// Possible to set input resolution (must be supported by the DCU)
#define WIDTH 1280       ///< width of DDR buffer in pixels
#define HEIGHT 800       ///< height of DDR buffer in pixels
#define DDR_BUFFER_CNT 3 ///< number of DDR buffers per ISP stream

//***************************************************************************

#define STREAM_CYCLE_FRM_CNT 150 ///< number of frames until stream switche
#define STREAM_CNT 4             ///< total number of availabe camear streams

#define FRM_TIME_MSR 300 ///< number of frames to measure the time and fps

//***************************************************************************
// macros
//***************************************************************************

#ifdef __STANDALONE__
//extern SEQ_Buf_t  producerless_buffer_1;
extern "C" {
unsigned long get_uptime_microS(void);
}

#define GETTIME(time) (*(time) = get_uptime_microS())
#else // ifdef __STANDALONE__
#define GETTIME(time)                                                                                                  \
  {                                                                                                                    \
    struct timeval lTime;                                                                                              \
    gettimeofday(&lTime, 0);                                                                                           \
    *time = (lTime.tv_sec * 1000000 + lTime.tv_usec);                                                                  \
  }
#endif // else from #ifdef __STANDALONE__

//***************************************************************************
// types
//***************************************************************************
struct AppContext
{
  sdi_grabber* mpGrabber; ///< pointer to grabber instance
  sdi_FdmaIO*  mpFdma;    ///< pointer to fdma object

  // ** event counters and flags **
  bool     mError;      ///< to signal ISP problems
  uint32_t mFrmDoneCnt; ///< number of frames done events
  uint32_t mFrmCnt;     ///< number of frames grabbed by the app
};                      // struct AppContext

/************************************************************************/
/** User defined call-back function for Sequencer event handling.
  * 
  * \param  aEventType defines Sequencer event type
  * \param  apUserVal  pointer to any user defined object 
  ************************************************************************/
static void SeqEventCallBack(uint32_t aEventType, void* apUserVal);

/************************************************************************/
/** Prepare everything before executing the main functionality .
  * 
  * \param arContext structure capturing the context of the application
  * 
  * \return 0 if all ok, <0 otherwise 
  ************************************************************************/
static int32_t Prepare(AppContext& arContext);

/************************************************************************/
/** Initial setup of application context.
  * 
  * \param arContext structure capturing the context of the application
  ************************************************************************/
static void ContextInit(AppContext& arContext);

/************************************************************************/
/** Prepares required libraries.
  * 
  * \param arContext structure capturing the context of the application
  * 
  * \return 0 if all ok, != 0 otherwise 
  ************************************************************************/
static int32_t LibsPrepare(AppContext& arContext);

/************************************************************************/
/** Prepares DDR buffers.
  * 
  * \param arContext structure capturing the context of the application
  * 
  * \return 0 if all ok, != 0 otherwise 
  ************************************************************************/
static int32_t DdrBuffersPrepare(AppContext& arContext);

/************************************************************************/
/** Execute main functionality of the application.
  * 
  * \param arContext structure capturing the context of the application
  * 
  * \return 0 if all ok, <0 otherwise 
  ************************************************************************/
static int32_t Run(AppContext& arContext);

/************************************************************************/
/** Cleanup all resources before application end.
  * 
  * \param arContext structure capturing the context of the application
  * 
  * \return 0 if all ok, <0 otherwise 
  ************************************************************************/
static int32_t Cleanup(AppContext& arContext);

#ifndef __STANDALONE__
/************************************************************************/
/** SIGINT handler.
  * 
  * \param  aSigNo 
  ************************************************************************/
static void SigintHandler(int);

/************************************************************************/
/** SIGINT handler.
  * 
  * \param  aSigNo 
  * 
  * \return SEQ_LIB_SUCCESS if all ok
  *         SEQ_LIB_FAILURE if failed
  ************************************************************************/
static int32_t SigintSetup(void);

//***************************************************************************

static bool sStop = false; ///< to signal Ctrl+c from command line

#endif // #ifndef __STANDALONE__

static uint8_t sCamChanel = 5;
static int8_t  sCsiPort   = -1;

int main(int argc, char** argv)
{


	APEX_Init();

#if 0

	int lRetVal = 0;

	vsdk::UMat lInput0 = cv::imread(INPUT_ROOT"in_grey_256x256.png", 0).getUMat(cv::ACCESS_READ);
	 //  ???????? aSrc ?? lInput0 

	vsdk::UMat lOutput0(256, 256, VSDK_CV_8UC1);
	// ???????? aDst??lOutput0

	//apexcv::BilateralFilter  BilateralFilter  ;
	    apexcv::BilateralFilter BilateralFilter;
		apexcv::Canny Canny;
	// ???? apexcv::BilateralFilter   ????

	lRetVal |= BilateralFilter.Initialize(lInput0,3,3,3,lOutput0);
	lRetVal |= Canny.Initialize(lInput0, lOutput0,100,150);
	if (lRetVal)
	{
	printf("Error on Initialize: %d \n", lRetVal);
	return lRetVal;
	}
	printf("Initialize Done \n");
	 // apexcv::BilateralFilter :: Initialize ,???? Initialize?????

	lRetVal |= BilateralFilter.Process();
	if (lRetVal)
	{
	printf("Error on Process: %d \n", lRetVal);
	return lRetVal;
	}
	printf("Single Process Done \n");
	// ???? apexcv::BilateralFilter :: Process?????? Process


#endif
    //YUVTOBGR yuvtorgb;
    
  int lRet = 0;

  AppContext lContext;

  

  //*** process command line parameters ***
  const char helpMsg_str[] =
      "\n**************************************************************\n"
      "** Omnivision Ov10635 quad demo using Maxim Ser/Des HW setup\n"
      "** Description:\n"
      "**  o Maxim 9286 deserializer board with 4xOmnivision Ov10635\n"
      "**    cameras each with 9271 serializer (on MipiCsi_0) expected as\n"
      "**    image input.\n"
      "**  o ISP converts YUV422 10bit piexel data provided by the sensor\n"
      "**    to YUV422 8bit pixels and stores single camera images into\n"
      "**    separate DDR buffers.\n"
      "**  o Resulting YUV 1280x800 image are displayed live using DCU.\n"
      "**\n"
      "*********************************\n\n"
      "** Usage:\n"
      "**  o ./isp_ov10635_quad.elf <camera channel> [<csi port>]\n"
      "**\n"
      "** Options:\n"
      "**  o camera channel   1-5. 5: switch each camera between 150 frames\n"
      "**  o                      [default: 5]\n"
      "**  o csi port         0|1 [default: use graph's setting]\n"
      "**\n"
      "*********************************\n\n"
      "** Example:\n"
      "**  o Run camera #2, MAXIM pluged in CSI #1.\n"
      "**    ./isp_ov10635_quad.elf 2 1\n"
      "**  o Run all camera, use graph's setting for csi port.\n"
      "**    ./isp_ov10635_quad.elf\n"
      "**\n"
      "**************************************************************\n\n";
  int idxHelp = COMMON_HelpMessage(argc, argv, helpMsg_str);
  if(idxHelp < 0)
  { // print help message even if no help option is provided by the user
    printf("%s", helpMsg_str);
  }

  if(argc > 1)
  {
    sCamChanel = atoi(argv[1]);
  }

  if(argc > 2)
  {
    sCsiPort = atoi(argv[2]);
  }

  if((sCamChanel < 1) || (sCamChanel > 5) || (sCsiPort < -1) || (sCsiPort > 1))
  {
    lRet = -1;
  } // if check params OK
  else
  {
#ifndef __STANDALONE__
    fflush(stdout);
    sleep(1);
#endif // ifndef __STANDALONE__

    if(Prepare(lContext) == 0)
    {
      if(Run(lContext) != 0)
      {
        printf("Demo execution failed.\n");
        lRet = -1;
      } // if Run() failed
    }   // if Prepare() ok
    else
    {
      printf("Demo failed in preparation phase.\n");
      lRet = -1;
    } // else from if Prepare() ok

    if(Cleanup(lContext) != 0)
    {
      printf("Demo failed in cleanup phase.\n");
      lRet = -1;
    } // if cleanup failed
  }   // else from if check params OK

  return lRet;
} // main()

//***************************************************************************

static int32_t Prepare(AppContext& arContext)
{
  // init app context
  ContextInit(arContext);

  // enable LIBS
  if(LibsPrepare(arContext) != 0)
  {
    printf("Failed to prepare libraries.\n");
    return -1;
  } // if failed to configure decoder
    // enable OAL

#ifndef __STANDALONE__

#endif // #ifndef __STANDALONE__

  if(DdrBuffersPrepare(arContext) != 0)
  {
    printf("Failed to prepare DDR buffers.\n");
    return -1;
  } // if fialed to prepare DDR buffers

  // *** prestart grabber ***
  if(arContext.mpGrabber->PreStart() != LIB_SUCCESS)
  {
    printf("Failed to prestart the grabber.\n");
    return -1;
  } // if PreStart() failed

  if(arContext.mpGrabber->SeqEventCallBackInstall(&SeqEventCallBack, &arContext) != LIB_SUCCESS)
  {
    printf("Failed to install Sequencer event callback.\n");
    return -1;
  } // if callback setup failed

  return 0;
} // Prepare()

//***************************************************************************

static void ContextInit(AppContext& arContext)
{
  arContext.mpGrabber   = NULL;
  arContext.mpFdma      = NULL;
  arContext.mError      = false;
  arContext.mFrmCnt     = 0;
  arContext.mFrmDoneCnt = 0;
} // ContextInit()

//***************************************************************************

static int32_t LibsPrepare(AppContext& arContext)
{
  // *** Initialize SDI ***
  if(sdi::Initialize(0) != LIB_SUCCESS)
  {
    printf("Failed to initialzie SDI.\n");
    return -1;
  } // if failed to initialize SDI

  // create grabber
  arContext.mpGrabber = new(sdi_grabber);
  if(arContext.mpGrabber == NULL)
  {
    printf("Failed to create sdi grabber.\n");
    return -1;
  } // if failed to create grabber

  if(arContext.mpGrabber->ProcessSet(gpGraph, &gGraphMetadata) != LIB_SUCCESS)
  {
    printf("Failed to set ISP graph to grabber.\n");
    return -1;
  } // if ISP graph not set

  if(arContext.mpGrabber->IoGet(SEQ_OTHRIX_MIPICSI0) && sCsiPort > -1)
  {
    printf("helloworld\n");
    arContext.mpGrabber->CsiSwap(sCsiPort + SEQ_OTHRIX_MIPICSI0, SEQ_OTHRIX_MIPICSI0);
  } // if(!lCamIo.mCsiIdxs.empty() && sCsiPort > -1)

  // get IOs
  arContext.mpFdma = (sdi_FdmaIO*)arContext.mpGrabber->IoGet(SEQ_OTHRIX_FDMA);
  if(arContext.mpFdma == NULL)
  {
    printf("Failed to get FDMA object.\n");
    return -1;
  } // if no FDMA object

  return 0;
} // LibsPrepare(AppContext &arContext)

//***************************************************************************

static int32_t DdrBuffersPrepare(AppContext& arContext)
{
  // *** 4x YUV full frame buffer array ***
  // modify DDR frame geometry to fit display output
  SDI_ImageDescriptor lFrmDesc;
  lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);

  if(arContext.mpFdma->DdrBufferDescSet(FDMA_IX_FastDMA_Out, lFrmDesc) != LIB_SUCCESS)
  {
    printf("Failed to set image descriptor 0.\n");
    return -1;
  } // if frame descriptor setup failed

  if(arContext.mpFdma->DdrBufferDescSet(FDMA_IX_FastDMA_Out1, lFrmDesc) != LIB_SUCCESS)
  {
    printf("Failed to set image descriptor 1.\n");
    return -1;
  } // if frame descriptor setup failed

  if(arContext.mpFdma->DdrBufferDescSet(FDMA_IX_FastDMA_Out2, lFrmDesc) != LIB_SUCCESS)
  {
    printf("Failed to set image descriptor 2.\n");
    return -1;
  } // if frame descriptor setup failed

  if(arContext.mpFdma->DdrBufferDescSet(FDMA_IX_FastDMA_Out3, lFrmDesc) != LIB_SUCCESS)
  {
    printf("Failed to set image descriptor 3.\n");
    return -1;
  } // if frame descriptor setup failed

  // allocate DDR buffers
  if(arContext.mpFdma->DdrBuffersAlloc(DDR_BUFFER_CNT) != LIB_SUCCESS)
  {
    printf("Failed to allocate DDR buffers.\n");
    return -1;
  } // if ddr buffers not allocated

  return 0;
} // DdrBuffersPrepare(AppContext &arContext)

//***************************************************************************

static int32_t Run(AppContext& arContext)
{
	//APEX_Init(); 
	
//*** Init DCU Output ***
#ifdef __STANDALONE__
  //io::FrameOutputDCU lDcuOutput(WIDTH, HEIGHT, io::IO_DATA_DEPTH_08, CHNL_CNT, DCU_BPP_YCbCr422);
  io::FrameOutputDCU lDcuOutput(WIDTH, HEIGHT, io::IO_DATA_DEPTH_08, CHNL_CNT, DCU_BPP_24);

#else  // #ifdef __STANDALONE__
  // setup Ctrl+C handler
  if(SigintSetup() != SEQ_LIB_SUCCESS)
  {
    VDB_LOG_ERROR("Failed to register Ctrl+C signal handler.");
    return -1;
  }

  printf("Press Ctrl+C to terminate the demo.\n");

  io::FrameOutputV234Fb lDcuOutput(WIDTH, HEIGHT, io::IO_DATA_DEPTH_08, io::IO_DATA_CH3, DCU_BPP_24);
#endif // else from #ifdef __STANDALONE__

  unsigned long lTimeStart = 0, lTimeEnd = 0, lTimeDiff = 0;

  // *** start grabbing ***
  GETTIME(&lTimeStart);
  if(arContext.mpGrabber->Start() != LIB_SUCCESS)
  {
    printf("Failed to start the grabber.\n");
    return -1;
  } // if Start() failed

  uint32_t  lActiveStreamIndex = 0;
  SDI_Frame lpFrame[STREAM_CNT];
  // *** grabbing/processing loop ***

  //KALPA ADD CODE//////////////////////
  //////////////////////////////////////
  //////////////////////////////////////
  //******** undistortion model ***************//
#if 1
    Size image_size = Size(1280, 800);//Size square_size = Size(20, 20);
    //cout << " image_size is  " << image_size << endl;
    Mat mapx = Mat(image_size, CV_32FC1);
    Mat mapy = Mat(image_size, CV_32FC1);
    Mat R = Mat::eye(3, 3, CV_32F);
    Matx33d intrinsic_matrix(442.599320628354, 0, 638.5263070981157, 0, 443.0719690041739, 402.2470486657329, 0, 0, 1);  
	//Matx33d intrinsic_matrix(443.2754928651194, 0, 639.1900153853104, 0, 443.1982173244536, 404.4138874483186, 0, 0, 1); 
                                 //No.01:442.9401943070878, 0, 639.1914133294234, 0, 442.8709935177127, 404.2649230134551, 0, 0, 1  
                                 
    double distortion_coeffs_array[4] = {-0.0201426, 0.00705835, -0.0064847, 0.00105044};
								//-0.0201426, 0.00705835, -0.0064847, 0.00105044
								//-0.0110265, -0.0213384, 0.0279957, -0.012062	
                                 //No.01:-0.0111159, -0.0196826, 0.0262563, -0.0115046
    Vec4d distortion_coeffs;
    for(int i = 0; i < 4; i++ )
    {
        distortion_coeffs[i] = distortion_coeffs_array[i];
    }
    fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
#endif

//////////////////////////////////////////////
  
#if 1
	//Mat SourceImg;

	vector<float> input_pt(6, 0);
	Kalman_kalpa_six::KalmanFilter kf(input_pt);
	Kalman_kalpa_six::KalmanFilter kf_right(input_pt);

	#if 1
	//Presetting parameter
	vector<Point2f> src_lane_points(4);
	vector<Point2f> dst_lane_perspective_points(4);

	src_lane_points[0] = Point2f(150.85, 360);
	src_lane_points[1] = Point2f(269, 250);
	src_lane_points[2] = Point2f(329.71, 250);
	src_lane_points[3] = Point2f(486.86, 360);

	dst_lane_perspective_points[0] = Point2f(195, 360);
	dst_lane_perspective_points[1] = Point2f(195, 0);
	dst_lane_perspective_points[2] = Point2f(424, 0);
	dst_lane_perspective_points[3] = Point2f(424, 360);

	vector<uchar> inliers_Lane_convert(src_lane_points.size());
 	Mat Transform_Matrix_Lane_convert = findHomography(src_lane_points, dst_lane_perspective_points, RANSAC, 0.1, inliers_Lane_convert);
	Mat Transform_Matrix_Lane_convert_inconvert = findHomography(dst_lane_perspective_points, src_lane_points, RANSAC, 0.1, inliers_Lane_convert);

	#endif

#endif
	int time_calculate_flag;
 	double FPS_data_tmp;
	double FPS_data;

	ofstream oFile;
	oFile.open("Ap_Op_Time.txt",ios::out|ios::trunc);

	//APEX_Init();
	
  for(;;)
  {
  	#if 1
    // pop all
    for(int i = 0; i < 4; i++)
    {
      lpFrame[i] = arContext.mpGrabber->FramePop(i);
      if(lpFrame[i].mUMat.empty())
      {
        printf("Failed to grab image number %u\n", arContext.mFrmCnt);
        arContext.mError = true;
        return -1;
      } // if pop failed
    }   // for all channels
    #endif
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    double time_start = getTickCount();
	#if 1
	Kalman_time_calcalate++;
	if(Kalman_update_time == Kalman_time_calcalate){
		Kalman_time_calcalate = 0;
		//cout << "kalman update" << endl;
		KalmanFilter();
	}
	#endif
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
#if 0	
    Mat output_mat = lpFrame[0].mUMat.getMat(cv::ACCESS_WRITE | OAL_USAGE_CACHED);
    Mat output_mat_B = output_mat.clone();

    Mat image_rgb;//tmp qu xiao
    //image_rgb.create(output_mat.rows, output_mat.cols, CV_8UC3);//first 
    //cout << output_mat.rows << "  " << output_mat.cols << endl;
 
    //********** UYVY TO BGR *********/ 
    //cvtColor(output_mat_B, image_rgb, COLOR_YUV2BGR_UYVY);//tmp qu xiao
	#if 0
//test 
    //Mat image_rgb_output = image_rgb.clone();
    //imwrite("Test.jpg",image_rgb_output); 
	resize(image_rgb, image_rgb, image_size,0,0,INTER_LINEAR); 
	lDcuOutput.PutFrame(image_rgb.getUMat(cv::ACCESS_READ));
	#endif

	#if 0// 42 fps
		double time_end = getTickCount();
		double before_fps = (time_end - time_start) / getTickFrequency();//s
		double FPS_data = 1.0 / (before_fps + 0.0167);
		//cout << "FPS_data is " << FPS_data << endl; 
	#endif


    //*********** Undistortion ***********/
#if 1//01

	//remap(image_rgb, SourceImg, mapx, mapy, INTER_LINEAR);
	#if 0//apex-cv

	APEX_Init(); 
	vsdk::UMat lInput0 = image_rgb.getUMat(cv::ACCESS_READ);
	vsdk::UMat lOutput0(lInput0.rows, lInput0.cols, VSDK_CV_8UC3);
	apexcv::Remap  myRemap; 
	float lTransform[] = {442.599320628354, 0, 638.5263070981157, 0, 443.0719690041739, 402.2470486657329, 0, 0, 1};
	int lRetVal = 0;
	lRetVal |= myRemap.Initialize(lTransform, lInput0.cols, lInput0.rows, lInput0.cols, apexcv::INTER_TYPE::INTER_LINEAR, apexcv::BORDER_TYPE::BORDER_CONSTANT, 0);
	if (lRetVal)
	{
	printf("Error on Initialize: %d \n", lRetVal);
	return lRetVal;
	}
	printf("Initialize Done \n");

	lRetVal |= myRemap.Process(lInput0, lOutput0);
	if (lRetVal)
	{
	printf("Error on Process: %d \n", lRetVal);
	return lRetVal;
	}

	SourceImg = lOutput0.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

	#endif

	thread Cvtcolor_Remap_thread(Remap_process, ref(output_mat_B), ref(mapx), ref(mapy));
	Cvtcolor_Remap_thread.join();

#endif

	#if 0// 25 fps//18  fps
		double time_end = getTickCount();
		double before_fps = (time_end - time_start) / getTickFrequency();//s
		double FPS_data = 1.0 / (before_fps);
		//cout << "FPS_data is " << FPS_data << endl; 
	#endif

	#if 0//test
		double time_end = getTickCount();
		double before_fps = (time_end - time_start) / getTickFrequency();//s
		double FPS_data = 1.0 / before_fps;
		char FPS_string[20];//10
		sprintf(FPS_string, "%.0f", FPS_data);//output is mm
		string FPS_add_String("");
		FPS_add_String += FPS_string;
		putText(SourceImg, FPS_add_String, Point(80, 120), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1.5, 4 );
		
		resize(SourceImg, SourceImg, image_size,0,0,INTER_LINEAR);//test
		lDcuOutput.PutFrame(SourceImg.getUMat(cv::ACCESS_READ));//test
	#endif

    //*********** Undistortion Over***********/
	
    //*********** LDW ***********************/
	//SourceImg = image_rgb_out.clone();first

	//tmp qu xiao 2019-03-15-16-31
	//resize(SourceImg, SourceImg, Size(640, 400),0,0,INTER_LINEAR);

	#if 1
	//double T0 = getTickCount();
	thread Ldw_thread_process(Globel_Process_Ldw_Mat, ref(SourceImg));
	//Median_thread_process.detach();
	Ldw_thread_process.join();
	//double T1 = getTickCount();
    //cout << "0 is :" << (T1 - T0) * 1000 / getTickFrequency() << " ms!" << endl;//11 ms
	#endif

	#if 0
	//double T4 = getTickCount();
	Globel_Process_Ldw_Mat(SourceImg);
	//double T5 = getTickCount();
    //cout << "2 is :" << (T5 - T4) * 1000 / getTickFrequency() << " ms!" << endl;//10 ms
	#endif
	
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
#if 0

	#if 0
	//Presetting parameter
	vector<Point2f> src_lane_points(4);
	vector<Point2f> dst_lane_perspective_points(4);

	src_lane_points[0] = Point2f(150.85, 360);
	src_lane_points[1] = Point2f(269, 250);
	src_lane_points[2] = Point2f(329.71, 250);
	src_lane_points[3] = Point2f(486.86, 360);

	dst_lane_perspective_points[0] = Point2f(195, 360);
	dst_lane_perspective_points[1] = Point2f(195, 0);
	dst_lane_perspective_points[2] = Point2f(424, 0);
	dst_lane_perspective_points[3] = Point2f(424, 360);

	vector<uchar> inliers_Lane_convert(src_lane_points.size());
 	Mat Transform_Matrix_Lane_convert = findHomography(src_lane_points, dst_lane_perspective_points, RANSAC, 0.1, inliers_Lane_convert);
	Mat Transform_Matrix_Lane_convert_inconvert = findHomography(dst_lane_perspective_points, src_lane_points, RANSAC, 0.1, inliers_Lane_convert);

	#endif

	#if 0// 16 fps
	double time_end = getTickCount();
	double before_fps = (time_end - time_start) / getTickFrequency();//s
	double FPS_data = 1.0 / (before_fps + 0.0167);
	//cout << "FPS_data is " << FPS_data << endl; 
	#endif

	#if 1	
	//Presetting
	Rect Lane_section_roi(143.5, 250, 350, 110);//using
	Rect Perspective_Lane_section_roi(160 - 20, 200, 320 + 2 * 20, 120);//using bf
	
	//Preprocess Model
	Mat Src_imge_process = SourceImg.clone();

	Mat SourceImg_Lane_section = Src_imge_process(Lane_section_roi);//bf  20190308 16:14
	//imshow("SourceImg", SourceImg);//test and varify

	//Preprocess to reduce invalid area of image
	Mat Mask = Mat::zeros(SourceImg.size(),CV_8UC3);
	Mat Lane_Paste_Mask_MAT = Mask(Lane_section_roi);
	SourceImg_Lane_section.copyTo(Lane_Paste_Mask_MAT, SourceImg_Lane_section);
	//imshow("Mask", Mask);//test and varify

	#if 0// 16  fps
	double time_end = getTickCount();
	double before_fps = (time_end - time_start) / getTickFrequency();//s
	double FPS_data = 1.0 / (before_fps + 0.0167);
	//cout << "FPS_data is " << FPS_data << endl; 
	#endif

	#if 1
	//perspective Process Model
	Mat Perspective_convert_Lane;
	//Perspective_process(Mask, Transform_Matrix_Lane_convert, Perspective_convert_Lane);
	//imshow("Perspective_convert_Lane", Perspective_convert_Lane);//test and varify

	thread Perspective_process_thread(Perspective_process, ref(Mask), ref(Transform_Matrix_Lane_convert), ref(Perspective_convert_Lane));
	Perspective_process_thread.join();
\
	#endif

	#if 0//  12 fps
	double time_end = getTickCount();
	double before_fps = (time_end - time_start) / getTickFrequency();//s
	double FPS_data = 1.0 / (before_fps + 0.0167);
	//cout << "FPS_data is " << FPS_data << endl; 
	#endif

	//Curve fit and calculate Model
	Mat Perspective_Lane_Process_MAT = Perspective_convert_Lane(Perspective_Lane_section_roi);

	//Preprocess to threshold the perspective image Model
	Mat Perspective_Lane_Process_MAT_SRC;
	Preprocess_bilateralFilter_threshold(Perspective_Lane_Process_MAT, Perspective_Lane_Process_MAT_SRC);//test and verify temperorily 20190222
	//Preprocess_bilateralFilter_threshold_third(Perspective_Lane_Process_MAT, Perspective_Lane_Process_MAT_SRC);
	//imshow("Perspective_Lane_Process_MAT_SRC", Perspective_Lane_Process_MAT_SRC);//test and varify

	#if 0// 9  fps
	double time_end = getTickCount();
	double before_fps = (time_end - time_start) / getTickFrequency();//s
	double FPS_data = 1.0 / (before_fps + 0.0167);
	//cout << "FPS_data is " << FPS_data << endl; 
	#endif

	//curve calculate Model
	Mat First_and_second_and_third_window_Mask = Mat::zeros(Perspective_Lane_Process_MAT_SRC.size(),CV_8UC3);
	#endif

	#if 0//9 fps
	double time_end = getTickCount();
	double before_fps = (time_end - time_start) / getTickFrequency();//s
	double FPS_data = 1.0 / (before_fps + 0.0167);
	//cout << "FPS_data is " << FPS_data << endl; 
	#endif

	#if 1 //must be located here, not to delete
	//Curve fitting to calculate the radius of curve
	Mat Third_window_Curve_calculate_addtional_curve_fitting;
	//vector<float> Current_Fitting_X_Point_kalman;
	vector<int> Lane_flag(2);
	Third_window_Curve_calculate_addtional_curve_fitting = Curve_fit_and_calculate_radius_of_curvature_choices(Perspective_Lane_Process_MAT_SRC, Lane_flag);
	//imshow("addtional", Third_window_Curve_calculate_addtional_curve_fitting);//test and varify

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	#if 0//8 fps
	double time_end = getTickCount();
	double before_fps = (time_end - time_start) / getTickFrequency();//s
	double FPS_data = 1.0 / (before_fps + 0.0167);
	//cout << "FPS_data is " << FPS_data << endl; 
	#endif
	
	vector<Point> L_Left_kalmanx_display;
	vector<Point> R_Left_kalmanx_display;
	vector<Point> Left_kalmanx_display;
	
	vector<Point> L_Right_kalmanx_display;
	vector<Point> Right_kalmanx_display;
	vector<Point> R_Right_kalmanx_display;
	
	#if 1
	int Departure_location = 10;
	//left correct
	for(int i = 0; i < 3; i++){
		int A = i*2;
		int B = i*2 + 1;
		Left_kalmanx_display.push_back(Point(Correct_X_Point_kalman[A], Correct_X_Point_kalman[B]));
		L_Left_kalmanx_display.push_back(Point(Correct_X_Point_kalman[A] - Departure_location, Correct_X_Point_kalman[B]));
		R_Left_kalmanx_display.push_back(Point(Correct_X_Point_kalman[A] + Departure_location, Correct_X_Point_kalman[B]));
	}
	//right correct
	for(int i = 0; i < 3; i++){
		int A = i*2;
		int B = i*2 + 1;
		Right_kalmanx_display.push_back(Point(right_Correct_X_Point_kalman[A], right_Correct_X_Point_kalman[B]));
		L_Right_kalmanx_display.push_back(Point(right_Correct_X_Point_kalman[A] - Departure_location, right_Correct_X_Point_kalman[B]));
		R_Right_kalmanx_display.push_back(Point(right_Correct_X_Point_kalman[A] + Departure_location, right_Correct_X_Point_kalman[B]));

	}

	#endif

	#if 1
	double right_x = right_Current_Fitting_X_Point_kalman[4];//right_Current_Fitting_X_Point_kalman  //right_Correct_X_Point_kalman
	double left_x = Current_Fitting_X_Point_kalman[4];
	Right_sub_Left_X_pixel_width = right_x - left_x;
	//cout << "D-value:  " << right_Correct_X_Point_kalman[4] - Correct_X_Point_kalman[4] << endl;//206.5 = 3570
	//lane departure process
	double Median_location_X = (right_x + left_x) / 2;
	double Left_lane_distance = Median_location_X - left_x;
	double Right_lane_distance = right_x - Median_location_X;
	
	int Departure_left_flag;
	int Departure_right_flag;
	if(Left_lane_distance < Warning_width)
		Departure_left_flag = 1;
	if(Right_lane_distance < Warning_width)
		Departure_right_flag = 1;
	
	
	Scalar scalar(136, 218, 230);//30, 250, 230//60, 170, 250//225, 75, 225//230, 250, 50
	int Line_thick = 8;
	//display left lane
	if(Lane_flag[0] != 0){
		//departure display
		if(Departure_left_flag == 1){
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(0, 0, 255), 8, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);

			Left_warning_time_accumulate++;
			if(Left_warning_time_accumulate == Warning_max_times){
				Left_Warning_sign(SourceImg);
				Left_warning_time_accumulate = 0;
			}
			Departure_left_flag = 0;
		}
		else{
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(0, 255, 0), 8, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
		}
	}
	else{
		#if 0
		//departure display
		if(Departure_left_flag == 1){
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(0, 0, 255), 8, 16, 0);
			Left_Warning_sign(Src_imge);
			Departure_left_flag = 0;
		}
		else{
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(0, 255, 255), 8, 16, 0);
		}
		#endif
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(250, 50, 80), 8, 16, 0);//0, 255, 255
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
	}

	//display right lane
	if(Lane_flag[1] != 0){	
		//departure display
		if(Departure_right_flag == 1){
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(0, 0, 255), 8, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
			
			Right_warning_time_accumulate++;
			if(Right_warning_time_accumulate == Warning_max_times){
				Right_Warning_sign(SourceImg);
				Right_warning_time_accumulate = 0;
			}
			Departure_right_flag = 0;
		}
		else{
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(0, 255, 0), 8, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
		}
	}
	else{
		#if 0
		//departure display
		if(Departure_right_flag == 1){
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(0, 0, 255), 8, 16, 0);
			Right_Warning_sign(Src_imge);
			Departure_right_flag = 0;
		}
		else{
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(0, 255, 255), 8, 16, 0);
		}
		#endif
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(250, 50, 80), 8, 16, 0);//0, 255, 255
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
		
	}
	#endif
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

	Third_window_Curve_calculate_addtional_curve_fitting.copyTo(First_and_second_and_third_window_Mask, Third_window_Curve_calculate_addtional_curve_fitting);
	//imshow("after addtional", First_and_second_and_third_window_Mask);//test and varify
	//imshow("Perspective_Lane_Process_MAT_threshold", Perspective_Lane_Process_MAT_threshold);//test and varify

	rectangle(First_and_second_and_third_window_Mask, Point(0, 0), Point(First_and_second_and_third_window_Mask.cols, First_and_second_and_third_window_Mask.rows), Scalar(0, 0, 255), 8, 16, 0);
	line(First_and_second_and_third_window_Mask, Point(First_and_second_and_third_window_Mask.cols / 2, 0), Point(First_and_second_and_third_window_Mask.cols / 2, First_and_second_and_third_window_Mask.rows),
				Scalar(0, 0, 255), 2, 4);
	#endif

	//Mask to process the data of lane detected
	Mat First_and_second_third_window_Lane_Mask = Mat::zeros(Perspective_convert_Lane.size(),CV_8UC3);
	Mat First_and_second_window_Lane_Mask_inconvert = First_and_second_third_window_Lane_Mask(Perspective_Lane_section_roi);
	First_and_second_and_third_window_Mask.copyTo(First_and_second_window_Lane_Mask_inconvert, First_and_second_and_third_window_Mask);
	//imshow("First_and_second_window_Lane_Mask", First_and_second_window_Lane_Mask);//test and varify

	//Inconveret perspective Process Model
	Mat Inconvert_Perspective_First_and_second_window_Mask;
	Perspective_process(First_and_second_third_window_Lane_Mask, Transform_Matrix_Lane_convert_inconvert, Inconvert_Perspective_First_and_second_window_Mask);
	//imshow("Inconvert_Perspective_First_and_second_window_Mask", Inconvert_Perspective_First_and_second_window_Mask);//test and varify

	Inconvert_Perspective_First_and_second_window_Mask.copyTo(SourceImg, Inconvert_Perspective_First_and_second_window_Mask);
	//imshow("Src_imge", Src_imge);

#endif
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
	
	kf.run(Current_Fitting_X_Point_kalman, Predict_X_Point_kalman, Correct_X_Point_kalman);
	kf_right.run(right_Current_Fitting_X_Point_kalman, right_Predict_X_Point_kalman, right_Correct_X_Point_kalman);

	#if 1
	double time_end = getTickCount();
	double before_fps = (time_end - time_start) / getTickFrequency();//s
	double FPS_data = 1.0 / (before_fps + 0.019);
	//cout << "FPS_data is " << FPS_data << endl; 
	#endif

	double time_start_two = getTickCount();
	PutText_Model(SourceImg, Radius_of_Curve_left, FPS_data);
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	#if 0
	double time_end = getTickCount();
	double before_fps = (time_end - time_start) / getTickFrequency();//s
	double FPS_data = 1.0 / before_fps;
	char FPS_string[20];//10
	sprintf(FPS_string, "%.0f", FPS_data);//output is mm
	string FPS_add_String("");
	FPS_add_String += FPS_string;
	putText(SourceImg, FPS_add_String, Point(80, 120), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 4, 16 );
	#endif
	
	resize(SourceImg, SourceImg, image_size,0,0,INTER_LINEAR); 
    lDcuOutput.PutFrame(SourceImg.getUMat(cv::ACCESS_READ));


	double time_end_two = getTickCount();
	double after_fps = (time_end_two - time_start_two) / getTickFrequency();//s
	//cout << "after_fps is " << after_fps << endl;
#endif	

	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//

	#if 0
	double time_start = getTickCount();
	SourceImg = imread("Test03.jpg", 1);
	
	resize(SourceImg, SourceImg, Size(640, 400),0,0,INTER_LINEAR);
	
	thread Ldw_thread_process(Globel_Process_Ldw_Mat, ref(SourceImg));
	Ldw_thread_process.join();

	kf.run(Current_Fitting_X_Point_kalman, Predict_X_Point_kalman, Correct_X_Point_kalman);
	kf_right.run(right_Current_Fitting_X_Point_kalman, right_Predict_X_Point_kalman, right_Correct_X_Point_kalman);

	double time_end = getTickCount();
	double before_fps = (time_end - time_start) / getTickFrequency();//s
	double FPS_data = 1.0 / (before_fps + 0.019);
	//cout << "FPS_data is " << FPS_data << endl; 
	PutText_Model(SourceImg, Radius_of_Curve_left, FPS_data);
	
	resize(SourceImg, SourceImg, image_size,0,0,INTER_LINEAR);//test
	lDcuOutput.PutFrame(SourceImg.getUMat(cv::ACCESS_READ));//test
	#endif

	#if 1
	VideoCapture capture;
	SourceImg = capture.open("LaneNew.mp4");
    if(!capture.isOpened())
    {
        printf("can not open ...\n");
        return -1;
    }
    while (capture.read(SourceImg)){
		#if 1
		Mat SourceImg_apex = SourceImg.clone();
		double Tre0 = getTickCount();
		resize(SourceImg, SourceImg, Size(640, 400),0,0,INTER_LINEAR);
		double Tre1 = getTickCount();
		//cout << "opencv resize is :" << (T14 - T13) * 1000 / getTickFrequency() << " ms!" << endl;
		Apex_opencv_time[14] = (Tre1 - Tre0) * 1000 / getTickFrequency();
		#endif
		#if 1
		//Mat SourceImg_apex = SourceImg.clone();
		double Tresize_apx0 = getTickCount();
		int lRetVal = 0;	
		vsdk::UMat lInput0 = SourceImg_apex.getUMat(cv::ACCESS_READ);
		//vsdk::UMat lInput0 = SourceImg_apex.getUMat(cv::ACCESS_READ);
		vsdk::UMat lOutput0(400, 640, VSDK_CV_8UC3); 
		
		apexcv::Resize resizeApex;

		lRetVal |= resizeApex.Initialize(lInput0, lOutput0);
		lRetVal |= resizeApex.ReconnectIO(lInput0, lOutput0);
		double Tresize_apx1 = getTickCount();
		//cout << "apexcv resize is :" << (T16 - T15) * 1000 / getTickFrequency() << " ms!" << endl;
		Apex_opencv_time[15] = (Tresize_apx1 - Tresize_apx0) * 1000 / getTickFrequency();
		Mat resize_apex_SourceImg = lOutput0.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
		//Mat output_resize = lOutput0.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
		//cout << output_resize.rows << "  " << output_resize.cols << endl;
		
		#endif

		#if 1
		//thread Ldw_thread_process(Globel_Process_Ldw_Mat, ref(output_resize));
		thread Ldw_thread_process(Globel_Process_Ldw_Mat, ref(SourceImg));
		Ldw_thread_process.join();

		kf.run(Current_Fitting_X_Point_kalman, Predict_X_Point_kalman, Correct_X_Point_kalman);
		kf_right.run(right_Current_Fitting_X_Point_kalman, right_Predict_X_Point_kalman, right_Correct_X_Point_kalman);

		double time_end = getTickCount();
		double before_fps = (time_end - time_start) / getTickFrequency();//s
		double FPS_data = 1.0 / (before_fps + 0.019);
		//cout << "FPS_data is " << FPS_data << endl; 
		PutText_Model(SourceImg, Radius_of_Curve_left, FPS_data);

		 oFile<< Apex_opencv_time[0] <<","<< Apex_opencv_time[1] <<","<< Apex_opencv_time[2] <<","<< Apex_opencv_time[3] <<","<< Apex_opencv_time[4] 
    	<<","<< Apex_opencv_time[5] <<","<< Apex_opencv_time[6] <<","<< Apex_opencv_time[7] <<","<< Apex_opencv_time[8] <<","<< Apex_opencv_time[9] 
    	<<","<< Apex_opencv_time[10] <<","<< Apex_opencv_time[11] <<","<< Apex_opencv_time[12] <<","<< Apex_opencv_time[13] <<","<< Apex_opencv_time[14] <<","<< Apex_opencv_time[15] <<endl;
		#endif
		resize(SourceImg, SourceImg, image_size,0,0,INTER_LINEAR);//test
		
		lDcuOutput.PutFrame(SourceImg.getUMat(cv::ACCESS_READ));//test

    }
	
	#endif
			
    #if 0
    if(sCamChanel < 5)
    {
      lDcuOutput.PutFrame(lpFrame[sCamChanel - 1].mUMat);
    }
    else
    {
      if(((++arContext.mFrmCnt) % STREAM_CYCLE_FRM_CNT) == 0)
      {
        ++lActiveStreamIndex;
        lActiveStreamIndex = lActiveStreamIndex % STREAM_CNT;
        printf("Selected camera = %u ", lActiveStreamIndex);
      } // if stream to be switched

      lDcuOutput.PutFrame(lpFrame[lActiveStreamIndex].mUMat);
    }
    #endif


    for(int i = 0; i < 4; i++)
    {
      if(arContext.mpGrabber->FramePush(lpFrame[i]) != LIB_SUCCESS)
      {
        printf("Failed to push image number %u\n", arContext.mFrmCnt);
        arContext.mError = true;
        break;
      } // if push failed
    }

    if((arContext.mFrmCnt % FRM_TIME_MSR) == 0)
    {
      GETTIME(&lTimeEnd);
      lTimeDiff  = lTimeEnd - lTimeStart;
      lTimeStart = lTimeEnd;

      printf("%u frames took %lu usec (%5.2ffps)\n", FRM_TIME_MSR, lTimeDiff,
             (FRM_TIME_MSR * 1000000.0) / ((float)lTimeDiff));
    } // if time should be measured

#ifndef __STANDALONE__
    if(sStop)
    {
      break; // break if Ctrl+C pressed
    }        // if Ctrl+C
#endif       //#ifndef __STANDALONE__

  }          // for ever

  return 0;
} // Run()

//***************************************************************************

static int32_t Cleanup(AppContext& arContext)
{
  int32_t lRet = 0;
  if(arContext.mpGrabber != NULL)
  {
    if(arContext.mpGrabber->Stop())
    {
      printf("Failed to stop the grabber.\n");
      lRet = -1;
    } // if grabber stop failed

    if(arContext.mpGrabber->Release())
    {
      printf("Failed to release grabber resources.\n");
      lRet = -1;
    } // if grabber resources not released

    delete(arContext.mpGrabber);
    arContext.mpGrabber = NULL;
  } // if grabber exists

#ifdef __STANDALONE__
  for(;;)
    ;  // *** don't return ***
#endif // #ifdef __STANDALONE__

  if(sdi::Close(0) != LIB_SUCCESS)
  {
    printf("Failed to terminate use of SDI.\n");
    lRet = -1;
  } // if SDI use termination failed

  return lRet;
} // Cleanup()

//***************************************************************************

static void SeqEventCallBack(uint32_t aEventType, void* apUserVal)
{
  AppContext* lpAppContext = (AppContext*)apUserVal;

  if(lpAppContext)
  {
    if(aEventType == SEQ_MSG_TYPE_FRAMEDONE)
    {
      printf("Frame done message arrived #%u.\n", lpAppContext->mFrmDoneCnt++);
    } // if frame done arrived
  }   // if user pointer is NULL
} // SeqEventCallBack()

  //***************************************************************************

#ifndef __STANDALONE__
static void SigintHandler(int)
{
  sStop = true;
} // SigintHandler()

//***************************************************************************

int32_t SigintSetup()
{
  static int32_t lRet = SEQ_LIB_SUCCESS;

  // prepare internal signal handler
  struct sigaction lSa;
  memset(&lSa, 0, sizeof(lSa));
  lSa.sa_handler = SigintHandler;

  if(sigaction(SIGINT, &lSa, NULL) != 0)
  {
    VDB_LOG_ERROR("Failed to register signal handler.\n");
    lRet = SEQ_LIB_FAILURE;
  } // if signal not registered

  return lRet;
} // SigintSetup()

//***************************************************************************
#endif // #ifndef __STANDALONE__

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//

void PutText_Model(Mat& image, double radius_of_curve, double FPS_data){
	//Radius of Curve display
	Point Location = Point(40, 40);
	putText(image, "Cur's radius: ", Location, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1.5, 8 );

	int result = isinf(radius_of_curve);
	if(1 == result){
		//cout << "infinite" << endl;
		putText(image, "Infinite", Point(160, 40), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1.5, 8 );
	}
	else if(0 == result){
		//cout << "mean is " << mean << endl;
		//Left_departure_X_Space
		char Left_string[10];//10
		sprintf(Left_string, "%.1f", radius_of_curve);//output is mm
		string LDW_Left_String("");
		LDW_Left_String += Left_string;
		putText(image, LDW_Left_String, Point(160, 40), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1.5, 8 );
		
	}
	
	//display the width of lane
	//cout << " Pixel_Unit_to_width " << Pixel_Unit_to_width << endl;
	putText(image, "Lane's Width: ", Point(40, 80), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1.5, 8 );
	double Lane_detection = Right_sub_Left_X_pixel_width * Pixel_Unit_to_width;
	char Lane_width_string[20];//10
	sprintf(Lane_width_string, "%.0f", Lane_detection);//output is mm
	string Lane_width_add_String("");
	Lane_width_add_String += Lane_width_string;
	putText(image, Lane_width_add_String, Point(160, 80), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1.5, 8 );

	#if 1
	putText(image, "FPS: ", Point(40, 120), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1.5, 4 );
	double FPS_calculate = FPS_data;
	char FPS_string[20];//10
	sprintf(FPS_string, "%.0f", FPS_calculate);//output is mm
	string FPS_add_String("");
	FPS_add_String += FPS_string;
	putText(image, FPS_add_String, Point(80, 120), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1.5, 4 );

	#endif

	return;
}
#if 0
void OpenMorphologyEx(Mat& Image_inputK, Mat& Image_outK){
	int lRetVal = 0;
	vsdk::UMat lInput0 = Image_inputK.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0(Image_inputK.rows, Image_inputK.cols, VSDK_CV_8UC1);
	apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lInput0, 3, lOutput0);
	lRetVal |= ErodeFilter.Process();
	Image_outK = lOutput0.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	return;
}

void CloseMorphologyEx(Mat& Image_input, Mat& Image_out){
	int lRetVal = 0;
	vsdk::UMat lInput0 = Image_input.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0(Image_input.rows, Image_input.cols, VSDK_CV_8UC1);
	apexcv::DilateFilter  DilateFilter;
	lRetVal |= DilateFilter.Initialize(lInput0, 3, lOutput0);
	lRetVal |= DilateFilter.Process();
	Image_out = lOutput0.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	return;
}

#endif

//Preprocess_bilateralFilter_threshold_APEX_CV
void Preprocess_bilateralFilter_threshold_APEX_CV(Mat& image, Mat& Image_threshold)
{
	//APEX_Init();
	int lRetVal = 0;
	
	Mat Image_Tmp = image.clone();
	//convertScaleAbs(image, image, 1.2, 30);
	//imshow("image",image);//test and verify

	//equalizeHist_Mat(Image_Tmp);
	//imshow("00",Image_Tmp);//test and verify
	#if 1
	double T20 = getTickCount();
	vsdk::UMat lInput0E = Image_Tmp.getUMat(ACCESS_READ);
    vsdk::UMat lOutput0E(Image_Tmp.rows, Image_Tmp.cols, VSDK_CV_8UC3);
	apexcv::HistogramEqualization HistogramEqualization;
	
	lRetVal |= HistogramEqualization.Initialize(lInput0E, lOutput0E);
	
	Image_Tmp = lOutput0E.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T21 = getTickCount();
	//cout << "apexcv equalize is :" << (T21 - T20) * 1000 / getTickFrequency() << " ms!" << endl;
	#endif

#if 1
	double minv = 0.0, maxv = 0.0;
	double* minp = &minv;
	double* maxp = &maxv;

	minMaxIdx(Image_Tmp,minp,maxp);
	//cout << "Mat minv = " << minv << endl;
	//cout << "Mat maxv = " << maxv << endl;

	int MAX = 255, MIN = 0;

	for (int i=0;i<Image_Tmp.rows;i++)
	{
		uchar* pdata=Image_Tmp.ptr<uchar>(i);
		
		for (int j=0;j<Image_Tmp.cols*Image_Tmp.channels();j++)
		{
			pdata[j] = (pdata[j] - MIN) * (MAX - MIN)/(maxv - minv) + MIN;
		}
	}
	//imshow("Image_Tmp",Image_Tmp);//test and verify

#endif

	Mat Image_cvt;
	//Mat Image_cvt_apex = Image_Tmp.clone();
	//cvtColor(image, Image_cvt, COLOR_BGR2GRAY,0);
	//cvtColor(Image_Tmp, Image_cvt, COLOR_BGR2GRAY,0);
	#if 1
	double T03 = getTickCount();
	int Width_apex = Image_Tmp.cols;
	int Height_apex = Image_Tmp.rows;
	vsdk::UMat lInput0_cvt = Image_Tmp.getUMat(cv::ACCESS_READ);
	vsdk::UMat lOutput0_cvt(Height_apex, Width_apex, VSDK_CV_8UC1); 
	apexcv::ColorConverter ColorConverter;
	lRetVal |= ColorConverter.Initialize(lInput0_cvt, apexcv::ColorConverter::eBGR888_TO_GREY,lOutput0_cvt);
	lRetVal |= ColorConverter.Process();
	
	Image_cvt = lOutput0_cvt.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T04 = getTickCount();
	//cout << "apexcv cvtColor is :" << (T04 - T03) * 1000 / getTickFrequency() << " ms!" << endl;
	#endif
	//imshow("Image_cvt",Image_cvt);//test and verify

	#if 1
	Mat Image_morph;
	double T09 = getTickCount();
	//Mat Image_cvt_after_apex = Image_cvt.clone();
	//open one time
	vsdk::UMat lInput0K = Image_cvt.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0K(Height_apex, Width_apex, VSDK_CV_8UC1);
	vsdk::UMat lOutput1K(Height_apex, Width_apex, VSDK_CV_8UC1);
	apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lInput0K, 3, lOutput0K);
	lRetVal |= ErodeFilter.Process();

	apexcv::DilateFilter  DilateFilter;
	lRetVal |= DilateFilter.Initialize(lOutput0K, 3, lOutput1K);
	lRetVal |= DilateFilter.Process();

	//close two times
	vsdk::UMat lOutput1C(Height_apex, Width_apex, VSDK_CV_8UC1);
	vsdk::UMat lOutput2C(Height_apex, Width_apex, VSDK_CV_8UC1);
	//apexcv::DilateFilter DilateFilter;
	lRetVal |= DilateFilter.Initialize(lOutput1K, 3, lOutput1C);
	lRetVal |= DilateFilter.Process();
	//apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lOutput1C, 3, lOutput2C);
	lRetVal |= ErodeFilter.Process();

	vsdk::UMat lOutput3C(Height_apex, Width_apex, VSDK_CV_8UC1);
	vsdk::UMat lOutput4C(Height_apex, Width_apex, VSDK_CV_8UC1);
	//apexcv::DilateFilter DilateFilter;
	lRetVal |= DilateFilter.Initialize(lOutput2C, 3, lOutput3C);
	lRetVal |= DilateFilter.Process();
	//apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lOutput3C, 3, lOutput4C);
	lRetVal |= ErodeFilter.Process();

	Image_morph = lOutput4C.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	
	double T10 = getTickCount();
	//cout << "apexcv Open and close is :" << (T10 - T09) * 1000 / getTickFrequency() << " ms!" << endl;
	#endif

	#if 1
	double T00 = getTickCount();
	//int Width = Image_apexcv.cols;
	//int Height = Image_apexcv.rows;
	Mat Image_canny;
	
	//APEX-CV test
	//APEX_Init();
	//int lRetVal = 0;
	vsdk::UMat lInput0 = Image_morph.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0(Height_apex, Width_apex, VSDK_CV_8UC1);
	apexcv::BilateralFilter testBilateralFilter;
	lRetVal |= testBilateralFilter.Initialize(lInput0, 3, 3, 3, lOutput0);
	lRetVal |= testBilateralFilter.Process();
	
	apexcv::Canny Canny_apex;
	vsdk::UMat lOutput2(Height_apex, Width_apex, VSDK_CV_8UC1);
	lRetVal |= Canny_apex.Initialize(lOutput0, lOutput2,100,200);
	lRetVal |= Canny_apex.Process();
	
	Image_canny = lOutput2.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T01 = getTickCount();
	//cout << "apexcv bilaterlfilter and canny is :" << (T01 - T00) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif
	#if 1
	//Mat Image_threshold_apex = Image_canny.clone();
	double T05 = getTickCount();
	//imshow("Image_canny",Image_canny);//test and verify
	//threshold(Image_canny,Image_threshold, 120, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);//CV_THRESH_OTSU  CV_THRESH_TRIANGLE
	adaptiveThreshold(Image_canny, Image_threshold, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 0);//ADAPTIVE_THRESH_MEAN_C  ADAPTIVE_THRESH_GAUSSIAN_C
	double T06 = getTickCount();
	//cout << "opencv adaptiveThreshold is :" << (T06 - T05) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif
	
	#if 0
	Mat Image_canny_apex = Image_canny.clone();

	
	double T07 = getTickCount();
	vsdk::UMat Input0_thresh = Image_canny_apex.getUMat(ACCESS_READ);
	vsdk::UMat Output1_thresh(Height_apex, Width_apex, VSDK_CV_8UC1);
	/*
	apexcv::Threshold Threshold;
	lRetVal |= Threshold.Initialize(Input0_thresh, 120, Output1_thresh);
	lRetVal |= Threshold.ReconnectIO(Input0_thresh, Output1_thresh);
	lRetVal |= Threshold.Process();
	*/
	apexcv::ThresholdRange ThresholdRange;
	lRetVal |= ThresholdRange.Initialize(Input0_thresh, 120, 200 ,Output1_thresh);
	lRetVal |= ThresholdRange.ReconnectIO(Input0_thresh, Output1_thresh);

	
	lRetVal |= ThresholdRange.Process();
	
	Image_threshold = Output1_thresh.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T08 = getTickCount();
	//cout << "apexcv Threshold is :" << (T08 - T07) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif
	
	//imshow("Image_threshold",Image_threshold);//test and verify
	
  return;
}

void Preprocess_bilateralFilter_threshold_Calculate_time(Mat& image, Mat& Image_threshold)
{
	//APEX_Init();
	int lRetVal = 0;
	
	Mat Image_Tmp = image.clone();
	//convertScaleAbs(image, image, 1.2, 30);
	//imshow("image",image);//test and verify
	int width_tmp = Image_Tmp.cols;
	int height_tmp = Image_Tmp.rows;
	
	Mat Image_Tmp_apex = image.clone();
	#if 1
	double Tequa0 = getTickCount();
	equalizeHist_Mat(Image_Tmp);
	double Tequa1 = getTickCount();
	Apex_opencv_time[12] = (Tequa1 - Tequa0) * 1000 / getTickFrequency();
	//imshow("00",Image_Tmp);//test and verify
	#endif
	
	#if 1
	double T15 = getTickCount();
	vsdk::UMat lInput0E = Image_Tmp_apex.getUMat(ACCESS_READ);
    vsdk::UMat lOutput0E(Image_Tmp.rows, Image_Tmp.cols, VSDK_CV_8UC3);
	apexcv::HistogramEqualization HistogramEqualization;
	
	lRetVal |= HistogramEqualization.Initialize(lInput0E, lOutput0E);
	
	Mat Equlization_apex_result = lOutput0E.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T16 = getTickCount();
	Apex_opencv_time[13] = (T16 - T15) * 1000 / getTickFrequency();
	//cout << "apexcv equalize is :" << (T21 - T20) * 1000 / getTickFrequency() << " ms!" << endl;
	#endif

#if 1
	double minv = 0.0, maxv = 0.0;
	double* minp = &minv;
	double* maxp = &maxv;

	minMaxIdx(Image_Tmp,minp,maxp);
	//cout << "Mat minv = " << minv << endl;
	//cout << "Mat maxv = " << maxv << endl;

	int MAX = 255, MIN = 0;

	for (int i=0;i<Image_Tmp.rows;i++)
	{
		uchar* pdata=Image_Tmp.ptr<uchar>(i);
		
		for (int j=0;j<Image_Tmp.cols*Image_Tmp.channels();j++)
		{
			pdata[j] = (pdata[j] - MIN) * (MAX - MIN)/(maxv - minv) + MIN;
		}
	}
	//imshow("Image_Tmp",Image_Tmp);//test and verify

#endif

	
	Mat Image_cvt_apex = Image_Tmp.clone();
	double Tcvt0 = getTickCount();
	Mat Image_cvt;	
	cvtColor(Image_Tmp, Image_cvt, COLOR_BGR2GRAY,0);
	double Tcvt1 = getTickCount();
	//cout << "apexcv Open and close is :" << (Tcvt1 - Tcvt0) * 1000 / getTickFrequency() << " ms!" << endl;
	double Tcvt_result = (Tcvt1 - Tcvt0) * 1000 / getTickFrequency();
	Apex_opencv_time[0] = Tcvt_result;
	#if 1
	double T02 = getTickCount();

	vsdk::UMat lInput0_cvt = Image_cvt_apex.getUMat(cv::ACCESS_READ);
	vsdk::UMat lOutput0_cvt(height_tmp, width_tmp, VSDK_CV_8UC1); 
	apexcv::ColorConverter ColorConverter;
	lRetVal |= ColorConverter.Initialize(lInput0_cvt, apexcv::ColorConverter::eBGR888_TO_GREY,lOutput0_cvt);
	lRetVal |= ColorConverter.Process();
	
	Mat Image_cvt_apex_out = lOutput0_cvt.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T03 = getTickCount();
	//cout << "apexcv cvtColor is :" << (T04 - T03) * 1000 / getTickFrequency() << " ms!" << endl;
	double Tcvt_result_apex = (T03 - T02) * 1000 / getTickFrequency();
	Apex_opencv_time[1] = Tcvt_result_apex;
	#endif
	//imshow("Image_cvt",Image_cvt);//test and verify
	/////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	
	#if 1
	double Top_mor0 = getTickCount();
	Mat Image_morph;
	Mat All_element = getStructuringElement(MORPH_CROSS, Size(3, 3), Point(-1,-1));// 7 7//Size(1, 3)//Size(3, 3)
	//Mat All_element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1,-1));// 7 7//Size(1, 3)//Size(3, 3)	
	morphologyEx(Image_cvt, Image_morph, MORPH_OPEN, All_element, Point(-1,-1), 1, BORDER_CONSTANT);//MORPH_OPEN
	double Top_mor1 = getTickCount();
	double Top_result = (Top_mor1 - Top_mor0) * 1000 / getTickFrequency();
	Apex_opencv_time[2] = Top_result;

	double Tcl_mor0 = getTickCount();
	morphologyEx(Image_cvt, Image_morph, MORPH_CLOSE, All_element, Point(-1,-1), 2, BORDER_CONSTANT);//6  //MORPH_OPEN
	//imshow("Image_morph",Image_morph);//test and verify
	double Tcl_mor1 = getTickCount();
	//cout << "opencv Open and Close is :" << (T12 - T11) * 1000 / getTickFrequency() << " ms!" << endl;
	double Tcl_result = (Tcl_mor1 - Tcl_mor0) * 1000 / getTickFrequency();
	Apex_opencv_time[4] = Tcl_result;
	#endif

	#if 1
	double T05 = getTickCount();
	Mat Image_cvt_after_apex = Image_cvt.clone();
	//open one time
	vsdk::UMat lInput0K = Image_cvt_after_apex.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0K(height_tmp, width_tmp, VSDK_CV_8UC1);
	vsdk::UMat lOutput1K(height_tmp, width_tmp, VSDK_CV_8UC1);
	apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lInput0K, 3, lOutput0K);
	lRetVal |= ErodeFilter.Process();

	apexcv::DilateFilter  DilateFilter;
	lRetVal |= DilateFilter.Initialize(lOutput0K, 3, lOutput1K);
	lRetVal |= DilateFilter.Process();
	double T06 = getTickCount();
	double Top_result_apex = (T06 - T05) * 1000 / getTickFrequency();
	Apex_opencv_time[3] = Top_result_apex;
	

	//close two times
	double T07 = getTickCount();
	vsdk::UMat lOutput1C(height_tmp, width_tmp, VSDK_CV_8UC1);
	vsdk::UMat lOutput2C(height_tmp, width_tmp, VSDK_CV_8UC1);
	//apexcv::DilateFilter DilateFilter;
	lRetVal |= DilateFilter.Initialize(lOutput1K, 3, lOutput1C);
	lRetVal |= DilateFilter.Process();
	//apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lOutput1C, 3, lOutput2C);
	lRetVal |= ErodeFilter.Process();

	vsdk::UMat lOutput3C(height_tmp, width_tmp, VSDK_CV_8UC1);
	vsdk::UMat lOutput4C(height_tmp, width_tmp, VSDK_CV_8UC1);
	//apexcv::DilateFilter DilateFilter;
	lRetVal |= DilateFilter.Initialize(lOutput2C, 3, lOutput3C);
	lRetVal |= DilateFilter.Process();
	//apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lOutput3C, 3, lOutput4C);
	lRetVal |= ErodeFilter.Process();
	double T08 = getTickCount();
	//cout << "apexcv Open and close is :" << (T10 - T09) * 1000 / getTickFrequency() << " ms!" << endl;
	Apex_opencv_time[5] = (T08 - T07) * 1000 / getTickFrequency();

	Mat Image_cvt_after_apex_outK = lOutput4C.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	#endif

	Mat Image_apexcv = Image_morph.clone();
	
	#if 1
	//Mat Image_apexcv = Image_morph.clone();	
	double Tbilater0 = getTickCount();
	Mat Image_Filter;
	int i = 5;//9//10//5
	bilateralFilter (Image_morph, Image_Filter, i, i*2, i/2 );//bilateralFilter ( src, dst, i, i*2, i/2 );
	double Tbilater1 = getTickCount();
	Apex_opencv_time[6] = (Tbilater1 - Tbilater0) * 1000 / getTickFrequency();

	double Tcanny0 = getTickCount();
	Mat Image_canny;
	Canny(Image_Filter, Image_canny, 100, 210);//50  210//200  400
	double Tcanny1 = getTickCount();
	Apex_opencv_time[8] = (Tcanny1 - Tcanny0) * 1000 / getTickFrequency();
	//cout << "opencv bilaterlfilter is :" << (T1 - T0) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif

	
	#if 0
	int Width = Image_morph.cols;
	int Height = Image_morph.rows;
	Mat Image_apexcv = Image_Filter.clone();
		
	double T2 = getTickCount();
	//APEX-CV test
	//APEX_Init();
	int lRetVal = 0;
	vsdk::UMat lInput0 = Image_apexcv.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0(Height, Width, VSDK_CV_8UC1);
	apexcv::BilateralFilter testBilateralFilter;
	lRetVal |= testBilateralFilter.Initialize(lInput0, 3, 3, 3, lOutput0);
	lRetVal |= testBilateralFilter.Process();
	
	Mat Image_apexcv_bilateralFilter = lOutput0.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T3 = getTickCount();
	cout << "apexcv bilaterlfilter is :" << (T3 - T2) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif
	
	#if 1
	double T09 = getTickCount();
	vsdk::UMat lInput0 = Image_apexcv.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0(height_tmp, width_tmp, VSDK_CV_8UC1);
	apexcv::BilateralFilter testBilateralFilter;
	lRetVal |= testBilateralFilter.Initialize(lInput0, 3, 3, 3, lOutput0);
	lRetVal |= testBilateralFilter.Process();
	double T10 = getTickCount();
	Apex_opencv_time[7] = (T10 - T09) * 1000 / getTickFrequency();
	
	double T11 = getTickCount();
	apexcv::Canny Canny_apex;
	vsdk::UMat lOutput2(height_tmp, width_tmp, VSDK_CV_8UC1);
	lRetVal |= Canny_apex.Initialize(lOutput0, lOutput2,100,200);
	lRetVal |= Canny_apex.Process();
	double T12 = getTickCount();
	Apex_opencv_time[9] = (T12 - T11) * 1000 / getTickFrequency();
	
	Mat apex_out = lOutput2.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

	//cout << "apexcv bilaterlfilter and canny is :" << (T01 - T00) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif

	#if 1
	Mat Image_threshold_apex = Image_canny.clone();
	double Tthresh0 = getTickCount();
	//threshold(Image_canny,Image_threshold, 120, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);//CV_THRESH_OTSU  CV_THRESH_TRIANGLE
	adaptiveThreshold(Image_canny, Image_threshold, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 0);//ADAPTIVE_THRESH_MEAN_C  ADAPTIVE_THRESH_GAUSSIAN_C
	double Tthresh1 = getTickCount();
	Apex_opencv_time[10] = (Tthresh1 - Tthresh0) * 1000 / getTickFrequency();
	//cout << "opencv adaptiveThreshold is :" << (T06 - T05) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif
	
	#if 1
	double T13 = getTickCount();
	vsdk::UMat lInput0_thresh = Image_threshold_apex.getUMat(ACCESS_READ);
	//Threshold
	vsdk::UMat lOutput1_thresh(height_tmp, width_tmp, VSDK_CV_8UC1);
	apexcv:: Threshold  Threshold;
	lRetVal |= Threshold.Initialize(lInput0_thresh, 120 ,lOutput1_thresh);
	lRetVal |= Threshold.ReconnectIO(lInput0_thresh,lOutput1_thresh);
	lRetVal |= Threshold.Process();
	double T14 = getTickCount();
	Apex_opencv_time[11] = (T14 - T13) * 1000 / getTickFrequency();
	Mat Image_threshold_apex_out = lOutput1_thresh.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);;

	#endif
	
	//imshow("Image_threshold",Image_threshold);//test and verify
	
  return;
}


void Preprocess_bilateralFilter_threshold(Mat& image, Mat& Image_threshold)
{
	//APEX_Init();
	int lRetVal = 0;
	
	Mat Image_Tmp = image.clone();
	//convertScaleAbs(image, image, 1.2, 30);
	//imshow("image",image);//test and verify
	int width_tmp = Image_Tmp.cols;
	int height_tmp = Image_Tmp.rows;
	
	Mat Image_Tmp_apex = image.clone();
	#if 1
	double Tequa0 = getTickCount();
	equalizeHist_Mat(Image_Tmp);
	double Tequa1 = getTickCount();
	Apex_opencv_time[12] = (Tequa1 - Tequa0) * 1000 / getTickFrequency();
	//imshow("00",Image_Tmp);//test and verify
	#endif
	
	#if 1
	double T15 = getTickCount();
	vsdk::UMat lInput0E = Image_Tmp_apex.getUMat(ACCESS_READ);
    vsdk::UMat lOutput0E(Image_Tmp.rows, Image_Tmp.cols, VSDK_CV_8UC3);
	apexcv::HistogramEqualization HistogramEqualization;
	
	lRetVal |= HistogramEqualization.Initialize(lInput0E, lOutput0E);
	
	Mat Equlization_apex_result = lOutput0E.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T16 = getTickCount();
	Apex_opencv_time[13] = (T16 - T15) * 1000 / getTickFrequency();
	//cout << "apexcv equalize is :" << (T21 - T20) * 1000 / getTickFrequency() << " ms!" << endl;
	#endif

#if 1
	double minv = 0.0, maxv = 0.0;
	double* minp = &minv;
	double* maxp = &maxv;

	minMaxIdx(Image_Tmp,minp,maxp);
	//cout << "Mat minv = " << minv << endl;
	//cout << "Mat maxv = " << maxv << endl;

	int MAX = 255, MIN = 0;

	for (int i=0;i<Image_Tmp.rows;i++)
	{
		uchar* pdata=Image_Tmp.ptr<uchar>(i);
		
		for (int j=0;j<Image_Tmp.cols*Image_Tmp.channels();j++)
		{
			pdata[j] = (pdata[j] - MIN) * (MAX - MIN)/(maxv - minv) + MIN;
		}
	}
	//imshow("Image_Tmp",Image_Tmp);//test and verify

#endif

	
	Mat Image_cvt_apex = Image_Tmp.clone();
	double Tcvt0 = getTickCount();
	Mat Image_cvt;	
	cvtColor(Image_Tmp, Image_cvt, COLOR_BGR2GRAY,0);
	double Tcvt1 = getTickCount();
	//cout << "apexcv Open and close is :" << (Tcvt1 - Tcvt0) * 1000 / getTickFrequency() << " ms!" << endl;
	double Tcvt_result = (Tcvt1 - Tcvt0) * 1000 / getTickFrequency();
	Apex_opencv_time[0] = Tcvt_result;
	#if 1
	double T02 = getTickCount();

	vsdk::UMat lInput0_cvt = Image_cvt_apex.getUMat(cv::ACCESS_READ);
	vsdk::UMat lOutput0_cvt(height_tmp, width_tmp, VSDK_CV_8UC1); 
	apexcv::ColorConverter ColorConverter;
	lRetVal |= ColorConverter.Initialize(lInput0_cvt, apexcv::ColorConverter::eBGR888_TO_GREY,lOutput0_cvt);
	lRetVal |= ColorConverter.Process();
	
	Mat Image_cvt_apex_out = lOutput0_cvt.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T03 = getTickCount();
	//cout << "apexcv cvtColor is :" << (T04 - T03) * 1000 / getTickFrequency() << " ms!" << endl;
	double Tcvt_result_apex = (T03 - T02) * 1000 / getTickFrequency();
	Apex_opencv_time[1] = Tcvt_result_apex;
	#endif
	//imshow("Image_cvt",Image_cvt);//test and verify
	/////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	
	#if 1
	double Top_mor0 = getTickCount();
	Mat Image_morph;
	Mat All_element = getStructuringElement(MORPH_CROSS, Size(3, 3), Point(-1,-1));// 7 7//Size(1, 3)//Size(3, 3)
	//Mat All_element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1,-1));// 7 7//Size(1, 3)//Size(3, 3)	
	morphologyEx(Image_cvt, Image_morph, MORPH_OPEN, All_element, Point(-1,-1), 1, BORDER_CONSTANT);//MORPH_OPEN
	double Top_mor1 = getTickCount();
	double Top_result = (Top_mor1 - Top_mor0) * 1000 / getTickFrequency();
	Apex_opencv_time[2] = Top_result;

	double Tcl_mor0 = getTickCount();
	morphologyEx(Image_cvt, Image_morph, MORPH_CLOSE, All_element, Point(-1,-1), 2, BORDER_CONSTANT);//6  //MORPH_OPEN
	//imshow("Image_morph",Image_morph);//test and verify
	double Tcl_mor1 = getTickCount();
	//cout << "opencv Open and Close is :" << (T12 - T11) * 1000 / getTickFrequency() << " ms!" << endl;
	double Tcl_result = (Tcl_mor1 - Tcl_mor0) * 1000 / getTickFrequency();
	Apex_opencv_time[4] = Tcl_result;
	#endif

	#if 1
	double T05 = getTickCount();
	Mat Image_cvt_after_apex = Image_cvt.clone();
	//open one time
	vsdk::UMat lInput0K = Image_cvt_after_apex.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0K(height_tmp, width_tmp, VSDK_CV_8UC1);
	vsdk::UMat lOutput1K(height_tmp, width_tmp, VSDK_CV_8UC1);
	apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lInput0K, 3, lOutput0K);
	lRetVal |= ErodeFilter.Process();

	apexcv::DilateFilter  DilateFilter;
	lRetVal |= DilateFilter.Initialize(lOutput0K, 3, lOutput1K);
	lRetVal |= DilateFilter.Process();
	double T06 = getTickCount();
	double Top_result_apex = (T06 - T05) * 1000 / getTickFrequency();
	Apex_opencv_time[3] = Top_result_apex;
	

	//close two times
	double T07 = getTickCount();
	vsdk::UMat lOutput1C(height_tmp, width_tmp, VSDK_CV_8UC1);
	vsdk::UMat lOutput2C(height_tmp, width_tmp, VSDK_CV_8UC1);
	//apexcv::DilateFilter DilateFilter;
	lRetVal |= DilateFilter.Initialize(lOutput1K, 3, lOutput1C);
	lRetVal |= DilateFilter.Process();
	//apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lOutput1C, 3, lOutput2C);
	lRetVal |= ErodeFilter.Process();

	vsdk::UMat lOutput3C(height_tmp, width_tmp, VSDK_CV_8UC1);
	vsdk::UMat lOutput4C(height_tmp, width_tmp, VSDK_CV_8UC1);
	//apexcv::DilateFilter DilateFilter;
	lRetVal |= DilateFilter.Initialize(lOutput2C, 3, lOutput3C);
	lRetVal |= DilateFilter.Process();
	//apexcv::ErodeFilter ErodeFilter;
	lRetVal |= ErodeFilter.Initialize(lOutput3C, 3, lOutput4C);
	lRetVal |= ErodeFilter.Process();
	double T08 = getTickCount();
	//cout << "apexcv Open and close is :" << (T10 - T09) * 1000 / getTickFrequency() << " ms!" << endl;
	Apex_opencv_time[5] = (T08 - T07) * 1000 / getTickFrequency();

	Mat Image_cvt_after_apex_outK = lOutput4C.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	#endif

	Mat Image_apexcv = Image_morph.clone();
	
	#if 1
	//Mat Image_apexcv = Image_morph.clone();	
	double Tbilater0 = getTickCount();
	Mat Image_Filter;
	int i = 5;//9//10//5
	bilateralFilter (Image_morph, Image_Filter, i, i*2, i/2 );//bilateralFilter ( src, dst, i, i*2, i/2 );
	double Tbilater1 = getTickCount();
	Apex_opencv_time[6] = (Tbilater1 - Tbilater0) * 1000 / getTickFrequency();

	double Tcanny0 = getTickCount();
	Mat Image_canny;
	Canny(Image_Filter, Image_canny, 100, 210);//50  210//200  400
	double Tcanny1 = getTickCount();
	Apex_opencv_time[8] = (Tcanny1 - Tcanny0) * 1000 / getTickFrequency();
	//cout << "opencv bilaterlfilter is :" << (T1 - T0) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif

	
	#if 0
	int Width = Image_morph.cols;
	int Height = Image_morph.rows;
	Mat Image_apexcv = Image_Filter.clone();
		
	double T2 = getTickCount();
	//APEX-CV test
	//APEX_Init();
	int lRetVal = 0;
	vsdk::UMat lInput0 = Image_apexcv.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0(Height, Width, VSDK_CV_8UC1);
	apexcv::BilateralFilter testBilateralFilter;
	lRetVal |= testBilateralFilter.Initialize(lInput0, 3, 3, 3, lOutput0);
	lRetVal |= testBilateralFilter.Process();
	
	Mat Image_apexcv_bilateralFilter = lOutput0.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T3 = getTickCount();
	cout << "apexcv bilaterlfilter is :" << (T3 - T2) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif
	
	#if 1
	double T09 = getTickCount();
	vsdk::UMat lInput0 = Image_apexcv.getUMat(ACCESS_READ);
	vsdk::UMat lOutput0(height_tmp, width_tmp, VSDK_CV_8UC1);
	apexcv::BilateralFilter testBilateralFilter;
	lRetVal |= testBilateralFilter.Initialize(lInput0, 3, 3, 3, lOutput0);
	lRetVal |= testBilateralFilter.Process();
	double T10 = getTickCount();
	Apex_opencv_time[7] = (T10 - T09) * 1000 / getTickFrequency();
	
	double T11 = getTickCount();
	apexcv::Canny Canny_apex;
	vsdk::UMat lOutput2(height_tmp, width_tmp, VSDK_CV_8UC1);
	lRetVal |= Canny_apex.Initialize(lOutput0, lOutput2,100,200);
	lRetVal |= Canny_apex.Process();
	double T12 = getTickCount();
	Apex_opencv_time[9] = (T12 - T11) * 1000 / getTickFrequency();
	
	Mat apex_out = lOutput2.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

	//cout << "apexcv bilaterlfilter and canny is :" << (T01 - T00) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif

	#if 1
	Mat Image_threshold_apex = Image_canny.clone();
	double Tthresh0 = getTickCount();
	//threshold(Image_canny,Image_threshold, 120, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);//CV_THRESH_OTSU  CV_THRESH_TRIANGLE
	adaptiveThreshold(Image_canny, Image_threshold, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 0);//ADAPTIVE_THRESH_MEAN_C  ADAPTIVE_THRESH_GAUSSIAN_C
	double Tthresh1 = getTickCount();
	Apex_opencv_time[10] = (Tthresh1 - Tthresh0) * 1000 / getTickFrequency();
	//cout << "opencv adaptiveThreshold is :" << (T06 - T05) * 1000 / getTickFrequency() << " ms!" << endl;//
	#endif
	
	#if 1
	double T13 = getTickCount();
	vsdk::UMat lInput0_thresh = Image_threshold_apex.getUMat(ACCESS_READ);
	//Threshold
	vsdk::UMat lOutput1_thresh(height_tmp, width_tmp, VSDK_CV_8UC1);
	apexcv:: Threshold  Threshold;
	lRetVal |= Threshold.Initialize(lInput0_thresh, 120 ,lOutput1_thresh);
	lRetVal |= Threshold.ReconnectIO(lInput0_thresh,lOutput1_thresh);
	lRetVal |= Threshold.Process();
	double T14 = getTickCount();
	Apex_opencv_time[11] = (T14 - T13) * 1000 / getTickFrequency();
	Mat Image_threshold_apex_out = lOutput1_thresh.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);;

	#endif
	
	//imshow("Image_threshold",Image_threshold);//test and verify
	
  return;
}

void Globel_Process_Ldw_Mat(Mat& Src_imge){
	//cout << " 00 " << endl;
#if 1
	//Presetting parameter
	vector<Point2f> src_lane_points(4);
	vector<Point2f> dst_lane_perspective_points(4);

	src_lane_points[0] = Point2f(150.85, 360);
	src_lane_points[1] = Point2f(269, 250);
	src_lane_points[2] = Point2f(329.71, 250);
	src_lane_points[3] = Point2f(486.86, 360);

	dst_lane_perspective_points[0] = Point2f(195, 360);
	dst_lane_perspective_points[1] = Point2f(195, 0);
	dst_lane_perspective_points[2] = Point2f(424, 0);
	dst_lane_perspective_points[3] = Point2f(424, 360);

	vector<uchar> inliers_Lane_convert(src_lane_points.size());
 	Mat Transform_Matrix_Lane_convert = findHomography(src_lane_points, dst_lane_perspective_points, RANSAC, 0.1, inliers_Lane_convert);
	Mat Transform_Matrix_Lane_convert_inconvert = findHomography(dst_lane_perspective_points, src_lane_points, RANSAC, 0.1, inliers_Lane_convert);

#endif
	//cout << " 11 " << endl;

#if 1	
	//Presetting
	Rect Lane_section_roi(143.5, 250, 350, 110);//using
	Rect Perspective_Lane_section_roi(160 - 20, 200, 320 + 2 * 20, 120);//using bf
	
	//Preprocess Model
	Mat Src_imge_process = Src_imge.clone();

	//cout << " 1100 " << endl;
	
	Mat SourceImg_Lane_section = Src_imge_process(Lane_section_roi);//bf  20190308 16:14
	//imshow("SourceImg", SourceImg);//test and varify

	//cout << " 1101 " << endl;
	
	//Preprocess to reduce invalid area of image
	Mat Mask = Mat::zeros(Src_imge.size(),CV_8UC3);
	Mat Lane_Paste_Mask_MAT = Mask(Lane_section_roi);
	SourceImg_Lane_section.copyTo(Lane_Paste_Mask_MAT, SourceImg_Lane_section);
	//imshow("Mask", Mask);//test and varify

	//perspective Process Model
	Mat Perspective_convert_Lane;
	Perspective_process(Mask, Transform_Matrix_Lane_convert, Perspective_convert_Lane);
	//imshow("Perspective_convert_Lane", Perspective_convert_Lane);//test and varify

	//cout << " 1102 " << endl;

	//Curve fit and calculate Model
	Mat Perspective_Lane_Process_MAT = Perspective_convert_Lane(Perspective_Lane_section_roi);

	//Preprocess to threshold the perspective image Model
	Mat Perspective_Lane_Process_MAT_SRC;
	//Preprocess_bilateralFilter_threshold(Perspective_Lane_Process_MAT, Perspective_Lane_Process_MAT_SRC);//test and verify temperorily 20190222
	//Preprocess_bilateralFilter_threshold_APEX_CV(Perspective_Lane_Process_MAT, Perspective_Lane_Process_MAT_SRC);
	Preprocess_bilateralFilter_threshold_Calculate_time(Perspective_Lane_Process_MAT, Perspective_Lane_Process_MAT_SRC);

	//imshow("Perspective_Lane_Process_MAT_SRC", Perspective_Lane_Process_MAT_SRC);//test and varify

	//cout << " 1103 " << endl;

	//curve calculate Model
	Mat First_and_second_and_third_window_Mask = Mat::zeros(Perspective_Lane_Process_MAT_SRC.size(),CV_8UC3);
#endif
	//cout << " 22 " << endl;

#if 1 //must be located here, not to delete
	//Curve fitting to calculate the radius of curve
	Mat Third_window_Curve_calculate_addtional_curve_fitting;
	//vector<float> Current_Fitting_X_Point_kalman;
	vector<int> Lane_flag(2);
	Third_window_Curve_calculate_addtional_curve_fitting = Curve_fit_and_calculate_radius_of_curvature_choices(Perspective_Lane_Process_MAT_SRC, Lane_flag);
	//imshow("addtional", Third_window_Curve_calculate_addtional_curve_fitting);//test and varify

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	
	vector<Point> L_Left_kalmanx_display;
	vector<Point> R_Left_kalmanx_display;
	vector<Point> Left_kalmanx_display;
	
	vector<Point> L_Right_kalmanx_display;
	vector<Point> Right_kalmanx_display;
	vector<Point> R_Right_kalmanx_display;
	
	#if 1
	int Departure_location = 10;
	//left correct
	for(int i = 0; i < 3; i++){
		int A = i*2;
		int B = i*2 + 1;
		Left_kalmanx_display.push_back(Point(Correct_X_Point_kalman[A], Correct_X_Point_kalman[B]));
		L_Left_kalmanx_display.push_back(Point(Correct_X_Point_kalman[A] - Departure_location, Correct_X_Point_kalman[B]));
		R_Left_kalmanx_display.push_back(Point(Correct_X_Point_kalman[A] + Departure_location, Correct_X_Point_kalman[B]));
	}
	//right correct
	for(int i = 0; i < 3; i++){
		int A = i*2;
		int B = i*2 + 1;
		Right_kalmanx_display.push_back(Point(right_Correct_X_Point_kalman[A], right_Correct_X_Point_kalman[B]));
		L_Right_kalmanx_display.push_back(Point(right_Correct_X_Point_kalman[A] - Departure_location, right_Correct_X_Point_kalman[B]));
		R_Right_kalmanx_display.push_back(Point(right_Correct_X_Point_kalman[A] + Departure_location, right_Correct_X_Point_kalman[B]));

	}

	#endif

	#if 1
	double right_x = right_Current_Fitting_X_Point_kalman[4];//right_Current_Fitting_X_Point_kalman  //right_Correct_X_Point_kalman
	double left_x = Current_Fitting_X_Point_kalman[4];
	Right_sub_Left_X_pixel_width = right_x - left_x;
	//cout << "D-value:  " << right_Correct_X_Point_kalman[4] - Correct_X_Point_kalman[4] << endl;//206.5 = 3570
	//lane departure process
	double Median_location_X = (right_x + left_x) / 2;
	double Left_lane_distance = Median_location_X - left_x;
	double Right_lane_distance = right_x - Median_location_X;
	
	int Departure_left_flag;
	int Departure_right_flag;
	if(Left_lane_distance < Warning_width)
		Departure_left_flag = 1;
	if(Right_lane_distance < Warning_width)
		Departure_right_flag = 1;
	#endif
	Scalar scalar(136, 218, 230);//30, 250, 230//60, 170, 250//225, 75, 225//230, 250, 50
	int Line_thick = 8;
	//display left lane
	if(Lane_flag[0] != 0){
		//departure display
		if(Departure_left_flag == 1){
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(0, 0, 255), 8, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);

			Left_warning_time_accumulate++;
			if(Left_warning_time_accumulate == Warning_max_times){
				Left_Warning_sign(Src_imge);
				Left_warning_time_accumulate = 0;
			}
			Departure_left_flag = 0;
		}
		else{
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(0, 255, 0), 8, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
		}
	}
	else{
		#if 0
		//departure display
		if(Departure_left_flag == 1){
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(0, 0, 255), 8, 16, 0);
			Left_Warning_sign(Src_imge);
			Departure_left_flag = 0;
		}
		else{
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(0, 255, 255), 8, 16, 0);
		}
		#endif
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, Left_kalmanx_display, false, Scalar(250, 50, 80), 8, 16, 0);//0, 255, 255
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Left_kalmanx_display, false, scalar, Line_thick, 16, 0);
	}

	//display right lane
	if(Lane_flag[1] != 0){	
		//departure display
		if(Departure_right_flag == 1){
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(0, 0, 255), 8, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
			
			Right_warning_time_accumulate++;
			if(Right_warning_time_accumulate == Warning_max_times){
				Right_Warning_sign(Src_imge);
				Right_warning_time_accumulate = 0;
			}
			Departure_right_flag = 0;
		}
		else{
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(0, 255, 0), 8, 16, 0);
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
		}
	}
	else{
		#if 0
		//departure display
		if(Departure_right_flag == 1){
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(0, 0, 255), 8, 16, 0);
			Right_Warning_sign(Src_imge);
			Departure_right_flag = 0;
		}
		else{
			polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(0, 255, 255), 8, 16, 0);
		}
		#endif
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, L_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, Right_kalmanx_display, false, Scalar(250, 50, 80), 8, 16, 0);//0, 255, 255
		polylines(Third_window_Curve_calculate_addtional_curve_fitting, R_Right_kalmanx_display, false, scalar, Line_thick, 16, 0);
		
	}
	
	//cout << " 33 " << endl;
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

	Third_window_Curve_calculate_addtional_curve_fitting.copyTo(First_and_second_and_third_window_Mask, Third_window_Curve_calculate_addtional_curve_fitting);
	//imshow("after addtional", First_and_second_and_third_window_Mask);//test and varify
	//imshow("Perspective_Lane_Process_MAT_threshold", Perspective_Lane_Process_MAT_threshold);//test and varify

	rectangle(First_and_second_and_third_window_Mask, Point(0, 0), Point(First_and_second_and_third_window_Mask.cols, First_and_second_and_third_window_Mask.rows), Scalar(0, 0, 255), 8, 16, 0);
	line(First_and_second_and_third_window_Mask, Point(First_and_second_and_third_window_Mask.cols / 2, 0), Point(First_and_second_and_third_window_Mask.cols / 2, First_and_second_and_third_window_Mask.rows),
				Scalar(0, 0, 255), 2, 4);
#endif

	//cout << " 44 " << endl;

	//Mask to process the data of lane detected
	Mat First_and_second_third_window_Lane_Mask = Mat::zeros(Perspective_convert_Lane.size(),CV_8UC3);
	Mat First_and_second_window_Lane_Mask_inconvert = First_and_second_third_window_Lane_Mask(Perspective_Lane_section_roi);
	First_and_second_and_third_window_Mask.copyTo(First_and_second_window_Lane_Mask_inconvert, First_and_second_and_third_window_Mask);
	//imshow("First_and_second_window_Lane_Mask", First_and_second_window_Lane_Mask);//test and varify

	//Inconveret perspective Process Model
	Mat Inconvert_Perspective_First_and_second_window_Mask;
	Perspective_process(First_and_second_third_window_Lane_Mask, Transform_Matrix_Lane_convert_inconvert, Inconvert_Perspective_First_and_second_window_Mask);
	//imshow("Inconvert_Perspective_First_and_second_window_Mask", Inconvert_Perspective_First_and_second_window_Mask);//test and varify

	//cout << " 55 " << endl;
	
	Inconvert_Perspective_First_and_second_window_Mask.copyTo(Src_imge, Inconvert_Perspective_First_and_second_window_Mask);
	//imshow("Src_imge", Src_imge);

	return;
}

Mat Curve_fit_and_calculate_radius_of_curvature_choices(Mat& Image_src_process, vector<int>& Lane_flag){

	//presetting parameters of windows
	int image_process_std_col = Image_src_process.cols;
	int image_process_std_row = Image_src_process.rows;
	//int No_window = 5, Margin = 30, Window_Min_Nonezero_Pixel_Num = 30;
	int No_window = 5, Window_Min_Nonezero_Pixel_Num = 30;
	
	double Appointed_y = image_process_std_col / 2;
	//imshow("original",Image_src_process);//test and verify

	#if 1
	Mat Image_src_process_third = Image_src_process.clone();
	Mat Image_src_process_third_Mask = Mat::zeros(Image_src_process_third.size(),CV_8UC1);
	vector<vector<Point>> G_contours;
	findContours(Image_src_process_third, G_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	//cout << "G_contours is " << G_contours_num << endl;
	
	for(size_t i = 0; i < G_contours.size(); i++){

		double length = arcLength(G_contours[i], true);
		//cout << "length--" << i << "---" << length << endl;

         if((length < 300) && (length > 20)){ 
		 //if((length < 300) && (length > 20) && (G_contours_num < 15) && (G_contours_num > 0)){ 
             drawContours(Image_src_process_third_Mask, G_contours, i, Scalar(255), 2, 2);
         }
		
	}
	
	//drawContours(Image_src_process_third_Mask, G_contours, -1, Scalar(255), 1);
	//imshow("222", Image_src_process_third_Mask);//test and verify
	int x_start = image_process_std_col / 2 - 40;
	int x_distance = image_process_std_col - x_start * 2;
	int y_start = 40;
	int y_distance = image_process_std_row - y_start;
	
	Mat ROI = Image_src_process_third_Mask(Rect(x_start, y_start, x_distance, y_distance));
	ROI = {Scalar(0)};
	//imshow("333", Image_src_process_third_Mask);//test and verify
	
	#endif
	
	#if 1
	//calculate right and left contours's number
	Rect Left_contour(0, 0, image_process_std_col / 2, image_process_std_row);
	Rect Right_contour(image_process_std_col / 2, 0, image_process_std_col / 2, image_process_std_row);
	Mat Left_mat = Image_src_process_third_Mask(Left_contour);
	Mat Right_mat = Image_src_process_third_Mask(Right_contour);

	//imshow("Left_mat", Left_mat);//test and verify
	//imshow("Right_mat", Right_mat);//test and verify

	vector<vector<Point>> Left_contours;
	vector<vector<Point>> Right_contours;
	//vector<vector<Point>> Global_contours;

	//findContours(Image_src_process, Global_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	findContours(Left_mat, Left_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	findContours(Right_mat, Right_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	
	//int Global_Contours_num = Global_contours.size();
	int Left_Contours_num = Left_contours.size();
	int Right_Contours_num = Right_contours.size();
	//cout << "Left_Contours_num is " << Left_Contours_num << endl;
	//cout << "Right_Contours_num is " << Right_Contours_num << endl;

	Lane_flag[0] = Left_Contours_num;
	Lane_flag[1] = Right_Contours_num;
	//cout << "zong - is " << Global_Contours_num - Left_Contours_num - Right_Contours_num << endl;

	#endif

	#if 0
	//test and verify Mat
	Mat Image_src_process_tmp = Image_src_process_third_Mask.clone();
	cvtColor(Image_src_process_tmp, Image_src_process_tmp, COLOR_GRAY2BGR, 0);
	//imshow("Image_src_process_tmp",Image_src_process_tmp);
	#endif
#if 1

	//preprocess section
	Mat Image_src_process_copy_display = Mat::zeros(Image_src_process_third_Mask.size(),CV_8UC3);
	//imshow("Image_src_process_copy_display",Image_src_process_copy_display);//test and verify

	//Mat convert to vector
	Mat Image_threshold_datatype_convert(image_process_std_col, image_process_std_row, CV_64FC1);
	Image_src_process_third_Mask.convertTo(Image_threshold_datatype_convert, CV_64FC1);
	//cout << Image_threshold_datatype_convert << endl;
	Mat histogram(1, image_process_std_col, CV_64FC1);
	reduce(Image_threshold_datatype_convert, histogram, 0, CV_REDUCE_SUM);//, CV_64F);
	//cout << histogram << endl;//test and verify

	//one vector is interrupted to be two sections that left and right ones
	vector<double> Mat_to_vector_Left;
	Mat_to_vector_Left = Mat_<double>(histogram);
	vector<double> Mat_to_vector_Right;
	Mat_to_vector_Right.assign(Mat_to_vector_Left.begin(), Mat_to_vector_Left.end());

	Mat_to_vector_Left.erase(Mat_to_vector_Left.begin() + image_process_std_col / 2, Mat_to_vector_Left.end()); //cout << "new Mat_to_vector_Left size is " << Mat_to_vector_Left.size() << endl;
	Mat_to_vector_Right.erase(Mat_to_vector_Right.begin(), Mat_to_vector_Right.end() - image_process_std_col / 2);

	vector<double>::iterator Left_biggest = max_element(begin(Mat_to_vector_Left), end(Mat_to_vector_Left));
	vector<double>::iterator Right_biggest = max_element(begin(Mat_to_vector_Right), end(Mat_to_vector_Right));
    int Left_startPoint_index = distance(begin(Mat_to_vector_Left), Left_biggest);
    int Right_startPoint_index = distance(begin(Mat_to_vector_Right), Right_biggest) + image_process_std_col / 2;
	//test and verify
	//cout << "Left_startPoint_index is " << Left_startPoint_index << "  Right_startPoint_index is " << Right_startPoint_index + image_process_std_col / 2 << endl;//test and verify

	vector<Point> NonZero_Locations;//error version: vector< vector<float> > NonZero_Locations;//test and verify
	findNonZero(Image_src_process_third_Mask, NonZero_Locations);
	int NonZero_Locations_Size = NonZero_Locations.size();

#endif

#if 1
	//int No_window = 5, Margin = 100, Window_Min_Nonezero_Pixel_Num = 50;
	int Window_height = image_process_std_row / No_window;

	int Leftx_current = Left_startPoint_index;
	int Rightx_current = Right_startPoint_index;
	//cout << "Rightx_current is " << Rightx_current << endl;
	//cout << endl;
	//cout << endl;
	#if 1
	//initialization the region in the window of zero
	int Half = (int)(Window_Min_Nonezero_Pixel_Num / Window_height) + 1;
	//ut << "zuo_half " << Half << endl;//test
	//ut << "total - Window_Min_Nonezero_Pixel_Num is " << Half * 2 * Window_height - Window_Min_Nonezero_Pixel_Num << endl;//test
	int Y_max = image_process_std_row;
	int Y_min = image_process_std_row - Window_height;

	//left lane initialization with locations of x and y
	int Left_x_current_zero_window = Leftx_current;
	int Left_X_max = Left_x_current_zero_window + Half;
	int Left_X_min = Left_x_current_zero_window - Half;
	
	//cout << "Left_x_current_zero_window is " << Left_x_current_zero_window << "   " << Half << "  " << Window_height << endl;
	vector<Point> Zero_window_Left_initialization_vector;
	for(int i = Left_X_min; i < Left_X_max; i++){
		for(int j = Y_min; j < Y_max; j++){
			Zero_window_Left_initialization_vector.push_back(Point(i, j)); 
			//cout << i << "," << j << "   ";
		}
		//cout << endl;
	}

	//right lane initialization with locations of x and y
	int Right_x_current_zero_window = Rightx_current;
	int Right_X_max = Right_x_current_zero_window + Half;
	int Right_X_min = Right_x_current_zero_window - Half;

	//cout << Rightx_current << "   " << Half << endl;
	vector<Point> Zero_window_Right_initialization_vector;
	for(int i = Right_X_min; i < Right_X_max; i++){
		for(int j = Y_min; j < Y_max; j++){
			Zero_window_Right_initialization_vector.push_back(Point(i, j)); 
			//cout << i << "," << j << "   ";
		}
	}

	#endif

	vector<Point> Last_Left_lane_indices;
	vector<Point> Last_Right_lane_indices;
	vector<Point> Left_lane_indices, Right_lane_indices;
	vector<Point> Left_lane_indices_to_fit, Right_lane_indices_to_fit;

	int Win_y_low, Win_y_high, Win_left_X_low, Win_left_X_high, Win_right_X_low, Win_right_X_high;
	int Left_lane_indices_Number, Right_lane_indices_Number;
	//cout << endl;
	//cout << endl;
	//cout << "start" << endl;
	for(int Window = 0; Window < No_window; Window++){
		//cout << "window is " << Window << endl;

		int Leftx_current_last = Leftx_current;
		int Rightx_current_last = Rightx_current;

		int Left_Margin = Leftx_current;
		int Right_Margin = image_process_std_col - Rightx_current;
		
		Win_y_low = image_process_std_row - (Window + 1) * Window_height;
		Win_y_high = image_process_std_row - Window * Window_height;
		//cout << Win_y_high << "  " << Win_y_low << endl;//test and verify
		#if 1
		Win_left_X_low = Leftx_current - Left_Margin;
		Win_left_X_high = Leftx_current + Left_Margin;
		Win_right_X_low = Rightx_current - Right_Margin;
		Win_right_X_high = Rightx_current + Right_Margin;
		#endif

		
		#if 0//bf 2019-03-16-11-39
		Win_left_X_low = Leftx_current - Margin;
		Win_left_X_high = Leftx_current + Margin;
		Win_right_X_low = Rightx_current - Margin;
		Win_right_X_high = Rightx_current + Margin;
		#endif

		#if 0
		//display windows from bottom to top
		rectangle(Image_src_process_tmp, Point(Win_left_X_low, Win_y_low), Point(Win_left_X_high, Win_y_high), Scalar(0, 255, 0), 2, 4, 0);
		rectangle(Image_src_process_tmp, Point(Win_right_X_low, Win_y_low), Point(Win_right_X_high, Win_y_high), Scalar(0, 0, 255), 2, 4, 0);
		//imshow("Image_src_process_tmp",Image_src_process_tmp);//test and verify
		#endif
		//cout << "Left_lane_indices in the beginning is " << Left_lane_indices << endl;//test and verify
		for(int i = 0; i < NonZero_Locations_Size; i++){
			if((NonZero_Locations[i].y > Win_y_low) && (NonZero_Locations[i].y < Win_y_high)){
				//left none_zero pixels's location
				if((NonZero_Locations[i].x > Win_left_X_low) && (NonZero_Locations[i].x < Win_left_X_high)){
					Left_lane_indices.push_back(NonZero_Locations[i]);
					//circle(image_copy, Point(NonZero_Locations[i].x, NonZero_Locations[i].y), 4, Scalar(255, 0, 0));//test and verify
				}

				//right none_zero pixels's location
				if((NonZero_Locations[i].x > Win_right_X_low) && (NonZero_Locations[i].x < Win_right_X_high)){
					Right_lane_indices.push_back(NonZero_Locations[i]);
					//circle(image_copy, Point(NonZero_Locations[i].x, NonZero_Locations[i].y), 4, Scalar(255, 0, 0));//test and verify
				}
			}
		}

		NonZero_Locations.clear();

		Left_lane_indices_Number = Left_lane_indices.size();
		Right_lane_indices_Number = Right_lane_indices.size();
	
		//cout << "original Left_lane_indices_Number is " << Left_lane_indices_Number<< endl;
#if 1
//process the window of right lane being empty
		if(0 == Window){
			if(0 == Left_lane_indices_Number){
				Left_lane_indices.assign(Zero_window_Left_initialization_vector.begin(), Zero_window_Left_initialization_vector.end());//test
				Last_Left_lane_indices.assign(Zero_window_Left_initialization_vector.begin(), Zero_window_Left_initialization_vector.end());//test
			}
			else{
				Last_Left_lane_indices.assign(Left_lane_indices.begin(), Left_lane_indices.end());//test
			}
		}
		else{
		//else if(1 == Window){
			if(0 == Left_lane_indices_Number){
				//cout << "inner last left is " << Last_Left_lane_indices.size() << endl;
				int Last_Left_lane_indices_number = Last_Left_lane_indices.size();

				Left_lane_indices.assign(Last_Left_lane_indices.begin(), Last_Left_lane_indices.end());//test
				
				for(int i = 0; i < Last_Left_lane_indices_number; i++){
					Left_lane_indices[i].y = Last_Left_lane_indices[i].y + Window_height;

				}
				//cout << "inner right is " << Right_lane_indices.size() << endl;

				Last_Left_lane_indices.assign(Left_lane_indices.begin(), Left_lane_indices.end());//test
				
			}

		}

#endif

#if 1
		//process the window of right lane being empty
		if(0 == Window){
			if(0 == Right_lane_indices_Number){
				Right_lane_indices.assign(Zero_window_Right_initialization_vector.begin(), Zero_window_Right_initialization_vector.end());//test
				Last_Right_lane_indices.assign(Zero_window_Right_initialization_vector.begin(), Zero_window_Right_initialization_vector.end());//test
			}
			else{
				Last_Right_lane_indices.assign(Right_lane_indices.begin(), Right_lane_indices.end());//test
			}
		}
		else{
		//else if(1 == Window){
			if(0 == Right_lane_indices_Number){
				//cout << "inner last right is " << Last_Right_lane_indices.size() << endl;
				int Last_Right_lane_indices_number = Last_Right_lane_indices.size();

				Right_lane_indices.assign(Last_Right_lane_indices.begin(), Last_Right_lane_indices.end());//test
				
				for(int i = 0; i < Last_Right_lane_indices_number; i++){
					Right_lane_indices[i].y = Last_Right_lane_indices[i].y + Window_height;

				}
				//cout << "inner right is " << Right_lane_indices.size() << endl;

				Last_Right_lane_indices.assign(Right_lane_indices.begin(), Right_lane_indices.end());//test
				
			}

		}

#endif


		Left_lane_indices_to_fit.insert(Left_lane_indices_to_fit.end(), Left_lane_indices.begin(), Left_lane_indices.end());
		Right_lane_indices_to_fit.insert(Right_lane_indices_to_fit.end(), Right_lane_indices.begin(), Right_lane_indices.end());

		vector<int> Left_NonZero_vector_x, Right_NonZero_vector_x;
		for(int i = 0; i < Left_lane_indices_Number; i++){
			Left_NonZero_vector_x.push_back(Left_lane_indices[i].x);
		}

		int Right_lane_indices_Number_new = Right_lane_indices.size();
		
		//cout << "new right is " << Right_lane_indices_Number_new << endl;
		for(int i = 0; i < Right_lane_indices_Number_new; i++){
			Right_NonZero_vector_x.push_back(Right_lane_indices[i].x);
		}
		//cout << "one is " << Left_NonZero_vector_x.size() << "  " << Right_NonZero_vector_x.size() << endl;//test and verify

		Left_lane_indices.clear();
		Right_lane_indices.clear();
		
		//cout << "Left_lane_indices.size is " << Left_lane_indices.size() << endl;	//test and verify

		if(0 == Left_lane_indices_Number){
			//cout << "here is none " << endl;//test and verify
			Leftx_current = Leftx_current_last;
		}
		else if(Left_lane_indices_Number > Window_Min_Nonezero_Pixel_Num){
				int Leftx_current_sum = accumulate(begin(Left_NonZero_vector_x), end(Left_NonZero_vector_x), 0.0);
				//cout << "Leftx_current_sum is " << Leftx_current_sum << endl;//test and verify
				Leftx_current = (int)(Leftx_current_sum / Left_lane_indices_Number);
				//cout << "Leftx_current is " << Leftx_current << endl;//test and verify
		}

		if(0 == Right_lane_indices_Number){
				//cout << "here is none " << endl;//test and verify
				Rightx_current = Rightx_current_last;
		}
		else if(Right_lane_indices_Number > Window_Min_Nonezero_Pixel_Num){
				int Rightx_current_sum = accumulate(begin(Right_NonZero_vector_x), end(Right_NonZero_vector_x), 0.0);
				//cout << "Rightx_current_sum is " << Rightx_current_sum << endl;//test and verify
				Rightx_current = (int)(Rightx_current_sum / Right_lane_indices_Number);
				//cout << "Rightx_current is " << Rightx_current << endl;//test and verify
		}

		Left_NonZero_vector_x.clear();
		Right_NonZero_vector_x.clear();
		

	}
	Zero_window_Right_initialization_vector.clear();
	Zero_window_Left_initialization_vector.clear();
#endif	

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

#if 1
	//preprocess left data of nonzero pixels of lane

	vector<Point> Left_points_fitted_display;
	int left_num_tmp = Left_lane_indices_to_fit.size();
	//if(left_num_tmp != 0){

	vector<Point> Left_lane_indices_to_fit_X_TO_Y(left_num_tmp);
	for(int i =0; i < left_num_tmp; i++){
		//circle(image_copy, Point(Left_lane_indices_to_fit[i].x, Left_lane_indices_to_fit[i].y), 4, Scalar(0, 255, 0));//test and verify
		int src_y = Left_lane_indices_to_fit[i].y;
		int src_x = Left_lane_indices_to_fit[i].x;
		Left_lane_indices_to_fit_X_TO_Y[i].x = src_y;
		Left_lane_indices_to_fit_X_TO_Y[i].y = src_x;
		//cout << Left_lane_indices_to_fit_X_TO_Y[i].x << "  " << Left_lane_indices_to_fit_X_TO_Y[i].y << "   " << Left_lane_indices_to_fit[i].x << "  " << Left_lane_indices_to_fit[i].y << endl;//test and verify
	}

	Left_lane_indices_to_fit.clear();

	//Left lane fitting section
	Mat Left_A;
	polynomial_curve_fit(Left_lane_indices_to_fit_X_TO_Y, 2, Left_A);
	Left_lane_indices_to_fit_X_TO_Y.clear();

	double Left_Equation_c = Left_A.at<double>(0, 0);
	double Left_Equation_b = Left_A.at<double>(1, 0);
	double Left_Equation_a = Left_A.at<double>(2, 0);

	#if 0//bf
	vector< double > Left_x_calculate_vector;
	for (int y = 0; y < image_process_std_row ; y++)
	{
		double x = Left_Equation_a * pow(y, 2) + Left_Equation_b * y + Left_Equation_c;// + A.at<double>(3, 0) * pow(x, 3);//bf
		Left_x_calculate_vector.push_back(x);
		Left_points_fitted_display.push_back(Point(x, y));
	}
	polylines(Image_src_process_copy_display, Left_points_fitted_display, false, Scalar(0, 255, 0), 8, 16, 0);
	Left_points_fitted_display.clear();
	#endif

	//calculate radius of curvature
	double derivatives_one_time = 2.0 * Left_Equation_a * Appointed_y + Left_Equation_b;
	double derivatives_two_time = 2.0 * Left_Equation_a;
	double numerator = 1.0 + pow(derivatives_one_time, 2.0);
	double numerator_two = pow(numerator, 2.0 / 3.0);
	double Curevature_rate = fabs(derivatives_two_time) / numerator_two;
	//cout << Curevature_rate << endl;
	//double Cur_radius = 1 / Curevature_rate;
	
	//if(Curevature_rate > 0)
	//	Radius_of_Curve_left = 1.0 / Curevature_rate;
	
	Radius_of_Curve_left = 1.0 / Curevature_rate;
	//cout << Radius_of_Curve_left << endl;

	vector<float> Left_points_fitted_to_kalman(6);
	Left_points_fitted_to_kalman[0] = (float)(Left_Equation_c);
	Left_points_fitted_to_kalman[1] = 0;
	float Median_y = image_process_std_row / 2;
	Left_points_fitted_to_kalman[2] = (float)(Left_Equation_a * pow(Median_y, 2) + Left_Equation_b * Median_y + Left_Equation_c);
	Left_points_fitted_to_kalman[3] = Median_y;
	float End_y = image_process_std_row;
	Left_points_fitted_to_kalman[4] = (float)(Left_Equation_a * pow(End_y, 2) + Left_Equation_b * End_y + Left_Equation_c);
	Left_points_fitted_to_kalman[5] = End_y;

	if((0 == Leftx_current) || (0 == Left_Contours_num) || (Left_Contours_num > 6)){
		Lane_flag[0] = 0;
	}
	else{
		Lane_flag[0] = 1;
	}
	Current_Fitting_X_Point_kalman.assign(Left_points_fitted_to_kalman.begin(), Left_points_fitted_to_kalman.end());//test

#endif

#if 1
	//preprocess right data of nonzero pixels of lane

	vector<Point> Right_points_fitted_display;
	int right_num_tmp = Right_lane_indices_to_fit.size();

	vector<Point> Right_lane_indices_to_fit_X_TO_Y(right_num_tmp);
	for(int i =0; i < right_num_tmp; i++){
		//circle(image_copy, Point(Right_lane_indices_to_fit[i].x, Right_lane_indices_to_fit[i].y), 4, Scalar(0, 255, 0));//test and verify
		int src_y = Right_lane_indices_to_fit[i].y;
		int src_x = Right_lane_indices_to_fit[i].x;
		Right_lane_indices_to_fit_X_TO_Y[i].x = src_y;
		Right_lane_indices_to_fit_X_TO_Y[i].y = src_x;
	}
	Right_lane_indices_to_fit.clear();

	//Right lane fitting section
	Mat Right_A;
	polynomial_curve_fit(Right_lane_indices_to_fit_X_TO_Y, 2, Right_A);
	Right_lane_indices_to_fit_X_TO_Y.clear();

	double Right_Equation_c = Right_A.at<double>(0, 0);
	double Right_Equation_b = Right_A.at<double>(1, 0);
	double Right_Equation_a = Right_A.at<double>(2, 0);

	//cout << "Right_Equation_a is " << Right_Equation_a << endl;

	#if 0 //bf
	for (int y = 0; y < image_process_std_row; y++)

	{
		double x = Right_Equation_a * pow(y, 2) + Right_Equation_b * y + Right_Equation_c;
		//double y = Right_Equation_a * pow(x, 2) + Right_Equation_b * x + Right_Equation_c;// + A.at<double>(3, 0) * pow(x, 3);
		Right_points_fitted_display.push_back(Point(x, y));
	}
	polylines(Image_src_process_copy_display, Right_points_fitted_display, false, Scalar(0, 255, 0), 8, 16, 0);
	#endif
	
	//Calculate Left lane department
	Right_points_fitted_display.clear();//test and verify temperorily 20190222  // 20190227  17.14

	vector<float> Right_points_fitted_to_kalman(6);
	Right_points_fitted_to_kalman[0] = (float)(Right_Equation_c);
	Right_points_fitted_to_kalman[1] = 0;
	//float Median_y = image_process_std_row / 2;
	Right_points_fitted_to_kalman[2] = (float)(Right_Equation_a * pow(Median_y, 2) + Right_Equation_b * Median_y + Right_Equation_c);
	Right_points_fitted_to_kalman[3] = Median_y;
	//float End_y = image_process_std_row;
	Right_points_fitted_to_kalman[4] = (float)(Right_Equation_a * pow(End_y, 2) + Right_Equation_b * End_y + Right_Equation_c);
	Right_points_fitted_to_kalman[5] = End_y;

	if((Rightx_current == image_process_std_col / 2) || (0 == Right_Contours_num) || (Right_Contours_num > 6)){
		Lane_flag[1] = 0;
	}
	else{
		Lane_flag[1] = 1;
	}
	right_Current_Fitting_X_Point_kalman.assign(Right_points_fitted_to_kalman.begin(), Right_points_fitted_to_kalman.end());//test
#endif

	return Image_src_process_copy_display;
}

bool Perspective_process(Mat& image, Mat& Transform_Matrix, Mat& frame_Out_Perspective_convert_result){
  if(image.empty())
     return false;
  //frame_Out_Perspective_convert_result = Mat::zeros(400, 640, image.type());
  frame_Out_Perspective_convert_result = Mat::zeros(image.rows, image.cols, image.type());

  //perspective collineation
  warpPerspective(image, frame_Out_Perspective_convert_result, Transform_Matrix, Size(image.cols, image.rows), INTER_LINEAR);

  return true;
}

bool polynomial_curve_fit(vector<Point>& key_point, int n, Mat& A){
	//B * X = Y
	//Number of key points
	//cout << key_point << endl;//test and verify
	int N = key_point.size();
	//cout << "N is " << N << endl;//test and verify
 	//construct matrix of X
	Mat X = Mat::zeros(n + 1, n + 1, CV_64FC1);
	//cout << "One of X is " << X << endl;//test and verify
	for (int i = 0; i < n + 1; i++)
	{
		for (int j = 0; j < n + 1; j++)
		{
			for (int k = 0; k < N; k++)
			{
				X.at<double>(i, j) = X.at<double>(i, j) + pow(key_point[k].x, i + j);
				//cout << k << endl;//test and verify
				//cout << X.at<double>(i, j) << "  " << k << "  " << key_point[k].x << endl;//test and verify
			}
		}
	}
	//cout << "Two of X is " << X << endl;//test and verify

	Mat Y = Mat::zeros(n + 1, 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int k = 0; k < N; k++)
		{
			Y.at<double>(i, 0) = Y.at<double>(i, 0) + pow(key_point[k].x, i) * key_point[k].y;
		}
	}

	A = Mat::zeros(n + 1, 1, CV_64FC1);
	#if 1
	solve(X, Y, A, DECOMP_LU);
	//solve(X, Y, A, DECOMP_LU);
	#endif
	//cout << A << endl;//test and verify
	//cout << "radius of curvature is " <<  A.at<double>(2, 0) << endl;//test and verify

	return true;
}

void Preprocess_bilateralFilter_threshold_third(Mat& image, Mat& Image_threshold)
{
	equalizeHist_Mat(image);
	
	Mat Image_cvt;
	//cvtColor(Bottom_frame_one, Image_cvt, COLOR_BGR2GRAY,0);
	cvtColor(image, Image_cvt, COLOR_BGR2GRAY,0);	//imshow("Image_cvt",Image_cvt);//test and verify
	
	threshold(Image_cvt,Image_threshold, 70, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);//CV_THRESH_OTSU
	//imshow("Image_threshold",Image_threshold);//test and verify

	Mat Image_morph;
	Mat All_element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1,-1));// 7 7//Size(1, 3)//Size(3, 3)
	morphologyEx(Image_threshold, Image_morph, MORPH_OPEN, All_element, Point(-1,-1), 1, BORDER_CONSTANT);//MORPH_OPEN
	//imshow("Image_morph",Image_morph);//test and verify

	Mat Image_bilateralFilter;
	int i = 5;//10//5
	bilateralFilter (Image_threshold, Image_bilateralFilter, i, i*2, i/2 );//bilateralFilter ( src, dst, i, i*2, i/2 );

	Mat Image_canny;
	Canny(Image_bilateralFilter, Image_threshold, 50, 210, 3 );//200	400
	//imshow("Image_canny",Image_canny);//test and verify
	
	return;
}

void equalizeHist_Mat(Mat& image){
	
	Mat image_apex = image.clone();
	#if 1	
	double T30 = getTickCount();
	Mat imageRGB[3];
	split(image, imageRGB);
	for (int i = 0; i < 3; i++)
	{
		equalizeHist(imageRGB[i], imageRGB[i]);
	}
	merge(imageRGB, 3, image);
	double T31 = getTickCount();
	//cout << "opencv equalize is :" << (T31 - T30) * 1000 / getTickFrequency() << " ms!" << endl;
	#endif
	
	#if 0
	double T20 = getTickCount();
	int lRetVal = 0;
	vsdk::UMat lInput0E = image.getUMat(ACCESS_READ);
    vsdk::UMat lOutput0E(image.rows, image.cols, VSDK_CV_8UC3);
	apexcv::HistogramEqualization HistogramEqualization;
	
	lRetVal |= HistogramEqualization.Initialize(lInput0E, lOutput0E);
	
	image = lOutput0E.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	double T21 = getTickCount();
	//cout << "apexcv equalize is :" << (T21 - T20) * 1000 / getTickFrequency() << " ms!" << endl;
	#endif

	return;
}

void Left_Warning_sign(Mat& img)
{
#if 1
	int Adjust_height = 50;
	Point root_points[1][3];
	root_points[0][0] = Point(50, 120 + Adjust_height);
	root_points[0][1] = Point(100, 60 + Adjust_height);
	root_points[0][2] = Point(150, 120 + Adjust_height);

	const Point* ppt[1] = {root_points[0]};
	int npt[] = {3};
	polylines(img, ppt, npt, 1, 1, Scalar(0, 255, 255), 2, 8, 0);

	Point p = Point(65, 115 + Adjust_height);
	putText(img, "WARNING", p, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1, 8 );
	Point Q = Point(80, 100 + Adjust_height);
	putText(img, "LEFT", Q, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1, 8 );
#endif

#if 0	
	Point center = Point(100, 140);
	int Radius = 50;
	circle(img, center, Radius, Scalar(0, 0, 255), 4, 8);
	Point Core_Point = Point(50, 140);
	putText(img, "WARNING", Core_Point, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1, 8 );


#endif
	return;
}

void Right_Warning_sign(Mat& img)
{
	int Adjust_height = 50;
	Point root_points[1][3];
	root_points[0][0] = Point(590, 120 + Adjust_height);
	root_points[0][1] = Point(540, 60 + Adjust_height);
	root_points[0][2] = Point(490, 120 + Adjust_height);

	const Point* ppt[1] = {root_points[0]};
	int npt[] = {3};
	polylines(img, ppt, npt, 1, 1, Scalar(0, 255, 255), 2, 8, 0);

	Point p = Point(505, 115 + Adjust_height);
	putText(img, "WARNING", p, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1, 8 );
	Point Q = Point(515, 100 + Adjust_height);
	putText(img, "RIGHT", Q, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1, 8 );

	return;
}

