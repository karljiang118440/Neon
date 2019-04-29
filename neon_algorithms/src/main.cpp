

#ifdef __STANDALONE__
#include "../../../data/common/headers/in_grey_256x256.h"
#include <frame_output_dcu.h>
#endif

#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <cstdlib>
#include <stdio.h>
#include <oal.h>
#include <common_helpers.h>
#include <string>
#include <arm_neon.h>



#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>




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






/*
//*** to dispaly


#include "../../../data/common/headers/in_grey_256x256.h"


#ifdef __STANDALONE__
#include "frame_output_dcu.h"
#define CHNL_CNT io::IO_DATA_CH3
#else // #ifdef __STANDALONE__
#include "frame_output_v234fb.h"
#define CHNL_CNT io::IO_DATA_CH3
#endif // else from #ifdef __STANDALONE__


#ifdef __STANDALONE__
  io::FrameOutputDCU output(1280, 720,	io::IO_DATA_DEPTH_08, CHNL_CNT);
#else	
  io::FrameOutputV234Fb output(1280, 720, io::IO_DATA_DEPTH_08, CHNL_CNT);
#endif


//**************   to dispaly


*/








inline void BGR888ToYUV444(unsigned char * __restrict__ yuv, unsigned char * __restrict__ bgr, int pixel_num);
inline void BGR2YUV_parrel_inter(unsigned char * __restrict__ yuv, unsigned char * __restrict__ bgr, int pixel_num);
inline void BGR2YUV_map(unsigned char * __restrict__ yuv, unsigned char * __restrict__ bgr, int pixel_num);

inline void neon_memcpy_karl(void* dst, void* src, long size);


#define INPUT_ROOT "data/common/"
#define OUTPUT_ROOT "data/output/"

#define CHNL_CNT io::IO_DATA_CH3





int main(int argc, char** argv)
{




  const char  inputImgName[] = INPUT_ROOT "in_grey_256x256.png";

  const char  inputImgName1[] = INPUT_ROOT "in_color_256x256.png";
  
  std::string helpMsg = std::string("Applies the Gauss 3x3  smoothing filter on an image with NEON.\n\tUsage: ") +
                        COMMON_ExtractProgramName(argv[0]) + "\n\n\tUses input image: " + inputImgName;
  int idxHelp = COMMON_HelpMessage(argc, argv, helpMsg.c_str());
  if(idxHelp > 0)
  {
    //found help in application arguments thus exiting application
    return -1;
  }

  using namespace cv;
  int lRetVal = 0;
  // Read the input using OpenCV
#ifdef __STANDALONE__
  Mat in(256, 256, CV_8UC1, in_grey_256x256);
#else

  Mat in = imread(inputImgName, 0);

  Mat in0 = imread(inputImgName1,1);
 
  Mat in1 = imread(inputImgName1,1);

  Mat in2 = imread(inputImgName1,1);
  

#endif

  using namespace cv;
  using namespace std;

  if(in.empty())
  {
    printf("no image %s \n", inputImgName);
    lRetVal = -1;
  }
  else
  {
    unsigned char* inputImage  = (unsigned char*)(in.data);
    Mat            out         = in.clone();
    unsigned char* outputImage = (unsigned char*)(out.data);

    Size s    = in.size();
    int  rows = s.height;
    int  cols = s.width;
    int  size = rows * cols;

 


	//***** add the BGR888ToYUV444 no neon
	
	
	
		double t0 = (double)cvGetTickCount();
	
		unsigned char* inputImage0	= (unsigned char*)(in0.data);
		Mat 		   out0 		= in0.clone();
		unsigned char* outputImage0 = (unsigned char*)(out0.data);
	
		Size s0    = in0.size();
		int  rows0 = s0.height;
		int  cols0 = s0.width;
		int  size0 = rows0 * cols;
	
		BGR2YUV_parrel_inter(outputImage0, inputImage0, rows0);
	
	
		//	算法过程
		t0 = (double)cvGetTickCount() - t0;
		printf( "BGR2YUV_parrel_inter run time = %gms\n", t0/(cvGetTickFrequency()*1000) );
	
	
	
	//************************ add the BGR888ToYUV444 no neon



//***** add the BGR888ToYUV444 by Neon



	double t1 = (double)cvGetTickCount();

    unsigned char* inputImage1  = (unsigned char*)(in1.data);
    Mat            out1         = in1.clone();
    unsigned char* outputImage1 = (unsigned char*)(out1.data);

    Size s1    = in1.size();
    int  rows1 = s1.height;
    int  cols1 = s1.width;
    int  size1 = rows1 * cols;

    BGR888ToYUV444(outputImage1, inputImage1, rows1);


	//	算法过程
	t1 = (double)cvGetTickCount() - t1;
	printf( "RGB2YUV_neon run time = %gms\n", t1/(cvGetTickFrequency()*1000) );



//************************ add the BGR888ToYUV444



//***** add the BGR888ToYUV444 by BGR2YUV_map


	double t = (double)cvGetTickCount();


    unsigned char* inputImage2  = (unsigned char*)(in2.data);
    Mat            out2         = in2.clone();
    unsigned char* outputImage2 = (unsigned char*)(out2.data);

    Size s2    = in2.size();
    int  rows2 = s2.height;
    int  cols2 = s2.width;
    int  size2 = rows2 * cols;

    BGR2YUV_map(outputImage2, inputImage2, rows2);

	//	算法过程
	t = (double)cvGetTickCount() - t;
	printf( "BGR2YUV_map run time = %gms\n", t/(cvGetTickFrequency()*1000) );
	//printf( "run time = %gs\n", t/(cvGetTickFrequency()*1000000) );

	



#if 0


    unsigned char* outputImage3 = (unsigned char*)(out2.data);
    unsigned char* outputImage4 = (unsigned char*)(out2.data);


   double t3 = (double)cvGetTickCount();

      memcpy(outputImage3, inputImage2,256*256);

    t3 = (double)cvGetTickCount() - t3;
    printf( "memcpy(256*256) run time = %gms\n", t3/(cvGetTickFrequency()*1000) );






   double t4 = (double)cvGetTickCount();

      neon_memcpy_karl(outputImage4, inputImage2,256*256);

    t4 = (double)cvGetTickCount() - t4;
    printf( "neon_memcpy_karl(256*256) run time = %gms\n", t4/(cvGetTickFrequency()*1000) );





      neon_memcpy_1280((char*)out_buffer_umat.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED).data,
                       (char*)lFrameRgbmat.data);
      



#endif




    #if 1

    ofstream oFile;
    oFile.open("Ap_Op_Time.txt",ios::out|ios::trunc);

	
    ofstream oFile1;
    oFile1.open("Ap_Op_Time1.txt",ios::out|ios::trunc);




    VideoCapture capture;
    SourceImg = capture.open("mvi_0050.avi");

    if(!capture.isOpened())
    {
        printf("can not open ...\n");
        return -1;
    }
    while (capture.read(SourceImg)){


   printf("SourceImg.cols=%d ,SourceImg.rows=%d\n", SourceImg.cols,SourceImg.rows);

        #if 0
        Mat SourceImg_apex = SourceImg.clone();
        double Tre0 = getTickCount();

        memcpy((char*)outputImage3.data, (char*)SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows);

        double Tre1 = getTickCount();
        //cout << "opencv resize is :" << (T14 - T13) * 1000 / getTickFrequency() << " ms!" << endl;
       double Apex_opencv_time[14] = (Tre1 - Tre0) * 1000 / getTickFrequency();


        #endif


#if 1



Mat SourceImg_apex = SourceImg.clone();
//Mat outputImage3= SourceImg.clone();
//Mat outputImage4= SourceImg.clone();


 Mat outputImage3(Size(640, 480), CV_8UC3, Scalar(0));
 Mat outputImage4(Size(640, 480), CV_8UC3, Scalar(0));





   double t3 = (double)cvGetTickCount();

        memcpy((char*)outputImage3.data, (char*)SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows*3);
       // memcpy(outputImage3.data, SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows);


    t3 = (double)cvGetTickCount() - t3;

    double Apex_opencv_time1 =  t3/(cvGetTickFrequency()*1000);

	oFile1<<Apex_opencv_time1 <<endl;	

    printf( "memcpy run time = %gms\n", t3/(cvGetTickFrequency()*1000) );



 #endif 




        #if 0
        //Mat SourceImg_apex = SourceImg.clone();
        double Tresize_apx0 = getTickCount();
    
        neon_memcpy_karl((char*)outputImage4.data, (char*)SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows);

        double Tresize_apx1 = getTickCount();
        //cout << "apexcv resize is :" << (T16 - T15) * 1000 / getTickFrequency() << " ms!" << endl;
       double Apex_opencv_time[15] = (Tresize_apx1 - Tresize_apx0) * 1000 / getTickFrequency();

        //Mat output_resize = lOutput0.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        //cout << output_resize.rows << "  " << output_resize.cols << endl;
        
        #endif





#if 1

   double t4 = (double)cvGetTickCount();

        neon_memcpy_karl((char*)outputImage4.data, (char*)SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows*3);

    t4 = (double)cvGetTickCount() - t4;

    double Apex_opencv_time2 =  t4/(cvGetTickFrequency()*1000);


    printf( "neon_memcpy_karl run time = %gms\n", t4/(cvGetTickFrequency()*1000) );

        // oFile<<Apex_opencv_time1 <<","<< Apex_opencv_time2 <<endl; 
         oFile<< Apex_opencv_time2 <<endl; 

 #endif 


    }
    
 
    #endif




























#ifdef __STANDALONE__


	
#else
//    imwrite(OUTPUT_ROOT "out_gauss3x3.png", out);

 //   imwrite( "RGB2YUV_neon.png", out1);

 //   imwrite( "RGB2YUV_opencv.png", lOutout0);

#endif
  }

  if(0 != lRetVal)
  {
    printf("Program Ended Error 0x%X [ERROR]\n", lRetVal);
  }
  else
  {
    printf("Program Ended [SUCCESS]\n");
  }
}

















inline void BGR888ToYUV444(unsigned char * __restrict__ yuv, unsigned char * __restrict__ bgr, int pixel_num)
{
    const uint8x8_t u8_zero = vdup_n_u8(0);
    const uint16x8_t u16_rounding = vdupq_n_u16(128);
    const int16x8_t s16_rounding = vdupq_n_s16(128);
    const int8x16_t s8_rounding = vdupq_n_s8(128);

    int count = pixel_num / 16;

    int i;
    for (i = 0; i < count; ++i) {
        // Load bgr
        uint8x16x3_t pixel_bgr = vld3q_u8(bgr);

        uint8x8_t high_r = vget_high_u8(pixel_bgr.val[0]);
        uint8x8_t low_r = vget_low_u8(pixel_bgr.val[0]);
        uint8x8_t high_g = vget_high_u8(pixel_bgr.val[1]);
        uint8x8_t low_g = vget_low_u8(pixel_bgr.val[1]);
        uint8x8_t high_b = vget_high_u8(pixel_bgr.val[2]);
        uint8x8_t low_b = vget_low_u8(pixel_bgr.val[2]);
        int16x8_t signed_high_r = vreinterpretq_s16_u16(vaddl_u8(high_r, u8_zero));
        int16x8_t signed_low_r = vreinterpretq_s16_u16(vaddl_u8(low_r, u8_zero));
        int16x8_t signed_high_g = vreinterpretq_s16_u16(vaddl_u8(high_g, u8_zero));
        int16x8_t signed_low_g = vreinterpretq_s16_u16(vaddl_u8(low_g, u8_zero));
        int16x8_t signed_high_b = vreinterpretq_s16_u16(vaddl_u8(high_b, u8_zero));
        int16x8_t signed_low_b = vreinterpretq_s16_u16(vaddl_u8(low_b, u8_zero));

        // NOTE:
        // declaration may not appear after executable statement in block
        uint16x8_t high_y;
        uint16x8_t low_y;
        uint8x8_t scalar = vdup_n_u8(76);
        int16x8_t high_u;
        int16x8_t low_u;
        int16x8_t signed_scalar = vdupq_n_s16(-43);
        int16x8_t high_v;
        int16x8_t low_v;
        uint8x16x3_t pixel_yuv;
        int8x16_t u;
        int8x16_t v;

        // 1. Multiply transform matrix (Y′: unsigned, U/V: signed)
        high_y = vmull_u8(high_r, scalar);
        low_y = vmull_u8(low_r, scalar);

        high_u = vmulq_s16(signed_high_r, signed_scalar);
        low_u = vmulq_s16(signed_low_r, signed_scalar);

        signed_scalar = vdupq_n_s16(127);
        high_v = vmulq_s16(signed_high_r, signed_scalar);
        low_v = vmulq_s16(signed_low_r, signed_scalar);

        scalar = vdup_n_u8(150);
        high_y = vmlal_u8(high_y, high_g, scalar);
        low_y = vmlal_u8(low_y, low_g, scalar);

        signed_scalar = vdupq_n_s16(-84);
        high_u = vmlaq_s16(high_u, signed_high_g, signed_scalar);
        low_u = vmlaq_s16(low_u, signed_low_g, signed_scalar);

        signed_scalar = vdupq_n_s16(-106);
        high_v = vmlaq_s16(high_v, signed_high_g, signed_scalar);
        low_v = vmlaq_s16(low_v, signed_low_g, signed_scalar);

        scalar = vdup_n_u8(29);
        high_y = vmlal_u8(high_y, high_b, scalar);
        low_y = vmlal_u8(low_y, low_b, scalar);

        signed_scalar = vdupq_n_s16(127);
        high_u = vmlaq_s16(high_u, signed_high_b, signed_scalar);
        low_u = vmlaq_s16(low_u, signed_low_b, signed_scalar);

        signed_scalar = vdupq_n_s16(-21);
        high_v = vmlaq_s16(high_v, signed_high_b, signed_scalar);
        low_v = vmlaq_s16(low_v, signed_low_b, signed_scalar);

        // 2. Scale down (">>8") to 8-bit values with rounding ("+128") (Y′: unsigned, U/V: signed)
        // 3. Add an offset to the values to eliminate any negative values (all results are 8-bit unsigned)

        high_y = vaddq_u16(high_y, u16_rounding);
        low_y = vaddq_u16(low_y, u16_rounding);

        high_u = vaddq_s16(high_u, s16_rounding);
        low_u = vaddq_s16(low_u, s16_rounding);

        high_v = vaddq_s16(high_v, s16_rounding);
        low_v = vaddq_s16(low_v, s16_rounding);

        pixel_yuv.val[0] = vcombine_u8(vqshrn_n_u16(low_y, 8), vqshrn_n_u16(high_y, 8));

        u = vcombine_s8(vqshrn_n_s16(low_u, 8), vqshrn_n_s16(high_u, 8));

        v = vcombine_s8(vqshrn_n_s16(low_v, 8), vqshrn_n_s16(high_v, 8));

        u = vaddq_s8(u, s8_rounding);
        pixel_yuv.val[1] = vreinterpretq_u8_s8(u);

        v = vaddq_s8(v, s8_rounding);
        pixel_yuv.val[2] = vreinterpretq_u8_s8(v);

        // Store
        vst3q_u8(yuv, pixel_yuv);

        bgr += 3 * 16;
        yuv += 3 * 16;
    }

    // Handle leftovers
    for (i = count * 16; i < pixel_num; ++i) {
        uint8_t r = bgr[i * 3];
        uint8_t g = bgr[i * 3 + 1];
        uint8_t b = bgr[i * 3 + 2];

        uint16_t y_tmp = 76 * r + 150 * g + 29 * b;
        int16_t u_tmp = -43 * r - 84 * g + 127 * b;
        int16_t v_tmp = 127 * r - 106 * g - 21 * b;

        y_tmp = (y_tmp + 128) >> 8;
        u_tmp = (u_tmp + 128) >> 8;
        v_tmp = (v_tmp + 128) >> 8;

        yuv[i * 3] = (uint8_t) y_tmp;
        yuv[i * 3 + 1] = (uint8_t) (u_tmp + 128);
        yuv[i * 3 + 2] = (uint8_t) (v_tmp + 128);
    }
}



// Refer to: https://en.wikipedia.org/wiki/YUV (Full swing for BT.601)
inline void BGR2YUV_parrel_inter(unsigned char *yuv, unsigned char *bgr, int pixel_num)
{
    int i;
    for (i = 0; i < pixel_num; ++i) {
        uint8_t r = bgr[i * 3];
        uint8_t g = bgr[i * 3 + 1];
        uint8_t b = bgr[i * 3 + 2];

        // 1. Multiply transform matrix (Y′: unsigned, U/V: signed)
        uint16_t y_tmp = 76 * r + 150 * g + 29 * b;
        int16_t u_tmp = -43 * r - 84 * g + 127 * b;
        int16_t v_tmp = 127 * r - 106 * g - 21 * b;

        // 2. Scale down (">>8") to 8-bit values with rounding ("+128") (Y′: unsigned, U/V: signed)
        y_tmp = (y_tmp + 128) >> 8;
        u_tmp = (u_tmp + 128) >> 8;
        v_tmp = (v_tmp + 128) >> 8;

        // 3. Add an offset to the values to eliminate any negative values (all results are 8-bit unsigned)
        yuv[i * 3] = (uint8_t) y_tmp;
        yuv[i * 3 + 1] = (uint8_t) (u_tmp + 128);
        yuv[i * 3 + 2] = (uint8_t) (v_tmp + 128);
    }
}





// 两张图片进行相加进行计算

inline void BGR2YUV_map(unsigned char *yuv, unsigned char *bgr, int pixel_num)
{
    int i;
    for (i = 0; i < pixel_num; ++i) {
        uint8_t r = bgr[i * 3];
        uint8_t g = bgr[i * 3 + 1];
        uint8_t b = bgr[i * 3 + 2];

        uint8_t y = 0.299 * r + 0.587 * g + 0.114 * b;
        uint8_t u = -0.169 * r - 0.331 * g + 0.5 * b + 128;
        uint8_t v = 0.5 * r - 0.419 * g - 0.081 * b + 128;

        yuv[i * 3] = y;
        yuv[i * 3 + 1] = u;
        yuv[i * 3 + 2] = v;
    }

}


inline void neon_memcpy_karl(void* dst, void* src, long size)
{
  long simd_pixels = size & ~15; 
  long simd_iterations = simd_pixels / 64;
  long simd_sync = 20;
  
  char *dst_local = (char *)dst;
  char *src_local = (char *)src;
  __asm volatile( "neon_memcpy_loop: \n\t"
              "LD1 {V0.16B, V1.16B, V2.16B, V3.16B}, [%[src_local]], #64    \n\t"
              "ST1 {V0.16B, V1.16B, V2.16B, V3.16B}, [%[dst_local]], #64    \n\t"
              
              "subs %[sync],%[sync],#1 \n\t"
              "bne neon_memcpy_next \n\t"
              "mov %[sync], #20 \n\t"
              "neon_memcpy_next: \n\t"
              "subs %[iterations],%[iterations],#1 \n\t"
              "bne neon_memcpy_loop \n\t"
              
              : [src_local]"+r"(src_local), [dst_local] "+r"(dst_local), [iterations]"+r"(simd_iterations), [sync]"+r"(simd_sync)
              : 
           );
}


































