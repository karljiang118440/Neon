

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


inline void neon_memcpy_karl(void* dst, void* src, long size);
inline void memcpy_neon_asm(void *dst,  void *src, size_t len);





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

 


    #if 1

    ofstream oFile;
    oFile.open("neon_memcpy.txt",ios::out|ios::trunc);

	
    ofstream oFile1;
    oFile1.open("memcpy.txt",ios::out|ios::trunc);




    VideoCapture capture;
    SourceImg = capture.open("mvi_0050.avi");

    if(!capture.isOpened())
    {
        printf("can not open ...\n");
        return -1;
    }
    while (capture.read(SourceImg)){


	resize(SourceImg, SourceImg, Size(1280, 960),0,0,INTER_LINEAR);

   printf("SourceImg.cols=%d ,SourceImg.rows=%d\n", SourceImg.cols,SourceImg.rows);





Mat SourceImg_apex = SourceImg.clone();

// Mat outputImage1(Size(SourceImg.cols, SourceImg.rows), CV_8UC3, Scalar(0));

 Mat outputImage3(Size(SourceImg.cols, SourceImg.rows), CV_8UC3, Scalar(0));
 Mat outputImage4(Size(SourceImg.cols, SourceImg.rows), CV_8UC3, Scalar(0));









   double t3 = (double)cvGetTickCount();

        memcpy((char*)outputImage3.data, (char*)SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows*3);
       // memcpy(outputImage3.data, SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows);


    t3 = (double)cvGetTickCount() - t3;

    double Apex_opencv_time1 =  t3/(cvGetTickFrequency()*1000);

	oFile1<<Apex_opencv_time1 <<endl;	

    printf( "memcpy run time = %gms\n", t3/(cvGetTickFrequency()*1000) );





   double t4 = (double)cvGetTickCount();

        neon_memcpy_karl((char*)outputImage4.data, (char*)SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows*3);

    t4 = (double)cvGetTickCount() - t4;

    double Apex_opencv_time2 =  t4/(cvGetTickFrequency()*1000);


    printf( "neon_memcpy_karl run time = %gms\n", t4/(cvGetTickFrequency()*1000) );

        // oFile<<Apex_opencv_time1 <<","<< Apex_opencv_time2 <<endl; 
         oFile<< Apex_opencv_time2 <<endl; 




#if 0
		 double t1 = (double)cvGetTickCount();
		 
			  memcpy_neon_asm((char*)outputImage3.data, (char*)SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows*3);
			 // memcpy(outputImage3.data, SourceImg_apex.data,SourceImg_apex.cols*SourceImg_apex.rows);
		 
		 
		  t1 = (double)cvGetTickCount() - t1;
		 
		  double memcpy_neon_asm =  t1/(cvGetTickFrequency()*1000);
		 
		  //oFile1<<Apex_opencv_time1 <<endl;   
		 
		  printf( "memcpy_neon_asm run time = %gms\n", t1/(cvGetTickFrequency()*1000) );


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














inline void neon_memcpy_karl(void* dst, void* src, long size)
{
  long simd_pixels = size & ~15; //内存对齐
  long simd_iterations = simd_pixels / 64; //每次传送 64 bit
//  long simd_sync = 20;
  
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


inline void memcpy_neon_asm(void *dst,  void *src, size_t len)
{
	 char *s = (char *)src;
	char *d = (char *)dst;

	while (len--)

    asm ("PLD [%0, #128]"::"r" (src)); //add1: asm 128bit 
	
	//	*d++ = *s++;



// add 2: add 3 loops reduce cpu times
	*d++ = *s++;
    *d++ = *s++;
    *d++ = *s++;
    *d++ = *s++;

	//return dst;





	
} 
































