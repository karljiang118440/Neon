

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

#include <uchar.h>
#include <wchar.h>

 
using namespace cv;
using namespace std;







inline void Gaussian(unsigned char* src, unsigned char* dst, int width, int size);
inline void reference_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n);
inline void neon_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n);



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

  Mat out1;


  Mat in0 = imread(inputImgName1,1);

  
  Mat in1 = imread(inputImgName1,1);


  

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



	double t2 = (double)cvGetTickCount();
	
    Gaussian(inputImage, outputImage, cols, size);

	t2 = (double)cvGetTickCount() - t2;
	printf( "Gaussian run time = %gms\n", t2/(cvGetTickFrequency()*1000) );



	
	
	unsigned char* inputImage0	= (unsigned char*)(in0.data);
	Mat 		   out0 		= in0.clone();
	unsigned char* outputImage0 = (unsigned char*)(out0.data);
	
	Size s0    = in0.size();
	int  rows0 = s0.height;
	int  cols0 = s0.width;
	int  size0 = rows0 * cols;

	double t0 = (double)cvGetTickCount();
	
	neon_convert(outputImage, inputImage, rows);
	
	
	//	算法过程
	t0 = (double)cvGetTickCount() - t0;
	printf( "neon_convert run time = %gms\n", t0/(cvGetTickFrequency()*1000) );
	
	
	





	double t3 = (double)cvGetTickCount();  
	reference_convert(outputImage, inputImage, rows);
	t3 = (double)cvGetTickCount() - t3;
	printf( "reference_convert run time = %gms\n", t3/(cvGetTickFrequency()*1000) );



	
	
	




	




	

#ifdef __STANDALONE__
    io::FrameOutputDCU output(1280, 720, io::IO_DATA_DEPTH_08, CHNL_CNT);

    // Output buffer (screen size) and it's mapped version (using cv mat in order to have copyTo functions)
    vsdk::UMat output_umat = vsdk::UMat(720, 1280, VSDK_CV_8UC3);
    cv::Mat    output_mat  = output_umat.getMat(ACCESS_WRITE | OAL_USAGE_CACHED);
    memset(output_mat.data, 0, 720 * 1280 * 3);

    cv::UMat inRGB(in.rows, in.cols, CV_8UC3);
    cv::UMat outRGB(in.rows, in.cols, CV_8UC3);

    cvtColor(out, outRGB, CV_GRAY2RGB);
    cvtColor(in, inRGB, CV_GRAY2RGB);

    inRGB.copyTo(output_mat(cv::Rect(0, 232, 256, 256)));
    outRGB.copyTo(output_mat(cv::Rect(300, 232, 256, 256)));

    output.PutFrame(output_umat);



    io::FrameOutputDCU output1(1280, 720, io::IO_DATA_DEPTH_08, CHNL_CNT);

    // Output buffer (screen size) and it's mapped version (using cv mat in order to have copyTo functions)
    vsdk::UMat output_umat1 = vsdk::UMat(720, 1280, VSDK_CV_8UC3);
    cv::Mat    output_mat1  = output_umat1.getMat(ACCESS_WRITE | OAL_USAGE_CACHED);
    memset(output_mat1.data, 0, 720 * 1280 * 3);

    cv::UMat inRGB1(in1.rows, in1.cols, CV_8UC3);
    cv::UMat outRGB1(in1.rows, in1.cols, CV_8UC3);

    cvtColor(out1, outRGB1, CV_GRAY2RGB);
    cvtColor(in1, inRGB1, CV_GRAY2RGB);

    inRGB1.copyTo(output_mat1(cv::Rect(0, 232, 256, 256)));
    outRGB1.copyTo(output_mat1(cv::Rect(300, 232, 256, 256)));

    output1.PutFrame(output_umat1);



	
#else
    imwrite(OUTPUT_ROOT "out_gauss3x3.png", out);

   // imwrite( "RGB2YUV_neon.png", out1);

   // imwrite( "RGB2YUV_opencv.png", lOutout0);

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

inline void Gaussian(unsigned char* src, unsigned char* dst, int width, int size)
{
  uint64_t simd_iteration = (size - (2 * width) - 1) & ~7;
  simd_iteration >>= 5;

  uint64_t* offset = new uint64_t[3];

  offset[0] = (-width - 1);
  offset[1] = (-1);
  offset[2] = (width - 1);

  asm volatile(
      "1:                                                 \n\t"
      "mov   x7, %x0                                      \n\t"

      //midle point
      "LD1   {v4.8B, v5.8B, v6.8B,v7.8B},  [%[src]]       \n\t"

      "UXTL  v4.8H, v4.8B                                 \n\t"
      "UXTL  v5.8H, v5.8B                                 \n\t"
      "UXTL  v6.8H, v6.8B                                 \n\t"
      "UXTL  v7.8H, v7.8B                                 \n\t"

      "UQSHL v4.8H, v4.8H, #2                             \n\t"
      "UQSHL v5.8H, v5.8H, #2                             \n\t"
      "UQSHL v6.8H, v6.8H, #2                             \n\t"
      "UQSHL v7.8H, v7.8H, #2                             \n\t"

      //1.step
      "ldr   x10, [%x1]                                   \n\t"
      "add   x6, x7, x10                                  \n\t"

      "LD1   {v8.8B, v9.8B, v10.8B,v11.8B},  [x6]         \n\t"

      "UXTL  v8.8H, v8.8B                                 \n\t"
      "UXTL  v9.8H, v9.8B                                 \n\t"
      "UXTL  v10.8H, v10.8B                               \n\t"
      "UXTL  v11.8H, v11.8B                               \n\t"

      "ADD   v4.8H, v4.8H, v8.8H                          \n\t"
      "ADD   v5.8H, v5.8H, v9.8H                          \n\t"
      "ADD   v6.8H, v6.8H, v10.8H                         \n\t"
      "ADD   v7.8H, v7.8H, v11.8H                         \n\t"

      //2.step
      "add       x6, x6, #1                               \n\t"

      "LD1   {v0.8B, v1.8B, v2.8B,v3.8B},  [x6]           \n\t"

      "UXTL  v8.8H, v0.8B                                 \n\t"
      "UXTL  v9.8H, v1.8B                                 \n\t"
      "UXTL  v10.8H, v2.8B                                \n\t"
      "UXTL  v11.8H, v3.8B                                \n\t"

      "UQSHL v8.8H, v8.8H, #1                             \n\t"
      "UQSHL v9.8H, v9.8H, #1                             \n\t"
      "UQSHL v10.8H, v10.8H, #1                           \n\t"
      "UQSHL v11.8H, v11.8H, #1                           \n\t"

      "ADD   v4.8H, v4.8H, v8.8H                          \n\t"
      "ADD   v5.8H, v5.8H, v9.8H                          \n\t"
      "ADD   v6.8H, v6.8H, v10.8H                         \n\t"
      "ADD   v7.8H, v7.8H, v11.8H                         \n\t"

      //3.step
      "add       x6, x6, #1                               \n\t"

      "LD1   {v0.8B, v1.8B, v2.8B,v3.8B},  [x6]           \n\t"

      "UXTL  v8.8H, v0.8B                                 \n\t"
      "UXTL  v9.8H, v1.8B                                 \n\t"
      "UXTL  v10.8H, v2.8B                                \n\t"
      "UXTL  v11.8H, v3.8B                                \n\t"

      "ADD   v4.8H, v4.8H, v8.8H                          \n\t"
      "ADD   v5.8H, v5.8H, v9.8H                          \n\t"
      "ADD   v6.8H, v6.8H, v10.8H                         \n\t"
      "ADD   v7.8H, v7.8H, v11.8H                         \n\t"

      //4.step - new line offset
      "add   %x1, %x1, #8                                 \n\t"
      "ldr   x10, [%x1]                                   \n\t"
      "add   x6, x7, x10                                  \n\t"

      "LD1   {v0.8B, v1.8B, v2.8B,v3.8B},  [x6]           \n\t"

      "UXTL  v8.8H, v0.8B                                 \n\t"
      "UXTL  v9.8H, v1.8B                                 \n\t"
      "UXTL  v10.8H, v2.8B                                \n\t"
      "UXTL  v11.8H, v3.8B                                \n\t"

      "UQSHL v8.8H, v8.8H, #1                             \n\t"
      "UQSHL v9.8H, v9.8H, #1                             \n\t"
      "UQSHL v10.8H, v10.8H, #1                           \n\t"
      "UQSHL v11.8H, v11.8H, #1                           \n\t"

      "ADD   v4.8H, v4.8H, v8.8H                          \n\t"
      "ADD   v5.8H, v5.8H, v9.8H                          \n\t"
      "ADD   v6.8H, v6.8H, v10.8H                         \n\t"
      "ADD   v7.8H, v7.8H, v11.8H                         \n\t"

      //5.step - middle point

      //6.step
      "add       x6, x6, #2                               \n\t"

      "LD1   {v0.8B, v1.8B, v2.8B,v3.8B},  [x6]           \n\t"

      "UXTL  v8.8H, v0.8B                                 \n\t"
      "UXTL  v9.8H, v1.8B                                 \n\t"
      "UXTL  v10.8H, v2.8B                                \n\t"
      "UXTL  v11.8H, v3.8B                                \n\t"

      "UQSHL v8.8H, v8.8H, #1                             \n\t"
      "UQSHL v9.8H, v9.8H, #1                             \n\t"
      "UQSHL v10.8H, v10.8H, #1                           \n\t"
      "UQSHL v11.8H, v11.8H, #1                           \n\t"

      "ADD   v4.8H, v4.8H, v8.8H                          \n\t"
      "ADD   v5.8H, v5.8H, v9.8H                          \n\t"
      "ADD   v6.8H, v6.8H, v10.8H                         \n\t"
      "ADD   v7.8H, v7.8H, v11.8H                         \n\t"

      //7.step
      "add   %x1, %x1, #8                                 \n\t"
      "ldr   x10, [%x1]                                   \n\t"
      "add   x6, x7, x10                                  \n\t"

      "LD1   {v0.8B, v1.8B, v2.8B,v3.8B},  [x6]           \n\t"

      "UXTL  v8.8H, v0.8B                                 \n\t"
      "UXTL  v9.8H, v1.8B                                 \n\t"
      "UXTL  v10.8H, v2.8B                                \n\t"
      "UXTL  v11.8H, v3.8B                                \n\t"

      "ADD   v4.8H, v4.8H, v8.8H                          \n\t"
      "ADD   v5.8H, v5.8H, v9.8H                          \n\t"
      "ADD   v6.8H, v6.8H, v10.8H                         \n\t"
      "ADD   v7.8H, v7.8H, v11.8H                         \n\t"

      //8.step
      "add       x6, x6, #1                               \n\t"

      "LD1   {v0.8B, v1.8B, v2.8B,v3.8B},  [x6]           \n\t"

      "UXTL  v8.8H, v0.8B                                 \n\t"
      "UXTL  v9.8H, v1.8B                                 \n\t"
      "UXTL  v10.8H, v2.8B                                \n\t"
      "UXTL  v11.8H, v3.8B                                \n\t"

      "UQSHL v8.8H, v8.8H, #1                             \n\t"
      "UQSHL v9.8H, v9.8H, #1                             \n\t"
      "UQSHL v10.8H, v10.8H, #1                           \n\t"
      "UQSHL v11.8H, v11.8H, #1                           \n\t"

      "ADD   v4.8H, v4.8H, v8.8H                          \n\t"
      "ADD   v5.8H, v5.8H, v9.8H                          \n\t"
      "ADD   v6.8H, v6.8H, v10.8H                         \n\t"
      "ADD   v7.8H, v7.8H, v11.8H                         \n\t"

      //9.step
      "add       x6, x6, #1                               \n\t"

      "LD1   {v0.8B, v1.8B, v2.8B,v3.8B},  [x6]           \n\t"

      "UXTL  v8.8H, v0.8B                                 \n\t"
      "UXTL  v9.8H, v1.8B                                 \n\t"
      "UXTL  v10.8H, v2.8B                                \n\t"
      "UXTL  v11.8H, v3.8B                                \n\t"

      "ADD   v4.8H, v4.8H, v8.8H                          \n\t"
      "ADD   v5.8H, v5.8H, v9.8H                          \n\t"
      "ADD   v6.8H, v6.8H, v10.8H                         \n\t"
      "ADD   v7.8H, v7.8H, v11.8H                         \n\t"

      "subs  %x1, %x1, #16                                \n\t" /*decrementation of offset*/
      "add   %x0, %x0, #32                                \n\t" /*incrementation of source data*/

      "USHR  v0.8H, v4.8H, #4                             \n\t" /*division by 16*/
      "USHR  v1.8H, v5.8H, #4                             \n\t"
      "USHR  v2.8H, v6.8H, #4                             \n\t"
      "USHR  v3.8H, v7.8H, #4                             \n\t"

      "XTN   v0.8B, v0.8H                                 \n\t" /*narrowing */
      "XTN   v1.8B, v1.8H                                 \n\t"
      "XTN   v2.8B, v2.8H                                 \n\t"
      "XTN   v3.8B, v3.8H                                 \n\t"

      "ST1   {v0.8B, v1.8B, v2.8B,v3.8B},  [%[dst]], #32  \n\t"

      "subs  %[simd_it], %[simd_it], #1                   \n\t"
      "bne   1b                                           \n\t"

      :
      : [src] "r"(src + width + 1), [offset] "r"(offset), [dst] "r"(dst + width), [simd_it] "r"(simd_iteration)
      : "x6", "x7", "x10"

  );
}


inline void reference_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)
{
  int i;
  for (i=0; i<n; i++)
  {
    int r = *src++; // load red
    int g = *src++; // load green
    int b = *src++; // load blue 

    // build weighted average:
    int y = (r*77)+(g*151)+(b*28);

    // undo the scale by 256 and write to memory:
    *dest++ = (y>>8);
  }
}




inline void neon_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)
{
  int i;
  uint8x8_t rfac = vdup_n_u8 (77);
  uint8x8_t gfac = vdup_n_u8 (151);
  uint8x8_t bfac = vdup_n_u8 (28);
  n/=8;

  for (i=0; i<n; i++)
  {
    uint16x8_t  temp;
    uint8x8x3_t rgb  = vld3_u8 (src);
    uint8x8_t result;

    temp = vmull_u8 (rgb.val[0],      rfac);
    temp = vmlal_u8 (temp,rgb.val[1], gfac);
    temp = vmlal_u8 (temp,rgb.val[2], bfac);

    result = vshrn_n_u16 (temp, 8);
    vst1_u8 (dest, result);
    src  += 8*3;
    dest += 8;
  }
}


















