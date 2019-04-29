

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








inline void Gaussian(unsigned char* src, unsigned char* dst, int width, int size);
inline void BGR888ToYUV444(unsigned char * __restrict__ yuv, unsigned char * __restrict__ bgr, int pixel_num);
inline void BGR888ToYUV444_noSIMD(unsigned char * __restrict__ yuv, unsigned char * __restrict__ bgr, int pixel_num);
//inline void gaussian3x3(uchar *p_u8Src, uchar *p_u8Dst, uint u32Rows, uint u32Cols);



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







	double t3 = (double)cvGetTickCount();
	
   // GaussianBlur(inputImage, outputImage, cols, size);
	cv::GaussianBlur(in, out1, Size(3, 3), 3, 3);

	t3 = (double)cvGetTickCount() - t3;
	printf( "GaussianBlur run time = %gms\n", t3/(cvGetTickFrequency()*1000) );















	//***** add the BGR888ToYUV444 no neon
	
	
	

	
		unsigned char* inputImage0	= (unsigned char*)(in0.data);
		Mat 		   out0 		= in0.clone();
		unsigned char* outputImage0 = (unsigned char*)(out0.data);
	
		Size s0    = in0.size();
		int  rows0 = s0.height;
		int  cols0 = s0.width;
		int  size0 = rows0 * cols;


		double t0 = (double)cvGetTickCount();
		
		BGR888ToYUV444_noSIMD(outputImage0, inputImage0, rows0);
	
	
		//	算法过程
		t0 = (double)cvGetTickCount() - t0;
		printf( "RGB2YUV_ no neon run time = %gms\n", t0/(cvGetTickFrequency()*1000) );
	
	
	
	//************************ add the BGR888ToYUV444 no neon









//stopwatch(true);

//***** add the BGR888ToYUV444 by Neon





    unsigned char* inputImage1  = (unsigned char*)(in1.data);
    Mat            out1         = in1.clone();
    unsigned char* outputImage1 = (unsigned char*)(out1.data);

    Size s1    = in1.size();
    int  rows1 = s1.height;
    int  cols1 = s1.width;
    int  size1 = rows1 * cols;

	double t1 = (double)cvGetTickCount();

    BGR888ToYUV444(outputImage1, inputImage1, rows1);


	//	算法过程
	t1 = (double)cvGetTickCount() - t1;
	printf( "RGB2YUV_neon run time = %gms\n", t1/(cvGetTickFrequency()*1000) );



//************************ add the BGR888ToYUV444


//stopwatch(false, "BGR888ToYUV444 by Neon");





//stopwatch(true);

//***** add the BGR888ToYUV444 by opencv


	double t = (double)cvGetTickCount();


	
	//vsdk::UMat lInput0 = cv::imread(INPUT_ROOT"in_color_256x256.png");  
	cv::Mat lInput0 = cv::imread(INPUT_ROOT"in_color_256x256.png");  
	cv::Mat lOutout0;
	cvtColor(lInput0, lOutout0, CV_RGB2YUV);




	//	算法过程
	t = (double)cvGetTickCount() - t;
	printf( "RGB2YUV_opencv run time = %gms\n", t/(cvGetTickFrequency()*1000) );
	//printf( "run time = %gs\n", t/(cvGetTickFrequency()*1000000) );






	

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

    imwrite( "RGB2YUV_neon.png", out1);

    imwrite( "RGB2YUV_opencv.png", lOutout0);

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
inline void BGR888ToYUV444_noSIMD(unsigned char *yuv, unsigned char *bgr, int pixel_num)
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


















