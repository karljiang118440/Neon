

������ NEON intrinsics �ĵ��� 


1��Neon ��ô��֤��ʹ���ˣ���Ҫ��Neon �Ŀ�λ�ô�log ֤����ִ���ˡ�






2����� Neon ͷ�ļ�������û�б���

--- #include <arm_neon.h> ��ӽ�ȥ����   
    �ܹ�����ͨ�������ǲ�û����ص�˵����������һ���������صĴ�������˵��.







3�����ֱ���� demo ���� Neon �����Ƿ��ܹ��ɹ���

---�����һ��С���򣬲鿴Ч����Ρ�

3.1��RGB �� YUV ��ʽ��Neon ��ʵ�֣�


Gaussian(unsigned char* src, unsigned char* dst, int width, int size);

BGR888ToYUV444(unsigned char * __restrict__ yuv, unsigned char * __restrict__ bgr, int pixel_num)

��Ӧ��Ŀ�꺯���� src ��Ӧ __restrict__ yuv 


  *****�����޸Ĳ���
        
  const char  inputImgName1[] = INPUT_ROOT "in_color_256x256.png";

  Mat in1 = imread(inputImgName1);



3.2��RGB �� YUV ��ʽ��opencv ��ʵ�֣�

    cvCvtColor ����ʵ��

	cv::Mat lInput0 = cv::imread(INPUT_ROOT"in_color_256x256.png");  
	cv::Mat lOutout0;
	cvtColor(lInput0, lOutout0, CV_RGB2YUV);




4������ʱ����жԱȣ�Խ�в��Խ�ã�

���  Clock ����ʱ��ĺ���
using Clock = std::chrono::high_resolution_clock;


�����

RGB2YUV_neon run time = 0.364ms
RGB2YUV_opencv run time = 10.4885ms















����neon Դ��鿴

D:\NXP\VisionSDK_S32V2_RTM_1_2_0\s32v234_sdk\libs\utils\neon\src

neon_memcpy.cpp ��ʹ��







�ġ��Լ��Ĵ�������



4.1��BGR888ToYUV444_noSIMD

RGB2YUV_ no neon run time = 0.3665ms
RGB2YUV_neon run time = 0.39625ms


4.2���ܵ�ָ������ʱ��

����δ������ǲ��ѷ��֣�32-bit��float���㱻16-bit�ļӼ����˷�����λ���������档�����Ļ������ǿ��԰����128/16=8���������ŵ�Q-register����SIMD���㣬һ����8��BGR���8��YUV�����������̶�����һ����ʹ���������㻹��һ���ô���һ����ԣ���������ָ������Ҫ��ʱ���������ڸ��������ʱ�����ڡ����ԣ�������δ���Ϊ��׼��baseline������NEON������������ϸ�ĵĿ���Ҳ���Ѿ�������˵���е��©����Ȼ��������ָ�������С�ڵ�����ͬ����ĸ���ָ������ڣ������Ͱ汾��BGR888ToYUV444���𸡵�汾�Ķ�����λ�ͼӷ���overhead��ָ����Ŀ�ǲ�ͬ�ģ��ܵ�ʱ�����ڲ�һ���Ͷ̡�













