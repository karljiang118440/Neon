

二、对 NEON intrinsics 的调用 


1、Neon 怎么被证明使用了？需要在Neon 的库位置打log 证明被执行了。






2、添加 Neon 头文件能运行没有报错。

--- #include <arm_neon.h> 添加进去编译   
    能够编译通过，但是并没有相关的说明。可以下一步中添加相关的代码运行说明.







3、如果直接在 demo 中跑 Neon 工程是否能够成功？

---跑随便一个小程序，查看效率如何。

3.1、RGB → YUV 格式（Neon 中实现）


Gaussian(unsigned char* src, unsigned char* dst, int width, int size);

BGR888ToYUV444(unsigned char * __restrict__ yuv, unsigned char * __restrict__ bgr, int pixel_num)

对应的目标函数中 src 对应 __restrict__ yuv 


  *****代码修改部分
        
  const char  inputImgName1[] = INPUT_ROOT "in_color_256x256.png";

  Mat in1 = imread(inputImgName1);



3.2、RGB → YUV 格式（opencv 中实现）

    cvCvtColor 函数实现

	cv::Mat lInput0 = cv::imread(INPUT_ROOT"in_color_256x256.png");  
	cv::Mat lOutout0;
	cvtColor(lInput0, lOutout0, CV_RGB2YUV);




4、测量时间进行对比（越有差距越好）

添加  Clock 计算时间的函数
using Clock = std::chrono::high_resolution_clock;


结果：

RGB2YUV_neon run time = 0.364ms
RGB2YUV_opencv run time = 10.4885ms















三、neon 源码查看

D:\NXP\VisionSDK_S32V2_RTM_1_2_0\s32v234_sdk\libs\utils\neon\src

neon_memcpy.cpp 的使用







四、自己的代码运行



4.1、BGR888ToYUV444_noSIMD

RGB2YUV_ no neon run time = 0.3665ms
RGB2YUV_neon run time = 0.39625ms


4.2、总的指令运行时间

从这段代码我们不难发现，32-bit的float运算被16-bit的加减、乘法和移位运算所代替。这样的话，我们可以把最多128/16=8个整型数放到Q-register中做SIMD运算，一次拿8个BGR算出8个YUV，把向量化程度再提一倍。使用整型运算还有一个好处：一般而言，整型运算指令所需要的时钟周期少于浮点运算的时钟周期。所以，我以这段代码为基准（baseline），用NEON来加速它。（细心的看官也许已经看到我说法中的纰漏：虽然单个整型指令的周期小于单个相同运算的浮点指令的周期，但整型版本的BGR888ToYUV444比起浮点版本的多了移位和加法的overhead，指令数目是不同的，总的时钟周期不一定就短。













