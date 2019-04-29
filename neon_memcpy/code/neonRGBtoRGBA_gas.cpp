
void __attribute__ ((noinline)) neonRGBtoRGBA_gas(unsigned char* src, unsigned char* dst, int numPix) 
{ 
	asm( 
		// numpix/8              
		"        mov      r2,  r2, lsr #3\n"   // numpix/8    逻辑左移三位再赋值，为什么要除以8下面会详解
		// load alpha channel value
		"        vmov.u8  d3, #0xff\n"         //额外增加的CvMat的第四通道alpha通道，这是在HLS中AXI转化为MAT结构，见图3
		"loop1:\n"                             //循环开始
		// load 8 rgb pixels with deinterleave //见图2，及分析
		"        vld3.8   {d0,d1,d2}, [r0]!\n"
		// preload next values                 //预取在下一次循环中要用到的数据
		"        pld      [r0,#40]\n"
		"        pld      [r0,#48]\n"
		"        pld      [r0,#56]\n"
		// substract loop counter              
		"        subs     r2, r2, #1\n"        //一次循环操作就可以取走24个8位单通道像素,也就是8个R,R个G，8个B，循环次数row*col/8
		//"        vswp     d0, d2\n"          
		// store as 4*8bit values
		"        vst4.8   {d0-d3}, [r1]!\n"    //将VLD的三通道数据D0-D2连同增加的alpha通道一同写入到目的地址r1（dst）中
		// loop if not ready
		"        bgt      loop1\n"             //循环跳转判断
	); 

}