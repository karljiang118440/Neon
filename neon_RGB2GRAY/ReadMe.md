

#1、RGB → GRAY格式

1.1、参考文档：https://blog.csdn.net/jacke121/article/details/55258262

void neon_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)


void reference_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)

#2、时间测量

neon_convert run time = 0.0075ms
reference_convert run time = 0.012167ms

加速大约为1.5倍左右。



##2.1、不对啊，完全没有任何说服力。

-- 实际上是由于计算时间错误导致的问题，包含的时间太多。



RGB2YUV_ no neon run time = 0.045084ms
RGB2YUV_neon run time = 0.028166ms





原文档中给出的test：

C-version:       15.1 cycles per pixel.
NEON-version:     9.9 cycles per pixel.
That’s only a speed-up of factor 1.5. 




#3、汇编指令说明

##3.1、原文档中的说明
  C-version:       15.1 cycles per pixel.
  NEON-version:     9.9 cycles per pixel.
  Assembler:        2.0 cycles per pixel.









