

#1��RGB �� GRAY��ʽ

1.1���ο��ĵ���https://blog.csdn.net/jacke121/article/details/55258262

void neon_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)


void reference_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)

#2��ʱ�����

neon_convert run time = 0.0075ms
reference_convert run time = 0.012167ms

���ٴ�ԼΪ1.5�����ҡ�



##2.1�����԰�����ȫû���κ�˵������

-- ʵ���������ڼ���ʱ������µ����⣬������ʱ��̫�ࡣ



RGB2YUV_ no neon run time = 0.045084ms
RGB2YUV_neon run time = 0.028166ms





ԭ�ĵ��и�����test��

C-version:       15.1 cycles per pixel.
NEON-version:     9.9 cycles per pixel.
That��s only a speed-up of factor 1.5. 




#3�����ָ��˵��

##3.1��ԭ�ĵ��е�˵��
  C-version:       15.1 cycles per pixel.
  NEON-version:     9.9 cycles per pixel.
  Assembler:        2.0 cycles per pixel.









