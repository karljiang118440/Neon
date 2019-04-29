void __attribute__ ((noinline)) neonMemCopy_gas(unsigned char* src, unsigned char* dst, int num_bytes)
{
	// source <a target=_blank href="http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka13544.html">http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka13544.html</a>
	asm(
	"neoncopypld:\n"
        "		pld 		[r0, #0xC0]\n" //预取数据
	"		vldm 		r0!,{d0-d7}\n" //从参数一r0（src）加载8*8=64个单通道8位数据
    	"		vstm 		r1!,{d0-d7}\n" //存储在目的地址r1（dst）中，同样是64个8位单通道8位数据
    	"		subs 		r2,r2,#0x40\n" //循环跳转参数，每次减64，总共循环次数=row*col*4/64
    	"		bge 		neoncopypld\n"
	);

}