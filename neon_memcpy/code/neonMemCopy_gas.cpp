void __attribute__ ((noinline)) neonMemCopy_gas(unsigned char* src, unsigned char* dst, int num_bytes)
{
	// source <a target=_blank href="http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka13544.html">http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka13544.html</a>
	asm(
	"neoncopypld:\n"
        "		pld 		[r0, #0xC0]\n" //Ԥȡ����
	"		vldm 		r0!,{d0-d7}\n" //�Ӳ���һr0��src������8*8=64����ͨ��8λ����
    	"		vstm 		r1!,{d0-d7}\n" //�洢��Ŀ�ĵ�ַr1��dst���У�ͬ����64��8λ��ͨ��8λ����
    	"		subs 		r2,r2,#0x40\n" //ѭ����ת������ÿ�μ�64���ܹ�ѭ������=row*col*4/64
    	"		bge 		neoncopypld\n"
	);

}