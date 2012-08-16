#ifndef _NS115_ARCH_COMMON_H_
#define	_NS115_ARCH_COMMON_H_


#define S(p0,p1,p2,p3) ((p0)|(p1<<2)|(p2<<4)|(p3<<6))
#define PINMUX(s0,s1,s2,s3) ((s0)|(s1<<8)|(s2<<16)|(s3<<24))

#define NS115_PINMUX_SIZE 10
#define NS115_PINMUX(board) unsigned int ns115_##board##_pinmux[NS115_PINMUX_SIZE] 
#define NS115_PINMUX_DECLARE(board) extern unsigned int ns115_##board##_pinmux[NS115_PINMUX_SIZE]
#define NS115_PINMUX_INIT(board) pinmux_init(ns115_##board##_pinmux, \
					ARRAY_SIZE(ns115_##board##_pinmux))

extern int pinmux_init(unsigned int * setting, int len);

extern void common_init(void);
extern void ddr_pm_init(void);
extern void scm_init(void);
#endif
