//
// Copyright (C) ENE TECHNOLOGY INC. 2009

#ifndef _ENEEC_IOC_H_
#define _ENEEC_IOC_H_

typedef struct RWREGS_PARAM
{
    unsigned short start_reg;
    unsigned int byte_cnt;
    unsigned char *buf;
} RWREGS_PARAM;

struct PWPARM
{
	unsigned char cmd;
	unsigned char data;	/*set value or read value*/
};
#define ENEEC_IOC_MAGIC             ('e' + 0x80)

#define ENEEC_IOC_GET_CHIPID        _IOR (ENEEC_IOC_MAGIC,  1, unsigned long)  // *arg=output chipIdL-chipIdH-revId.
#define ENEEC_IOC_ENTER_CODE_IN_RAM _IO  (ENEEC_IOC_MAGIC,  2)
#define ENEEC_IOC_EXIT_CODE_IN_RAM  _IO  (ENEEC_IOC_MAGIC,  3)
#define ENEEC_IOC_ENTER_CODE_IN_ROM _IO  (ENEEC_IOC_MAGIC,  4)
#define ENEEC_IOC_EXIT_CODE_IN_ROM  _IO  (ENEEC_IOC_MAGIC,  5)
#define ENEEC_IOC_READ_REG          _IOWR(ENEEC_IOC_MAGIC, 11, unsigned short) // *arg=input regL-regH, also, =output byte.
#define ENEEC_IOC_WRITE_REG         _IOW (ENEEC_IOC_MAGIC, 12, unsigned long)  // *arg=input regL-regH-byte.
#define ENEEC_IOC_READ_REGS         _IOWR(ENEEC_IOC_MAGIC, 13, struct RWREGS_PARAM)
#define ENEEC_IOC_WRITE_REGS        _IOW (ENEEC_IOC_MAGIC, 14, struct RWREGS_PARAM)
#define ENEEC_IOC_PW        	    _IOWR (ENEEC_IOC_MAGIC, 16, struct PWPARM)
struct io373x;
extern struct class * getclass(struct io373x *io373x);

#endif // _ENEEC_IOC_H_
