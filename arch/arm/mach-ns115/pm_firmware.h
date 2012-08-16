#ifndef _PM_FIRMWARE_H_
#define _PM_FIRMWARE_H_

#define RESUME_ENTRY 	0x05800FFC
#define SYS_STAT	0x05800FF8
#define USAGE_CNT	0x05800FF4
#define FW_STAT		0x05800FF0
#define FW_READY	0xA115B115
#define PM_REQ_ENTRY	0x05800000
/*
 * pm req:
 * set SYS_STAT with request mode.
 * jump to PM_REQ_ENTRY.
 */

#endif
