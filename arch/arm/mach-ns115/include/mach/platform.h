#ifndef _ASM_ARCH_PLATFORM_NS115_H
#define _ASM_ARCH_PLATFORM_NS115_H

/*##############################################################*/
/*              NS115 Registers define                   */
/*##############################################################*/

/*
 * Timer and Watchdog registers
 */

#define NS115_SYS_TIMER_LOAD                    0x00
#define NS115_SYS_TIMER_COUNTER                 0x04
#define NS115_SYS_TIMER_CONTROL                 0x08
#define NS115_SYS_TIMER_INT_STATUS              0x0C
#define NS115_SYS_WATCHDOG_LOAD                 0x20
#define NS115_SYS_WATCHDOG_COUNTER              0x24
#define NS115_SYS_WATCHDOG_CONTROL              0x28
#define NS115_SYS_WATCHDOG_INT_STATUS           0x2C
#define NS115_SYS_WATCHDOG_RESET                0x30
#define NS115_SYS_WATCHDOG_DISABLE              0x34

#define NS115_SYS_WATCHDOG_CONTROL_ENABLE       (0x1 << 0)
#define NS115_SYS_WATCHDOG_CONTROL_MODE         (0x1 << 3)


/*
 * System controller bit assignment
 */
#define TICKS_PER_uSEC                          12

/* 
 *  These are useconds NOT ticks.  
 * 
 */
#define mSEC_1                                  1000
#define mSEC_5                                  (mSEC_1 * 5)
#define mSEC_10                                 (mSEC_1 * 10)
#define mSEC_25                                 (mSEC_1 * 25)
#define SEC_1                                   (mSEC_1 * 1000)





///////////////delete later
//#define REALVIEW_SYS_BASE                     0x10000000
//#define REALVIEW_SYS_LED_OFFSET               0x08
//#define REALVIEW_SYS_LED                      (REALVIEW_SYS_BASE + REALVIEW_SYS_LED_OFFSET)
//#define REALVIEW_SYS_LED7                     (1 << 7)
//#define REALVIEW_SCTL_BASE                    0x10001000/* System controller */

//#define REALVIEW_SYS_RESETCTL_OFFSET          0x40
//#define REALVIEW_SYS_RESETCTL                 (REALVIEW_SYS_BASE + REALVIEW_SYS_RESETCTL_OFFSET)

//#define REALVIEW_SYS_LOCK_VAL                 0xA05F       /* Enable write access */
//#define REALVIEW_SYS_LOCK_OFFSET              0x20
//#define REALVIEW_SYS_LOCK                     (REALVIEW_SYS_BASE + REALVIEW_SYS_LOCK_OFFSET)



#endif /* _ASM_ARCH_PLATFORM_NS115_H */
