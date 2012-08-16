#ifndef _NS2416_POWER_MANAGEMENT_H_
#define _NS2416_POWER_MANAGEMENT_H_

/*
 * spooled CPU context is 1KB / CPU 
 */
#define CTX_SP      0   
#define CTX_CPSR    4   
#define CTX_SPSR    8   
#define CTX_CPACR   12  
#define CTX_CSSELR  16  
#define CTX_SCTLR   20  
#define CTX_ACTLR   24  
#define CTX_PCTLR   28  

#define CTX_FPEXC   32  
#define CTX_FPSCR   36  
#define CTX_DIAGNOSTIC  40

#define CTX_TTBR0   48  
#define CTX_TTBR1   52  
#define CTX_TTBCR   56  
#define CTX_DACR    60  
#define CTX_PAR     64  
#define CTX_PRRR    68  
#define CTX_NMRR    72  
#define CTX_VBAR    76  
#define CTX_CONTEXTIDR  80  
#define CTX_TPIDRURW    84  
#define CTX_TPIDRURO    88  
#define CTX_TPIDRPRW    92  

#define CTX_SVC_SP  0
#define CTX_SVC_LR  -1  @ stored on stack
#define CTX_SVC_SP  0
#define CTX_SVC_LR  -1  @ stored on stack
#define CTX_SVC_SPSR    8

#define CTX_SYS_SP  96
#define CTX_SYS_LR  100

#define CTX_ABT_SPSR    112
#define CTX_ABT_SP  116
#define CTX_ABT_LR  120

#define CTX_UND_SPSR    128
#define CTX_UND_SP  132
#define CTX_UND_LR  136

#define CTX_IRQ_SPSR    144
#define CTX_IRQ_SP  148
#define CTX_IRQ_LR  152

#define CTX_FIQ_SPSR    160
#define CTX_FIQ_R8  164
#define CTX_FIQ_R9  168
#define CTX_FIQ_R10 172
#define CTX_FIQ_R11 178
#define CTX_FIQ_R12 180
#define CTX_FIQ_SP  184
#define CTX_FIQ_LR  188

/* context only relevant for master cpu */
#ifdef CONFIG_CACHE_L2X0
#define CTX_L2_CTRL 224
#define CTX_L2_AUX  228
#define CTX_L2_TAG_CTRL 232
#define CTX_L2_DAT_CTRL 236
#define CTX_L2_PREFETCH 240
#endif

#define CTX_VFP_REGS    256
#define CTX_VFP_SIZE    (32 * 8)

#define CTX_CP14_REGS   512
#define CTS_CP14_DSCR   512
#define CTX_CP14_WFAR   516
#define CTX_CP14_VCR    520
#define CTX_CP14_CLAIM  524

/* Each of the folowing is 2 32-bit registers */
#define CTS_CP14_BKPT_0 528
#define CTS_CP14_BKPT_1 536
#define CTS_CP14_BKPT_2 544
#define CTS_CP14_BKPT_3 552
#define CTS_CP14_BKPT_4 560
#define CTS_CP14_BKPT_5 568

/* Each of the folowing is 2 32-bit registers */
#define CTS_CP14_WPT_0  576
#define CTS_CP14_WPT_1  584
#define CTS_CP14_WPT_2  592
#define CTS_CP14_WPT_3  600

#endif /* _NS2416_POWER_MANAGEMENT_H_ */
