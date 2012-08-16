
//SFR
#define CTRL (0x00)  	//6'b000000 01900000H I2S Control Register
#define STAT (0x04)	//6'b000100 00001100H I2S Status Register
#define SRR  (0x08)	//6'b001000 00000000H I2S Channels Sample Rate & Resolution Configuration Register
#define CID_CTRL (0x0C)	//6'b001100 00000000H Clock, Interrupt and DMA Control Register
#define TFIFO_STAT (0x10)	//6'b010000 00000000H Transmit FIFO Status Register
#define RFIFO_STAT (0x14)	//6'b010100 00000000H Receive FIFO Status Register
#define TFIFO_CTRL (0x18)	//6'b011000 000F0000H1 Transmit FIFO Control Register
#define RFIFO_CTRL (0x1C)	//6'b011100 000F0000H1 Receive FIFO Control Register
#define DEV_CONF   (0x20)	//6'b100000 00000208H2 Device Configuration Register

#define FIFO_ADDR (0x1000)

#define T_MS (1<<21)
#define R_MS (1<<22)
#define TFIFO_RST (1<<23)
#define RFIFO_RST (1<<24)
#define RSYNC_LOOP_BACK (1<<28)   // 1 = normal mode
#define TSYNC_LOOP_BACK (1<<27)

#define TSYNC_RST (1<<25)
#define RSYNC_RST (1<<26)
#define LOOPBACK_0_1 (1<<16)

#define EN_CHAN(x) (1 << x)
#define TR_CHAN(x) (1 << (x+8))

#define SR(s,r)         (s|((r-1)<<11))
#define SSR(s,r)        (SR(s,r)|(SR(s,r)<<16))

#define FIFOC(ae,af)    (ae|(af<<16)) 

//#define        PRINT_DBG_INFO

#ifdef  PRINT_DBG_INFO 
        #define DBG_PRINT(fmt, args...) printk( KERN_INFO "nusmart i2s: " fmt, ## args) 
#else
        #define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif


#define NUSMART_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
                SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
                SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

#define NUSMART_I2S_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
                         SNDRV_PCM_FMTBIT_S24_LE | \
                         SNDRV_PCM_FMTBIT_S32_LE)

extern struct snd_soc_dai_driver eva_i2s_dai;
