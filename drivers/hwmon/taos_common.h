/*******************************************************************************
*                                                                              *
*       File Name:      taos_common.h                                          *
*       Description:    Common file for ioctl and configuration definitions.   *
*       		Used by kernel driver and driver access applications.  *
*       		Please include this file, and <sys/ioctl.h> in your    *
*                       driver access application program source.	       *
*       Author:         John Koshi                                             *
*       History:        09/16/2009 - Initial creation                          *
*       		02/07/2010 - Add proximity			       *
*                                                                              *
********************************************************************************
*       Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074        *
*******************************************************************************/

// ioctl numbers
#define TAOS_IOCTL_MAGIC        	0XCF
#define TAOS_IOCTL_ALS_ON       	_IO(TAOS_IOCTL_MAGIC, 1)
#define TAOS_IOCTL_ALS_OFF      	_IO(TAOS_IOCTL_MAGIC, 2)
#define TAOS_IOCTL_ALS_DATA     	_IOR(TAOS_IOCTL_MAGIC, 3, short)
#define TAOS_IOCTL_ALS_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 4)
#define TAOS_IOCTL_CONFIG_GET   	_IOR(TAOS_IOCTL_MAGIC, 5, struct taos_cfg)
#define TAOS_IOCTL_CONFIG_SET		_IOW(TAOS_IOCTL_MAGIC, 6, struct taos_cfg)
#define TAOS_IOCTL_PROX_ON		_IO(TAOS_IOCTL_MAGIC, 7)
#define TAOS_IOCTL_PROX_OFF		_IO(TAOS_IOCTL_MAGIC, 8)
#define TAOS_IOCTL_PROX_DATA		_IOR(TAOS_IOCTL_MAGIC, 9, struct taos_prox_info)
#define TAOS_IOCTL_PROX_EVENT           _IO(TAOS_IOCTL_MAGIC, 10)
#define TAOS_IOCTL_PROX_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 11)

#define TAOS_IOCTL_SENSOR_ON	_IO(TAOS_IOCTL_MAGIC, 12)
#define TAOS_IOCTL_SENSOR_OFF	_IO(TAOS_IOCTL_MAGIC, 13)
#define TAOS_IOCTL_SENSOR_CONFIG	_IOW(TAOS_IOCTL_MAGIC, 14, struct taos_cfg)
#define TAOS_IOCTL_SENSOR_CHECK	_IO(TAOS_IOCTL_MAGIC, 15)
#define TAOS_IOCTL_SENSOR_test	_IO(TAOS_IOCTL_MAGIC, 16)
// device name/id/address/counts
#define ALS_PS_INT                      IRQ_NS115_GPIO0_WAKEUP_4
#define TAOS_DEVICE_NAME                "taos"
#define TAOS_DEVICE_ID                  "tritonFN"
#define TAOS_ID_NAME_SIZE               10
#define TAOS_TRITON_CHIPIDVAL           0x00
#define TAOS_TRITON_MAXREGS             32
#define TAOS_DEVICE_ADDR1               0x29
#define TAOS_DEVICE_ADDR2               0x39
#define TAOS_DEVICE_ADDR3               0x49
#define TAOS_MAX_NUM_DEVICES            3
#define TAOS_MAX_DEVICE_REGS            32
// TRITON register offsets
#define TAOS_TRITON_CNTRL               0x00
#define TAOS_TRITON_ALS_TIME            0X01
#define TAOS_TRITON_PRX_TIME            0x02
#define TAOS_TRITON_WAIT_TIME           0x03
#define TAOS_TRITON_ALS_MINTHRESHLO     0X04
#define TAOS_TRITON_ALS_MINTHRESHHI     0X05
#define TAOS_TRITON_ALS_MAXTHRESHLO     0X06
#define TAOS_TRITON_ALS_MAXTHRESHHI     0X07
#define TAOS_TRITON_PRX_MINTHRESHLO     0X08
#define TAOS_TRITON_PRX_MINTHRESHHI     0X09
#define TAOS_TRITON_PRX_MAXTHRESHLO     0X0A
#define TAOS_TRITON_PRX_MAXTHRESHHI     0X0B
#define TAOS_TRITON_INTERRUPT           0x0C
#define TAOS_TRITON_PRX_CFG             0x0D
#define TAOS_TRITON_PRX_COUNT           0x0E
#define TAOS_TRITON_GAIN                0x0F
#define TAOS_TRITON_REVID               0x11
#define TAOS_TRITON_CHIPID              0x12
#define TAOS_TRITON_STATUS              0x13
#define TAOS_TRITON_ALS_CHAN0LO         0x14
#define TAOS_TRITON_ALS_CHAN0HI         0x15
#define TAOS_TRITON_ALS_CHAN1LO         0x16
#define TAOS_TRITON_ALS_CHAN1HI         0x17
#define TAOS_TRITON_PRX_LO              0x18
#define TAOS_TRITON_PRX_HI              0x19
#define TAOS_TRITON_TEST_STATUS         0x1F
// Triton cmd reg masks
#define TAOS_TRITON_CMD_REG             0X80
#define TAOS_TRITON_CMD_AUTO            0x20	//iVIZM  2011.07.21
#define TAOS_TRITON_CMD_BYTE_RW         0x00
#define TAOS_TRITON_CMD_WORD_BLK_RW     0x20
#define TAOS_TRITON_CMD_SPL_FN          0x60
#define TAOS_TRITON_CMD_PROX_INTCLR     0X05
#define TAOS_TRITON_CMD_ALS_INTCLR      0X06
#define TAOS_TRITON_CMD_PROXALS_INTCLR  0X07
#define TAOS_TRITON_CMD_TST_REG         0X08
#define TAOS_TRITON_CMD_USER_REG        0X09
// Triton cntrl reg masks
#define TAOS_TRITON_CNTL_PROX_INT_ENBL  0X20
#define TAOS_TRITON_CNTL_ALS_INT_ENBL   0X10
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL  0X08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL  0X04
#define TAOS_TRITON_CNTL_ADC_ENBL       0x02
#define TAOS_TRITON_CNTL_PWRON          0x01
// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID     0x01
#define TAOS_TRITON_STATUS_PRXVALID     0x02
#define TAOS_TRITON_STATUS_ADCINTR      0x10
#define TAOS_TRITON_STATUS_PRXINTR      0x20
// lux constants
#define TAOS_MAX_LUX                    10000
#define TAOS_SCALE_MILLILUX             3
#define TAOS_FILTER_DEPTH               3
#define CHIP_ID                         0x3d
#define TAOS_INPUT_NAME                 "lightsensor"

// forward declarations
// device configuration
struct taos_cfg {
	u32 calibrate_target;
	u16 als_time;
	u16 scale_factor;
	u16 gain_trim;
	u8 filter_history;
	u8 filter_count;
	u8 gain;
	u16 prox_threshold_hi;
	u16 prox_threshold_lo;
	u16 als_threshold_hi;
	u16 als_threshold_lo;
	u8 prox_int_time;
	u8 prox_adc_time;
	u8 prox_wait_time;
	u8 prox_intr_filter;
	u8 prox_config;
	u8 prox_pulse_cnt;
	u8 prox_gain;
};

// proximity data
struct taos_prox_info {
	u16 prox_clear;
	u16 prox_data;
	int prox_event;
};
