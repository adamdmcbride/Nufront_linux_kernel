 #ifndef SDIO_HOST_H
#define SDIO_HOST_H

enum evatronix_state {          
        STATE_IDLE = 0,      
        STATE_SENDING_CMD,           
        STATE_BUSY_RSP,           
        STATE_SENDING_DATA,                  

       	STATE_DATA_BUSY,                     
        STATE_SENDING_STOP,
        STATE_DATA_ERROR,                    
};

enum {                                       
        EVENT_CMD_COMPLETE = 0,                      
        EVENT_XFER_COMPLETE,
	EVENT_CMD_ERROR,
	EVENT_DATA_ERROR,

	EVENT_CARD_INTR,
	EVENT_DMA_FIN,
	EVENT_CARD_DETECT,
	EVENT_DMA_ERROR,
};
       


typedef struct DmaDescStruct {  
	unsigned int desc_1;           
	unsigned int desc_2;           
}DmaDesc;


#define EVATRONIX_DMA_SUPPORT

#define EVATRONIX_MAX_SLOTS		4
#define EVATRONXI_MAX_DIVIDER_VALUE	250
#define DEBOUNCING_TIME         	0x300000ul
#define COMMANDS_TIMEOUT        	0x100000ul


#define SFR11_TIMEOUT_MASK                  0x00F0000ul



#define SDMA_MODE       0x00
#define ADMA1_MODE      0x01
#define ADMA2_MODE      0x02
#define NONEDMA_MODE    0xFF
//XXX
#define EVATRONIX_DMA_SIZE	PAGE_SIZE


#define GET_BASE_CLK_FREQ_MHZ(val) ( ((val)>> 8) & 0x3F )


#define SFR10_DMA_SELECT_SDMA   (0x0 << 3)                                          
#define SFR10_DMA_SELECT_ADMA1  (0x1 << 3)                                          
#define SFR10_DMA_SELECT_ADMA2  (0x2 << 3)                                          
#define SFR10_DMA_SELECT_MASK   (0x3 << 3)                                              
                                                                                        
#define SFR11_INT_CLOCK_ENABLE             0x0000001ul
#define SFR11_SEL_FREQ_BASE_MASK            0x000FF00ul


#define ADMA2_DESCRIPTOR_LENGHT(val) ((val) << 16)
#define ADMA2_DESCRIPTOR_END 		(0x1 << 1)


//#define SDIO_REG_OFFSET		0xc5070000	

#define SDIO_REG_HSFR0 			0x0000
#define SDIO_REG_HSFR1  		0x0004
#define SDIO_REG_HSFR2 			0x0008


#define SDIO_REG_SLOT_SFR0_OFFSET       0x00
#define SDIO_REG_SLOT_SFR1_OFFSET       0x04
#define SDIO_REG_SLOT_SFR2_OFFSET       0x08
#define SDIO_REG_SLOT_SFR3_OFFSET       0x0C
#define SDIO_REG_SLOT_SFR4_OFFSET       0x10
#define SDIO_REG_SLOT_SFR5_OFFSET       0x14
#define SDIO_REG_SLOT_SFR6_OFFSET       0x18
#define SDIO_REG_SLOT_SFR7_OFFSET       0x1C
#define SDIO_REG_SLOT_SFR8_OFFSET       0x20
#define SDIO_REG_SLOT_SFR9_OFFSET       0x24
#define SDIO_REG_SLOT_SFR10_OFFSET      0x28
#define SDIO_REG_SLOT_SFR11_OFFSET      0x2C
#define SDIO_REG_SLOT_SFR12_OFFSET      0x30
#define SDIO_REG_SLOT_SFR13_OFFSET      0x34
#define SDIO_REG_SLOT_SFR14_OFFSET      0x38
#define SDIO_REG_SLOT_SFR15_OFFSET      0x3C
#define SDIO_REG_SLOT_SFR16_OFFSET      0x40
#define SDIO_REG_SLOT_SFR17_OFFSET      0x44
#define SDIO_REG_SLOT_SFR18_OFFSET      0x48
#define SDIO_REG_SLOT_SFR20_OFFSET      0x50
#define SDIO_REG_SLOT_SFR21_OFFSET      0x54
#define SDIO_REG_SLOT_SFR22_OFFSET      0x58
#define SDIO_REG_SLOT_SFR28_OFFSET      0x70


#define SDIO_REG_COMM_SFR63     	0x00FC

#define host_readl(port,reg)   readl((port)->virt_base + SDIO_REG_##reg)
#define host_writel(port, reg, val) writel((u32)(val), (port)->virt_base + SDIO_REG_##reg)


#define slot_readl(host, reg, id)	\
	__raw_readl((host)->virt_base + SDIO_REG_SLOT_##reg##_OFFSET + (id+1)*(0x100))
#define slot_writel(host, reg, id, val)	\
	__raw_writel((u32)(val), (host)->virt_base + SDIO_REG_SLOT_##reg##_OFFSET + (id+1)*(0x100))

/* verbose debug */
//#define	EVATRONIX_DBG

#define evatronix_pr(x, args...) 	printk(KERN_EMERG x "\n", ##args)
#define err(x, args...) 		printk(KERN_ERR x "\n", ##args)

#ifdef EVATRONIX_DBG
#define VDBG(x, args...)		//printk(KERN_EMERG x "\n", ##args)
#define eng(x, args...) 		printk(KERN_ERR x "\n", ##args)

#define ENTER()		/*\
	printk(KERN_EMERG "L%d: %s", __LINE__, __func__) */
#define LEAVE()		/*\
	printk(KERN_EMERG "=== L%d: %s", __LINE__, __func__) */
#else

#define VDBG(x, arg...) 
#define ENTER()
#define LEAVE()
#define eng(x, args...) 
#endif


#define SFR3_CMD_CICE		0x00100000
#define SFR3_CMD_DPS           	0x00200000
#define SFR3_CMD_DTDS		0x00000010	/* read from card */

#define SFR3_CMD_RSP_136	0x00010000
#define SFR3_CMD_RSP_48		0x00020000
#define SFR3_CMD_RSP_48_BUSY	0x00030000
#define SFR3_CMD_RSP_CRCCE     	0x00080000 


#define SFR3_AUTO_CMD12		0x00000004
#define SFR3_AUTO_CMD23		0x00000008



#define SFR10_DATA_WIDTH_1BIT           0x00000000ul
#define SFR10_DATA_WIDTH_8BIT           0x00000020ul


#define SFR10_SET_3_3V_BUS_VOLTAGE      0x0000E00ul
#define SFR10_SET_3_0V_BUS_VOLTAGE      0x0000C00ul
#define SFR10_SET_1_8V_BUS_VOLTAGE      0x0000A00ul
#define SFR10_BUS_VOLTAGE_MASK              0x0000E00ul
#define SFR10_SD_BUS_POWER		    0x0000100ul	


#define SFR11_SD_CLOCK_ENABLE               0x0000004ul


//#define BIT(nr)				(1<<(nr))
#define SFR12_ETING			BIT(26)
#define SFR12_EADMA			BIT(25)
#define SFR12_EAC			BIT(24)
#define SFR12_ECL			BIT(23)
#define SFR12_EDEB			BIT(22)
#define SFR12_EDCRC			BIT(21)
#define SFR12_EDT			BIT(20)
#define SFR12_ECI			BIT(19)
#define SFR12_ECEB			BIT(18)
#define SFR12_ECCRC			BIT(17)
#define SFR12_ECT			BIT(16)
#define SFR12_INT			BIT(15)

//GAP
#define SFR12_RTNGE			BIT(12)
#define SFR12_INT_C			BIT(11)
#define SFR12_INT_B			BIT(10)
#define SFR12_INT_A			BIT(9)
#define SFR12_CINT			BIT(8)
#define SFR12_CR			BIT(7)
#define SFR12_CIN			BIT(6)
#define SFR12_BRR			BIT(5)
#define SFR12_BWR			BIT(4)
#define SFR12_DMAINT			BIT(3)
#define SFR12_BGE			BIT(2)
#define SFR12_TC			BIT(1)
#define SFR12_CC			BIT(0)






//Interrupt bits
#define EVATRONIX_DATA_ERROR		0x0070000ul
#define EVATRONIX_CMD_ERROR		0x000F000ul
#define EVATRONIX_INT_ON_LINE		0x0000E00ul
#define EVATRONIX_CARD_INT		
#define EVATRONIX_BUFFER_INT		
#define EVATRONIX_DMA_INT
#define EVATRONIX_BLK_GAP_EVNET
#define EVATRONIX_XFER_COMPLETE
#define EVATRONIX_CMD_COMPLETE


//========================================================
#define HSFR0_SOFTWARE_RESET        (1ul << 0)


//========================================================
/// Lock card command
#define SD_MMC_CARD_CONF_CARD_LOCK          0x4
/// Unlock card command
#define SD_MMC_CARD_CONF_CARD_UNLOCK        0x0
/// Set password to the card
#define SD_MMC_CARD_CONF_SET_PASSWORD       0x1
/// Reset password to the card
#define SD_MMC_CARD_CONF_RESET_PASSWORD     0x2









/// Enable interrupt signal support form SDIO Host driver 
#define SDIO_INTERRUPT_SUPPORT  1

/// SDMA support for SDIO host driver
#define SDIO_SDMA_SUPPORTED     1
/// ADMA1 support for SDIO host driver
#define SDIO_ADMA1_SUPPORTED    0
/// ADMA2 support for SDIO host driver
#define SDIO_ADMA2_SUPPORTED    1

/// size of DSR circual buffer, it shouldn't be greater than 127
#define DSR_COUNT               30
/// Debug level possible values (0, 1, 2, 3)
#define DEBUG_LEVEL             1
/// Choose SD bus mode SD or SPI ( currently only SD bus mode is supported )
#define HOST_BUS_MODE           SDIOHOST_BUS_MODE_SD
/// Set debouncing period
#define DEBOUNCING_TIME         0x300000ul
/// Commands timeout is a iteration count after which timeout error will be report
/// if a command will not execute (mainly using in WaitForValue function)
/// Enable or disable detecting on the card interrupt
/// You should enable this option only if you have implemented 
/// procedure to hanlde card interrupt
#define ENABLE_CARD_INTERRUPT   0
/// define this definition if you use Tasking compiler. Define it as 0 otherwise.
#define TASKING                 1
// system clock in Hz 
#define SYTEM_CLK_KHZ (8000uL)





#define ADMA1_SIZE_OF_DESCRIPTOR 32u
#define ADMA2_SIZE_OF_DESCRIPTOR 64u

#define SDMA_MODE       0x00
#define ADMA1_MODE      0x01
#define ADMA2_MODE      0x02
#define NONEDMA_MODE    0xFF

/** ADMA1 Descriptor Fields */

/// Set address/lenght field
#define ADMA1_DESCRIPTOR_ADDRESS_LENGHT(Val) (Val << 12)
/// No operation  go to next descriptor on the list.
#define ADMA1_DESCRIPTOR_TYPE_NOP   (0x0 << 4)
/// Set data page length and go to next descriptor on the list.
#define ADMA1_DESCRIPTOR_TYPE_SET   (0x1 << 4)
/// Transfer data from the pointed page and go to next descriptor on the list.
#define ADMA1_DESCRIPTOR_TYPE_TRAN  (0x2 << 4)
/// Go to the next descriptor list
#define ADMA1_DESCRIPTOR_TYPE_LINK  (0x3 << 4)
/// the ADMA interrupt is generated 
/// when the ADMA1 engine finishes processing the descriptor.
#define ADMA1_DESCRIPTOR_INT        (0x1 << 2)
/// it signals termination of the transfer
/// and generates Transfer Complete Interrupt 
/// when this transfer is completed
#define ADMA1_DESCRIPTOR_END        (0x1 << 1)
/// it indicates the valid descriptor on a list
#define ADMA1_DESCRIPTOR_VAL        (0x1 << 0)

/// Set address/lenght field
/// No operation  go to next descriptor on the list.
#define ADMA2_DESCRIPTOR_TYPE_NOP   (0x0 << 4)
/// Transfer data from the pointed page and go to next descriptor on the list.
#define ADMA2_DESCRIPTOR_TYPE_TRAN  (0x2 << 4)
/// Go to the next descriptor list
#define ADMA2_DESCRIPTOR_TYPE_LINK  (0x3 << 4)
/// the ADMA interrupt is generated 
/// when the ADMA1 engine finishes processing the descriptor.
#define ADMA2_DESCRIPTOR_INT        (0x1 << 2)
/// it signals termination of the transfer
/// and generates Transfer Complete Interrupt 
/// when this transfer is completed
#define ADMA2_DESCRIPTOR_END        (0x1 << 1)
/// it indicates the valid descriptor on a list
#define ADMA2_DESCRIPTOR_VAL        (0x1 << 0)






/// There is no error during execute procedure
#define SDIO_ERR_NO_ERROR                   0x00    
/// Error Host base clock is zero
#define SDIO_ERR_BASE_CLK_IS_ZERO           0x01
/// Error can't power supply
#define SDIO_ERR_NO_POWER_SUPPLY            0x02
/// Error timeout
#define SDIO_ERR_TIMEOUT                    0x03
/// Error during configuration - current limit error
#define SDIO_ERR_CURRENT_LIMIT_ERROR        0x04
/// Error during transfer data - end bit error
#define SDIO_ERR_DATA_END_BIT_ERROR         0x05
/// Error during transfer data - crc bit error
#define SDIO_ERR_DATA_CRC_ERROR             0x06
/// Error during transfer data - timeout error
#define SDIO_ERR_DATA_TIMEOUT_ERROR         0x07
/// Error during execute command - command index error
#define SDIO_ERR_COMMAND_INDEX_ERROR        0x08
/// Error during execute command - command line conflict
#define SDIO_ERR_CMD_LINE_CONFLICT          0x09
/// Error during execute command - timeout error
#define SDIO_ERR_COMMAND_TIMEOUT_ERROR      0x0A
/// Error during execute command - end bit error
#define SDIO_ERR_COMMAND_END_BIT_ERROR      0x0B
/// Error during execute command - CRC error
#define SDIO_ERR_COMMAND_CRC_ERROR          0x0C
/// Card is in inactive state, it is unusable
#define SDIO_ERR_UNUSABLE_CARD              0x0D
/// Wrong argument for SDIO card function
#define SDIO_ERR_PARAM_ERR                  0x0E
/// Wrong SDIO function number error
#define SDIO_ERR_FUN_NUM_ERR                0x0F
/// The CRC check for the previous command failed
#define SDIO_ERR_COM_CRC_ERR                0x10
/// Illegal command error
#define SDIO_ERR_ILLEGAL_CMD                0x11
/// An error in the sequence of erase commands occurred. 
#define SDIO_ERR_ERASE_SEQUENCE_ERR         0x12
/// A general or an unknown error occurred during the operation
#define SDIO_ERR_GENERAL                    0x13
/// A misaligned address that did not match the block length was used in the command
#define SDIO_ERR_ADDRESS_ERR                0x14
/// Can't switch device to high speed mode because it is not supported either by controller or by card
#define SDIO_ERR_HIGH_SPEED_UNSUPP          0x15
/// Command not inssued by auto CMD 12 error 
/// CMD_wo_DAT is not executed due to an Auto CMD12 error (D04-D01)
/// in the Auto CMD12 error status register
#define SIO_ERR_AUTOCMD12_NOT_ISSUED        0x16
/// Command index error occurs in response to a command
#define SIO_ERR_AUTOCMD12_INDEX_ERROR       0x17
/// End bit of command response is 0
#define SIO_ERR_AUTOCMD12_END_BIT_ERROR     0x18
/// Error CRC in the Auto CMD 12 command response
#define SIO_ERR_AUTOCMD12_CRC_ERROR         0x19
/// No response is returned within 64 SDCLK cycles from the end bit of command
/// In this error occur then other error status bits are meaningless
#define SIO_ERR_AUTOCMD12_TIMEOUT           0x1A
/// Host controller cannot issue Auto CMD12 to stop memory multiple 
/// block data transfer due to some error.
#define SIO_ERR_AUTOCMD12_NOT_EXECUTED      0x1B
/// Auto CMD12 error - CMD line conflict
#define SDIO_ERR_AUTOCMD12_LINE_CONFLICT    0x1C
/// Touple not found in CIS register error
#define SDIO_ERR_TUPLE_NOT_FOUND            0x1D
/// Software driver to service inserted card was not implemented
#define SDIO_ERR_DRIVER_NOT_IMPLEMENTED     0x1E
/// Function malloc error
#define SDIO_ERR_MEM_ALLOC                  0x1F               
/// Error DAT line is busy, can't execute command
#define SDIO_ERR_DAT_LINE_BUSY              0x20
/// Error CMD line is busy, can't execute command
#define SDIO_ERR_CMD_LINE_BUSY              0x21
/// Internal card controller error
#define SDIO_ERR_CC_ERROR                   0x22
/// Card internal ECC was applied but failure
#define SDIO_ERR_CARD_ECC                   0x23
/// Sequence or password error has been detected in lock/unlock card command
#define SDIO_ERR_LOCK_UNLOCK_FAILED         0x24
/// Host attempts to write to a protected block or to the temporary or permanent write protected card.
#define SDIO_ERR_WP_VIOLATION               0x25
/// An invalid selection of write-blocks for erase occurred
#define SDIO_ERR_ERASE_PARAM                0x26
/// The transferred block length is not allowed for this card, or the number 
/// of transferred bytes does not match the block length
#define SDIO_ERR_BLOCK_LEN_ERROR            0x27
/// The command s argument was out of the allowed range for this card
#define SDIO_ERR_OUT_OF_RANGE               0x28
/// Unrecoverable error ocured
#define SDIO_ERR_NON_RECOVERABLE_ERROR      0x29
/// An error occurred in CMD_wo_DAT, but not in the SD memory transfer.
#define SDIO_ERR_ACMD12_RECOVERABLE_A       0x2A
/// An error occurred in CMD_wo_DAT, and also occurred in the SD memory transfer
#define SDIO_ERR_ACMD12_RECOVERABLE_B       0x2B
/// An error did not occur in CMD_wo_DAT, but an error occurred in the SD memory transfer
#define SDIO_ERR_ACMD12_RECOVERABLE_C       0x2C
/// CMD_wo_DAT was not issued, and an error occurred in the SD memory transfer
#define SDIO_ERR_ACMD12_RECOVERABLE_D       0x2D
/// Card is card locked by the host
#define SDIO_ERR_CARD_IS_LOCKED             0x2E
/// The card could not sustain data transfer in stream read mode
#define SDIO_ERR_UNDERRUN                   0x2F
/// The card could not sustain data transfer in stream write mode
#define SDIO_ERR_OVERRUN                    0x30
/// Problem with erase part of memory because it is protected
#define SDIO_ERR_WP_ERASE_SKIP              0x31
/// Card didn't switch to the expected mode as requested by the SWITCH command
#define SDIO_ERR_SWITCH_ERROR               0x32
/// Error authentication process
#define SDIO_ERR_AKE_SEQ_ERROR              0x33
/// SLOT is busy can't execute a command
#define SDIO_ERR_SLOT_IS_BUSY               0x34
// Error command is not supported by the card device
#define SDIO_ERR_UNSUPPORTED_COMMAND        0x35
/// Device is null pointer
#define SDIO_ERR_DEV_NULL_POINTER           0x36
/// ADMA error
#define SDIO_ERR_ADMA                       0x37
/// unexepted DMA interrupt (probably wrong descriptor settings)
#define SDIO_ERR_DMA_UNEXCEPTED_INTERRUPT   0x38

/// invalid function parameter
#define SDIO_ERR_INVALID_PARAMETER          0x39
/// error trying write or erase locked card.
#define SDIO_ERR_CARD_WRITE_PROTECTED       0x3A
/// error function is not supported (CMD6 command)
#define SDIO_ERR_FUNCTION_UNSUPP            0x3B

/// Request is currently executing
#define SDIO_STATUS_PENDING                 0xFF
/// Request was aborted
#define SDIO_STATUS_ABORTED                 0xFE
/// Undefined error
#define SDIO_ERR_UNDEFINED                  0xEF
/// Hardware problem error
#define SDIO_ERR_HARDWARE_PROBLEM           0xEE
/// Driver procedure was called with wrong value of parameter
#define SDIO_ERR_WRONG_VALUE                0xED
// Cant execute function
#define SDIO_ERR_CANT_EXECUTE               0xEC









/// Abort CMD12, CMD52 for writing I/O Abort in CCCR 
#define SFR3_ABORT_CMD          0xC00000ul
/// Resume CMD52 for writing Function Select in CCCR
#define SRF3_RESUME_CMD         0x800000ul
///Suspend CMD52 for writing Bus Suspend in CCCR  
#define SFR3_SUSPEND_CMD        0x400000ul
/// data is present and will be transferred using the DAT line
#define SFR3_DATA_PRESENT       0x200000ul
/// index check enable
#define SFR3_INDEX_CHECK_EN     0x100000ul
/// response CRC check enable
#define SFR3_CRC_CHECK_EN       0x080000ul
/// response type - no response
#define SFR3_NO_RESPONSE        0x000000ul
/// response type - response length 136 
#define SFR3_RESP_LENGTH_136    0x010000ul
/// response type - response length 48
#define SFR3_RESP_LENGTH_48     0x020000ul
/// response type - response length 48 and check Busy after response 
#define SFR3_RESP_LENGTH_48B    0x030000ul
/// multi block DAT line data transfers
#define SFR3_MULTI_BLOCK_SEL    0x000020ul
/// data transfer direction - write
#define SFR3_TRANS_DIRECT_WRITE 0x000000ul
/// data transfer direction - read
#define SFR3_TRANS_DIRECT_READ  0x000010ul
/// Auto CMD12 enable
#define SFR3_AUTOCMD12_ENABLE   0x000004ul
/// Block count enable
#define SFR3_BLOCK_COUNT_ENABLE 0x000002ul
/// DMA enable
#define SFR3_DMA_ENABLE         0x000001ul
//@}

//-------------------------------------------------------------------------------------------
/// @name General information register (HSFR0) - masks
//-------------------------------------------------------------------------------------------
//@{
/// 4 slots are accesible in the SDIO host
#define HSFR0_4_ACCESABLE_SLOTS     (0x1uL << 19)
/// 3 slots are accesible in the SDIO host
#define HSFR0_3_ACCESABLE_SLOTS     (0x1uL << 18)
/// 2 slots are accesible in the SDIO host
#define HSFR0_2_ACCESABLE_SLOTS     (0x1uL << 17)
/// 1 slots are accesible in the SDIO host
#define HSFR0_1_ACCESABLE_SLOT      (0x1uL << 16)
/// Reset all internal registers including SFR RAM. The card detection unit is reset also.
#define HSFR0_SOFTWARE_RESET        (1ul << 0)
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Initialization setting register (HSFR1) - masks and macros
//-------------------------------------------------------------------------------------------
//@{
/// SDIO host uses SD mode to initialize card
#define HSFR1_INITIALIZATION_MODE_SD    0x10000000ul
/// SDIO host uses SPI mode to initialize card
#define HSFR1_INITIALIZATION_MODE_SPI   0x00000000ul
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name DMA settings register (HSFR2) - values   
//-------------------------------------------------------------------------------------------
//@{
/// set 2048 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_2048    0xC
/// set 1024 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_1024    0xB
/// set 512 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_512     0xA
/// set 256 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_256     0x9
/// set 128 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_128     0x8
/// set 64 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_64      0x7
/// set 32 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_32      0x6
/// set 16 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_16      0x5
/// set 8 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_8       0x4
/// set 4 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_4       0x3
/// set 2 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_2       0x2
/// set 1 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_1       0x1
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Block count and size register (SFR1) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Block count for current transfer mask
#define SFR1_BLOCK_COUNT            0xFFFF0000ul
/// DMA buffer size 4kB
#define SFR1_DMA_BUFF_SIZE_4KB      0x00000000ul
/// DMA buffer size 8kB
#define SFR1_DMA_BUFF_SIZE_8KB      0x00001000ul
/// DMA buffer size 16kB
#define SFR1_DMA_BUFF_SIZE_16KB     0x00002000ul
/// DMA buffer size 32kB
#define SFR1_DMA_BUFF_SIZE_32KB     0x00003000ul
/// DMA buffer size 64kB
#define SFR1_DMA_BUFF_SIZE_64KB     0x00004000ul
/// DMA buffer size 128kB
#define SFR1_DMA_BUFF_SIZE_128KB    0x00005000ul
/// DMA buffer size 265kB
#define SFR1_DMA_BUFF_SIZE_256KB    0x00006000ul
/// DMA buffer size 512kB
#define SFR1_DMA_BUFF_SIZE_512KB    0x00007000ul
/// DMA buffer size mask            
#define SFR1_DMA_BUFF_SIZE_MASK     0x00007000ul
/// Transfer block size mask
#define SFR1_BLOCK_SIZE             0x00000FFFul
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Transfer mode and command information register (SFR3) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Abort CMD12, CMD52 for writing I/O Abort in CCCR 
#define SFR3_ABORT_CMD          0xC00000ul
/// Resume CMD52 for writing Function Select in CCCR
#define SRF3_RESUME_CMD         0x800000ul
///Suspend CMD52 for writing Bus Suspend in CCCR  
#define SFR3_SUSPEND_CMD        0x400000ul
/// data is present and will be transferred using the DAT line
#define SFR3_DATA_PRESENT       0x200000ul
/// index check enable
#define SFR3_INDEX_CHECK_EN     0x100000ul
/// response CRC check enable
#define SFR3_CRC_CHECK_EN       0x080000ul
/// response type - no response
#define SFR3_NO_RESPONSE        0x000000ul
/// response type - response length 136 
#define SFR3_RESP_LENGTH_136    0x010000ul
/// response type - response length 48
#define SFR3_RESP_LENGTH_48     0x020000ul
/// response type - response length 48 and check Busy after response 
#define SFR3_RESP_LENGTH_48B    0x030000ul
/// multi block DAT line data transfers
#define SFR3_MULTI_BLOCK_SEL    0x000020ul
/// data transfer direction - write
#define SFR3_TRANS_DIRECT_WRITE 0x000000ul
/// data transfer direction - read
#define SFR3_TRANS_DIRECT_READ  0x000010ul
/// Auto CMD12 enable
#define SFR3_AUTOCMD12_ENABLE   0x000004ul
/// Block count enable
#define SFR3_BLOCK_COUNT_ENABLE 0x000002ul
/// DMA enable
#define SFR3_DMA_ENABLE         0x000001ul
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Present state register masks (SFR9) - masks
//-------------------------------------------------------------------------------------------
//@{
/// CMD line signal level
#define SFR9_CMD_SIGNAL_LEVEL   0x1000000ul
/// DAT3 signal level
#define SFR9_DAT3_SIGNAL_LEVEL  0x0800000ul
/// DAT2 signal level
#define SFR9_DAT2_SIGNAL_LEVEL  0x0400000ul
/// DAT1 signal level
#define SFR9_DAT1_SIGNAL_LEVEL  0x0200000ul  
/// DAT0 signal level
#define SFR9_DAT0_SIGNAL_LEVEL  0x0100000ul 
/// Write protect switch pin level
#define SFR9_WP_SWITCH_LEVEL    0x0080000ul 
/// Card detect pin level
#define SFR9_CARD_DETECT_LEVEL  0x0040000ul 
/// Card state stable
#define SFR9_CARD_STATE_STABLE  0x0020000ul
/// Card inserted
#define SFR9_CARD_INSERTED      0x0010000ul 
/// Buffer read enable
#define SFR9_BUFF_READ_EN       0x0000800ul
/// Buffer write enable
#define SFR9_BUFF_WRITE_EN      0x0000400ul
/// Read transfer active
#define SFR9_READ_TRANS_ACTIVE  0x0000200ul
/// Write transfer active
#define SFR9_WRITE_TRANS_ACTIVE 0x0000100ul
/// DAT line active
#define SFR9_DAT_LINE_ACTIVE    0x0000004ul
/// Command Inhibit (DAT)
#define SFR9_CMD_INHIBIT_DAT    0x0000002ul
/// Command Inhibit (CMD)
#define SFR9_CMD_INHIBIT_CMD    0x0000001ul

//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @anchor Voltages
/// @name Host control settings #0 (SFR10) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Wakeup event enable on SD card removal
#define SFR10_WAKEUP_EVENT_CARD_REM_ENABLE  0x4000000ul
/// Wakeup event enable on SD card inserted
#define SFR10_WAKEUP_EVENT_CARD_INS_ENABLE  0x2000000ul
/// Wakeup event enable on SD card interrupt
#define SFR10_WAKEUP_EVENT_CARD_INT_ENABLE  0x1000000ul
/// Interrupt at block gap for a muliple transfer
#define SFR10_INTERRUPT_BLOCK_GAP           0x0080000ul
/// Read wait control
#define SFR10_READ_WAIT_CONTROL             0x0040000ul
/// Continue request
#define SFR10_CONTINUE_REQUEST              0x0020000ul
/// Stop at block gap request
#define SFR10_STOP_AT_BLOCK_GAP             0x0010000ul
/// SD bus voltage - 3.3V
#define SFR10_SET_3_3V_BUS_VOLTAGE          0x0000E00ul
/// SD bus voltage - 3.0V
#define SFR10_SET_3_0V_BUS_VOLTAGE          0x0000C00ul
/// SD bus voltage - 1.8V
#define SFR10_SET_1_8V_BUS_VOLTAGE          0x0000A00ul
/// SD bus voltage mask
#define SFR10_BUS_VOLTAGE_MASK              0x0000E00ul
/// SD bus power. The SD device is powered.
#define SFR10_SD_BUS_POWER                  0x0000100ul


#define SFR10_DMA_SELECT_SDMA   (0x0 << 3)
#define SFR10_DMA_SELECT_ADMA1  (0x1 << 3)
#define SFR10_DMA_SELECT_ADMA2  (0x2 << 3)
#define SFR10_DMA_SELECT_MASK   (0x3 << 3)

/// High speed enable.
#define SFR10_HIGH_SPEED_ENABLE             0x0000004ul
/// Set 4 bit data transfer width
#define SFR10_DATA_WIDTH_4BIT               0x0000002ul
/// Turning on the LED.
#define SFR10_TURN_ON_LED                   0x0000001ul
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Host control settings #1 (SFR11) - masks
//-------------------------------------------------------------------------------------------
//{@
/// Software reset for DAT line
#define SFR11_SOFT_RESET_DAT_LINE           0x4000000ul
/// Software reset for CMD line
#define SFR11_SOFT_RESET_CMD_LINE           0x2000000ul
/// Software reset for all. Restart entrie controller except the card detection circuit.
#define SFR11_SOFT_RESET_ALL                0x1000000ul

/// Data timeout TMCLK x 2 raised to the 27-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_27    0x00E0000ul
/// Data timeout TMCLK x 2 raised to the 26-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_26    0x00D0000ul
/// Data timeout TMCLK x 2 raised to the 25-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_25    0x00C0000ul
/// Data timeout TMCLK x 2 raised to the 24-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_24    0x00B0000ul
/// Data timeout TMCLK x 2 raised to the 23-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_23    0x00A0000ul
/// Data timeout TMCLK x 2 raised to the 22-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_22    0x0090000ul
/// Data timeout TMCLK x 2 raised to the 21-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_21    0x0080000ul
/// Data timeout TMCLK x 2 raised to the 20-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_20    0x0070000ul
/// Data timeout TMCLK x 2 raised to the 19-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_19    0x0060000ul
/// Data timeout TMCLK x 2 raised to the 18-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_18    0x0050000ul
/// Data timeout TMCLK x 2 raised to the 17-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_17    0x0040000ul
/// Data timeout TMCLK x 2 raised to the 16-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_16    0x0030000ul
/// Data timeout TMCLK x 2 raised to the 15-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_15    0x0020000ul
/// Data timeout TMCLK x 2 raised to the 14-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_14    0x0010000ul
/// Data timeout TMCLK x 2 raised to the 13-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_13    0x0000000ul
/// Data timeout mask
#define SFR11_TIMEOUT_MASK                  0x00F0000ul

/// SDCLK Frequency select. Divide base clock by 256.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_256     0x0008000ul
/// SDCLK Frequency select. Divide base clock by 128.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_128     0x0004000ul
/// SDCLK Frequency select. Divide base clock by 64.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_64      0x0002000ul
/// SDCLK Frequency select. Divide base clock by 32.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_32      0x0001000ul
/// SDCLK Frequency select. Divide base clock by 16.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_16      0x0000800ul
/// SDCLK Frequency select. Divide base clock by 8.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_8       0x0000400ul
/// SDCLK Frequency select. Divide base clock by 4.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_4       0x0000200ul
/// SDCLK Frequency select. Divide base clock by 2.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_2       0x0000100ul
/// SDCLK Frequency select. Divide base clock by 1.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_1       0x0000000ul
/// SDCLK Frequency mask                    
#define SFR11_SEL_FREQ_BASE_MASK            0x000FF00ul

/// SD clock enable
#define SFR11_SD_CLOCK_ENABLE              0x0000004ul
/// Internal clock stable
#define SFR11_INT_CLOCK_STABLE             0x0000002ul
/// internal clock enable
#define SFR11_INT_CLOCK_ENABLE             0x0000001ul
//@}
//-------------------------------------------------------------------------------------------


/// @name Interrupt status register (SFR12) - masks
//-------------------------------------------------------------------------------------------
//@{
/// ADMA error
#define SFR12_ADMA_ERROR	        0x2000000ul
/// Auto CMD12 error
#define SFR12_AUTO_CMD12_ERROR      0x1000000ul
/// Current limit error host controller is not supplying power to SD card due some failure.
#define SFR12_CURRENT_LIMIT_ERROR   0x0800000ul
/// Data end bit error
#define SFR12_DATA_END_BIT_ERROR    0x0400000ul
/// Data CRC error
#define SFR12_DATA_CRC_ERROR        0x0200000ul
/// Data timeout error
#define SFR12_DATA_TIMEOUT_ERROR    0x0100000ul
/// Command index error. Index error occurs in the command response.
#define SFR12_COMMAND_INDEX_ERROR   0x0080000ul
/// Command end bit error
#define SFR12_COMMAND_END_BIT_ERROR 0x0040000ul
/// Command CRC error
#define SFR12_COMMAND_CRC_ERROR     0x0020000ul
/// Command timeout error
#define SFR12_COMMAND_TIMEOUT_ERROR 0x0010000ul
/// Error interrupt
#define SFR12_ERROR_INTERRUPT       0x0008000ul
/// Card interrupt
#define SFR12_CARD_INTERRUPT        0x0000100ul
/// Card removal
#define SFR12_CARD_REMOVAL          0x0000080ul
/// Card insertion
#define SFR12_CARD_INSERTION        0x0000040ul
/// Buffer read ready. Host is ready to read the buffer.
#define SFR12_BUFFER_READ_READY     0x0000020ul
/// Buffer write ready. Host is ready for writing data to the buffer.
#define SFR12_BUFFER_WRITE_READY    0x0000010ul
/// DMA interrupt
#define SFR12_DMA_INTERRUPT         0x0000008ul
/// Block gap event
#define SFR12_BLOCK_GAP_EVENT       0x0000004ul
/// Transfer complete
#define SFR12_TRANSFER_COMPLETE     0x0000002ul
/// Command complete
#define SFR12_COMMAND_COMPLETE      0x0000001ul
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Interrupt status enable register (SFR13) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Auto CMD12 error status enable
#define SFR13_AUTO_CMD12_ERR_STAT_EN        0x1000000ul
/// Current limit error status enable
#define SFR13_CURRENT_LIMIT_ERR_STAT_EN     0x0800000ul
/// Data end bit error status enable
#define SFR13_DATA_END_BIT_ERR_STAT_EN      0x0400000ul
/// Data CRC error status enable
#define SFR13_DATA_CRC_ERR_STAT_EN          0x0200000ul
/// Data timeout error status enable
#define SFR13_DATA_TIMEOUT_ERR_STAT_EN      0x0100000ul
/// Command index error status enable
#define SFR13_COMMAND_INDEX_ERR_STAT_EN     0x0080000ul
/// Command end bit error status enable
#define SFR13_COMMAND_END_BIT_ERR_STAT_EN   0x0040000ul
/// Command CRC error status enable
#define SFR13_COMMAND_CRC_ERR_STAT_EN       0x0020000ul
/// Command timeout error status enable
#define SFR13_COMMAND_TIMEOUT_ERR_STAT_EN   0x0010000ul
/// Card interrupt status enable
#define SFR13_CARD_INTERRUPT_STAT_EN        0x0000100ul
/// Card removal status enable
#define SFR13_CARD_REMOVAL_STAT_EN          0x0000080ul
/// Card insertion status enable
#define SFR13_CARD_INERTION_STAT_EN         0x0000040ul
/// Buffer read ready status enable
#define SFR13_BUFFER_READ_READY_STAT_EN     0x0000020ul
/// Buffer write ready status enable
#define SFR13_BUFFER_WRITE_READY_STAT_EN    0x0000010ul
/// DMA interrupt status enable
#define SFR13_DMA_INTERRUPT_STAT_EN         0x0000008ul
/// Block gap event status enable
#define SFR13_BLOCK_GAP_EVENT_STAT_EN       0x0000004ul
/// Transfer complete status enable
#define SFR13_TRANSFER_COMPLETE_STAT_EN     0x0000002ul
/// Command complete status enable
#define SFR13_COMMAND_COMPLETE_STAT_EN      0x0000001ul
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Interrupt signal enable register (SFR14) - masks
//-------------------------------------------------------------------------------------------
//{@
/// Auto CMD12 error signal enable
#define SFR14_AUTO_CMD12_ERR_SIG_EN        0x1000000ul
/// Current limit error signal enable
#define SFR14_CURRENT_LIMIT_ERR_SIG_EN     0x0800000ul
/// Data end bit error signal enable
#define SFR14_DATA_END_BIT_ERR_SIG_EN      0x0400000ul
/// Data CRC error signal enable
#define SFR14_DATA_CRC_ERR_SIG_EN          0x0200000ul
/// Data timeout error signal enable
#define SFR14_DATA_TIMEOUT_ERR_SIG_EN      0x0100000ul
/// Command index error signal enable
#define SFR14_COMMAND_INDEX_ERR_SIG_EN     0x0080000ul
/// Command end bit error signal enable
#define SFR14_COMMAND_END_BIT_ERR_SIG_EN   0x0040000ul
/// Command CRC error signal enable
#define SFR14_COMMAND_CRC_ERR_SIG_EN       0x0020000ul
/// Command timeout error signal enable
#define SFR14_COMMAND_TIMEOUT_ERR_SIG_EN   0x0010000ul
/// Card interrupt signal enable
#define SFR14_CARD_INTERRUPT_SIG_EN        0x0000100ul
/// Card removal signal enable
#define SFR14_CARD_REMOVAL_SIG_EN          0x0000080ul
/// Card insertion signal enable
#define SFR14_CARD_INERTION_SIG_EN         0x0000040ul
/// Buffer read ready signal enable
#define SFR14_BUFFER_READ_READY_SIG_EN     0x0000020ul
/// Buffer write ready signal enable
#define SFR14_BUFFER_WRITE_READY_SIG_EN    0x0000010ul
/// DMA interrupt signal enable
#define SFR14_DMA_INTERRUPT_SIG_EN         0x0000008ul
/// Block gap event signal enable
#define SFR14_BLOCK_GAP_EVENT_SIG_EN       0x0000004ul
/// Transfer complete signal enable
#define SFR14_TRANSFER_COMPLETE_SIG_EN     0x0000002ul
/// Command complete signal enable
#define SFR14_COMMAND_COMPLETE_SIG_EN      0x0000001ul
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Auto CMD12 error status register (SFR15) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Command not issued bu auto CMD12 error
#define SFR15_CMD_NOT_ISSUED_ERR        0x80ul
/// Auto CMD12 index error
#define SFR15_AUTO_CMD12_INDEX_ERR      0x10ul
/// Auto CMD12 end bit error
#define SFR15_AUTO_CMD12_END_BIT_ERR    0x08ul
/// Auto CMD12 CRC error
#define SFR15_AUTO_CMD12_CRC_ERR        0x04ul
/// Auto CMD12 timeout error
#define SFR15_AUTO_CMD12_TIMEOUT_ERR    0x02ul
/// Autp CMD12 not executed
#define SFR15_AUTO_CMD12_NOT_EXECUTED   0x01ul
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Capabilities register (SFR16) - masks
//-------------------------------------------------------------------------------------------
//@{
/// 64-bit System Bus Support
#define SFR16_VOLTAGE_64BIT_SUPPORT     0x08000000ul
/// Voltage 1.8V is supported
#define SFR16_VOLTAGE_1_8V_SUPPORT      0x04000000ul
/// Voltage 3.0V is supported
#define SFR16_VOLTAGE_3_0V_SUPPORT      0x02000000ul
/// Voltage 3.3V is supported
#define SFR16_VOLTAGE_3_3V_SUPPORT      0x01000000ul
/// Suspend and resume functionality are supported
#define SFR16_RESUME_SUPPORT            0x00800000ul
/// Host controller is capable of using SDMA
#define SFR16_DMA_SUPPORT               0x00400000ul
/// Host controller and the host system support High Speed mode.
#define SFR16_HIGH_SPEED_SUPPORT        0x00200000ul
/// Host controller is capable of using ADMA1
#define SFR16_ADMA1_SUPPORT        		0x00100000ul
/// Host controller is capable of using ADMA2
#define SFR16_ADMA2_SUPPORT        		0x00080000ul
/// 512 is the maximum block size that can be written to the buffer in the Host Controller.
#define SFR16_MAX_BLOCK_LENGTH_512      0x00000000ul
/// 1024 is the maximum block size that can be written to the buffer in the Host Controller.
#define SFR16_MAX_BLOCK_LENGTH_1024     0x00010000ul
/// 2048 is the maximum block size that can be written to the buffer in the Host Controller.
#define SFR16_MAX_BLOCK_LENGTH_2048     0x00020000ul
/// timeout unit clock is MHz
#define SFR16_TIMEOUT_CLOCK_UNIT_MHZ    0x00000100ul
/// this macro can be used to get base clock frequency for SD clock, from a value which was read from the SFR16 register
#define SFR16_GET_BASE_CLK_FREQ_MHZ(VALUE) ( ( VALUE >> 8 ) & 0x3F )
/// this macro can be used to get timeout clock frequency from a value which was read from the SFR16 register
#define SFR16_GET_TIMEOUT_CLK_FREQ(VALUE) ( VALUE & 0x1F )
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Maximum current capabilities register (SFR18) - masks
//-------------------------------------------------------------------------------------------
//@{
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name ADMA Error Status Register
//-------------------------------------------------------------------------------------------
//@{
/// ADMA Length Mismatch Error
#define SFR21_ADMA_ERROR_LENGTH_MISMATCH	(0x1uL << 2)
/// ADMA machine Stopped
#define SFR21_ADMA_ERROR_STATE_STOP			(0x0uL << 0)
/// ADMA Fetching descriptor
#define SFR21_ADMA_ERROR_STATE_FDS			(0x1uL << 0)
/// ADMA machine Transfer data
#define SFR21_ADMA_ERROR_STATE_TRF			(0x3uL << 0)

//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Host controller version/ slot interrupt status register (SFR63) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Interrupt signal for slot 0
#define SFR63_SLOT0_INTERRUPT       0x0
/// Interrupt signal for slot 1
#define SFR63_SLOT1_INTERRUPT       0x1
/// Interrupt signal for slot 2
#define SFR63_SLOT2_INTERRUPT       0x2
/// Interrupt signal for slot 3
#define SFR63_SLOT3_INTERRUPT       0x3
/// Interrupt signal for slot 4
#define SFR63_SLOT4_INTERRUPT       0x4
/// Interrupt signal for slot 5
#define SFR63_SLOT5_INTERRUPT       0x5
/// Interrupt signal for slot 6
#define SFR63_SLOT6_INTERRUPT       0x6
/// Interrupt signal for slot 7
#define SFR63_SLOT7_INTERRUPT       0x7
//-------------------------------------------------------------------------------------------

/// Maximum size of response in bytes
#define MAX_CARD_RESPONSE_BYTES 120 

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// @anchor WakeupCondition
/// @name Wakeup condition definition
//---------------------------------------------------------------------------
//@{
/// Wake up host when card is instert
#define SDIOHOST_WAKEUP_COND_CARD_INS       0x1
/// Wake up host when card is remove
#define SDIOHOST_WAKEUP_COND_CARD_REM       0x2
/// Wake up host when card interrupt occur
#define SDIOHOST_WAKEUP_COND_CARD_INT       0x4
//@}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// @name CCCR transfer direction definitions
//---------------------------------------------------------------------------
//@{
/// Read data from CCCR register
#define SDIOHOST_CCCR_READ  0
/// Write data to CCCR register
#define SDIOHOST_CCCR_WRITE 1
//@}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// @anchor HostConfigureCmd
/// @name Host configure commands
//---------------------------------------------------------------------------
//@{
/// Set host clock frequency
#define SDIOHOST_CONFIG_SET_CLK             0x1
/// Set host bus width
#define SDIOHOST_CONFIG_SET_BUS_WIDTH       0x2
/// Set timout on DAT line
#define SDIOHOST_CONFIG_SET_DAT_TIMEOUT     0x3
/// Disable signal interrupts
#define SDIOHOST_DISABLE_SIGNAL_INTERRUPT   0x4
/// Restore signal interrupts
#define SDIOHOST_RESTORE_SIGNAL_INTERRUPT   0x5
/***************************************************************/

#endif	/*SDIO_HOST_H*/
