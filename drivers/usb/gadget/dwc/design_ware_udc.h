#ifndef __DESIGN_WARE_UDC_H__
#define __DESIGN_WARE_UDC_H__

#define  DRIVER_NAME            "design ware udc"
#define  PERR(format,...)       printk(DRIVER_NAME "-[%s#%d:%s]:"format, \
                                	__FILE__,__LINE__,__func__ , ##__VA_ARGS__ )
#define  PLOG(format,...)		printk(DRIVER_NAME "-[%s#%d:%s]:"format, \
                                	__FILE__,__LINE__,__func__ , ##__VA_ARGS__ )
#define  PRINT(format,...)		printk(format, ##__VA_ARGS__ )                                                                                          

#ifdef   EN_DEBUG
#define  PDBG(format,...)		printk(DRIVER_NAME "-[%lu#%d:%s]:"format, \
                                	jiffies,__LINE__,__func__ , ##__VA_ARGS__ )
#define  DUMP_DATA(...)			dump_data(__VA_ARGS__)
#else   
#define  PDBG(format,...)		do{}while(0)
#define  DUMP_DATA(...)			do{}while(0)
#endif

#define  ENTER_FUNC()                        PDBG("enter func: %s\n",__func__);
#define  LEAVE_FUNC()                        PDBG("leave func: %s\n",__func__);

void dump_data(char * data, int len, int prntAll);
//////

typedef struct os_dependent {
	/** Base address returned from ioremap() */
	void *base;

	/** Register offset for Diagnostic API */
	uint32_t reg_offset;
	
	int irq;

} os_dependent_t;

/**
 * This structure is a wrapper that encapsulates the driver components used to
 * manage a single DWC_otg controller.
 */
typedef struct dwc_otg_device {
	 struct delayed_work reset_work;
	/** Structure containing OS-dependent stuff. KEEP THIS STRUCT AT THE
	 * VERY BEGINNING OF THE DEVICE STRUCT. OSes such as FreeBSD and NetBSD
	 * require this. */
	struct os_dependent os_dep;

	/** Pointer to the core interface structure. */
	dwc_otg_core_if_t *core_if;

	/** Pointer to the PCD structure. */
	struct dwc_otg_pcd *pcd;

	/** Pointer to the HCD structure. */
	struct dwc_otg_hcd *hcd;

	/** Flag to indicate whether the common IRQ handler is installed. */
	uint8_t common_irq_installed;

} dwc_otg_device_t;

#endif //__DESIGN_WARE_UDC_H__


