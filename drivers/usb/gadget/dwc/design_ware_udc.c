#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/types.h>
#include <asm/io.h>
#include <mach/gpio.h>

#include "dwc_otg_type.h"
#include "dwc_os.h"
#include "dwc_list.h"
#include "dwc_otg_core_if.h"
#include "dwc_otg_regs.h"
#include "dwc_otg_cil.h"
#include "dwc_otg_dbg.h"
#include "dwc_otg_pcd_if.h"
#include "design_ware_udc.h"

//#define USB_DET_GPIO		6 	// wakeup_gpio[6]
//
static const char driver_name [] = "design_ware_udc";

uint32_t g_dbg_lvl = -1;
//
extern int pcd_init(struct platform_device * _dev);
extern int pcd_remove(struct platform_device * _dev);
static void dwc_reset_work_func(struct work_struct *w);
//

static dwc_otg_device_t * g_otg_dev;
#ifdef EN_DEBUG
void dump_data(char * data, int len, int prntAll)
{
#define COL             16
#define ROW             5
        int     i,
				j,
				row,
				index;
                        
        if( prntAll )
        {
                row = 10000;
        }
        else
        {
                row = ROW;
        }
                        
        PRINT("=======<DUMP_DATA>[%lu]=======\n", jiffies);
        PRINT("           0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"
                                                "    0123456789ABCDEF\n");
        for(i = 0, index = 0 ; i < row ; i++)
        {
                PRINT("0x%04x:    ",i*COL);
                for(j = 0 ; j < COL ; j++)
                {
                        if( index + j >= len )
                        {
                                break;
                        }
                        
                        PRINT("%02x ",(unsigned char)data[index + j]);
                }
                
                for(; j < COL ; j++)
                {
                        PRINT("-- ");
                }
                PRINT(" |");
                for(j = 0 ; j < COL ; j++)
                {
                        if( index >= len )
                        {
                                break;
                        }
                        
                        if( data[index] >= 32 && data[index] <= 126 )
                        {
                                PRINT("%c",(unsigned char)data[index]);
                        }
                        else
                        {
                                PRINT(".");
                        }
                        
                        index ++;
                }
                PRINT("\n");    
                
                if( index >= len )
                {
                        break;
                }
        }
        PRINT("=========================\n");
#undef COL
#undef ROW
}
#endif

////////////////////////


/**
 * This function is called during module intialization
 * to pass module parameters to the DWC_OTG CORE.
 */
static int set_parameters(dwc_otg_core_if_t * core_if)
{
	int retval = 0;
	int i;

	// otg_cap, "OTG Capabilities 0=HNP&SRP 1=SRP Only 2=None"
	retval += dwc_otg_set_param_otg_cap(core_if, 2);
	
	// dma_enable, "DMA Mode 0=Slave 1=DMA enabled"
	retval += dwc_otg_set_param_dma_enable(core_if, 1);

	// dma_desc_enable, "DMA Desc Mode 0=Address DMA 1=DMA Descriptor enabled"
	retval += dwc_otg_set_param_dma_desc_enable(core_if, 0);

	// opt, "OPT Mode"
	retval += dwc_otg_set_param_opt(core_if, 0);

	// dma_burst_size "DMA Burst Size 1, 4, 8, 16, 32, 64, 128, 256"
	retval += dwc_otg_set_param_dma_burst_size(core_if, 4);
	
	// host_support_fs_ls_low_power, "Support Low Power w/FS or LS 0=Support 1=Don't Support"
//	retval += dwc_otg_set_param_host_support_fs_ls_low_power(core_if, 0);
	
	// enable_dynamic_fifo, "0=cC Setting 1=Allow Dynamic Sizing"
	retval += dwc_otg_set_param_enable_dynamic_fifo(core_if, 1);

	// data_fifo_size, "Total number of words in the data FIFO memory 32-32768"
	retval += dwc_otg_set_param_data_fifo_size(core_if, 654);

	// dev_rx_fifo_size, "Number of words in the Rx FIFO 16-32768"
	retval += dwc_otg_set_param_dev_rx_fifo_size(core_if, 276);

	 // dev_nperio_tx_fifo_size, "Number of words in the non-periodic Tx FIFO 16-32768"
	retval += dwc_otg_set_param_dev_nperio_tx_fifo_size(core_if, 128);

//	retval += dwc_otg_set_param_host_rx_fifo_size(core_if,
//							dwc_otg_module_params.host_rx_fifo_size);
	
//	retval += dwc_otg_set_param_host_nperio_tx_fifo_size(core_if,
//							       dwc_otg_module_params.
//							       host_nperio_tx_fifo_size);
	
//	retval +=
//		    dwc_otg_set_param_host_perio_tx_fifo_size(core_if,
//							      dwc_otg_module_params.
//							      host_perio_tx_fifo_size);
		
	// max_transfer_size, "The maximum transfer size supported in bytes 2047-65535"	???				      
	retval += dwc_otg_set_param_max_transfer_size(core_if, 131071);

	// max_packet_count, "The maximum number of packets in a transfer 15-511"
	retval += dwc_otg_set_param_max_packet_count(core_if, 511);

	
//	retval += dwc_otg_set_param_host_channels(core_if,
//						    dwc_otg_module_params.
//						    host_channels);

	// dev_endpoints, "The number of endpoints in addition to EP0 available for device mode 1-15"
	retval += dwc_otg_set_param_dev_endpoints(core_if, 2);
	
	// phy_type, "0=Reserved 1=UTMI+ 2=ULPI"
	retval += dwc_otg_set_param_phy_type(core_if, 1);

	// speed, "Speed 0=High Speed 1=Full Speed"
	retval += dwc_otg_set_param_speed(core_if, 0);


//	retval +=
//		    dwc_otg_set_param_host_ls_low_power_phy_clk(core_if,
//								dwc_otg_module_params.
//								host_ls_low_power_phy_clk);

//	retval +=
//		    dwc_otg_set_param_phy_ulpi_ddr(core_if,
//						   dwc_otg_module_params.
//						   phy_ulpi_ddr);

//	if (dwc_otg_module_params.phy_ulpi_ext_vbus != -1) {
//		retval +=
//		    dwc_otg_set_param_phy_ulpi_ext_vbus(core_if,
//							dwc_otg_module_params.
//							phy_ulpi_ext_vbus);
//	}

	// phy_utmi_width, "Specifies the UTMI+ Data Width 8 or 16 bits"
	retval += dwc_otg_set_param_phy_utmi_width(core_if, 8);

//	if (dwc_otg_module_params.ulpi_fs_ls != -1) {
//		retval +=
//		    dwc_otg_set_param_ulpi_fs_ls(core_if,
//						 dwc_otg_module_params.ulpi_fs_ls);
//	}

//	if (dwc_otg_module_params.ts_dline != -1) {
//		retval +=
//		    dwc_otg_set_param_ts_dline(core_if,
//					       dwc_otg_module_params.ts_dline);
//	}
//	if (dwc_otg_module_params.i2c_enable != -1) {
//		retval +=
//		    dwc_otg_set_param_i2c_enable(core_if,
//						 dwc_otg_module_params.
//						 i2c_enable);
//	}

	// en_multiple_tx_fifo, "Dedicated Non Periodic Tx FIFOs 0=disabled 1=enabled"
	retval += dwc_otg_set_param_en_multiple_tx_fifo(core_if, 0);

//	for (i = 0; i < 15; i++) {
//		if (dwc_otg_module_params.dev_perio_tx_fifo_size[i] != -1) {
//			retval +=
//			    dwc_otg_set_param_dev_perio_tx_fifo_size(core_if,
//								     dwc_otg_module_params.
//								     dev_perio_tx_fifo_size
//								     [i], i);
//		}
//	}

//	for (i = 0; i < 15; i++) {
//		if (dwc_otg_module_params.dev_tx_fifo_size[i] != -1) {
//			retval += dwc_otg_set_param_dev_tx_fifo_size(core_if,
//								     dwc_otg_module_params.
//								     dev_tx_fifo_size
//								     [i], i);
//		}
//	}

//	if (dwc_otg_module_params.thr_ctl != -1) {
//		retval +=
//		    dwc_otg_set_param_thr_ctl(core_if,
//					      dwc_otg_module_params.thr_ctl);
//	}
//	if (dwc_otg_module_params.mpi_enable != -1) {
//		retval +=
//		    dwc_otg_set_param_mpi_enable(core_if,
//						 dwc_otg_module_params.
//						 mpi_enable);
//	}
//	if (dwc_otg_module_params.pti_enable != -1) {
//		retval +=
//		    dwc_otg_set_param_pti_enable(core_if,
//						 dwc_otg_module_params.
//						 pti_enable);
//	}

	// lpm_enable, "LPM Enable 0=LPM Disabled 1=LPM Enabled"
	retval += dwc_otg_set_param_lpm_enable(core_if, 0);

	// ic_usb_cap, "IC_USB Capability 0=IC_USB Disabled 1=IC_USB Enabled"
	retval += dwc_otg_set_param_ic_usb_cap(core_if,0);

//	if (dwc_otg_module_params.tx_thr_length != -1) {
//		retval +=
//		    dwc_otg_set_param_tx_thr_length(core_if,
//						    dwc_otg_module_params.tx_thr_length);
//	}
//	if (dwc_otg_module_params.rx_thr_length != -1) {
//		retval +=
//		    dwc_otg_set_param_rx_thr_length(core_if,
//						    dwc_otg_module_params.
//						    rx_thr_length);
//	}
//	if (dwc_otg_module_params.ahb_thr_ratio != -1) {
//		retval +=
//		    dwc_otg_set_param_ahb_thr_ratio(core_if,
//						    dwc_otg_module_params.ahb_thr_ratio);
//	}
//	if (dwc_otg_module_params.power_down != -1) {
//		retval +=
//		    dwc_otg_set_param_power_down(core_if,
//						 dwc_otg_module_params.power_down);
//	}
//	if (dwc_otg_module_params.reload_ctl != -1) {
//		retval +=
//		    dwc_otg_set_param_reload_ctl(core_if,
//						 dwc_otg_module_params.reload_ctl);
//	}

//	if (dwc_otg_module_params.dev_out_nak != -1) {
//		retval +=
//			dwc_otg_set_param_dev_out_nak(core_if,
//			dwc_otg_module_params.dev_out_nak);
//	}

	// otg_ver, "OTG revision supported 0=OTG 1.3 1=OTG 2.0"
	retval += dwc_otg_set_param_otg_ver(core_if, 1);
	
	// adp_enable, "ADP Enable 0=ADP Disabled 1=ADP Enabled"
	retval += dwc_otg_set_param_adp_enable(core_if, 0);

	return retval;
}

/**
 * This function is the top level interrupt handler for the Common
 * (Device and host modes) interrupts.
 */
 irqreturn_t dwc_otg_pcd_irq(int irq, void *dev);
static irqreturn_t dwc_otg_common_irq(int irq, void *dev)
{
	dwc_otg_device_t *otg_dev = dev;
	int32_t retval = IRQ_NONE;

	retval = dwc_otg_handle_common_intr(otg_dev->core_if);
	retval |= dwc_otg_pcd_irq(irq, otg_dev->pcd);

	return IRQ_RETVAL(retval);
}

#if 0
static irqreturn_t dwc_otg_usb_det_irq(int irq, void *dev)
#else
void dwc_otg_usb_det(void)
#endif
{
	dwc_otg_device_t *otg_dev = g_otg_dev;

	if (!g_otg_dev){
		return;
	}
	PDBG("USB cable plug out\n");
	if (otg_dev->core_if->pcd_cb && otg_dev->core_if->pcd_cb->stop) {
                otg_dev->core_if->pcd_cb->stop(otg_dev->core_if->pcd_cb->p);
        }

	if (otg_dev->core_if->pcd_cb && otg_dev->core_if->pcd_cb->start) {
                otg_dev->core_if->pcd_cb->start(otg_dev->core_if->pcd_cb->p);
        }

	return ;
}

static int dw_udc_remove(struct platform_device *_dev)
{
	dwc_otg_device_t *otg_dev = dev_get_drvdata(&_dev->dev);

	DWC_DEBUGPL(DBG_ANY, "%s(%p)\n", __func__, _dev);

	if (!otg_dev) {
		/* Memory allocation for the dwc_otg_device failed. */
		DWC_DEBUGPL(DBG_ANY, "%s: otg_dev NULL!\n", __func__);
		return 0;
	}
#ifndef DWC_DEVICE_ONLY
	if (otg_dev->hcd) {
		hcd_remove(_dev);
	} else {
		DWC_DEBUGPL(DBG_ANY, "%s: otg_dev->hcd NULL!\n", __func__);
		return 0;
	}
#endif

#ifndef DWC_HOST_ONLY
	if (otg_dev->pcd) {
		pcd_remove(_dev);
	} else {
		DWC_DEBUGPL(DBG_ANY, "%s: otg_dev->pcd NULL!\n", __func__);
		return 0;
	}
#endif
	/*
	 * Free the IRQ
	 */
	if (otg_dev->common_irq_installed) {
		free_irq(otg_dev->os_dep.irq, otg_dev);
		//free_irq(gpio_to_irq(USB_DET_GPIO), otg_dev);
	} else {
		DWC_DEBUGPL(DBG_ANY, "%s: There is no installed irq!\n", __func__);
		return 0;
	}

	if (otg_dev->core_if) {
		dwc_otg_cil_remove(otg_dev->core_if);
	} else {
		DWC_DEBUGPL(DBG_ANY, "%s: otg_dev->core_if NULL!\n", __func__);
		return 0;
	}

	/*
	 * Remove the device attributes
	 */
//	dwc_otg_attr_remove(_dev);

	/*
	 * Return the memory.
	 */
	if (otg_dev->os_dep.base) {
		iounmap(otg_dev->os_dep.base);
	}
	DWC_FREE(otg_dev);

	/*
	 * Clear the drvdata pointer.
	 */
	dev_set_drvdata(&_dev->dev, NULL);
	g_otg_dev = NULL;

	return 0;
}

static int dw_udc_probe(struct platform_device *pdev)
{
	struct resource *res, *ires;
	int retval = 0;
	dwc_otg_device_t *dwc_otg_device;

	ENTER_FUNC();
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		PERR("platform_get_resource IORESOURCE_MEM error.\n");
		return -ENODEV;
	}
	PDBG("io_start=0x%08x\n", (unsigned)res->start);
	
	ires = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!ires) {
		PERR("platform_get_resource IORESOURCE_IRQ error.\n");
		return -ENODEV;
	}

	dwc_otg_device = DWC_ALLOC(sizeof(dwc_otg_device_t));

	if (!dwc_otg_device) {
		PERR("kmalloc of dwc_otg_device failed\n");
		return -ENOMEM;
	}

	memset(dwc_otg_device, 0, sizeof(*dwc_otg_device));
	dwc_otg_device->os_dep.reg_offset = 0xFFFFFFFF;

	dwc_otg_device->os_dep.base = ioremap(res->start, resource_size(res));
	if (dwc_otg_device->os_dep.base == NULL) {
		PERR("ioremap error.\n");
		return -ENOMEM;
	}
	PDBG("base=0x%08x\n", (unsigned)dwc_otg_device->os_dep.base);
	
	dev_set_drvdata(&pdev->dev, dwc_otg_device);
	g_otg_dev = dwc_otg_device;

	dwc_otg_device->core_if = dwc_otg_cil_init(dwc_otg_device->os_dep.base);
	if (!dwc_otg_device->core_if) {
		PERR("CIL initialization failed!\n");
		retval = -ENOMEM;
		goto fail;
	}
	
	/*
	 * Attempt to ensure this device is really a DWC_otg Controller.
	 * Read and verify the SNPSID register contents. The value should be
	 * 0x45F42XXX, which corresponds to "OT2", as in "OTG version 2.XX".
	 */

	if ((dwc_otg_get_gsnpsid(dwc_otg_device->core_if) & 0xFFFFF000) !=
	    0x4F542000) {
		PERR("Bad value for SNPSID: 0x%08x\n",
			dwc_otg_get_gsnpsid(dwc_otg_device->core_if));
		retval = -EINVAL;
		goto fail;
	}
	
	/*
	 * Validate parameter values.
	 */
	if (set_parameters(dwc_otg_device->core_if)) {
		retval = -EINVAL;
		goto fail;
	}

	/*
	 * Create Device Attributes in sysfs
	 */
	//dwc_otg_attr_create(_dev);

	/*
	 * Disable the global interrupt until all the interrupt
	 * handlers are installed.
	 */
	dwc_otg_disable_global_interrupts(dwc_otg_device->core_if);

	/*
	 * Install the interrupt handler for the common interrupts before
	 * enabling common interrupts in core_init below.
	 */
	dwc_otg_device->os_dep.irq = ires->start;
	PLOG("registering (common) handler for irq%d\n",
		    ires->start);
	retval = request_irq(ires->start, dwc_otg_common_irq,
			     IRQF_SHARED | IRQF_DISABLED |  IRQF_TRIGGER_HIGH, "dwc_otg",
			     dwc_otg_device);
	if (retval) {
		PERR("request of irq%d failed\n", ires->start);
		retval = -EBUSY;
		goto fail;
	} 

#if 0
	PLOG("registering USB_DET irq\n");
	retval = request_irq(gpio_to_irq(USB_DET_GPIO), dwc_otg_usb_det_irq,
			     IRQF_SHARED | IRQF_DISABLED |  IRQF_TRIGGER_FALLING, "dwc_otg",
			     dwc_otg_device);
	if (retval) {
		PERR("request usb_det irq failed\n");
		retval = -EBUSY;
		free_irq(dwc_otg_device->os_dep.irq, dwc_otg_device);
		goto fail;
	} else {
		dwc_otg_device->common_irq_installed = 1;
	}
#else
	dwc_otg_device->common_irq_installed = 1;
#endif

	/*
	 * Initialize the DWC_otg core.
	 */
	dwc_otg_core_init(dwc_otg_device->core_if);
	
#ifndef DWC_HOST_ONLY
	/*
	 * Initialize the PCD
	 */
	retval = pcd_init(pdev);
	if (retval != 0) {
		DWC_ERROR("pcd_init failed\n");
		dwc_otg_device->pcd = NULL;
		goto fail;
	}	
#endif
#ifndef DWC_DEVICE_ONLY
	/*
	 * Initialize the HCD
	 */
	retval = hcd_init(pdev);
	if (retval != 0) {
		DWC_ERROR("hcd_init failed\n");
		dwc_otg_device->hcd = NULL;
		goto fail;
	}
#endif
	/*
	 * Enable the global interrupt after all the interrupt
	 * handlers are installed if there is no ADP support else 
	 * perform initial actions required for Internal ADP logic.
	 */
	if (!dwc_otg_get_param_adp_enable(dwc_otg_device->core_if)) {
		PDBG("ADP disable\n");
		dwc_otg_enable_global_interrupts(dwc_otg_device->core_if);
	} else {
		PDBG("ADP enable\n");
		dwc_otg_adp_start(dwc_otg_device->core_if);
	}

	INIT_DELAYED_WORK(&dwc_otg_device->reset_work, dwc_reset_work_func);
	PDBG("succes\n");	
	return 0;
	
fail:
	dw_udc_remove(pdev);
	return retval;
}

static int dwc_suspend(struct platform_device * dev, pm_message_t state)
{
	dwc_otg_device_t *otg_dev = dev_get_drvdata(&dev->dev);

	PLOG("UDC suspend\n");
	/*
	dwc_otg_pcd_pullup(otg_dev->pcd, 0);
	if (otg_dev->core_if->pcd_cb && otg_dev->core_if->pcd_cb->stop) {
                otg_dev->core_if->pcd_cb->stop(otg_dev->core_if->pcd_cb->p);
        }
	*/
	return 0;
}

static void dwc_reset_work_func(struct work_struct *w)
{
	struct delayed_work *dw = to_delayed_work(w);
        dwc_otg_device_t *otg_dev = container_of(dw, dwc_otg_device_t, reset_work);

	PDBG("+++++\n");
	dwc_otg_core_init(otg_dev->core_if);
	dwc_otg_enable_global_interrupts(otg_dev->core_if);
	dwc_otg_pcd_pullup(otg_dev->pcd, 1);
}

static int dwc_resume(struct platform_device * dev)
{
	dwc_otg_device_t *otg_dev = dev_get_drvdata(&dev->dev);

	PLOG("UDC resume\n");
	schedule_delayed_work(&otg_dev->reset_work, HZ * 2);

	return 0;
}

static struct platform_driver dw_udc_driver = {
	.remove		= __exit_p(dw_udc_remove),
	.suspend	= dwc_suspend,
	.resume		= dwc_resume,
	.driver		= {
		.name	= (char *) driver_name,
		.owner	= THIS_MODULE,
	},
};

static int __init udc_init_module(void)
{
	ENTER_FUNC();
	return platform_driver_probe(&dw_udc_driver, dw_udc_probe);
}
module_init(udc_init_module);

static void __exit udc_exit_module(void)
{
	platform_driver_unregister(&dw_udc_driver);
}
module_exit(udc_exit_module);

MODULE_DESCRIPTION("design ware udc driver");
MODULE_AUTHOR("Guo Jianxin");
MODULE_LICENSE("GPL");

