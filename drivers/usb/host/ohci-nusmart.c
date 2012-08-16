/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * Copyright (C) 2010 Nusmart Corp.
 *
 * Author : zeyuan <zeyuan.xu@nufrountsoft.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/platform_device.h>

static int ohci_nusmart_start(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);

	ohci_hcd_init(ohci);
	ohci_init(ohci);
	ohci_run(ohci);
	hcd->state = HC_STATE_RUNNING;
	return 0;
}

static const struct hc_driver ohci_nusmart_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"Nusmart OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_nusmart_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_nusmart_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct usb_hcd *hcd = NULL;
	const struct hc_driver *driver = &ohci_nusmart_hc_driver;
	int irq = -1;
	int ret;

	if (usb_disabled())
		return -ENODEV;
	
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if(!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ.Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	/* initialize hcd */
	hcd = usb_create_hcd(&ohci_nusmart_hc_driver, &pdev->dev, (char *)hcd_name);
	if (!hcd) {
		ret = -ENOMEM;
		goto fail_create_hcd;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register register addr.Check %s setup!\n",
			dev_name(&pdev->dev));
		ret = -ENODEV;
		goto fail_request_resource;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start +1;

	if(!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use.\n");
		ret = -EBUSY;
		goto fail_request_resource;
	}

	hcd->regs = ioremap_nocache(hcd->rsrc_start, hcd->rsrc_len);
	if(hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory.\n");
		ret = -EFAULT;
		goto fail_ioremap;
	}

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret != 0) {
		goto fail_add_hcd;
	}

	dev_dbg(&pdev->dev, "probe ok!\n");
	return ret;

fail_add_hcd:
	iounmap(hcd->regs);
fail_ioremap:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
fail_request_resource:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), ret);
	return ret;

}

static int ohci_hcd_nusmart_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	return 0;
}

static struct platform_driver ohci_hcd_nusmart_driver = {
	.probe		= ohci_hcd_nusmart_probe,
	.remove		= ohci_hcd_nusmart_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "ns2816-ohci",
		.owner	= THIS_MODULE,
	},
};

MODULE_ALIAS("platform:ns2816-ohci");
