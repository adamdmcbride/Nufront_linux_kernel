/*
        wangweilin modify
        add:suspend() and resume();
*/
#define VERSION "3.0"   //2011.5.24 wangweilin_add

#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mii.h>
#include <asm/io.h>
#include "nusmart_emac.h"

#define DEBUG 0
 

#define nusmartEMACPhyAddrSize	8192
#define NUM_OF_DESC		256
#define ETHERNET_HEADER             14
#define ETHERNET_CRC                 4
#define MACBASE		0x0000
#define DMABASE		0x1000
//#define DEFAULT_MAC_ADDRESS {0x00, 0x55, 0x7B, 0xB5, 0x7D, 0xF7}
#define DEFAULT_MAC_ADDRESS {0x00, 0x55, 0x7B, 0xB5, 0x45, 0x7A}
#define DEFAULT_LOOP_VARIABLE   10
#define TR0(fmt, args...) printk(KERN_CRIT "nusmartEMAC: " fmt, ##args)
#define TR1(fmt, args...) printk(KERN_DEBUG "nusmartEMAC: " fmt, ##args)

//#define MAC_ADDR_RANDOM


static int nusmart_probe(struct platform_device *pdev);
static int nusmart_remove (struct platform_device *pdev);
static int phy_find(struct net_device *netdev);
irqreturn_t nusmartEMAC_intr_handler(int intr_num, void * dev_id);
static int txdesc_init(struct net_device *netdev);
static int rxdesc_init(struct net_device *netdev);
static void txdesc_exit(struct net_device *netdev);
static void rxdesc_exit(struct net_device *netdev);
static void mac_init(struct net_device *netdev);
static void dma_init(struct net_device *netdev);
//static void dma_init_t(struct net_device *netdev);
static void emac_start(struct net_device *netdev);
static void emac_stop(struct net_device *netdev);
static void nusmartEMAC_linux_cable_unplug_function(unsigned int timerdata);
static int nusmartEMAC_linux_open(struct net_device *netdev);
static int nusmartEMAC_linux_close(struct net_device *netdev);
static int nusmartEMAC_read_phy_reg(u32 RegBase,u32 PhyBase, u32 RegOffset, u16 * data);
static int nusmartEMAC_linux_xmit_frames(struct sk_buff *skb, struct net_device *netdev);
static int nusmart_handle_transmit_over(struct net_device *netdev);
static int nusmart_handle_receive_over(struct net_device *netdev);
static struct net_device_stats *  nusmartEMAC_linux_get_stats(struct net_device *netdev);
static int nusmartEMAC_linux_set_mac_address(struct net_device *netdev, void * macaddr);
static void nusmartEMAC_linux_set_multicast_list(struct net_device *netdev);
static int nusmartEMAC_linux_ioctl(struct net_device *dev,struct ifreq *ifr,int cmd);

static unsigned char nusmartEMAC_driver_name[] = "ns2816-emac";
//static unsigned char nusmartEMAC_driver_name[] = "ns2416-emac";

static const struct net_device_ops nusmart_netdev_ops = {
        .ndo_open               = nusmartEMAC_linux_open,
        .ndo_stop               = nusmartEMAC_linux_close,
        .ndo_start_xmit         = nusmartEMAC_linux_xmit_frames,
        .ndo_set_mac_address    = nusmartEMAC_linux_set_mac_address,
        .ndo_get_stats		= nusmartEMAC_linux_get_stats,
        .ndo_set_multicast_list = nusmartEMAC_linux_set_multicast_list,
        .ndo_do_ioctl		= nusmartEMAC_linux_ioctl,
        .ndo_change_mtu		= eth_change_mtu,
        .ndo_validate_addr	= eth_validate_addr,
        //	.ndo_set_mac_address	= eth_mac_addr,

};

static int nusmartEMAC_linux_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{

        struct mii_ioctl_data *data = if_mii(ifr);
        struct nusmartEMACprivdataStruct *privdata = netdev_priv(dev);
        struct nusmartEMACDeviceStruct 	 *emacdev;
        struct sockaddr *addr;
        unsigned int temp;

#if DEBUG
        TR0("nusmartEMAC_linux_ioctl\n");
#endif
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        switch(cmd) {
                case SIOCGMIIPHY:
                        data->phy_id = privdata->phy_addr;

                case SIOCGMIIREG: 

                        nusmartEMAC_read_phy_reg(emacdev->MacBase,data->phy_id,data->reg_num, &data->val_out);
                        return 0;

                case SIOCGIFHWADDR:

                        addr = (struct sockaddr *) &ifr->ifr_hwaddr;
                        copy_to_user((void __user __force *) addr->sa_data,
                                        (void  __force *) dev->dev_addr,
                                        IFHWADDRLEN);

                        return 0;

                case SIOCSIFHWADDR:

                        addr = (struct sockaddr *) &ifr->ifr_hwaddr;
                        /* Copy MAC address in from user space */
                        copy_from_user((void __force *) dev->dev_addr,
                                        (void __user __force *) addr->sa_data,
                                        IFHWADDRLEN);

                        //set dev_addr
                        temp = (dev->dev_addr[5] << 8) | dev->dev_addr[4];
                        wmb();
                        writel(temp,(void*)(emacdev->MacBase + GmacAddr0High));
                        temp = (dev->dev_addr[3] << 24) | (dev->dev_addr[2] << 16) | (dev->dev_addr[1] << 8) | dev->dev_addr[0] ;
                        wmb();
                        writel(temp,(void*)(emacdev->MacBase + GmacAddr0Low));

                        return 0;	
                        //	case SIOCSMIIREG:

                        //		return 0;
                default:
                        return -EOPNOTSUPP;
        }


}
static void nusmartEMAC_linux_set_multicast_list(struct net_device *netdev)
{
#if DEBUG
        TR0("nusmartEMAC_linux_set_multicast_list\n");
#endif
}

/*static int nusmartEMAC_poll(struct napi_struct *napi,int budget)
  {

  struct nusmartEMACprivdataStruct *privdata = 
  container_of(napi,struct nusmartEMACprivdataStruct ,napi);
  struct net_device *netdev = privdata->nusmartEMACnetdev;
  struct nusmartEMACDeviceStruct 		*emacdev;
  int i=0;
  int ret;

  emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

  while(i < budget){

  ret = nusmart_handle_receive_over(netdev);
  if(ret!=0)
  {
  napi_complete(napi);
//enable rx interrupt
writel(readl((void*)(emacdev->DmaBase + DmaInterrupt)) | DmaIntRxNormMask,
(void*)(emacdev->DmaBase + DmaInterrupt));
break;
}
i++;
}

return i;
}
*/
/* wwl_rm 2011.5.24
static int nusmart_mii_read(struct mii_bus *bus, int phyaddr, int regidx)
{
        struct nusmartEMACprivdataStruct *privdata = (struct nusmartEMACprivdataStruct *)bus->priv;
        struct nusmartEMACDeviceStruct          *emacdev = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);
        unsigned long flags;
        int i, reg;

#if DEBUG
        TR0("nusmart_mii_read\n");
#endif
        spin_lock_irqsave(&privdata->lock, flags);

        // Confirm MII not busy 
        if (unlikely(readl((void*)(emacdev->MacBase + GmacGmiiAddr)) & GmiiBusy)) {
                reg = -EIO;
                goto out;
        }

        // Set the address, index & direction (read from PHY) //
        wmb();
        writel((((phyaddr & 0x1F)<< 11) | GmiiCsrClk4 | GmiiBusy) & (~GmiiWrite), (void *)(emacdev->MacBase + GmacGmiiAddr));

        // Wait for read to complete w/ timeout //
        for (i = 0; i < 100; i++)
                if (!(readl((void*)(emacdev->MacBase + GmacGmiiAddr)) & GmiiBusy)) {
                        reg = readl((void*)(emacdev->MacBase + GmacGmiiData));
                        goto out;
                }

        reg = -EIO;

out:
        spin_unlock_irqrestore(&privdata->lock, flags);
        return reg;
}

static int nusmart_mii_write(struct mii_bus *bus, int phyaddr, int regidx,
                u16 val)
{
        struct nusmartEMACprivdataStruct *privdata = (struct nusmartEMACprivdataStruct *)bus->priv;
        struct nusmartEMACDeviceStruct          *emacdev = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);
        unsigned long flags;
        int i, reg;

#if DEBUG
        TR0("nusmart_mii_write\n");
#endif
        spin_lock_irqsave(&privdata->lock, flags);

        if (unlikely(readl((void*)(emacdev->MacBase + GmacGmiiAddr)) & GmiiBusy)) {
                reg = -EIO;
                goto out;
        }

        // Put the data to write in the MAC //
        wmb();
        writel(val,(void *)(emacdev->MacBase + GmacGmiiData));

        // Set the address, index & direction (read from PHY) //
        wmb();
        writel(((phyaddr & 0x1F)<< 11) | GmiiCsrClk4 | GmiiBusy | ((regidx & 0x1F) << 6) | GmiiWrite, (void *)(emacdev->MacBase + GmacGmiiAddr));

        // Wait for write to complete w/ timeout //
        for (i = 0; i < 100; i++)
                if (!(readl((void*)(emacdev->MacBase + GmacGmiiAddr)) & GmiiBusy)) {
                        reg = 0;
                        goto out;
                }

        reg = -EIO;

out:
        spin_unlock_irqrestore(&privdata->lock, flags);
        return reg;
}

*/
/*  wwl_rm 2011.5.24
static void nusmart_phy_update_flowcontrol(struct nusmartEMACprivdataStruct *privdata)
{
#if DEBUG
        TR0("nusmart_phy_update_flowcontrol\n");
#endif
}
*/
/* wwl_rm 2011.5.24
static void nusmart_phy_adjust_link(struct net_device *netdev)
{	
        struct nusmartEMACprivdataStruct *privdata = netdev_priv(netdev);
        struct phy_device *phy_dev = privdata->phy_dev;
        unsigned long flags;
        int carrier;

#if DEBUG
        TR0("nusmart_phy_adjust_link\n");
#endif

        if (phy_dev->duplex != privdata->last_duplex) {

                spin_lock_irqsave(&privdata->lock, flags);

                if (phy_dev->duplex) {
                } else {
                }

                spin_unlock_irqrestore(&privdata->lock, flags);

                nusmart_phy_update_flowcontrol(privdata);
                privdata->last_duplex = phy_dev->duplex;
        }

        carrier = netif_carrier_ok(netdev);

}
*/
/* wwl_rm 2011.5.24
static int nusmart_mii_probe(struct net_device *netdev)
{
        struct nusmartEMACprivdataStruct *privdata = netdev_priv(netdev);
        struct phy_device *phydev = NULL;
        int phy_addr;

#if DEBUG
        TR0("nusmart_mii_probe\n");
#endif

        // find the first phy //
        for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
                if (privdata->mii_bus->phy_map[phy_addr]) {
                        phydev = privdata->mii_bus->phy_map[phy_addr];
                        break;
                }
        }

        if (!phydev) {
                return -ENODEV;
        }

        phydev = phy_connect(netdev, dev_name(&phydev->dev),
                        &nusmart_phy_adjust_link, 0, PHY_INTERFACE_MODE_MII);

        if (IS_ERR(phydev)) {
                return PTR_ERR(phydev);
        }

        // mask with MAC supported features //
        phydev->supported &= (PHY_BASIC_FEATURES | SUPPORTED_Pause |
                        SUPPORTED_Asym_Pause);
        phydev->advertising = phydev->supported;

        privdata->phy_dev = phydev;
        privdata->last_duplex = -1;
        privdata->last_carrier = -1;

        return 0;

}
*/
/*  //wwl_rm 2011.5.24
static int nusmart_mii_init(struct platform_device *pdev,struct net_device * netdev)
{
        struct nusmartEMACprivdataStruct *privdata = NULL;
        int err = -ENXIO;
        int i;	

#if DEBUG
        TR0("nusmart_mii_init\n");
#endif
        privdata = netdev_priv(netdev);

        privdata->mii_bus = mdiobus_alloc();

        if(!privdata->mii_bus){

                err = -ENOMEM;
                goto err_mdiobus_alloc;

        }

        privdata->mii_bus->name = "nusmart_mdio";
        snprintf(privdata->mii_bus->id, MII_BUS_ID_SIZE, "%x", pdev->id);
        privdata->mii_bus->priv = privdata;
        privdata->mii_bus->read = nusmart_mii_read;
        privdata->mii_bus->write = nusmart_mii_write;
        privdata->mii_bus->irq = privdata->phy_irq;
        for(i = 0;i < PHY_MAX_ADDR;i++)
                privdata->mii_bus->irq[i] = PHY_POLL;

        privdata->mii_bus->parent = &netdev->dev;			
        if(mdiobus_register(privdata->mii_bus))
                goto err_mdiobus_register;	

        if(nusmart_mii_probe(netdev) < 0)
                goto err_mii_probe;

        return 0;

err_mii_probe:
        mdiobus_unregister(privdata->mii_bus);
err_mdiobus_register:
        mdiobus_free(privdata->mii_bus);
err_mdiobus_alloc:	

        TR0("mii_init failure!\n");
        return err;

}
*/

static int nusmart_probe (struct platform_device *pdev)
{

        int i;
        struct resource *res;
        int irq;
        struct net_device *netdev;
        struct nusmartEMACprivdataStruct *privdata = NULL;
        unsigned char mac_addr0[6] = DEFAULT_MAC_ADDRESS;

#ifdef MAC_ADDR_RANDOM
        int j;
        struct timeval tv;
        do_gettimeofday(&tv);

        //	j=jiffies;
        j=tv.tv_usec;

        TR0("MAC ADDRESS RAND:%d",j);

        mac_addr0[5] = (unsigned char)j;
        mac_addr0[4] = (unsigned char)(j + 1);
        j=tv.tv_usec;
        mac_addr0[3] = (unsigned char)(j + 2);

#endif

        TR0("nusmart_probe \n");
        TR0("version:%s\n", VERSION);

        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res)
        {
                TR0("Get IORESOURCE_MEM failure!\n");
                return -ENXIO;
        }

        irq = platform_get_irq(pdev, 0);
        if (irq < 0)
        {
                TR0("Get IORESOURCE_IRQ failure!\n");
                return irq;
        }

        netdev = alloc_etherdev(sizeof(struct nusmartEMACprivdataStruct));

        if(!netdev)
                return -ENOMEM;

        privdata = netdev_priv(netdev);

        if(!request_mem_region (res->start, res->end -res->start +1,nusmartEMAC_driver_name))
                goto err_request_mem;

        privdata->nusmartEMACdev.membase = (u32)ioremap_nocache(res->start, res->end - res->start + 1);
        privdata->nusmartEMACdev.phyaddr = res->start;
        privdata->nusmartEMACdev.physize = res->end - res->start + 1;
        privdata->nusmartEMACdev.irq	 = irq;

        if(!privdata->nusmartEMACdev.membase)
                goto err_ioremap;

        privdata->nusmartEMACnetdev	= netdev;

        /*set nusmartEMACdevice struct*/
        privdata->nusmartEMACdev.MacBase = privdata->nusmartEMACdev.membase + MACBASE;
        privdata->nusmartEMACdev.DmaBase = privdata->nusmartEMACdev.membase + DMABASE;

        ether_setup(netdev);

        netdev->dma		= (-1);
        netdev->base_addr 	= res->start;
        netdev->irq		= irq;
        //netdev->features	|= NETIF_F_NO_CSUM ;
        netdev->flags		|= IFF_MULTICAST;


#ifdef CONFIG_NET_POLL_CONTROLLER
        TR0("*********defined config_NET_POLL_CONTROLLER\n");

#endif 

        memcpy(netdev->dev_addr,mac_addr0,6);
        for(i = 0; i <6; i++){
                netdev->broadcast[i] = 0xff;
        }


        if(!is_valid_ether_addr(netdev->dev_addr))
                TR0("invalid ether addr! \n");

        netdev->netdev_ops                  = &nusmart_netdev_ops;
        //	netdev->set_multicast_list      = &nusmartEMAC_linux_set_multicast_list;
        //	netdev->change_mtu              = &nusmartEMAC_linux_change_mtu;
        //	netdev->do_ioctl                = &nusmartEMAC_linux_do_ioctl;
        //	netdev->tx_timeout              = &nusmartEMAC_linux_tx_timeout;
        netdev->watchdog_timeo          = msecs_to_jiffies(5000);

        /*register*/
        //netif_napi_add(netdev,&privdata->napi,nusmartEMAC_poll,16);
        spin_lock_init(&(privdata->lock));
        spin_lock_init(&(privdata->nusmartEMACdev.lock));
        platform_set_drvdata(pdev,netdev);
        SET_NETDEV_DEV(netdev,&pdev->dev);

        if(register_netdev(netdev))
                goto err_register_netdev;

        /*	if(nusmart_mii_init(pdev,netdev))
            goto err_mii_init;
            */
        return 0;
/*
err_mii_init:
        unregister_netdev(netdev);	
*/
err_register_netdev:
        iounmap((void*)privdata->nusmartEMACdev.membase);

err_ioremap:
        release_mem_region (res->start, res->end - res->start + 1);

err_request_mem:
        free_netdev(netdev);

        TR0("net_device register failed!\n");

        return -1;

}

static int nusmart_remove (struct platform_device *pdev)
{
        struct net_device *netdev = (struct net_device *)platform_get_drvdata(pdev);
        struct nusmartEMACprivdataStruct *privdata = netdev_priv(netdev);

        TR0("nusmart_remove\n");

        phy_disconnect(privdata->phy_dev);
        privdata->phy_dev = NULL;
        /*	mdiobus_unregister(privdata->mii_bus);
            mdiobus_free(privdata->mii_bus);
            */
        if(privdata->nusmartEMACdev.membase)
                iounmap((void *)privdata->nusmartEMACdev.membase);
        if(privdata->nusmartEMACdev.phyaddr)
                release_mem_region(privdata->nusmartEMACdev.phyaddr,privdata->nusmartEMACdev.physize);

        if(netdev)
        {
                unregister_netdev(netdev);
                free_netdev(netdev);
        }

        return 0;
}


static int phy_find(struct net_device *netdev)
{
#if DEBUG
        TR0("phy_find\n");
#endif
        unsigned int addr=0;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        for(addr=0;addr<32;addr++)
        {

                while(readl((void*)(emacdev->MacBase + GmacGmiiAddr)) & GmiiBusy)
                        udelay(10);
                wmb();
                writel((addr << 11) | GmiiCsrClk4 | GmiiBusy , (void *)(emacdev->MacBase + GmacGmiiAddr));

                udelay(10);

                while(readl((void*)(emacdev->MacBase + GmacGmiiAddr)) & GmiiBusy)
                        udelay(10);

                if(readl((void*)(emacdev->MacBase + GmacGmiiData)) != 0xFFFF)
                {
                        return	addr;
                }

        }

        return -1;
}

static int nusmart_handle_receive_over(struct net_device *netdev)
{

        unsigned int rxbusy;
        DmaDesc *rxdesc;
        struct sk_buff *skb = NULL;
        dma_addr_t 	dma_addr = 0;
        unsigned int len;

        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

#if DEBUG
        TR0("nusmart_handle_receive_over\n");
#endif

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        rxbusy = emacdev->RxBusy;
        rxdesc = emacdev->RxBusyDesc;

        if((rxdesc->status & DescOwnByDma) == DescOwnByDma)
                return -1;

        if((rxdesc->length & DescSize1Mask) == 0)
                return -1;

        if(rxdesc->data1!=0)
        {
                skb = (struct sk_buff*)rxdesc->data1;
                dma_addr = (dma_addr_t)rxdesc->buffer1;

                if(((rxdesc->status & DescError) == 0) && ((rxdesc->status & DescRxFirst) == DescRxFirst) && ((rxdesc->status & DescRxLast) == DescRxLast))
                {
                        dma_unmap_single(NULL,dma_addr,(size_t)skb_tailroom(skb),DMA_FROM_DEVICE);
                        len = ((rxdesc->status & DescFrameLengthMask) >> DescFrameLengthShift) - 4;
                        skb_put(skb,len);
                        //	skb->dev = netdev;
                        skb->protocol = eth_type_trans(skb, netdev);
                        //skb->ip_summed = CHECKSUM_COMPLETE;

                        netdev->last_rx = jiffies;

                        netif_rx(skb);
                        privdata->nusmartEMACNetStats.rx_packets++;
                        privdata->nusmartEMACNetStats.rx_bytes +=len;

                        /*alloc new sk_buff*/
                        skb = netdev_alloc_skb(netdev,netdev->mtu + ETHERNET_HEADER + ETHERNET_CRC);
                        if(skb==NULL)
                                TR0("*************netdev_alloc_skb error!\n");
                        skb_reserve(skb,2);			
                        dma_addr = dma_map_single(NULL,skb->data,(size_t)skb_tailroom(skb),DMA_FROM_DEVICE);
                        if(dma_addr == 0)
                                TR0("*************dma_map_single error!\n");				
                }
                else
                {
                        privdata->nusmartEMACNetStats.rx_errors++;		

                }

        }

        rxdesc->status = 0;
        rxdesc->buffer1 = 0;
        rxdesc->data1 = 0;

        if((rxdesc->length & RxDescEndOfRing)==RxDescEndOfRing)
        {
                rxdesc->length = RxDescEndOfRing;
                emacdev->RxBusy = 0;
                emacdev->RxBusyDesc = emacdev->RxDesc;
        }
        else
        {
                rxdesc->length = 0;
                (emacdev->RxBusy)++;
                (emacdev->RxBusyDesc)++;
        }

        rxdesc->length |= (((skb_tailroom(skb)) <<DescSize1Shift) & DescSize1Mask);
        rxdesc->buffer1 = (unsigned int)dma_addr;
        rxdesc->data1 = (unsigned int)skb;
        rxdesc->status = DescOwnByDma;

        return 0;
}

static int nusmart_handle_transmit_over(struct net_device *netdev)
{
        unsigned int txbusy;
        DmaDesc *txdesc;
        struct sk_buff *skb = NULL;
        dma_addr_t 	dma_addr = 0;

        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

#if DEBUG
        TR0("nusmart_handle_transmit_over\n");
#endif
        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        txbusy = emacdev->TxBusy;
        txdesc = emacdev->TxBusyDesc;

        if((txdesc->status & DescOwnByDma) !=0)
        {
                return -1;
        }
        if(emacdev->BusyTxDesc == 0)
        {
                return -1;
        }

        (emacdev->BusyTxDesc)--;

        /**/
        if(txdesc->data1 !=0)
        {
                skb = (struct sk_buff*)txdesc->data1;
                dma_addr = (dma_addr_t)txdesc->buffer1;
                dma_unmap_single(NULL,dma_addr,(size_t)skb->len,DMA_TO_DEVICE);		
                dev_kfree_skb_irq(skb);

                if((txdesc->status & DescError)==0)
                {
                        privdata->nusmartEMACNetStats.tx_bytes += skb->len;
                        privdata->nusmartEMACNetStats.tx_packets++;
                }	
                else
                {
                        privdata->nusmartEMACNetStats.tx_errors++;
                }

        }

        txdesc->buffer1 = 0;
        txdesc->data1 	= 0;	

        if((txdesc->length & TxDescEndOfRing)==TxDescEndOfRing)
        {
                emacdev->TxBusy = 0;
                emacdev->TxBusyDesc = emacdev->TxDesc;
                txdesc->length = TxDescEndOfRing;
        }
        else
        {
                (emacdev->TxBusy)++;
                (emacdev->TxBusyDesc)++;
                txdesc->length = 0;
        }

        if(emacdev->queue_stopped)
        {	
                emacdev->queue_stopped=0;
                netif_wake_queue(netdev);	
        }
        return 0;
}

irqreturn_t nusmartEMAC_intr_handler(int intr_num, void * dev_id)
{
        unsigned long flags;
        int ret;
        unsigned int dma_status_reg;
        struct net_device *netdev;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;
        unsigned long i;

#if DEBUG
        TR0("nusmartEMAC_intr_handler\n");
#endif
        netdev = (struct net_device *)dev_id;
        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        spin_lock_irqsave(&emacdev->lock,flags);


        dma_status_reg = readl((void*)(emacdev->DmaBase + DmaStatus));
        i = dma_status_reg & DmaRxState;
        /*	if(dma_status_reg == 0)
            {
            spin_unlock_irqrestore(&emacdev->lock,flags);
            return 	IRQ_NONE;
            }
            */

        /*disable the interrupt*/
        //	wmb();
        writel(DmaIntDisable,(void*)(emacdev->DmaBase + DmaInterrupt));

        /*clear the interrupt*/
        //	wmb();
        writel(dma_status_reg,(void*)(emacdev->DmaBase + DmaStatus));

        if(dma_status_reg & DmaIntTxCompleted)
        {

                do{
                        ret = nusmart_handle_transmit_over(netdev);
                }while(ret == 0);
        }

        if(dma_status_reg & DmaIntRxCompleted)
        {
                do{
                        ret = nusmart_handle_receive_over(netdev);
                }while(ret == 0);

        }

        if(i == DmaRxStopped || i == DmaRxSuspended)
        {
                wmb();
                writel(1,emacdev->DmaBase + DmaRxPollDemand);
        }


        if(dma_status_reg & DmaIntRxNoBuffer)
        {
                TR0("dma_status_reg:%x\n", dma_status_reg);
                TR0("DmaIntRxNoBuffer:%x\n", DmaIntRxNoBuffer);
                TR0("rx no buffer\n");
                privdata->nusmartEMACNetStats.rx_over_errors++;
                wmb();
                writel(1,emacdev->DmaBase + DmaRxPollDemand);
        }

        if(dma_status_reg & DmaIntTxUnderflow)
        {
                TR0("tx under flow\n");
                do{
                        ret = nusmart_handle_transmit_over(netdev);
                }while(ret == 0);
        }

        if(dma_status_reg & DmaIntRxStopped)
        {
                TR0("interrupt rx stopped called\n");

        }

        if(dma_status_reg & DmaIntTxStopped)
        {
                TR0("interrupt tx stopped called\n");

        }

        /*enable the interrupt*/
        //	wmb();
        writel(DmaIntEnable ,(void*)(emacdev->DmaBase + DmaInterrupt));

        spin_unlock_irqrestore(&emacdev->lock,flags);

        return IRQ_HANDLED;
}

static int txdesc_init(struct net_device *netdev)
{

        DmaDesc 	*first_desc = NULL;
        dma_addr_t 	dma_addr = 0;

        int i;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        TR0("txdesc_init\n");

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        first_desc = dma_alloc_coherent(NULL,(size_t)(sizeof(DmaDesc)*NUM_OF_DESC),&dma_addr,GFP_KERNEL | GFP_ATOMIC);


        if(!first_desc)
                return -1;	

        emacdev->TxDescCount	= NUM_OF_DESC;
        emacdev->TxDesc		= first_desc;
        emacdev->TxDescDma	= dma_addr;

        for(i=0;i<NUM_OF_DESC;i++)
        {
                first_desc->status  = 0;
                first_desc->length  = 0;
                first_desc->buffer1 = 0;
                first_desc->buffer2 = 0;
                first_desc->data1   = 0;
                first_desc->data2   = 0;

                if(i==NUM_OF_DESC - 1)
                        first_desc->length = TxDescEndOfRing;
                first_desc += 1;

        }

        emacdev->TxNext 	= 0;
        emacdev->TxBusy 	= 0;
        emacdev->TxNextDesc 	= emacdev->TxDesc;
        emacdev->TxBusyDesc 	= emacdev->TxDesc;
        emacdev->BusyTxDesc  	= 0;

        return 0;

}

static int rxdesc_init(struct net_device *netdev)
{

        DmaDesc 	*first_desc = NULL;
        dma_addr_t 	dma_addr = 0;
        struct sk_buff  *skb = NULL;

        int i;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        TR0("rxdesc_init\n");

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        first_desc = dma_alloc_coherent(NULL,(size_t)(sizeof(DmaDesc)*NUM_OF_DESC),&dma_addr,GFP_KERNEL | GFP_ATOMIC);
        if(!first_desc)
                return -1;	

        emacdev->RxDescCount	= NUM_OF_DESC;
        emacdev->RxDesc		= first_desc;
        emacdev->RxDescDma	= dma_addr;
        emacdev->BusyRxDesc  	= 0;

        for(i=0;i<NUM_OF_DESC;i++)
        {
                first_desc->status  = 0;
                first_desc->length  = 0;
                first_desc->buffer1 = 0;
                first_desc->buffer2 = 0;
                first_desc->data1   = 0;
                first_desc->data2   = 0;

                if(i==NUM_OF_DESC - 1)
                        first_desc->length = RxDescEndOfRing;

                skb = netdev_alloc_skb(netdev,netdev->mtu + ETHERNET_HEADER + ETHERNET_CRC);
                if(skb==NULL)
                        TR0("*************netdev_alloc_skb error!\n");
                skb_reserve(skb,2);			
                if(!skb)
                {
                        rxdesc_exit(netdev);
                        return -1;
                }

                first_desc->data1	= (unsigned int)skb;

                dma_addr = dma_map_single(NULL,skb->data,(size_t)skb_tailroom(skb),DMA_FROM_DEVICE);
                if(dma_addr == 0)
                        TR0("*************dma_map_single error!\n");				


                if(!dma_addr)
                {
                        rxdesc_exit(netdev);
                        return -1;

                }

                first_desc->status	= DescOwnByDma;
                first_desc->length	|=  ((skb_tailroom(skb) <<DescSize1Shift) & DescSize1Mask);
                first_desc->buffer1	= (unsigned int)dma_addr;

                (emacdev->BusyRxDesc)++;
                first_desc += 1;

        }

        emacdev->RxNext 	= 0;
        emacdev->RxBusy 	= 0;
        emacdev->RxNextDesc 	= emacdev->RxDesc;
        emacdev->RxBusyDesc 	= emacdev->RxDesc;

        return 0;
}

static void txdesc_exit(struct net_device *netdev)
{

        int i;
        DmaDesc 	*first_desc = NULL;
        struct sk_buff  *skb = NULL;
        dma_addr_t 	dma_addr = 0;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        TR0("txdesc_exit\n");

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        if(emacdev->TxDesc)
        {
                first_desc = emacdev->TxDesc;		

                for(i=0;i < emacdev->TxDescCount;i++)
                {
                        dma_addr = (dma_addr_t)first_desc->buffer1;
                        skb = (struct sk_buff*)first_desc->data1;

                        if(dma_addr)
                                dma_unmap_single(NULL,dma_addr,(size_t)skb->len,DMA_TO_DEVICE);
                        if(skb)
                                dev_kfree_skb(skb);			

                        first_desc++;
                }	

                dma_free_coherent(NULL,(size_t)(sizeof(DmaDesc)*emacdev->TxDescCount),emacdev->TxDesc,emacdev->TxDescDma);
        }
        emacdev->TxDesc = NULL;
        emacdev->TxDescDma = 0;


}

static void rxdesc_exit(struct net_device *netdev)
{

        int i;
        DmaDesc 	*first_desc = NULL;
        struct sk_buff  *skb = NULL;
        dma_addr_t 	dma_addr = 0;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        TR0("rxdesc_exit\n");

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        if(emacdev->RxDesc)
        {
                first_desc = emacdev->RxDesc;		

                for(i=0;i < emacdev->RxDescCount;i++)
                {
                        dma_addr = (dma_addr_t)first_desc->buffer1;
                        skb = (struct sk_buff*)first_desc->data1;

                        if(dma_addr)
                                dma_unmap_single(NULL,dma_addr,(size_t)skb_tailroom(skb),DMA_FROM_DEVICE);
                        if(skb)
                                dev_kfree_skb(skb);			

                        first_desc++;
                }

                dma_free_coherent(NULL,(size_t)(sizeof(DmaDesc)*emacdev->RxDescCount),emacdev->RxDesc,emacdev->RxDescDma);
        }
        emacdev->RxDesc = NULL;
        emacdev->RxDescDma = 0;

}

static int nusmartEMAC_read_phy_reg(u32 RegBase,u32 PhyBase, u32 RegOffset, u16 * data)
{

        u32 addr=0;
        //u32 loop_variable;
        addr = ((PhyBase << GmiiDevShift) & GmiiDevMask) | ((RegOffset << GmiiRegShift) & GmiiRegMask);
        addr = addr | GmiiBusy | GmiiCsrClk4;

#if DEBUG
        TR0("nusmartEMAC_read_phy_reg\n");
#endif
        while(readl((void*)(RegBase + GmacGmiiAddr)) & GmiiBusy)
                udelay(10);

        wmb();
        writel(addr,(void*)(RegBase + GmacGmiiAddr));

        while(readl((void*)(RegBase + GmacGmiiAddr)) & GmiiBusy)
                udelay(10);

        * data = (u16)(readl((void*)(RegBase + GmacGmiiData)) & 0xFFFF);

        return 0;
}


static void mac_init(struct net_device *netdev)
{

        int phy_addr;
        u16 shortdata;
        s32 status = 0;
        s32 loop_count;
        unsigned int data;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        TR0("mac_init\n");

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        /*verify the phy addr from 32 phy addrs */
        phy_addr = phy_find(netdev);
        privdata->phy_addr = phy_addr;
        if(phy_addr == -1)
        {
                TR0("carrier_off1\n");
                netif_carrier_off(netdev);
        }
        else
        {
                emacdev->PhyBase = phy_addr;
        }

        emacdev->ClockDivMdc = GmiiCsrClk4; 
        /*phy init*/
        if(phy_addr!=-1)
        {

                loop_count = DEFAULT_LOOP_VARIABLE;
                while(loop_count-- > 0)
                {

                        status = nusmartEMAC_read_phy_reg(emacdev->MacBase,emacdev->PhyBase,PHY_STATUS_REG, &shortdata);
                        if(status)
                        {
                                TR0("carrier_off2\n");
                                netif_carrier_off(netdev);
                                goto err_phy_addr;
                        }

                        if((shortdata & Mii_AutoNegCmplt) != 0){
                                break;
                        }
                }

                status = nusmartEMAC_read_phy_reg(emacdev->MacBase,emacdev->PhyBase,PHY_STATUS_REG, &shortdata);
                if(status)
                {
                        TR0("carrier_off3\n");
                        netif_carrier_off(netdev);
                        goto err_phy_addr;
                }

                if((shortdata & Mii_phy_status_link_up) == 0){
                        emacdev->LinkState = LINKDOWN;
                        TR0("carrier_off4\n");
                        netif_carrier_off(netdev);
                        goto err_phy_addr;
                }
                else{
                        emacdev->LinkState = LINKUP;
                }

                status = nusmartEMAC_read_phy_reg(emacdev->MacBase,emacdev->PhyBase,PHY_CONTROL_REG, &shortdata);
                if(status)
                {
                        TR0("carrier_off5\n");
                        netif_carrier_off(netdev);
                        goto err_phy_addr;
                }
                emacdev->DuplexMode = (shortdata & Mii_phy_status_full_duplex)  ? FULLDUPLEX: HALFDUPLEX ;

                status = nusmartEMAC_read_phy_reg(emacdev->MacBase,emacdev->PhyBase,PHY_CONTROL_REG, &shortdata);
                if(status)
                {
                        TR0("carrier_off6\n");
                        netif_carrier_off(netdev);
                        goto err_phy_addr;
                }
                if(shortdata & Mii_phy_status_speed_100)
                {
                        TR0("Ethernet:100M Speed");
                        emacdev->Speed      =   SPEED100;
                }
                else
                {
                        TR0("Ethernet:10M Speed");
                        emacdev->Speed      =   SPEED10;
                }

        }

err_phy_addr:	

        /*mac init*/
        //set dev_addr
        data = (netdev->dev_addr[5] << 8) | netdev->dev_addr[4];
        wmb();
        writel(data,(void*)(emacdev->MacBase + GmacAddr0High));
        data = (netdev->dev_addr[3] << 24) | (netdev->dev_addr[2] << 16) | (netdev->dev_addr[1] << 8) | netdev->dev_addr[0] ;
        wmb();
        writel(data,(void*)(emacdev->MacBase + GmacAddr0Low));
        //read version
        data = readl((void*)(emacdev->MacBase + GmacVersion));
        emacdev->Version = data;
        //set receive all
        data = readl((void*)(emacdev->MacBase + GmacFrameFilter));
        data |= GmacPromiscuousMode;
        wmb();
        writel(data,(void*)(emacdev->MacBase + GmacFrameFilter));
        //set flow control
        data = readl((void*)(emacdev->MacBase + GmacFlowControl));
        data |= GmacRxFlowControl | GmacTxFlowControl | 0xFFFF0000;
        wmb();
        writel(data,(void*)(emacdev->MacBase + GmacFlowControl));
        //set duplexmode	
        if(emacdev->DuplexMode == FULLDUPLEX)
        {
                TR0("Ethernet:FULLDUPLEX");
                data = readl((void*)(emacdev->MacBase + GmacConfig));
                data |= GmacDuplex;
                wmb();
                writel(data,(void*)(emacdev->MacBase + GmacConfig));
        }
        else
        {
                TR0("Ethernet:HALFDUPLEX");
                data = readl((void*)(emacdev->MacBase + GmacConfig));
                data &= (~GmacDuplex);
                wmb();
                writel(data,(void*)(emacdev->MacBase + GmacConfig));

        }
}

static void dma_init(struct net_device *netdev)
{

        unsigned int data;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        TR0("dma_init\n");

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        //reset
        wmb();
        writel(DmaResetOn,(void*)(emacdev->DmaBase + DmaBusMode));
        udelay(100);
        //set dma rx and tx
        wmb();
        writel((unsigned int)emacdev->TxDescDma,(void*)(emacdev->DmaBase + DmaTxBaseAddr));
        wmb();
        writel((unsigned int)emacdev->RxDescDma,(void*)(emacdev->DmaBase + DmaRxBaseAddr));
        //set burst and descriptorskip...
        wmb();
        writel(DmaBurstLength32 | DmaDescriptorSkip2,(void*)(emacdev->DmaBase + DmaBusMode));
        wmb();
        writel(DmaRxStoreAndForward | DmaStoreAndForward | DmaTxSecondFrame | DmaRxThreshCtrl128,(void*)(emacdev->DmaBase + DmaControl));	
        //set flowcontrol
        data = readl((void*)(emacdev->DmaBase + DmaControl));
        data |= DmaRxFlowCtrlAct2K | DmaRxFlowCtrlDeact5K | DmaEnHwFlowCtrl;
        wmb();
        writel(data,(void*)(emacdev->DmaBase + DmaControl));
}
/*
static void dma_init_t(struct net_device *netdev)
{

        unsigned int data;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        TR0("dma_init_t\n");

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);
        //reset
        wmb();
        writel(DmaResetOn,(void*)(emacdev->DmaBase + DmaBusMode));
        udelay(100);

        //set dma rx and tx ///////////////////////////////////////////
        wmb();
        writel((unsigned int)emacdev->TxDescDma,(void*)(emacdev->DmaBase + DmaTxBaseAddr));
        wmb();
        writel((unsigned int)emacdev->RxDescDma,(void*)(emacdev->DmaBase + DmaRxBaseAddr));
        
        //set burst and descriptorskip...
        wmb();
        writel(DmaBurstLength32 | DmaDescriptorSkip2,(void*)(emacdev->DmaBase + DmaBusMode));
        wmb();
        writel(DmaRxStoreAndForward | DmaStoreAndForward | DmaTxSecondFrame | DmaRxThreshCtrl128,(void*)(emacdev->DmaBase + DmaControl));	
        //set flowcontrol
        data = readl((void*)(emacdev->DmaBase + DmaControl));
        data |= DmaRxFlowCtrlAct2K | DmaRxFlowCtrlDeact5K | DmaEnHwFlowCtrl;
        wmb();
        writel(data,(void*)(emacdev->DmaBase + DmaControl));
}
*/


static void emac_start(struct net_device *netdev)
{

        unsigned int data;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        TR0("emac_start\n");

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);
        //clear interrupt
        data = readl((void*)(emacdev->DmaBase + DmaStatus));
        wmb();
        writel(data,(void*)(emacdev->DmaBase + DmaStatus));
        //enable interrupt
        wmb();
        writel(DmaIntEnable,(void*)(emacdev->DmaBase + DmaInterrupt));
        //enable dma rx and tx
        data = readl((void*)(emacdev->DmaBase + DmaControl));
        data |= DmaRxStart | DmaTxStart;
        wmb();
        writel(data,(void*)(emacdev->DmaBase + DmaControl));
        //enable mac tx and rx
        data = readl((void*)(emacdev->MacBase + GmacConfig));
        data |= GmacRx | GmacTx;
        wmb();
        writel(data,(void*)(emacdev->MacBase + GmacConfig));

}

static void emac_stop(struct net_device *netdev)
{

        unsigned int data;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

        TR0("emac_stop\n");

        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);
        //disable mac tx and rx
        data = readl((void*)(emacdev->MacBase + GmacConfig));
        data &= (~GmacRx & ~GmacTx);
        wmb();
        writel(data,(void*)(emacdev->MacBase + GmacConfig));
        //disable dma tx and rx
        data = readl((void*)(emacdev->DmaBase + DmaControl));
        data &= (~DmaRxStart & ~DmaTxStart);
        wmb();
        writel(data,(void*)(emacdev->DmaBase + DmaControl));
        //disable interrupt
        wmb();
        writel(DmaIntDisable,(void*)(emacdev->DmaBase + DmaInterrupt));

}

static void nusmartEMAC_linux_cable_unplug_function(unsigned int timerdata)
{
        s32 status;
        u16 data;
        struct net_device *netdev = (struct net_device *)timerdata;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

#if DEBUG
        TR0("nusmartEMAC_linux_cable_unplug_function\n");
#endif
        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        init_timer(&privdata->nusmartEMAC_cable_unplug_timer);
        privdata->nusmartEMAC_cable_unplug_timer.expires = HZ*5 + jiffies;
        status = nusmartEMAC_read_phy_reg(emacdev->MacBase,emacdev->PhyBase,PHY_STATUS_REG, &data);

        if(status)
                goto err_impossible;

        if((data & Mii_phy_status_link_up) == 0){
                /*No Link*/
                emacdev->LinkState = 0;
                emacdev->DuplexMode = 0;
                emacdev->Speed = 0;
                emacdev->LoopBackMode = 0;

                //TR0("carrier_off\n");
                netif_carrier_off(netdev);

        }
        else{
                /*Link UP*/
                if(!emacdev->LinkState){

                        mac_init(netdev);
                        //emac_start(netdev);
                        TR0("carrier_on\n");
                        netif_carrier_on(netdev);

                }
        }
err_impossible:
        add_timer(&privdata->nusmartEMAC_cable_unplug_timer);

}

struct net_device *netdev_t; /////////////////////////////////wwl_a

static int nusmartEMAC_linux_open(struct net_device *netdev)
{

        struct nusmartEMACprivdataStruct *privdata = netdev_priv(netdev);

        TR0("nusmartEMAC_linux_open\n");

        netdev_t = netdev;
        /*Request for an interrupt.*/

        if(txdesc_init(netdev))
                goto err_tx_init;

        if(rxdesc_init(netdev))
                goto err_rx_init;

        dma_init(netdev);
        mac_init(netdev);

        //	privdata->last_duplex = -1;
        //	privdata->last_carrier = -1;
        //	phy_start(privdata->phy_dev);
        //	napi_enable(&privdata->napi);
        emac_start(netdev);

        if(request_irq (privdata->nusmartEMACdev.irq, nusmartEMAC_intr_handler, IRQF_SHARED, netdev->name,netdev)){
                TR0("request irq failure\n");
                return -1;
        }

        netif_start_queue(netdev);

        /*Setting up the cable unplug timer*/
        init_timer(&privdata->nusmartEMAC_cable_unplug_timer);
        privdata->nusmartEMAC_cable_unplug_timer.function = (void *)nusmartEMAC_linux_cable_unplug_function;
        privdata->nusmartEMAC_cable_unplug_timer.data = (unsigned int)netdev;
        privdata->nusmartEMAC_cable_unplug_timer.expires = HZ + jiffies;
        add_timer(&privdata->nusmartEMAC_cable_unplug_timer);


        return 0;

err_rx_init:
        TR0("rxdesc init failure\n");
        txdesc_exit(netdev);	
err_tx_init:
        TR0("txdesc init failure\n");
        free_irq(privdata->nusmartEMACdev.irq,netdev);	

        return -1;
}

static int nusmartEMAC_linux_close(struct net_device *netdev)
{

        struct nusmartEMACprivdataStruct *privdata = netdev_priv(netdev);

        TR0("nusmartEMAC_linux_close\n");

        del_timer(&privdata->nusmartEMAC_cable_unplug_timer);

        netif_stop_queue(netdev);


        /* Bring the PHY down */
        //	if (privdata->phy_dev)
        //		phy_stop(privdata->phy_dev);

        emac_stop(netdev);

        //	napi_disable(&privdata->napi);

        rxdesc_exit(netdev);

        txdesc_exit(netdev);

        free_irq(privdata->nusmartEMACdev.irq,netdev);

        return 0;
}

static int nusmartEMAC_linux_xmit_frames(struct sk_buff *skb, struct net_device *netdev)
{	

        unsigned long flags;
        DmaDesc 	*txdesc;
        dma_addr_t	dma_addr = 0;
        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;
        unsigned long i;

#if DEBUG
        TR0("nusmartEMAC_linux_xmit_frames\n");
#endif
        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        spin_lock_irqsave(&emacdev->lock,flags);
        /*check free descriptor*/
        txdesc = emacdev->TxNextDesc;

        if(emacdev->BusyTxDesc == NUM_OF_DESC)
        {
                netif_stop_queue(netdev);
                emacdev->queue_stopped = 1;
                TR0("**********xmit busy\n");
                goto err_all;
        }

        /*map the skb*/
        dma_addr = dma_map_single(NULL,skb->data,(size_t)skb->len,DMA_TO_DEVICE);
        if(dma_addr == 0)
                TR0("*************dma_map_single error!\n");				

        if(!dma_addr)
        {
                TR0("**********xmit err\n");
                goto err_all;
        }
        /*insert the skb to the tx descriptor RING*/


        txdesc->length |= (((skb->len) << DescSize1Shift) & DescSize1Mask);
        txdesc->length |= (DescTxFirst | DescTxLast | DescTxIntEnable);

        txdesc->buffer1 = (unsigned int)dma_addr;
        txdesc->data1 = (unsigned int)skb;


        if((txdesc->length & TxDescEndOfRing) == TxDescEndOfRing)
        {
                emacdev->TxNext = 0;
                emacdev->TxNextDesc = emacdev->TxDesc;
        }
        else
        {
                (emacdev->TxNext)++;
                (emacdev->TxNextDesc)++;

        }

        txdesc->status = DescOwnByDma;

        (emacdev->BusyTxDesc)++;

        netdev->trans_start = jiffies;

        i = readl((void*)(emacdev->DmaBase + DmaStatus)) & DmaTxState;

        if((i == DmaTxSuspended) || (i == DmaTxStopped))
        {
                wmb();
                /*force DMA to start transmission*/
                writel(1,(void*)(emacdev->DmaBase + DmaTxPollDemand));
        }

        spin_unlock_irqrestore(&emacdev->lock,flags);


        return NETDEV_TX_OK;

err_all:
        spin_unlock_irqrestore(&emacdev->lock,flags);
        return NETDEV_TX_BUSY;
}

static struct net_device_stats *  nusmartEMAC_linux_get_stats(struct net_device *netdev)
{
#if DEBUG
        TR0("nusmartEMAC_linux_get_stats\n");
#endif

        return ( &(((struct nusmartEMACprivdataStruct *)(netdev_priv(netdev)))->nusmartEMACNetStats) );
}

static int nusmartEMAC_linux_set_mac_address(struct net_device *netdev, void * macaddr)
{

        struct sockaddr *addr = macaddr;
        unsigned int data;

        struct nusmartEMACprivdataStruct 	*privdata;
        struct nusmartEMACDeviceStruct 		*emacdev;

#if DEBUG
        TR0("nusmartEMAC_linux_set_mac_address\n");
#endif
        privdata = (struct nusmartEMACprivdataStruct *)netdev_priv(netdev);
        emacdev  = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);


        if(!is_valid_ether_addr(addr->sa_data))
                return -EADDRNOTAVAIL;

        //set dev_addr
        data = (addr->sa_data[5] << 8) | addr->sa_data[4];
        wmb();
        writel(data,(void*)(emacdev->MacBase + GmacAddr0High));
        data = (addr->sa_data[3] << 24) | (addr->sa_data[2] << 16) | (addr->sa_data[1] << 8) | addr->sa_data[0] ;
        wmb();
        writel(data,(void*)(emacdev->MacBase + GmacAddr0Low));

        memcpy(netdev->dev_addr,addr->sa_data,6);

        return 0;
}

static int nusmart_suspend(struct platform_device *dev, pm_message_t state)
{

        struct net_device *netdev = platform_get_drvdata(dev);
        //struct nusmartEMACprivdataStruct *privdata = netdev_priv(netdev);
        //struct nusmartEMACDeviceStruct *emacdev = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        TR0("nusmart_suspend\n");

        if (netdev) {
                if (netif_running(netdev)) {
                        netif_device_detach(netdev);
                        emac_stop(netdev);
                        rxdesc_exit(netdev);    //wwl_add
                        txdesc_exit(netdev);    //wwl_add
                }
        }

        return 0;
}

static int nusmart_resume(struct platform_device *dev)
{

#if 1
        struct net_device *netdev = platform_get_drvdata(dev);
        //struct nusmartEMACprivdataStruct *privdata = netdev_priv(netdev);
        //struct nusmartEMACDeviceStruct *emacdev = (struct nusmartEMACDeviceStruct *)(&privdata->nusmartEMACdev);

        TR0("nusmart_resume\n");

        if (netdev) {
                if (netif_running(netdev)) {
                        if(txdesc_init(netdev)) //wwl_add
                                goto err_tx_init;

                        if(rxdesc_init(netdev)) //wwl_add
                                goto err_rx_init;

                        dma_init(netdev);
                        mac_init(netdev);
                        emac_start(netdev);

                        netif_device_attach(netdev);
                }
        }

        return 0;

err_rx_init:
        TR0("rxdesc init failure\n");
        txdesc_exit(netdev);	
err_tx_init:
        TR0("txdesc init failure\n");
        //free_irq(privdata->nusmartEMACdev.irq,netdev);	

        return -1;
#endif

}

static struct platform_driver nusmart_platform_driver =
{
        .probe		= nusmart_probe,
        .remove		= nusmart_remove,
        .suspend	= nusmart_suspend,
        .resume		= nusmart_resume,	
        .driver = {
                .name	= nusmartEMAC_driver_name,
                .owner	= THIS_MODULE,
        }
};

int __init nusmartEMAC_Module_init(void)
{
        return platform_driver_register (&nusmart_platform_driver);
}

void __exit nusmartEMAC_Module_exit(void)
{
        platform_driver_unregister(&nusmart_platform_driver);
}

module_init(nusmartEMAC_Module_init);
module_exit(nusmartEMAC_Module_exit);

MODULE_AUTHOR("Yang Shuai");
MODULE_DESCRIPTION("NUSMART EMAC NETWORK DRIVER WITH PLATFORM INTERFACE");
MODULE_LICENSE("GPL");

