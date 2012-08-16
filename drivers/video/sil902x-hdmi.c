/*
 * Base driver for  SIL902x chip (I2C)
 *
 * Copyright (C) 2011 NUFRONT INC.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/earlysuspend.h>
#include <asm/div64.h>
#include <video/edid.h>
#include "edid.h"

#include "sil902x-hdmi.h"

#define MODULE_NAME			"sil902x-hdmi"

#define PLUG_IN				1
#define PLUG_OUT			0

#define HDMI_ERROR			1
#define HDMI_OK				0





struct sil902x_data {
	unsigned char uc_gpio;
};

struct sil902x_info {
	unsigned char uc_connected;
	unsigned char uc_gpio;
	unsigned char uc_state;
	unsigned char uc_earlysuspend;
	struct i2c_client* p_client;
	struct work_struct kstr_workstruct;
	struct workqueue_struct* pkstr_workqueue;
};

struct sil902x_info g_kstr_info;



int Sil902x_Init(struct i2c_client* client);
static unsigned int Sil902x_AudioConfigure(struct i2c_client* client);
static void sil902x_earlysuspend(struct early_suspend* h);
static void sil902x_earlyresume(struct early_suspend* h);


unsigned char g_uc_VideoMode[9]= {
0x02, 0x60, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x0F
};

static const struct i2c_device_id sil902x_idtable[] = {
    { MODULE_NAME,   0},
    { }
};

MODULE_DEVICE_TABLE(i2c, sil902x_idtable);








static int Sil902x_RegWrite(struct i2c_client* p_client, unsigned int ui_num,  unsigned char uc_regoffset, unsigned char* uc_RegBuf)
{
	int i_ret, ui_count;
	unsigned char uc_i2cbuf[16];


	i_ret = -EIO;
	if (ui_num <= 15) {
		ui_count = ui_num + 1;
		uc_i2cbuf[0] = uc_regoffset;
		memcpy(&uc_i2cbuf[1], uc_RegBuf, ui_num);
		if(i2c_master_send(p_client, uc_i2cbuf, ui_count) == ui_count) {
			i_ret = HDMI_OK;
		}
	}	

	return i_ret;
}

static int Sil902x_RegRead(struct i2c_client* p_client, unsigned int ui_num, unsigned char uc_regoffset, unsigned char* uc_RegBuf)
{
	int i_ret;

	i_ret = -EIO;
	if(i2c_master_send(p_client, &uc_regoffset, 1) == 1) {
		if (i2c_master_recv(p_client, uc_RegBuf, ui_num) == ui_num) {
			i_ret = HDMI_OK;
		}
	}


	return i_ret;
}

static int sil902x_event_notify(struct notifier_block *self, unsigned long action, void *data)
{
        struct fb_event *event = data;
	int i_blank;
 
       switch(action) {
        case FB_EVENT_BLANK:
		i_blank = *(int*)event->data;
		if (i_blank) {
                	sil902x_earlysuspend(NULL);
		}
		else {
			sil902x_earlyresume(NULL);
		}
                break;
        }

        return 0;
}

static struct notifier_block hdmi_event_notifier = {
        .notifier_call  = sil902x_event_notify,
};

static int sil902x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i_ret;

	//init Sil902x hdmi
	i_ret = Sil902x_Init(client);

	return i_ret;

}

static int __devexit sil902x_i2c_remove(struct i2c_client *client)
{

	return 0;
}


#ifdef CONFIG_PM
static int sil902x_resume(struct device* dev)
{
	struct sil902x_info* psil902x_info;
	unsigned char uc_data[2];

	printk("HDMI resume.\r\n");
	psil902x_info = &g_kstr_info;
	// Reset Sil902x device
	gpio_direction_output(psil902x_info->uc_gpio, 0);
	msleep(10);
	gpio_direction_output(psil902x_info->uc_gpio, 1);
	psil902x_info = &g_kstr_info;
	psil902x_info->uc_earlysuspend = EARLYSUSPEND_NO;
	// Switch to TPI
	uc_data[0] = 0x00;
	Sil902x_RegWrite(psil902x_info->p_client, 1, 0xC7, uc_data);
	// Configure Audio
	Sil902x_AudioConfigure(psil902x_info->p_client);
	// Clear interrupt status
	uc_data[0] = 0xFF;
	Sil902x_RegWrite(psil902x_info->p_client, 1, REG_INTERRUPT_STATUS, uc_data);
	// Enable hot plug interrupt
	uc_data[0] = BIT_HOTPLUG_ENABLE;
	Sil902x_RegWrite(psil902x_info->p_client, 1, REG_INTERRUPT_CTRL, uc_data);
	// Read Interrupt Status Register
	Sil902x_RegRead(psil902x_info->p_client, 1, REG_INTERRUPT_STATUS, uc_data);
	printk("Sil9022 Interrupt status: 0x%x.\r\n", uc_data[0]);
	if (uc_data[0] != 0x0) {
		// Plugged in already, switch to D0 state
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_INTERRUPT_STATUS, uc_data);
		// Power up to D0 state
		uc_data[0] = BIT_POWER_D0;
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_POWER_CTRL, uc_data);
		// Enable TMDS output
		uc_data[0] = 0x1;
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_SYSTEM_CTRL, uc_data); 
		psil902x_info->uc_connected = HOTPLUG_CONNECTED;
		psil902x_info->uc_state = POWER_D0;
	}
	else {
		// Plugged out, switch to D3 state
		// Disable TMDS output
		uc_data[0] = 0x18;
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_SYSTEM_CTRL, uc_data); 
		// Power up to D3 state
		uc_data[0] = BIT_POWER_D3;
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_POWER_CTRL, uc_data);
		psil902x_info->uc_connected = HOTPLUG_DISCONNECTED;
		psil902x_info->uc_state = POWER_D3;
	}

        return HDMI_OK;
}


static int sil902x_suspend(struct device* dev)
{
	// do nothing
	printk("HDMI suspend.\r\n");

        return HDMI_OK;
}


static const struct dev_pm_ops sil902x_pm_ops = {
	.suspend	= sil902x_suspend,
	.resume		= sil902x_resume,
};

#endif


static void sil902x_earlysuspend(struct early_suspend* h)
{
	struct sil902x_info* psil902x_info;
	unsigned char uc_data[2];

	//printk("HDMI early suspend\r\n");
	psil902x_info = &g_kstr_info;
	if (psil902x_info->uc_connected == HOTPLUG_CONNECTED) {
		// Disable TMDS output
		uc_data[0] = 0x18;
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_SYSTEM_CTRL, uc_data); 
		psil902x_info->uc_earlysuspend = EARLYSUSPEND_YES;
	}
}

static void sil902x_earlyresume(struct early_suspend* h)
{
	struct sil902x_info* psil902x_info;
	unsigned char uc_data[2];

	//printk("HDMI early resume\r\n");
	psil902x_info = &g_kstr_info;
	if (psil902x_info->uc_connected == HOTPLUG_CONNECTED) {
		// Enable TMDS output
		uc_data[0] = 0x01;
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_SYSTEM_CTRL, uc_data); 
		psil902x_info->uc_earlysuspend = EARLYSUSPEND_NO;
	}
}

static struct i2c_driver sil902x_i2c_driver = {
	.driver = {
		.name   	= MODULE_NAME,
		.owner  	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm		= &sil902x_pm_ops,
#endif
	},
	.probe      = sil902x_i2c_probe,
	.remove     = __devexit_p(sil902x_i2c_remove),
	.id_table   = sil902x_idtable,
};



#if 0
static unsigned int Sil902x_VideoConfigure(struct i2c_client* client)
{
	byte Result;
	byte Status;


	vid_mode = g_uc_VideoMode[0];	//Command.Arg[0];//zhy 2009 11 24
	//DisableTMDS();                  // turn off TMDS output
	//DelayMS(T_RES_CHANGE_DELAY);    // allow control InfoFrames to pass through to the sink device.

        InitVideo(vid_mode, ((g_uc_VideoMode[1] >> 6) & TWO_LSBITS), MODE_CHANGE, g_uc_VideoMode[8]);        // Will set values based on VModesTable[Arg[0])
        g_uc_VideoMode[2] &= ~BITS_5_4;                         // No Deep Color in 9022A/24A, 9022/4
        WriteByteTPI(TPI_INPUT_FORMAT_REG, g_uc_VideoMode[2]);     // Write output formats to register 0x0A

        if ((g_uc_VideoMode[3] & TWO_LSBITS) == CS_DVI_RGB)
        {
		ReadClearWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, OUTPT_MODE_HDMI);        // Set 0x1A[0] to DVI
		ReadModifyWriteTPI(TPI_OUTPUT_FORMAT_REG, TWO_LSBITS, CS_DVI_RGB);   // Set 0x0A[1:0] to DVI RGB
	}
        else if (((g_uc_VideoMode[3] & TWO_LSBITS) >= CS_HDMI_RGB) && ((g_uc_VideoMode[3] & TWO_LSBITS) < CS_DVI_RGB)) // An HDMI CS
        {
		;
        }

	WriteByteTPI(TPI_PIX_REPETITION, g_uc_VideoMode[1]);  // 0x08
        WriteByteTPI(TPI_OUTPUT_FORMAT_REG, g_uc_VideoMode[3]);   // Write input and output formats to registers 0x09, 0x0A

	if (g_uc_VideoMode[4] & MSBIT)             // set embedded sync,
        {
		ReadSetWriteTPI(TPI_SYNC_GEN_CTRL, MSBIT); // set 0x60[7] = 1 for Embedded Sync
		if (g_uc_VideoMode[5] & BIT_6)
		{
	            	Result = SetEmbeddedSync(vid_mode);       // Call SetEmbeddedSync() with Video Mode as a parameter
			if (!Result) {
				return SET_EMBEDDED_SYC_FAILURE;
            		}
			EnableEmbeddedSync();
		}	
		else {
                	ReadClearWriteTPI(TPI_DE_CTRL, BIT_6);     // clear 0x63[6] = 0 to disable internal DE
		}	
        }
        else                                    // Set external sync
        {
            ReadClearWriteTPI(TPI_SYNC_GEN_CTRL, MSBIT);       // set 0x60[7] = 0 for External Sync
            if (g_uc_VideoMode[5] & BIT_6)     // set Internal DE Generator only if 0x60[7] == 0
            {
                ReadSetWriteTPI(TPI_DE_CTRL, BIT_6);               // set 0x63[6] = 1 for DE
                Status = SetDE(vid_mode);         // Call SetDE() with Video Mode as a parameter
                if (Status != DE_SET_OK) {
                    return Status;
                }
            }
            else if (!(g_uc_VideoMode[5] & BIT_6)) {
                ReadClearWriteTPI(TPI_DE_CTRL, BIT_6);     // clear 0x63[6] = 0 to disable internal DE
            }
        }

	TxPowerState(TX_POWER_STATE_D0);//zhy +
        ReadModifyWriteTPI(TPI_AVI_BYTE_2, BITS_7_6, g_uc_VideoMode[6] << 4);
        if ((g_uc_VideoMode & BITS_3_2) == SET_EX_COLORIMETRY) {
		ReadModifyWriteTPI(TPI_AVI_BYTE_3, BITS_6_5_4, g_uc_VideoMode[6]);
        }

        WriteByteTPI(TPI_YC_Input_Mode, g_uc_VideoMode[7]);
 	// This check needs to be changed to if HDCP is required by the content... once support has been added by RX-side library.
	if (HDCP_TxSupports == TRUE) {
		ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, LINK_INTEGRITY_MODE_MASK | TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK, LINK_INTEGRITY_DYNAMIC | TMDS_OUTPUT_CONTROL_ACTIVE | AV_MUTE_MUTED);
		tmdsPoweredUp = TRUE;
	}
	else {
		EnableTMDS();
	}

        return VIDEO_MODE_SET_OK;
}
#endif

static unsigned int Sil902x_AudioConfigure(struct i2c_client* client)
{
	unsigned char uc_cnt, uc_Regdata, uc_RegList[15];

	// Mute Audio
	Sil902x_RegRead(client, 1, REG_AUDIO_CONFIGURE, &uc_Regdata);
	uc_Regdata |= BIT_AUDIO_MUTE;
	Sil902x_RegWrite(client, 1, REG_AUDIO_CONFIGURE, &uc_Regdata);
	//Select I2S interface
	uc_Regdata &= (0x3F);
	uc_Regdata |= BIT_AUDIO_I2S;
	Sil902x_RegWrite(client, 1, REG_AUDIO_CONFIGURE, &uc_Regdata);
	//Configure I2S
	uc_Regdata = 0x80;
	Sil902x_RegWrite(client, 1, REG_I2S_CONFIGURE, &uc_Regdata);
	//Configure I2S Mapping
	uc_Regdata = 0x80;
	Sil902x_RegWrite(client, 1, REG_I2S_MAPPING, &uc_Regdata);
	uc_Regdata = 0x11;
	Sil902x_RegWrite(client, 1, REG_I2S_MAPPING, &uc_Regdata);
	uc_Regdata = 0x22;
	Sil902x_RegWrite(client, 1, REG_I2S_MAPPING, &uc_Regdata);
	uc_Regdata = 0x33;
	Sil902x_RegWrite(client, 1, REG_I2S_MAPPING, &uc_Regdata);
	//Configure sample rate and sample size
	uc_Regdata = (BIT_AUDIO_16BIT | BIT_AUDIO_48KHZ);
	Sil902x_RegWrite(client, 1, REG_AUDIO_SAMPLERATE, &uc_Regdata);
	//Configure I2S Channel status
	uc_Regdata = 0x00;
	Sil902x_RegWrite(client, 1, REG_I2S_CHSTATUS_0, &uc_Regdata);
	Sil902x_RegWrite(client, 1, REG_I2S_CHSTATUS_1, &uc_Regdata);
	Sil902x_RegWrite(client, 1, REG_I2S_CHSTATUS_2, &uc_Regdata);
	uc_Regdata = 0x02;
	Sil902x_RegWrite(client, 1, REG_I2S_CHSTATUS_3, &uc_Regdata);
	Sil902x_RegWrite(client, 1, REG_I2S_CHSTATUS_4, &uc_Regdata);

	//Configure Audio Infoframe Header
	memset(uc_RegList, 0, sizeof(uc_RegList));
	uc_RegList[0] = 0xC2;
	uc_RegList[1] = 0x84;
	uc_RegList[2] = 0x01;
	uc_RegList[3] = 0x0A;
	uc_RegList[5] = 0x00;
	uc_RegList[4] = 0x84 + 0x01 + 0x0A;
	uc_RegList[6] = 0x00;
	uc_RegList[8] = 0x00;
	for (uc_cnt = 5; uc_cnt < 15; uc_cnt ++)
        uc_RegList[4] += uc_RegList[uc_cnt];
	uc_RegList[4] = 0x100 - uc_RegList[4];
	Sil902x_RegWrite(client, 15, REG_AUDIO_INFOFRAME, uc_RegList);
	uc_Regdata = 0x80;
	Sil902x_RegWrite(client, 1, REG_AUDIO_CONFIGURE, &uc_Regdata);

	return HDMI_OK;
}


static unsigned int sil902x_switch(struct sil902x_info* p_info, unsigned char uc_workmode)
{
	struct sil902x_info* psil902x_info;
	unsigned char uc_data[4];

	psil902x_info = p_info;
	// Reset Sil902x device
	gpio_direction_output(psil902x_info->uc_gpio, 0);
	msleep(10);
	gpio_direction_output(psil902x_info->uc_gpio, 1);
	// Switch to TPI mode
	uc_data[0] = 0x0;
	Sil902x_RegWrite(psil902x_info->p_client, 1, 0xC7, uc_data);
	// Clear interrupt status
	uc_data[0] = 0xFF;
	Sil902x_RegWrite(psil902x_info->p_client, 1, REG_INTERRUPT_STATUS, uc_data);
	// Enable Hotplug interrupt
	uc_data[0] = BIT_HOTPLUG_ENABLE;
	Sil902x_RegWrite(psil902x_info->p_client, 1, REG_INTERRUPT_CTRL, uc_data);
	if (uc_workmode == POWER_D0) {
		// Configure Audio
		Sil902x_AudioConfigure(psil902x_info->p_client);
		// Power up to D0 state
		uc_data[0] = BIT_POWER_D0;
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_POWER_CTRL, uc_data);
		if (psil902x_info->uc_earlysuspend == EARLYSUSPEND_NO) {
			// Enable TMDS output
			uc_data[0] = 0x1;
		}
		else {
			// Disable TMDS output
			uc_data[0] = 0x18;
		} 
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_SYSTEM_CTRL, uc_data);
	}
	else {
		// Disable TMDS output
		uc_data[0] = 0x18;
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_SYSTEM_CTRL, uc_data); 
		// Power up to D3 state
		uc_data[0] = BIT_POWER_D3;
		Sil902x_RegWrite(psil902x_info->p_client, 1, REG_POWER_CTRL, uc_data);
	}

	return HDMI_OK;
}

static void sil902x_workhandler(struct work_struct* work_data)
{
	unsigned char uc_data[4];
	unsigned char uc_cnt;
	struct sil902x_info* psil902x_info;

        psil902x_info = container_of(work_data, struct sil902x_info, kstr_workstruct);
	if (psil902x_info->uc_state == POWER_D3) {
		// Switch to D0 state
		sil902x_switch(psil902x_info, POWER_D0);	
		psil902x_info->uc_state = POWER_D0;
	}
	else {
		Sil902x_RegRead(psil902x_info->p_client, 1, REG_INTERRUPT_STATUS, uc_data);
		if (uc_data[0] & 0x01) {
			if (psil902x_info->uc_connected == HOTPLUG_DISCONNECTED) {
				printk("hdmi connected, enter D0 state.\r\n");
				psil902x_info->uc_connected = HOTPLUG_CONNECTED;
				// Clear Interrupt
				Sil902x_RegWrite(psil902x_info->p_client, 1, REG_INTERRUPT_STATUS, uc_data);
			}
			else {
				uc_cnt = 10;
				// Check if unplug is reliable
				do {	
					Sil902x_RegRead(psil902x_info->p_client, 1, REG_INTERRUPT_STATUS, uc_data);
					msleep(200);
				} while (((uc_data[0] & 0x0F) != UNPLUG_CONFIRMED) && (-- uc_cnt));				
				if (uc_cnt) {
					// Disable all interrupts
					uc_data[0] = 0x0;
					Sil902x_RegWrite(psil902x_info->p_client, 1, REG_INTERRUPT_CTRL, uc_data);
					// Switch to D3 state
					sil902x_switch(psil902x_info, POWER_D3);
					psil902x_info->uc_state = POWER_D3;
					printk("hdmi disconnected, enter D3 state\r\n");
					psil902x_info->uc_connected = HOTPLUG_DISCONNECTED;
				}
				else {
					// Not a unplug, so just clear Interrupt
					Sil902x_RegWrite(psil902x_info->p_client, 1, REG_INTERRUPT_STATUS, uc_data);
				}
			}
		}
	}

	enable_irq(psil902x_info->p_client->irq);
	//printk("Sil902x work queue handler.\r\n");
}


static irqreturn_t sil902x_handler(int irq, void* dev_id)
{
	struct sil902x_info* psil902x_info;

	psil902x_info = (struct sil902x_info*)dev_id;
        disable_irq_nosync(psil902x_info->p_client->irq);
	queue_work(psil902x_info->pkstr_workqueue, &(psil902x_info->kstr_workstruct));

	return IRQ_HANDLED;
}


int Sil902x_Init(struct i2c_client* client)
{
	int i_ret;
	unsigned int ui_devid;
	unsigned char uc_data[4];
	struct sil902x_data* psil902x;
	struct sil902x_info* psil902x_info;

	i_ret = -EIO;
	printk("Sil902x_Init function.\r\n");
	psil902x = (struct sil902x_data*)(client->dev.platform_data);
	i_ret = gpio_request(psil902x->uc_gpio, "sil902x-hdmi");
	if (i_ret == 0) {
		// Reset Sil902x device
		gpio_direction_output(psil902x->uc_gpio, 0);
		msleep(10);
		gpio_direction_output(psil902x->uc_gpio, 1);
		psil902x_info = &g_kstr_info;
		psil902x_info->uc_gpio = psil902x->uc_gpio;
		psil902x_info->p_client = client;
		uc_data[0] = 0x00;
		if (Sil902x_RegWrite(client, 1, 0xC7, uc_data) == 0) {
			// Read Device ID
			Sil902x_RegRead(client, 3, REG_DEVID, uc_data);
			ui_devid = (uc_data[0] << 16) | (uc_data[1] << 8) | (uc_data[2]);
			printk("Sil902x HDMI: ID is 0x%x, irq is %d.\r\n", ui_devid, client->irq);
			psil902x_info->uc_earlysuspend = EARLYSUSPEND_NO;
			// Configure Audio
			Sil902x_AudioConfigure(client);
			// Clear interrupt status
			uc_data[0] = 0xFF;
			Sil902x_RegWrite(client, 1, REG_INTERRUPT_STATUS, uc_data);
			// Enable hot plug interrupt
			uc_data[0] = BIT_HOTPLUG_ENABLE;
			Sil902x_RegWrite(client, 1, REG_INTERRUPT_CTRL, uc_data);
			// Read Interrupt Status Register
			Sil902x_RegRead(client, 1, REG_INTERRUPT_STATUS, uc_data);
			printk(KERN_INFO "Sil9022 Interrupt status: 0x%x.\r\n", uc_data[0]);
			if (uc_data[0] != 0x0) {
				// Plugged in already, switch to D0 state
				Sil902x_RegWrite(client, 1, REG_INTERRUPT_STATUS, uc_data);
				// Power up to D0 state
				uc_data[0] = BIT_POWER_D0;
				Sil902x_RegWrite(client, 1, REG_POWER_CTRL, uc_data);
				// Enable TMDS output
				uc_data[0] = 0x1;
				Sil902x_RegWrite(client, 1, REG_SYSTEM_CTRL, uc_data); 
				psil902x_info->uc_connected = HOTPLUG_CONNECTED;
				psil902x_info->uc_state = POWER_D0;
			}
			else {
				// Plugged out, switch to D3 state
				// Disable TMDS output
				uc_data[0] = 0x18;
				Sil902x_RegWrite(client, 1, REG_SYSTEM_CTRL, uc_data); 
				// Power up to D3 state
				uc_data[0] = BIT_POWER_D3;
				Sil902x_RegWrite(client, 1, REG_POWER_CTRL, uc_data);
				psil902x_info->uc_connected = HOTPLUG_DISCONNECTED;
				psil902x_info->uc_state = POWER_D3;
			}
			//Create workqueue to deal with HDMI hot plug interrupt
			psil902x_info->pkstr_workqueue = create_singlethread_workqueue("sil902x-hdmi");
			INIT_WORK(&(psil902x_info->kstr_workstruct), sil902x_workhandler);
			if (request_irq(client->irq, sil902x_handler, IRQF_TRIGGER_FALLING , client->name, psil902x_info) == 0) {
				// Register FB notifier since we need to deal with early suspend event before/after LCDC control
				fb_register_client(&hdmi_event_notifier);
				i_ret = HDMI_OK;
			}
			else {
				gpio_free(psil902x->uc_gpio);
			}
		}
		else {
			gpio_free(psil902x->uc_gpio);
		}
	}


	return i_ret;
}

static int __init hdmi_sil902x_init(void)
{
	int ret;

	printk("Sil902x HDMI Driver Init.\r\n");
	ret = i2c_add_driver(&sil902x_i2c_driver);
	if(ret)
	{
		printk(KERN_EMERG "i2c_add_driver failed! exit...\n");
		i2c_del_driver(&sil902x_i2c_driver);
	}

	return ret;
}

static void __exit hdmi_sil902x_exit(void)
{
    
	i2c_del_driver(&sil902x_i2c_driver);
	printk(KERN_EMERG "Sil902x hdmi_i2c_exit\n");
}

module_init(hdmi_sil902x_init);
module_exit(hdmi_sil902x_exit);

MODULE_AUTHOR("mian.tang@nufront.com");
MODULE_DESCRIPTION("SIL9022 HDMI driver");
MODULE_LICENSE("GPL");
