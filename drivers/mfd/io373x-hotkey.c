/**
 * io373x-hotkey.c - TWL4030 hot key Input Driver
 *
*
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mfd/io373x.h>

struct io373x_hotkey {
    struct input_dev *pwr;
    unsigned long subdev_id;
    struct notifier_block nb;
    struct mutex lock;
};


static int io373x_hotkey_notify(struct notifier_block *nb, unsigned long val, void *data)
{
    struct io373x_hotkey *hotkey = container_of(nb, struct io373x_hotkey, nb);   
    unsigned int scancode = 0;
   
    if (val == SUBDEV_ID_KBD) {
        struct io373x_subdev_data *subdev_data = data;

	scancode = subdev_data->buf[0];
/*
        for (i = 0; i < subdev_data->count; i++) {
		scancode += (subdev_data->buf[i] << (i*8));
	}
*/

	switch(scancode)
	{
		case IO373X_KEY_BTN_MOUSE:
			printk(KERN_EMERG "touch board mouse...\n");
//			input_report_key(hotkey->pwr, KEY_FN_ESC, 1);
//			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, BTN_MOUSE, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_SLEEP_S3:
			printk(KERN_EMERG "key sleep s3...\n");
			input_report_key(hotkey->pwr, KEY_SLEEP, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_WIFI:
			printk(KERN_EMERG "key wifi...\n");
			input_report_key(hotkey->pwr, KEY_WLAN, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_CRTLCD_SWITCH:
			printk(KERN_EMERG "key CRTLCD switch...\n");
			input_report_key(hotkey->pwr, KEY_SWITCHVIDEOMODE, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_BRIGHTNESSDOWN:
			printk(KERN_EMERG "key brightness down...\n");
			input_report_key(hotkey->pwr, KEY_BRIGHTNESSDOWN, 1);
			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, KEY_BRIGHTNESSDOWN, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_BRIGHTNESSUP:
			printk(KERN_EMERG "key brightness up...\n");
			input_report_key(hotkey->pwr, KEY_BRIGHTNESSUP, 1);
			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, KEY_BRIGHTNESSUP, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_LID_SWITCH:
			printk(KERN_EMERG "key LID switch...\n");
			input_report_key(hotkey->pwr, SW_LID, 1);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_FN_F10:
			printk(KERN_EMERG "fn key f10...\n");
			input_report_key(hotkey->pwr, KEY_FN_F10, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_BLUETOOTH:
			printk(KERN_EMERG "key bluetooth...\n");
			input_report_key(hotkey->pwr, KEY_BLUETOOTH, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_FN_F12:
			printk(KERN_EMERG "fn key f12...\n");
			input_report_key(hotkey->pwr, KEY_FN_F12, 0);
			input_sync(hotkey->pwr);
			break;
/*
		case IO373X_KEY_TOUCHPAD:
			printk(KERN_EMERG "key touchpad...\n");
			input_report_key(hotkey->pwr, BTN_TOUCH, 1);
			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, BTN_TOUCH, 0);
			input_sync(hotkey->pwr);
			break;

*/
/*
		if(subdev_data->buf[i] == IO373X_KEY_BACKLIGHT) {
			printk("key back light...\n");
			input_report_key(hotkey->pwr, , 1);
			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, , 0);
			input_sync(hotkey->pwr);
		}
*/
/*
		if(subdev_data->buf[i] == IO373X_KEY_GSENSOR) {
			printk("key gsensor...\n");
			input_report_key(hotkey->pwr, , 1);
			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, , 0);
			input_sync(hotkey->pwr);
		}

		if(subdev_data->buf[i] == IO373X_KEY_3G) {
			printk("key 3G...\n");
			input_report_key(hotkey->pwr, , 1);
			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, , 0);
			input_sync(hotkey->pwr);
		}
*/
/*
		case IO373X_KEY_CAMERA:
			printk(KERN_EMERG "key camera...\n");
			input_report_key(hotkey->pwr, KEY_CAMERA, 1);
			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, KEY_CAMERA, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_WWW:
			printk(KERN_EMERG "key WWW...\n");
			input_report_key(hotkey->pwr, KEY_WWW, 1);
			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, KEY_WWW, 0);
			input_sync(hotkey->pwr);
			break;

		case IO373X_KEY_EMAIL:
			printk(KERN_EMERG "key EMAIL...\n");
			input_report_key(hotkey->pwr, KEY_EMAIL, 1);
			input_sync(hotkey->pwr);
			input_report_key(hotkey->pwr, KEY_EMAIL, 0);
			input_sync(hotkey->pwr);
			break;

*/		default:
			//printk("0x%0x not hot key!\n", subdev_data->buf[i]);
			break;
		}
    }
    return 0;
}

static int __devinit io373x_hotkey_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	struct io373x_hotkey *  hotkey;
    	struct io373x *io373x = dev_get_drvdata(pdev->dev.parent);
	int	err;
	if (!(hotkey = kzalloc(sizeof(struct io373x_hotkey), GFP_KERNEL)))
        	return -ENOMEM;

    	hotkey->subdev_id = SUBDEV_ID_KBD;
    	hotkey->nb.notifier_call = io373x_hotkey_notify;

	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate hot key\n");
		goto input_alloc_fail;
	}

	pwr->evbit[0] = BIT_MASK(EV_KEY);

	//touch board mouse
	pwr->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_MOUSE);

	//touch pad 
	pwr->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	//CRTLCD switch
	pwr->keybit[BIT_WORD(KEY_SWITCHVIDEOMODE)] = BIT_MASK(KEY_SWITCHVIDEOMODE);

	pwr->keybit[BIT_WORD(KEY_BRIGHTNESSDOWN)] = BIT_MASK(KEY_BRIGHTNESSDOWN);
	pwr->keybit[BIT_WORD(KEY_BRIGHTNESSUP)] = BIT_MASK(KEY_BRIGHTNESSUP);

	//pwr->keybit[BIT_WORD(KEY_BACK)] = BIT_MASK(KEY_BACK); ?? back light key

	pwr->keybit[BIT_WORD(KEY_SLEEP)] = BIT_MASK(KEY_SLEEP);
	pwr->keybit[BIT_WORD(KEY_WLAN)] = BIT_MASK(KEY_WLAN); //wifi??
	pwr->keybit[BIT_WORD(KEY_BLUETOOTH)] = BIT_MASK(KEY_BLUETOOTH);
	//pwr->keybit[BIT_WORD()] = BIT_MASK(); gsensor ??
	//pwr->keybit[BIT_WORD()] = BIT_MASK(); 3G ??

	pwr->keybit[BIT_WORD(KEY_CAMERA)] = BIT_MASK(KEY_CAMERA);
	pwr->keybit[BIT_WORD(KEY_WWW)] = BIT_MASK(KEY_WWW);
	pwr->keybit[BIT_WORD(KEY_EMAIL)] = BIT_MASK(KEY_EMAIL);
	pwr->keybit[BIT_WORD(SW_LID)] = BIT_MASK(SW_LID);


	pwr->name = "io373x_hotkey";
	pwr->phys = "io373x_hotkey/input0";
	pwr->dev.parent = &pdev->dev;

	err = input_register_device(pwr);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power button: %d\n", err);
		goto input_register_fail;
	}
	
	hotkey->pwr = pwr;
	platform_set_drvdata(pdev, hotkey);

    	if ((err = io373x_register_notifier(io373x, &hotkey->nb))) 
		goto io373x_register_fail;	
	

	return 0;
io373x_register_fail:
	input_unregister_device(pwr);
input_register_fail:
	input_free_device(pwr);
input_alloc_fail:
	kfree(hotkey);
	return err;
}

static int __devexit io373x_hotkey_remove(struct platform_device *pdev)
{
	struct io373x_hotkey *  hotkey = platform_get_drvdata(pdev);
	struct input_dev *pwr = hotkey->pwr;
	input_unregister_device(pwr);
    	kfree(hotkey);
	return 0;
}

struct platform_driver io373x_hotkey_driver = {
	.probe		= io373x_hotkey_probe,
	.remove		= __devexit_p(io373x_hotkey_remove),
	.driver		= {
		.name	= "io373x-hotkey",
		.owner	= THIS_MODULE,
	},
};

static int __init io373x_hotkey_init(void)
{
	return platform_driver_register(&io373x_hotkey_driver);
}
module_init(io373x_hotkey_init);

static void __exit io373x_hotkey_exit(void)
{
	platform_driver_unregister(&io373x_hotkey_driver);
}
module_exit(io373x_hotkey_exit);

MODULE_ALIAS("platform:io373x_hotkey");
MODULE_DESCRIPTION("Io373x Hot Key");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhijie.hao <zhijie.hao@nufront.com>");

