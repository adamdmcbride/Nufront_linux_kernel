/**
 * io373x-pwrbutton.c - TWL4030 Power Button Input Driver
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

struct io373x_pwrbutton {
    struct input_dev *pwr;
    unsigned long subdev_id;
    struct notifier_block nb;
    struct mutex lock;
};


static int io373x_pwrbutton_notify(struct notifier_block *nb, unsigned long val, void *data)
{
    struct io373x_pwrbutton *pwrbutton = container_of(nb, struct io373x_pwrbutton, nb);   
   
    if (val == SUBDEV_ID_KBD) {
        int i;
        struct io373x_subdev_data *subdev_data = data;

        for (i = 0; i < subdev_data->count; i++) {
		if(subdev_data->buf[i] == IO373X_KEY_POWER_DOWN) {
			input_report_key(pwrbutton->pwr, KEY_POWER, 1);
			input_sync(pwrbutton->pwr);
		} else if(subdev_data->buf[i] == IO373X_KEY_POWER_UP) {
			input_report_key(pwrbutton->pwr, KEY_POWER, 0);
			input_sync(pwrbutton->pwr);
		}
	}
    }
    return 0;
}

static int __devinit io373x_pwrbutton_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	struct io373x_pwrbutton *  pwrbutton;
	int err;
    	struct io373x *io373x = dev_get_drvdata(pdev->dev.parent);
	if (!(pwrbutton = kzalloc(sizeof(struct io373x_pwrbutton), GFP_KERNEL)))
        	return -ENOMEM;

    	pwrbutton->subdev_id = SUBDEV_ID_KBD;
    	pwrbutton->nb.notifier_call = io373x_pwrbutton_notify;

	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		goto input_alloc_fail;
	}

	pwr->evbit[0] = BIT_MASK(EV_KEY);
	pwr->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	pwr->name = "io373x_pwrbutton";
	pwr->phys = "io373x_pwrbutton/input0";
	pwr->dev.parent = &pdev->dev;

	err = input_register_device(pwr);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power button: %d\n", err);
		goto input_register_fail;
	}
	
	pwrbutton->pwr = pwr;
	platform_set_drvdata(pdev, pwrbutton);

    	if ((err = io373x_register_notifier(io373x, &pwrbutton->nb))) 
		goto io373x_register_fail;	
	

	return 0;
io373x_register_fail:
	input_unregister_device(pwr);
input_register_fail:
	input_free_device(pwr);
input_alloc_fail:
	kfree(pwrbutton);
	return err;
}

static int __devexit io373x_pwrbutton_remove(struct platform_device *pdev)
{
	struct io373x_pwrbutton *  pwrbutton = platform_get_drvdata(pdev);
	struct input_dev *pwr = pwrbutton->pwr;
	input_unregister_device(pwr);
    	kfree(pwrbutton);
	return 0;
}

static int io373x_pwrbutton_resume(struct platform_device *dev)
{
	struct io373x_pwrbutton *  pwrbutton = platform_get_drvdata(dev);
	struct input_dev *pwr = pwrbutton->pwr;
	input_report_key(pwrbutton->pwr, KEY_POWER, 1);
	input_sync(pwrbutton->pwr);
	input_report_key(pwrbutton->pwr, KEY_POWER, 0);
	input_sync(pwrbutton->pwr);
	return 0;
}

struct platform_driver io373x_pwrbutton_driver = {
	.probe		= io373x_pwrbutton_probe,
	.remove		= __devexit_p(io373x_pwrbutton_remove),
	/*.resume		= io373x_pwrbutton_resume,*/
	.driver		= {
		.name	= "io373x-pwrbutton",
		.owner	= THIS_MODULE,
	},
};

static int __init io373x_pwrbutton_init(void)
{
	return platform_driver_register(&io373x_pwrbutton_driver);
}
module_init(io373x_pwrbutton_init);

static void __exit io373x_pwrbutton_exit(void)
{
	platform_driver_unregister(&io373x_pwrbutton_driver);
}
module_exit(io373x_pwrbutton_exit);

MODULE_ALIAS("platform:io373x_pwrbutton");
MODULE_DESCRIPTION("Io373x Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhijie.hao <zhijie.hao@nufront.com>");

