#ifndef	_BLUETOOTH_H
#define _BLUETOOH_H
#ifdef CONFIG_BT_NW53
#include <linux/types.h>
#include <linux/device.h>
#include <linux/serial_core.h>
#include <linux/wakelock.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>	/* event notifications */

struct nw53_bt_platform_data {

	unsigned int reset;
	unsigned int bt_wake;
	unsigned int int_gpio;
	unsigned int bt_enable;

	struct uart_port *uport;
	struct wake_lock wake_lock;
	struct hci_dev *bluesleep_hdev;
};
#endif

#endif
