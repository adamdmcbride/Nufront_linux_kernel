#ifndef _IO373X_PW_H_
#define _IO373X_PW_H_

#define STS_ON		1
#define STS_OFF		0
#define STS_ENABLE		1
#define STS_DISABLE 	0

#define STS_S3		3
#define STS_S5		5
/*CMD List*/
enum {
POWER_SEQ_S3=0,		/*To S3 State*/	
POWER_SEQ_S5,		/*To S5 State*/		
WIFI_ON,		/*Power On Wifi*/		
WIFI_OFF,		/*Power Off Wifi*/		
BT_ON,			/*Power On Bluetooth*/		
BT_OFF,			/*Power off Bluetooth*/		
G3_ON,			/*Power On 3G*/		
G3_OFF,			/*Power off 3G*/			
TOUCH_ENABLE,		/*Enable Touch panel*/			
TOUCH_DISABLE,		/*Disable Touch Panel*/		
LCD_ON,			/*Power On LCD Screen*/			
LCD_OFF,		/*Power off LCD Screen*/			
CAM_ON,			/*Power On Camera*/			
CAM_OFF,		/*Power off Camera*/			
SND_ON,			/*Power On Sound*/			
SND_OFF,		/*Power Off Sound*/			
BRT_UP,			/*Bright Up*/			
BRT_DOWN,		/*Bright Down*/			
SND_UP,			/*Sound Up*/			
SND_DOWN,		/*Sound Down*/			
BRT_LEV,		/*Set Bright Level*/			
FAN_LEV,		/*Set Fan Level*/			
GPS_ON,			/*Set Fan Level*/			
GPS_OFF,		/*Set Fan Level*/			
POWER_STS,		/*Get Power Status*/			
WIFI_STS,		/*Get Wifi Status*/			
BT_STS,			/*Get Bluetooth Status*/			
G3_STS,			/*Get 3G Status*/			
TOUCH_STS,		/*Get Touch Status*/			
LCD_STS,		/*Get LCD Status*/			
CAM_STS,		/*Get Camera Status*/			
SND_STS,		/*Get Sound Status*/			
BRT_RLEV,		/*Get Bright Level*/			
BRT_RMAX,		/*Get Bright Level Max*/			
FAN_RLEV,		/*Get Fan Level*/			
FAN_RMAX,		/*Get Fan Level Max*/			
GPS_STS,		/*Get Fan Level Max*/			
CMD_MAX,		/*Reserve.*/			
};

extern void io373x_poweroff(void);
extern void io373x_suspend(void);
#endif
