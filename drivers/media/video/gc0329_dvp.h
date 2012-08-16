/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */

#define GC0329_ID  0xc0 

#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/*
 * Our nominal (default) frame rate.
 */



#define GC0329_FRAME_RATE 7//15

/* Registers */
#define GC0329_I2C_ADDR 0x62
#define COM7_FMT_VGA 0x00
#define REG_COM7	0xfe	/* Reset related */
#define COM7_RESET	  0x80	  /* Register reset */
#define REG_PID		0x00	/* Product ID */
#define REG_BRIGHT	0x13	/* Brightness */

/* Information we maintain about a known sensor.*/


struct gc0329_format_struct;	/* coming later */
struct gc0329_info {
	struct gc0329_format_struct *fmt;	/* Current format */
	unsigned char sat;	/* Saturation value */
	int hue;		/* Hue value */
};

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};


/*The default register settings, as obtained from Micron.*/

static struct regval_list gc0329_init_regs[] = {

	{ 0xfe, 0x80 },
	{ 0xfc, 0x12 }, //[4]Clock_en [2] A25_en [1]D18_en [0]Apwdn
	{ 0xfc, 0x12 },
	{ 0xfe, 0x00 },
	{ 0xf0, 0x00 }, //vsync_en
	{ 0xf1, 0x00 }, //data_en

	{ 0x73, 0x90 }, //98//R channle gain
	{ 0x74, 0x80 }, //G1 channle gain
	{ 0x75, 0x80 }, //G2 channle gain
	{ 0x76, 0x94 }, //88//B channle gain

	{ 0x42, 0x00 },
	{ 0x77, 0x57 },
	{ 0x78, 0x4d },
	{ 0x79, 0x45 },
	{ 0x42, 0xfc },

	////////////////////analog////////////////////

	{ 0xfc, 0x16 }, //

	{ 0x0a, 0x04 }, //row_start_low
	{ 0x0c, 0x04 }, //col_start_low
	{ 0x17, 0x14 }, //cisctl_mode1//[7]hsync_always ,[6] NA,[5:4] CFA sequence [3:2]NA,[1]upside_down,[0] mirror
	{ 0x19, 0x05 }, //cisctl_mode3
	{ 0x1b, 0x24 }, //04//44//rsh_width
	{ 0x1c, 0x04 }, // 1d//Tsp_width
	{ 0x1e, 0x00 }, //Analog_mode1//[7:6]rsv1,rsv0[5:3] Column bias(coln_r)[1] clk_delay_en
	{ 0x1f, 0xc0 }, //Analog_mode2//[7:6] comv_r
	{ 0x20, 0x00 }, //Analog_mode3//[6:4] cap_low_r for MPW [3:2] da18_r [1] rowclk_mode [0]adclk_mode
	{ 0x21, 0x48 }, //Hrst_rsg//[7] hrst[6:4] da_rsg[3]txhigh_en
	{ 0x23, 0x22 }, //ADC_r//[6:5]opa_r [1:0]sRef
	{ 0x24, 0x16 }, //PAD_drv//[7:6]NA,[5:4]sync_drv [3:2]data_drv [1:0]pclk_drv

	//==================================================
	////////////////////blk////////////////////
	{ 0x26, 0xf7 },
	{ 0x32, 0x04 },
	{ 0x33, 0x20 },
	{ 0x34, 0x20 },
	{ 0x35, 0x20 },
	{ 0x36, 0x20 },

	////////////////////ISP BLOCK ENABLE////////////////////
	{ 0x40, 0xff }, //fe //ff
	{ 0x41, 0x00 },//20  victor
	{ 0x42, 0xfe }, //
	{ 0x46, 0x02 }, //sync mode
	{ 0x4b, 0xcb },
	{ 0x4d, 0x01 }, //[1]In_buf
	{ 0x4f, 0x01 },
	{ 0x70, 0x48 }, //global gain 0x40, 0x1X

	//{ 0xb0, 0x00 },
	//{ 0xbc, 0x00 },
	//{ 0xbd, 0x00 },
	//{ 0xbe, 0x00 },

	////////////////////DNDD////////////////////
	{ 0x80, 0xe7 }, //87 //[7]auto_en [6]one_pixel [5]two_pixel
	{ 0x82, 0x55 }, // DN_inc
	{ 0x87, 0x4a }, //

	////////////////////ASDE////////////////////
	{ 0xfe, 0x01 },
	{ 0x18, 0x22 }, //[7:4]AWB LUMA X, 0x[3:0]ASDE LUMA X
	{ 0xfe, 0x00 },
	{ 0x9c, 0x0a }, //ASDE dn b slope
	{ 0xa4, 0x50 }, //40 //90// Auto Sa slope
	{ 0xa5, 0x21 }, // [7:4]Saturation limit x10
	{ 0xa7, 0x35 }, //low luma value th
	{ 0xdd, 0x54 }, //44//edge dec sat enable & slopes
	{ 0x95, 0x35 }, //Edge effect

	////////////////////RGB gamma////////////////////
	//s_gamma
	{ 0xbf, 0x08 },
	{ 0xc0, 0x13 },
	{ 0xc1, 0x1f },
	{ 0xc2, 0x31 },
	{ 0xc3, 0x48 },
	{ 0xc4, 0x5f },
	{ 0xc5, 0x71 },
	{ 0xc6, 0x90 },
	{ 0xc7, 0xa9 },
	{ 0xc8, 0xbc },
	{ 0xc9, 0xca },
	{ 0xca, 0xd7 },
	{ 0xcb, 0xe0 },
	{ 0xcc, 0xe8 },
	{ 0xcd, 0xf4 },
	{ 0xce, 0xfb },
	{ 0xcf, 0xff },

	//////////////////CC///////////////////
	/*{ 0xfe, 0x00 },
	{ 0xb3, 0x3d },
	{ 0xb4, 0xfd },
	{ 0xb5, 0x02 },
	{ 0xb6, 0xfa },
	{ 0xb7, 0x40 },
	{ 0xb8, 0xf0 },*/

      { 0xfe, 0x00 },
      { 0xb3, 0x44 },
      { 0xb4, 0xfd },
      { 0xb5, 0x02 },
      { 0xb6, 0xfa },
      { 0xb7, 0x48 },
      { 0xb8, 0xF0 },

	//skin
	/*{ 0xfe, 0x00 },
	{ 0xb3, 0x3c },
      { 0xb4, 0xFF },
	{ 0xb5, 0x03 },
	{ 0xb6, 0x01 },
	{ 0xb7, 0x3f },
	{ 0xb8, 0xF3 },*/

	////default CC
	//{ 0xfe, 0x00 },
	//{ 0xb3, 0x45 },
	//{ 0xb4, 0x00 },
	//{ 0xb5, 0x00 },
	//{ 0xb6, 0x00 },
	//{ 0xb7, 0x45 },
	//{ 0xb8, 0xF0 },

	// crop
	{ 0x50, 0x01 },
	{ 0x19, 0x05 },
	{ 0x20, 0x01 },
	{ 0x22, 0xba },
	{ 0x21, 0x48 },

	////////////////////YCP////////////////////
	{ 0xfe, 0x00 },
	{ 0xd1, 0x34 }, //38//saturation Cb  victor
	{ 0xd2, 0x34 }, //38//saturation Cr

	//d6 e8 d7  18   d8  18  victor  skin
	////////////////////AEC////////////////////
	{ 0xfe, 0x01 },
	{ 0x10, 0x40 },
	{ 0x11, 0x21 }, //a1
	{ 0x12, 0x07 }, //17 //27 center weight un
	{ 0x13, 0x50 }, //Y target
	{ 0x17, 0x88 }, //AEC ignore mode
	{ 0x21, 0xb0 },
	{ 0x22, 0x48 },
	{ 0x3c, 0x95 },
	{ 0x3d, 0x50 },
	{ 0x3e, 0x48 },

	////////////////////AWB////////////////////
	{ 0xfe, 0x01 },
	{ 0x06, 0x08},
	{ 0x07, 0x06 },
	{ 0x08, 0xa6 },
	{ 0x09, 0xee },

	{ 0x18, 0x00 }, //RGB high
	{ 0x50, 0xfc }, //RGB high
	{ 0x51, 0x28 }, //Y2C diff
	{ 0x52, 0x10 }, //20
	{ 0x53, 0x08 }, //20
	{ 0x54, 0x12 }, //30//C inter
	{ 0x55, 0x10 },
	{ 0x56, 0x0b },//10
	//{ 0x57, 0x40 },
	{ 0x58, 0x80 }, //number limit, 0xX4
	{ 0x59, 0x08 }, //AWB adjust temp curve  0c
	{ 0x5a, 0x02 }, //25//[3:0]light gain range x10
	{ 0x5b, 0x63 }, //62
	{ 0x5c, 0x34 }, //37 //show and mode [2]big C mode [1]dark mode [0] block move mode  34
	{ 0x5d, 0x43 }, //52//AWB margin  73
	{ 0x5e, 0x29 }, //19//temp curve_enable
	{ 0x5f, 0x40 }, //5K gain
	{ 0x60, 0x40 }, //5K gain
	{ 0x61, 0xc8 }, //sinT
	{ 0x62, 0xa0 }, //cosT
	{ 0x63, 0x40 }, //30//AWB X1 cut
	{ 0x64, 0x38 }, //60//AWB X2 cut
	{ 0x65, 0x98 }, //a0//AWB Y1 cut
	{ 0x66, 0xfa }, //ea//AWB Y2 cut
	{ 0x67, 0x80 }, //AWB R gain limit
	{ 0x68, 0x60 }, //58 //AWB G gain Limit
	{ 0x69, 0x90 }, //7d //AWB B gain limit
	{ 0x6a, 0x40 },
	{ 0x6b, 0x39 },
	{ 0x6c, 0x28 },
	{ 0x6d, 0x28 },
	{ 0x6e, 0x41 }, //41 //outdoor gain limit enable [7]use exp or luma value to adjust outdoor
	{ 0x70, 0x10 },
	{ 0x71, 0x00 }, //when outdoor , add high luma gray pixel weight
	{ 0x72, 0x08 },
	{ 0x73, 0x40 }, //95// when exp < th, outdoor mode open
	//{ 0x74, 0x70 },
	//{ 0x75, 0x40 },
	//{ 0x76, 0x30 },
	//{ 0x77, 0x48 },
	//{ 0x7a, 0x50 },
	//{ 0x7b, 0x20 }, // Yellow R2B, 0xB2G limit, >it, as Yellow
	{ 0x80, 0x70 }, //R gain high limit
	{ 0x81, 0x58 }, //G gain high limit
	{ 0x82, 0x42 }, //B gain high limit
	{ 0x83, 0x40 }, //R gain low limit
	{ 0x84, 0x40 }, //G gain low limit
	{ 0x85, 0x40 }, //B gain low limit

	////////////////////CC-AWB////////////////////
	{ 0xd0, 0x00 },
	{ 0xd2, 0x2c }, //D Xn
	{ 0xd3, 0x80 },

	///////////////////ABS///////////////////////
	{ 0x9c, 0x02 },
	{ 0x9d, 0x10 }, //08//20

	///////////////////LSC //////////////////////
	//// for xuye062d lens setting
	{ 0xfe, 0x01 },
	{ 0xa0, 0x00 },
	{ 0xa1, 0x3c },
	{ 0xa2, 0x50 },
	{ 0xa3, 0x00 },
	{ 0xa8, 0x0f },
	{ 0xa9, 0x08 },
	{ 0xaa, 0x00 },
	{ 0xab, 0x04 },
	{ 0xac, 0x00 },
	{ 0xad, 0x07 },
	{ 0xae, 0x0e },
	{ 0xaf, 0x00 },
	{ 0xb0, 0x00 },
	{ 0xb1, 0x09 },
	{ 0xb2, 0x00 },
	{ 0xb3, 0x00 },
	{ 0xb4, 0x31 },
	{ 0xb5, 0x19 },
	{ 0xb6, 0x24 },
	{ 0xba, 0x3a },
	{ 0xbb, 0x24 },
	{ 0xbc, 0x2a },
	{ 0xc0, 0x17 },
	{ 0xc1, 0x13 },
	{ 0xc2, 0x17 },
	{ 0xc6, 0x21 },
	{ 0xc7, 0x1c },
	{ 0xc8, 0x1c },
	{ 0xb7, 0x00 },
	{ 0xb8, 0x00 },
	{ 0xb9, 0x00 },
	{ 0xbd, 0x00 },
	{ 0xbe, 0x00 },
	{ 0xbf, 0x00 },
	{ 0xc3, 0x00 },
	{ 0xc4, 0x00 },
	{ 0xc5, 0x00 },
	{ 0xc9, 0x00 },
	{ 0xca, 0x00 },
	{ 0xcb, 0x00 },
	{ 0xa4, 0x00 },
	{ 0xa5, 0x00 },
	{ 0xa6, 0x00 },
	{ 0xa7, 0x00 },

	{ 0xfe, 0x00 },
	////////////////////asde ///////////////////
	{ 0xa0, 0xaf }, //[7:4]bright_slope for special point
	{ 0xa2, 0xff }, //[for special point

	{ 0x44, 0xa2 }, //23//output format
	{ 0xd5, 0x00 },
/*
	{ 0x05, 0x01 },//addd victor  2011-12 -22
	{ 0x06, 0xfa },//HB
	{ 0x07, 0x02 },
	{ 0x08, 0xc8 },//vb

	{ 0xfe, 0x01 },

	{ 0x29, 0x00 },
	{ 0x2a, 0x64 },//step
*/
	{ 0x05, 0x00 },//addd victor  2011-12 -22
	{ 0x06, 0x38 },//HB
	{ 0x07, 0x00 },
	{ 0x08, 0x25 },//vb

	{ 0xfe, 0x01 },

	{ 0x29, 0x00 },
	{ 0x2a, 0x50 },//step

	{ 0x13, 0x48 },

	{ 0x2b, 0x04 },
	{ 0x2c, 0xb0 },//LEVEL 20  fps
	{ 0x2d, 0x04 },
	{ 0x2e, 0xb0 },
	{ 0x2f, 0x04 },
	{ 0x30, 0xb0 },
	{ 0x31, 0x04 },
	{ 0x32, 0xb0 },

	{ 0xfe, 0x00 },
        { 0x17, 0x15 },
        { 0x7a, 0x80 },
        { 0x7b, 0x80 },
        { 0x7c, 0x82 },
	
	{ 0xfc, 0x17 },	// standby
 
	{ 0xff, 0xff }, /* END MARKER */
};


static struct regval_list gc0329_fmt_yuv422_vga[] = {

    { 0xfe, 0x00 },
    { 0x4b, 0xcb },
    { 0x50, 0x01 },
    { 0x51, 0x00 },
    { 0x52, 0x00 },
    { 0x53, 0x00 },
    { 0x54, 0x00 },
    { 0x55, 0x01 },
    { 0x56, 0xe0 },  
    { 0x57, 0x02 },
    { 0x58, 0x80 },
    
    { 0x59, 0x11 },  
    { 0x5a, 0x0e },
    { 0x5b, 0x00 },
    { 0x5c, 0x00 },
    { 0x5d, 0x00 },
    { 0x5e, 0x00 },
    { 0x5f, 0x00 },
    { 0x60, 0x00 },
    { 0x61, 0x00 },
    { 0x62, 0x00 },
    { 0x4c, 0x00 },
    
	{ 0xff, 0xff },

};

static struct regval_list gc0329_fmt_yuv422_qvga[] = {

    { 0xfe, 0x00 },
    { 0x4b, 0xca },
    { 0x50, 0x01 },
    { 0x51, 0x00 },
    { 0x52, 0x00 },
    { 0x53, 0x00 },
    { 0x54, 0x00 },
    { 0x55, 0x00 },
    { 0x56, 0xf0 },  
    { 0x57, 0x01 },
    { 0x58, 0x40 },
    
    { 0x59, 0x22 },  
    { 0x5a, 0x03 },
    { 0x5b, 0x00 },
    { 0x5c, 0x00 },
    { 0x5d, 0x00 },
    { 0x5e, 0x00 },
    { 0x5f, 0x00 },
    { 0x60, 0x00 },
    { 0x61, 0x00 },
    { 0x62, 0x00 },
    
	{ 0xff, 0xff },

};

static struct regval_list gc0329_fmt_yuv422_qcif[] = {

//    { 0xfe, 0x00 },
//    { 0x4b, 0xca },
//    { 0x50, 0x01 },
//    { 0x51, 0x00 },
//    { 0x52, 0x00 },
//    { 0x53, 0x00 },
//    { 0x54, 0x00 },
//    { 0x55, 0x00 },
//    { 0x56, 0x90 },  
//    { 0x57, 0x00 },
//    { 0x58, 0xb0 },
//    
//    { 0x59, 0xaa },  
//    { 0x5a, 0x03 },
//    { 0x5b, 0x04 },
//    { 0x5c, 0x08 },
//    { 0x5d, 0x00 },
//    { 0x5e, 0x00 },
//    { 0x5f, 0x04 },
//    { 0x60, 0x08 },
//    { 0x61, 0x00 },
//    { 0x62, 0x00 },
//    
	{ 0xff, 0xff },

};

