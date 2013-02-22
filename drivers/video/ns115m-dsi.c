#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <mach/memory.h>
#include <mach/get_bootargs.h>
#include <mach/lcdc.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/clk.h>

#include <asm/mach-types.h>
#include <linux/completion.h>

#define TR0(fmt, args...) printk(KERN_CRIT "dsi_______________________: " fmt, ##args)

#define	DSI_NAME "nusmart_dsi"

#define MIPI_RESET   (8 + 4)
#define MIPI_SPI_CS  (8 + 2)
#define MIPI_SPI_CLK (8 + 5)
#define MIPI_SPI_SDI (8 + 3)
#define MIPI_SPI_SDO (8 + 22)
#define MIPI_SHUT (64 + 8 + 30)
#define LCD_RESET (32 + 8 + 15)

#define HX_WR_REGISTER   (0x72)
#define HX_RD_REGISTER   (0x73)

static int set_reset_pin(int v);
static int set_gpio_out(int n, int v);

#define SET_RESET_PIN(v)    set_reset_pin((v))
#define SET_GPIO_OUT(n, v)  set_gpio_out((n), (v))

#define UDELAY(n) udelay(n)
#define MDELAY(n) mdelay(n)

#define LSCE_GPIO_PIN (MIPI_SPI_CS)
#define LSCK_GPIO_PIN (MIPI_SPI_CLK)
#define LSDA_GPIO_PIN (MIPI_SPI_SDI)
#define LSSHUT_GPIO_PIN (MIPI_SHUT)
#define LSDIO_GPIO_PIN (MIPI_SPI_SDO)

#define SET_HX_CS    SET_GPIO_OUT(LSCE_GPIO_PIN, 1)
#define CLR_HX_CS    SET_GPIO_OUT(LSCE_GPIO_PIN, 0)
#define SET_HX_SDI   SET_GPIO_OUT(LSDA_GPIO_PIN, 1)
#define CLR_HX_SDI   SET_GPIO_OUT(LSDA_GPIO_PIN, 0)
#define SET_HX_SCLK  SET_GPIO_OUT(LSCK_GPIO_PIN, 1)
#define CLR_HX_SCLK  SET_GPIO_OUT(LSCK_GPIO_PIN, 0)

#define CS1_L  CLR_HX_CS
#define CS1_H  SET_HX_CS
#define CLK1_L CLR_HX_SCLK
#define CLK1_H SET_HX_SCLK
#define SDI1_L CLR_HX_SDI
#define SDI1_H SET_HX_SDI
#define RESET1_H  SET_RESET_PIN(1)
#define RESET1_L  SET_RESET_PIN(0)

void SSD2825_0P(unsigned int address);
static struct clk *clk;

void SSD2825_1P(unsigned int address, unsigned int data0);
void SSD2825_2P(unsigned int address, unsigned int data0, unsigned int data1);
void SSD2825_3P(unsigned int address, unsigned int data0, unsigned int data1,
		unsigned int data2);
void SSD2825_15P(unsigned int address, unsigned int data0, unsigned int data1,
		 unsigned int data2, unsigned int data3, unsigned int data4,
		 unsigned int data5, unsigned int data6, unsigned int data7,
		 unsigned int data8, unsigned int data9, unsigned int data10,
		 unsigned int data11, unsigned int data12, unsigned int data13,
		 unsigned int data14);

void SSD2825_18P(unsigned int address, unsigned int data0, unsigned int data1,
		 unsigned int data2, unsigned int data3, unsigned int data4,
		 unsigned int data5, unsigned int data6, unsigned int data7,
		 unsigned int data8, unsigned int data9, unsigned int data10,
		 unsigned int data11, unsigned int data12, unsigned int data13,
		 unsigned int data14, unsigned int data15, unsigned int data16,
		 unsigned int data17);

void SSD2825_24P(unsigned int address, unsigned int data0, unsigned int data1,
		 unsigned int data2, unsigned int data3, unsigned int data4,
		 unsigned int data5, unsigned int data6, unsigned int data7,
		 unsigned int data8, unsigned int data9, unsigned int data10,
		 unsigned int data11, unsigned int data12, unsigned int data13,
		 unsigned int data14, unsigned int data15, unsigned int data16,
		 unsigned int data17, unsigned int data18, unsigned int data19,
		 unsigned int data20, unsigned int data21, unsigned int data22,
		 unsigned int data23);

void SSD2825_34P(unsigned int address, unsigned int data0, unsigned int data1,
		 unsigned int data2, unsigned int data3, unsigned int data4,
		 unsigned int data5, unsigned int data6, unsigned int data7,
		 unsigned int data8, unsigned int data9, unsigned int data10,
		 unsigned int data11, unsigned int data12, unsigned int data13,
		 unsigned int data14, unsigned int data15, unsigned int data16,
		 unsigned int data17, unsigned int data18, unsigned int data19,
		 unsigned int data20, unsigned int data21, unsigned int data22,
		 unsigned int data23, unsigned int data24, unsigned int data25,
		 unsigned int data26, unsigned int data27, unsigned int data28,
		 unsigned int data29, unsigned int data30, unsigned int data31,
		 unsigned int data32, unsigned int data33);

static int set_reset_pin(int v)
{
	gpio_direction_output(MIPI_RESET, v);

	return 0;
}

static int set_gpio_out(int n, int v)
{
	gpio_direction_output(n, v);

	return 0;
}

static int config_gpio(void)
{
	int ret;

	ret = gpio_request(LSCE_GPIO_PIN, "dsi HX_CS");
	if (ret != 0) {
		TR0("line %d: gpio %d request fail!\n", __LINE__,
		    LSCE_GPIO_PIN);
		return ret;
	}

	ret = gpio_request(LSDA_GPIO_PIN, "dsi HX_DSI");
	if (ret != 0) {
		TR0("line %d: gpio %d request fail!\n", __LINE__,
		    LSDA_GPIO_PIN);
		return ret;
	}

	ret = gpio_request(LSCK_GPIO_PIN, "dsi HX_SCLK");
	if (ret != 0) {
		TR0("line %d: gpio %d request fail!\n", __LINE__,
		    LSCK_GPIO_PIN);
		return ret;
	}

	ret = gpio_request(LSSHUT_GPIO_PIN, "dsi mipi_shut");
	if (ret != 0) {
		TR0("line %d: gpio %d request fail!\n", __LINE__,
		    LSSHUT_GPIO_PIN);
		return ret;
	}

	ret = gpio_request(LSDIO_GPIO_PIN, "dsi mipi_sdo");
	if (ret != 0) {
		TR0("line %d: gpio %d request fail!\n", __LINE__,
		    LSDIO_GPIO_PIN);
		return ret;
	}

	ret = gpio_request(MIPI_RESET, "dsi mipi_reset");
	if (ret != 0) {
		TR0("line %d: gpio %d request fail!\n", __LINE__, MIPI_RESET);
		return ret;
	}
	gpio_direction_input(LSDIO_GPIO_PIN);

	return 0;
}

static void WriteCommand(unsigned int comm)
{
	int i;
	CS1_L;
	CLK1_L;
	{
		SDI1_L;
		mdelay(1);
	}
	CLK1_H;
	mdelay(1);
	for (i = 0; i < 8; i++) {
		CLK1_L;
		//Select to use LSB or MSB?? here is use MSB
		if ((0x80 & (comm << i)) == 0)	//MSB
		{
			SDI1_L;
			mdelay(1);
		} else {
			SDI1_H;
			mdelay(1);
		}
		CLK1_H;
		mdelay(1);
	}
	CS1_H;
}

//data 8-bit write sub-function
static void WritePara(unsigned int data)
{
	int i;
	CS1_L;
	CLK1_L;
	{
		SDI1_H;
		mdelay(1);
	}
	CLK1_H;
	mdelay(1);
	for (i = 0; i < 8; i++) {
		CLK1_L;
		//Select to use LSB or MSB??  here is use MSB
		if ((0x80 & (data << i)) == 0)	//MSB
		{
			SDI1_L;
			mdelay(1);
		} else {
			SDI1_H;
			mdelay(1);
		}
		CLK1_H;
		mdelay(1);
	}
	CS1_H;
}

//  SSD2825 spi write register and data
void SSD2825_spi_reg_data_wr(int addr, int data1, int data2)
{
	WriteCommand(addr);
	WritePara(data2);
	WritePara(data1);
}

#if 1
//data 8-bit Read sub-function
static unsigned char ReadReg(void)
{
	int j;
	unsigned char value = 0;

	CS1_L;
	CLK1_H;
	mdelay(1);

	for (j = 0; j < 8; j++) {
		CLK1_L;		//CLR_HX_SCLK;
		UDELAY(20);
		CLK1_H;		//SET_HX_SCLK;
		value <<= 1;
		value |= gpio_get_value(LSDIO_GPIO_PIN);
		UDELAY(20);
	}
	CS1_H;

	return value;
}

//  SSD2825 spi read register
unsigned short SSD2825_spi_reg_data_rd(int addr)
{
	unsigned char val1 = 0;
	unsigned char val2 = 0;
	unsigned short tmp = 0;

	WriteCommand(addr);
	WriteCommand(0xFA);
	val1 = ReadReg();
	val2 = ReadReg();

	TR0("read1:0x%x", val1);
	TR0("read2:0x%x", val2);

	tmp |= val2;
	tmp = (tmp << 8);
	tmp |= val1;

	return tmp;
}
#endif
//SSD2825 spi write register and no data
void spi_reg_wr(int data1)
{
	WriteCommand(data1);
}

void Initial_Code_HX8394A_BOECD_LCD_20111220(void)
{
	mdelay(2);
	SSD2825_0P(0x11);
	mdelay(2);

	SSD2825_3P(0xB9, 0xFF, 0x83, 0x94);
	mdelay(2);

	WriteCommand(0xB7);
	WritePara(0x10);
	WritePara(0x03);
	mdelay(1);

	WriteCommand(0xBC);
	WritePara(0x02);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0xBD);
	WritePara(0x00);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0xBE);
	WritePara(0x02);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0xBF);
	WritePara(0xBA);
	WritePara(0x13);	//13
	mdelay(1);
	SSD2825_2P(0xB2, 0x08, 0xCB);	//c8
	mdelay(1);

	SSD2825_15P(0xB1, 0x7C, 0x00, 0x34, 0x09, 0x01, 0x11, 0x11, 0x36, 0x3E, 0x25, 0x25, 0x57, 0x12, 0x01, 0xE6);	//BU YINGXIANG
	mdelay(1);

	SSD2825_18P(0xB4, 0x00, 0x00, 0x00, 0x05, 0x06, 0x06, 0x60, 0x04, 0x06, 0x5B, 0x33, 0x37, 0x04, 0x64, 0x6C, 0x08, 0x05, 0x10);
	TR0("Initial Code 0xB4 download end.....!!\n");
	mdelay(1);

	SSD2825_24P(0xD5, 0x4C, 0x01, 0x00, 0x01, 0xCD, 0x23, 0xEF, 0x45, 0x67, 0x89, 0xAB, 0x11, 0x00, 0xDC, 0x10, 0xFE, 0x32, 0xBA, 0x98, 0x76, 0x54, 0x00, 0x11, 0x40);
	TR0("Initial Code 0xD5 download end.....!!\n");
	mdelay(1);

	SSD2825_34P(0xE0, 0x01, 0x24, 0x28, 0x3F, 0x3E, 0x3E, 0x36, 0x56, 0x06, 0x0D, 0x0F, 0x12, 0x15, 0x13, 0x14, 0x12, 0x1C, 0x01, 0x24, 0x28, 0x3F, 0x3E, 0x3E, 0x36, 0x56, 0x06, 0x0D, 0x0F, 0x12, 0x15, 0x13, 0x14, 0x12, 0x1C);

	mdelay(1);

	SSD2825_1P(0xB3, 0x09);
	mdelay(1);
	SSD2825_1P(0xCC, 0x01);
	mdelay(1);
	SSD2825_1P(0x3A, 0x70);
	mdelay(1);

	SSD2825_1P(0x36, 0x0A);
	mdelay(1);

	WriteCommand(0xB7);
	WritePara(0x10);
	WritePara(0x03);
	mdelay(1);

	WriteCommand(0xBC);
	WritePara(0x02);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0xBD);
	WritePara(0x00);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0xBE);
	WritePara(0x02);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0xBF);
	WritePara(0xCC);
	WritePara(0x01);
	mdelay(1);

	WriteCommand(0xBF);
	WritePara(0xB6);
	WritePara(0x3A);

	mdelay(1);

	WriteCommand(0xB7);
	WritePara(0x50);
	WritePara(0x03);
	mdelay(1);

	WriteCommand(0xBC);
	WritePara(0x01);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0xBD);
	WritePara(0x00);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0xBE);
	WritePara(0x02);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0x29);
	mdelay(5);
	// TR0("Initial Code 0x29 download end.....!!\n");
}

// TFT-LCD Initial code write
static void tft_power_on(void)
{
	config_gpio();

	CS1_L;
	SDI1_H;
	CLK1_H;

	RESET1_H;
	SET_GPIO_OUT(LSSHUT_GPIO_PIN, 1);
	MDELAY(20);
	SET_GPIO_OUT(LSSHUT_GPIO_PIN, 0);

	mdelay(20);
	RESET1_L;
	mdelay(20);
	RESET1_H;
	mdelay(20);

	TR0("SSD2825C Initial download start.....!!\n");

	CS1_L;
	mdelay(1);

	WriteCommand(0xB9);
	WritePara(0x00);
	WritePara(0x00);
	mdelay(1);

	WriteCommand(0xBA);
	WritePara(0x31);
	WritePara(0xc0);
	mdelay(2);

	WriteCommand(0xB9);
	WritePara(0x01);
	WritePara(0x00);
	mdelay(2);

#if 1
	//NULL, 60, 720, 1280, 16116, 0x3b, 0x20, 0x0b, 0x0b, 9, 2,
	WriteCommand(0xB1);
	WritePara(0x09);
	WritePara(0x02);
	mdelay(1);

	WriteCommand(0xB2);
	WritePara(0x3b);
	WritePara(0x0b);
	mdelay(1);

	WriteCommand(0xB3);
	WritePara(0x20);
	WritePara(0x0b);
	mdelay(1);

	WriteCommand(0xB4);
	WritePara(0xD0);
	WritePara(0x02);
	mdelay(1);

	WriteCommand(0xB5);
	WritePara(0x0D);
	WritePara(0x05);
	mdelay(1);

	WriteCommand(0xB6);
	WritePara(0x0B);
	WritePara(0x00);
	mdelay(1);
#endif

	WriteCommand(0xEB);
	WritePara(0x00);
	WritePara(0xB0);
	mdelay(1);

	TR0("RGB timing config end.....!!\n");
	WriteCommand(0xDE);
	WritePara(0x03);
	WritePara(0x00);
	TR0("read 0xb2:0x%04x", SSD2825_spi_reg_data_rd(0xB1));
	mdelay(2);

	CS1_L;
	mdelay(2);

	TR0("LCD Initial download start.....!!\n");

	Initial_Code_HX8394A_BOECD_LCD_20111220();

	TR0("Initial download end.....!!\n");
	WriteCommand(0xB7);
	WritePara(0x0B);
	WritePara(0x07);

	CS1_H;

	mdelay(2);
}

//SSD2825_1_to_16_parameter write sub-function

void SSD2825_0P(unsigned int address)
{
	SSD2825_spi_reg_data_wr(0xb7, 0x03, 0x50);	//seting Generic 02 packet
	SSD2825_spi_reg_data_wr(0xbc, 0x00, 0x01);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbd, 0x00, 0x00);	//Packet Size Control Register1 D[15:0]
	SSD2825_spi_reg_data_wr(0xbe, 0x00, 0x02);	//Packet Size Control Register2 D[31:16]
	WriteCommand(address);
}

void SSD2825_1P(unsigned int address, unsigned int data0)
{
	SSD2825_spi_reg_data_wr(0xb7, 0x03, 0x50);	//seting Generic packet
	SSD2825_spi_reg_data_wr(0xbc, 0x00, 0x01);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbd, 0x00, 0x00);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbe, 0x00, 0x02);	//Packet Size Control Register1 D[15:0]
	WriteCommand(address);
	WritePara(data0);
}

void SSD2825_2P(unsigned int address, unsigned int data0, unsigned int data1)
{
	SSD2825_spi_reg_data_wr(0xb7, 0x07, 0x10);	//seting Generic packet
	SSD2825_spi_reg_data_wr(0xbc, 0x00, 0x03);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbd, 0x00, 0x00);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbe, 0x00, 0x03);	//Packet Size Control Register1 D[15:0]
	WriteCommand(0xbf);
	WritePara(address);
	WritePara(data0);
	WritePara(data1);
}

void SSD2825_3P(unsigned int address, unsigned int data0, unsigned int data1,
		unsigned int data2)
{
	SSD2825_spi_reg_data_wr(0xb7, 0x07, 0x10);	//seting Generic packet
	SSD2825_spi_reg_data_wr(0xbc, 0x00, 0x04);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbd, 0x00, 0x00);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbe, 0x00, 0x04);	//Packet Size Control Register1 D[15:0]
	WriteCommand(0xbf);
	WritePara(address);
	WritePara(data0);
	WritePara(data1);
	WritePara(data2);
}

void SSD2825_15P(unsigned int address, unsigned int data0, unsigned int data1,
		 unsigned int data2, unsigned int data3, unsigned int data4,
		 unsigned int data5, unsigned int data6, unsigned int data7,
		 unsigned int data8, unsigned int data9, unsigned int data10,
		 unsigned int data11, unsigned int data12, unsigned int data13,
		 unsigned int data14)
{
	SSD2825_spi_reg_data_wr(0xb7, 0x07, 0x10);	//seting Generic packet
	SSD2825_spi_reg_data_wr(0xbc, 0x00, 0x10);	//seting Generic packet
	SSD2825_spi_reg_data_wr(0xbd, 0x00, 0x00);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbe, 0x00, 0x10);	//Packet Size Control Register1 D[15:0]
	WriteCommand(0xbf);
	WritePara(address);
	WritePara(data0);
	WritePara(data1);
	WritePara(data2);
	WritePara(data3);
	WritePara(data4);
	WritePara(data5);
	WritePara(data6);
	WritePara(data7);
	WritePara(data8);
	WritePara(data9);
	WritePara(data10);
	WritePara(data11);
	WritePara(data12);
	WritePara(data13);
	WritePara(data14);
}

void SSD2825_18P(unsigned int address, unsigned int data0, unsigned int data1,
		 unsigned int data2, unsigned int data3, unsigned int data4,
		 unsigned int data5, unsigned int data6, unsigned int data7,
		 unsigned int data8, unsigned int data9, unsigned int data10,
		 unsigned int data11, unsigned int data12, unsigned int data13,
		 unsigned int data14, unsigned int data15, unsigned int data16,
		 unsigned int data17)
{
	SSD2825_spi_reg_data_wr(0xb7, 0x07, 0x10);	//seting Generic packet
	SSD2825_spi_reg_data_wr(0xbc, 0x00, 0x13);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbd, 0x00, 0x00);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbe, 0x00, 0x13);	//Packet Size Control Register1 D[15:0]
	WriteCommand(0xbf);
	WritePara(address);
	WritePara(data0);
	WritePara(data1);
	WritePara(data2);
	WritePara(data3);
	WritePara(data4);
	WritePara(data5);
	WritePara(data6);
	WritePara(data7);
	WritePara(data8);
	WritePara(data9);
	WritePara(data10);
	WritePara(data11);
	WritePara(data12);
	WritePara(data13);
	WritePara(data14);
	WritePara(data15);
	WritePara(data16);
	WritePara(data17);
}

void SSD2825_24P(unsigned int address, unsigned int data0, unsigned int data1,
		 unsigned int data2, unsigned int data3, unsigned int data4,
		 unsigned int data5, unsigned int data6, unsigned int data7,
		 unsigned int data8, unsigned int data9, unsigned int data10,
		 unsigned int data11, unsigned int data12, unsigned int data13,
		 unsigned int data14, unsigned int data15, unsigned int data16,
		 unsigned int data17, unsigned int data18, unsigned int data19,
		 unsigned int data20, unsigned int data21, unsigned int data22,
		 unsigned int data23)
{
	SSD2825_spi_reg_data_wr(0xb7, 0x07, 0x10);	//seting Generic packet
	SSD2825_spi_reg_data_wr(0xbc, 0x00, 0x19);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbd, 0x00, 0x00);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbe, 0x00, 0x19);	//Packet Size Control Register1 D[15:0]
	WriteCommand(0xbf);
	WritePara(address);
	WritePara(data0);
	WritePara(data1);
	WritePara(data2);
	WritePara(data3);
	WritePara(data4);
	WritePara(data5);
	WritePara(data6);
	WritePara(data7);
	WritePara(data8);
	WritePara(data9);
	WritePara(data10);
	WritePara(data11);
	WritePara(data12);
	WritePara(data13);
	WritePara(data14);
	WritePara(data15);
	WritePara(data16);
	WritePara(data17);
	WritePara(data18);
	WritePara(data19);
	WritePara(data20);
	WritePara(data21);
	WritePara(data22);
	WritePara(data23);
}

void SSD2825_34P(unsigned int address, unsigned int data0, unsigned int data1,
		 unsigned int data2, unsigned int data3, unsigned int data4,
		 unsigned int data5, unsigned int data6, unsigned int data7,
		 unsigned int data8, unsigned int data9, unsigned int data10,
		 unsigned int data11, unsigned int data12, unsigned int data13,
		 unsigned int data14, unsigned int data15, unsigned int data16,
		 unsigned int data17, unsigned int data18, unsigned int data19,
		 unsigned int data20, unsigned int data21, unsigned int data22,
		 unsigned int data23, unsigned int data24, unsigned int data25,
		 unsigned int data26, unsigned int data27, unsigned int data28,
		 unsigned int data29, unsigned int data30, unsigned int data31,
		 unsigned int data32, unsigned int data33)
{
	SSD2825_spi_reg_data_wr(0xb7, 0x07, 0x10);	//seting Generic packet
	SSD2825_spi_reg_data_wr(0xbc, 0x00, 0x23);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbd, 0x00, 0x00);	//Packet Size Control Register2 D[31:16]
	SSD2825_spi_reg_data_wr(0xbe, 0x00, 0x23);	//Packet Size Control Register1 D[15:0]
	WriteCommand(0xbf);
	WritePara(address);
	WritePara(data0);	//
	WritePara(data1);	//
	WritePara(data2);	//
	WritePara(data3);	//
	WritePara(data4);	//
	WritePara(data5);	//
	WritePara(data6);	//
	WritePara(data7);	//
	WritePara(data8);	//
	WritePara(data9);	//
	WritePara(data10);	//
	WritePara(data11);	//
	WritePara(data12);	//
	WritePara(data13);	//
	WritePara(data14);	//
	WritePara(data15);	//
	WritePara(data16);	//
	WritePara(data17);	//
	WritePara(data18);	//
	WritePara(data19);	//
	WritePara(data20);	//
	WritePara(data21);	//
	WritePara(data22);	//
	WritePara(data23);	//
	WritePara(data24);	//
	WritePara(data25);	//
	WritePara(data26);	//
	WritePara(data27);	//
	WritePara(data28);	//
	WritePara(data29);	//
	WritePara(data30);	//
	WritePara(data31);	//
	WritePara(data32);	//
	WritePara(data33);	//
}

static int reset_lcd(void)
{
	int ret = 0;

	ret = gpio_request(LCD_RESET, "lcd");
	if (ret != 0) {
		TR0("line %d : gpio %d request fail!\n", __LINE__, LCD_RESET);
		return ret;
	}
	gpio_direction_output(LCD_RESET, 1);
	mdelay(10);
	gpio_direction_output(LCD_RESET, 0);
	mdelay(10);
	gpio_direction_output(LCD_RESET, 1);
	mdelay(10);
	gpio_free(LCD_RESET);

	return ret;
}

/*
 * probe
 */

int __devinit nusmart_dsi_probe(struct platform_device *pdev)
{
	//struct device *device = &pdev->dev;
	//struct resource *res = NULL;

	TR0("probe\n");

	clk = clk_get(NULL, "ns115_alt1");
	if (clk_set_rate(clk, 12000000) < 0)
		TR0("set clcd clock failed!\n");
	clk_enable(clk);

	reset_lcd();
	tft_power_on();
#if 0
	TR0("read 0xb1:0x%04x", SSD2825_spi_reg_data_rd(0xB1));
	TR0("read 0xb2:0x%04x", SSD2825_spi_reg_data_rd(0xB2));
	TR0("read 0xb3:0x%04x", SSD2825_spi_reg_data_rd(0xB3));
	TR0("read 0xb4:0x%04x", SSD2825_spi_reg_data_rd(0xB4));
	TR0("read 0xb5:0x%04x", SSD2825_spi_reg_data_rd(0xB5));
	TR0("read 0xb6:0x%04x", SSD2825_spi_reg_data_rd(0xB6));
	TR0("read 0xb7:0x%04x", SSD2825_spi_reg_data_rd(0xB7));
#endif
	return 0;
}

int nusmart_dsi_remove(struct platform_device *pdev)
{
	TR0("remove\n");

	return 0;
}

int nusmart_dsi_suspend(struct platform_device *dev, pm_message_t msg)
{
	TR0("suspend\n");

	clk_disable(clk);

	return 0;
}

int nusmart_dsi_resume(struct platform_device *dev)
{
	TR0("resume\n");

	clk = clk_get(NULL, "ns115_alt1");
	if (clk_set_rate(clk, 12000000) < 0)
		TR0("set clcd clock failed!\n");
	clk_enable(clk);

	reset_lcd();
	tft_power_on();

	return 0;
}

struct platform_driver nusmart_dsi_driver = {
	.probe = nusmart_dsi_probe,
	.remove = nusmart_dsi_remove,
	.suspend = nusmart_dsi_suspend,
	.resume = nusmart_dsi_resume,

	.driver = {
		   .name = DSI_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int nusmart_dsi_init(void)
{
	int ret;

	TR0("init\n");

	ret = platform_driver_register(&nusmart_dsi_driver);
	if (ret)
		TR0("nusmart_dsi_driver register failed! retval = %d", ret);

	return 0;
}

static void nusmart_dsi_exit(void)
{
	TR0("exit\n");

	platform_driver_unregister(&nusmart_dsi_driver);
}

module_init(nusmart_dsi_init);
module_exit(nusmart_dsi_exit);

MODULE_LICENSE("GPL");
