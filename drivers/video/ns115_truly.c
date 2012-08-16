/*
 */
#include <linux/gpio.h>
#include <linux/delay.h>
#include "ns115-clcd.h"

#define GPIO_PORTA_DR    0x0
#define GPIO_PORTA_DDR   0x4
#define GPIO_PORTA_CTRL  0x8
#define GPIO_PORTB_DR    0xc
#define GPIO_PORTB_DDR   0x10
#define GPIO_PORTB_CTRL  0x14

#define LCD_RESET 	        (8 + 4)			
#define LCD_SPI_CS   		(8 + 2)	
#define LCD_SPI_CLK 		(8 + 5)						
#define LCD_SPI_SDI 	        (8 + 3)	

void inline reset_lcd(void)
{
	int ret = 0;

	ret = gpio_request(LCD_RESET, "lcd truly");
	if(ret != 0) {
		printk("line %d : gpio %d request fail!\n", __LINE__, LCD_RESET);
		return ret;
	}
	gpio_direction_output(LCD_RESET, 1);
	mdelay(10);
	gpio_direction_output(LCD_RESET, 0);
	mdelay(10);
	gpio_direction_output(LCD_RESET, 1);
	mdelay(10);
	gpio_free(LCD_RESET);
}

void inline spi_write_9bit(unsigned short data)
{
	int i;
	int ret = 0;

	
	for (i=9; i>0; i--)
	{
		ret = gpio_request(LCD_SPI_CLK, "lcd truly");
		if(ret != 0) {
			printk("line %d: gpio %d request fail!\n", __LINE__, LCD_SPI_CLK);
			return ret;
		}
		gpio_direction_output(LCD_SPI_CLK, 0); 
		gpio_free(LCD_SPI_CLK);

		ret = gpio_request(LCD_SPI_SDI, "lcd truly");
		if(ret != 0) {
			printk("line %d: gpio %d request fail!\n", __LINE__, LCD_SPI_SDI);
			return ret;
		}
		if (data & (1<<(i-1)))
		{
			gpio_direction_output(LCD_SPI_SDI, 1); 
		}
		else
		{
			gpio_direction_output(LCD_SPI_SDI, 0); 
		}
		gpio_free(LCD_SPI_SDI);
	
		ret = gpio_request(LCD_SPI_CLK, "lcd truly");
		if(ret != 0) {
			printk("line %d: gpio %d request fail!\n", __LINE__, LCD_SPI_CLK);
			return ret;
		}
		gpio_direction_output(LCD_SPI_CLK, 1); 
		gpio_free(LCD_SPI_CLK);
	}
	
}

void inline lcd_spi_write(unsigned char *buf, int length)
{	
	unsigned short data;
	int i, ret = 0;
	
	ret = gpio_request(LCD_SPI_CS, "lcd truly");
	if(ret != 0) {
		printk("line %d: gpio %d request fail!\n", __LINE__, LCD_SPI_CS);
		return ret;
	}
	gpio_direction_output(LCD_SPI_CS, 0); 
	gpio_free(LCD_SPI_CS);

	for (i=0; i<length;i++)
	{
		data = 0;
		if (i)
		{
			data = 1<< 8 | buf[i];
		}
		else
		{
			data = buf[i];
		}
		
		spi_write_9bit(data);
	}

	ret = gpio_request(LCD_SPI_CS, "lcd truly");
	if(ret != 0) {
		printk("line %d: gpio %d request fail!\n", __LINE__, LCD_SPI_CS);
		return ret;
	}
	gpio_direction_output(LCD_SPI_CS, 1); //CS=1
	gpio_free(LCD_SPI_CS);

	return ;
}

int lcd_init_hx8369(void)
{	
	//Enable extention command
	unsigned char index1[]={0XB9,\
							0XFF,0X83,0X69 \
						   };
	
	unsigned char index2[]={0XB0, \
							0x01,0x0C \
						   };

	//Set Power
	unsigned char index3[]={0xB1,\
							0x01,0x00,0x34,0x06, \
							0x00,0x0F,0x0F,0x1A, \
							0x21,0x3F,0x3F,0x07, \
							0x1B,0x01,0xE6,0xE6, \
							0xE6,0xE6,0xE6		 \	
						   };

	// SET Display  480x800 
	unsigned char index4[]={0XB2,\
							0x00,0x2B,0x0A,0x0A,\
							0x70,0x00,0xFF,0x00,\
							0x00,0x00,0x00,0x03,\
							0x03,0x00,0x01		\
						   };

	// SETTING RGB SIGNAL
	unsigned char index5[]={0XB3,\
							0X05 \
						   };

	// SET Display CYC
	unsigned char index6[]={0XB4,\
							0x02,0x0A,0xA0,0x0E, \
							0x00 \
						   };

	// SET VCOM
	unsigned char index7[]={0XB6,\
							0x1E,0x1E \
						   };
	
	unsigned char index8[]={0xD5,\
							0x00,0x05,0x03,0x00, \
							0x01,0x09,0x10,0x80, \
							0x37,0x37,0x20,0x31, \
							0x46,0x8A,0x57,0x9B, \
							0x20,0x31,0x46,0x8A, \
							0x57,0x9B,0x07,0x0F, \
							0x07,0x00 \
						   };

	//SET GAMMA
	unsigned char index9[]={0xE0,\
							0x00,0x01,0x03,0x2B, \
							0x33,0x3F,0x0D,0x30, \
							0x06,0x0B,0x0D,0x10, \
							0x13,0x11,0x13,0x11, \
							0x17,0x00,0x01,0x03, \
							0x2B,0x33,0x3F,0x0D, \
							0x30,0x06,0x0B,0x0D, \
							0x10,0x13,0x11,0x13, \
							0x11,0x17 \
						   };
	//set DGC
	unsigned char index10[]={0xC1,\
							 0x01,0x01,0x08,0x10, \
							 0x18,0x1F,0x27,0x2F, \
							 0x34,0x3B,0x43,0x4B, \
							 0x53,0x5B,0x62,0x6A, \
							 0x72,0x7A,0x81,0x88, \
							 0x8F,0x97,0x9F,0xA7, \
							 0xAF,0xB8,0xC1,0xCA, \
							 0xD1,0xD9,0xE3,0xEB, \
							 0xF5,0xFE,0xCB,0x31, \
							 0x46,0x3E,0x03,0xA9, \
							 0x0A,0x2E,0x00,0x01, \
							 0x08,0x10,0x18,0x1F, \
							 0x27,0x2F,0x34,0x3B, \
							 0x43,0x4B,0x53,0x5B, \
							 0x62,0x6A,0x72,0x7A, \
							 0x81,0x88,0x8F,0x97, \
							 0x9F,0xA7,0xAF,0xB8, \
							 0xC1,0xCA,0xD1,0xD9, \
							 0xE3,0xEB,0xF5,0xFE, \
							 0xCB,0x31,0x46,0x3E, \
							 0x03,0xA9,0x0A,0x2E, \
							 0x00,0x01,0x08,0x10, \
							 0x18,0x1F,0x27,0x2F, \
							 0x34,0x3B,0x43,0x4B, \
							 0x53,0x5B,0x62,0x6A, \
							 0x72,0x7A,0x81,0x88, \
							 0x8F,0x97,0x9F,0xA7, \
							 0xAF,0xB8,0xC1,0xCA, \
							 0xD1,0xD9,0xE3,0xEB, \
							 0xF5,0xFE,0xCB,0x31, \
							 0x46,0x3E,0x03,0xA9, \
							 0x0A,0x2E,0x00 \
						    };

	//24BIT RGB 
	unsigned char index11[]={0x3A,\
							 0x77 \
						    };

	unsigned char index12[]={0x11\
						    };

	unsigned char index13[]={0X29\
						    };
	// Reset;
	reset_lcd();
	//LCD init
	lcd_spi_write(index1, sizeof(index1));
	lcd_spi_write(index2, sizeof(index2));
	lcd_spi_write(index3, sizeof(index3));
	lcd_spi_write(index4, sizeof(index4));
	lcd_spi_write(index5, sizeof(index5));
	lcd_spi_write(index6, sizeof(index6));
	lcd_spi_write(index7, sizeof(index7));
	lcd_spi_write(index8, sizeof(index8));
	lcd_spi_write(index9, sizeof(index9));
	lcd_spi_write(index10, sizeof(index10));
	lcd_spi_write(index11, sizeof(index11));
	lcd_spi_write(index12, sizeof(index12));
	mdelay(120);
	lcd_spi_write(index13, sizeof(index13));

	return 0 ;
}
