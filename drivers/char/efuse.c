/*
 *  linux/drivers/char/efuse.c  
 *
 *	Huangshan efuse info
 *  
 *  Author: Bai Nianfu
 *
 *  Copyright: Nusmart
 * 
 */



#include <linux/module.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>      /* Necessary because we use the proc fs */
#include <asm/uaccess.h>        /* for copy_from_user */
#include <mach/hardware.h>
#include <mach/board-ns2816.h>


#define PROCFS_MAX_SIZE         1024
#define PROCFS_NAME             "efuseinfo"

#define EFUSE_OFFSET			0x00000080
#define EFUSE_SIZE				(1024/8)
#define EFUSE_VALID_DATA_OFFSET	(512/8)
#define EFUSE_DATA_SIZE			8

/**
 * This structure hold information about the /proc file
 *
 */
static struct proc_dir_entry *Our_Proc_File;

/**
 * The buffer used to store character for this module
 *
 */
static char procfs_buffer[PROCFS_MAX_SIZE];

/**
 * The size of the buffer
 *
 */
static unsigned long procfs_buffer_size = 0;


/*
	efuse virtual address
*/
volatile unsigned int* addr;


/**
 * This function is called then the /proc file is read
 *
 */
static int
procfile_read(char *buffer,
              char **buffer_location,
              off_t offset, int buffer_length, int *eof, void *data)
{
        int ret;
        if (offset > 0) {
                /* we have finished to read, return 0 */
                ret = 0;
        } else {
                /* fill the buffer, return the buffer size */
                memcpy(buffer, procfs_buffer, procfs_buffer_size);
                ret = procfs_buffer_size;
        }
        return ret;
}


/**
 * This function is called with the /proc file is written
 *
 */
static int procfile_write(struct file *file, const char *buffer, unsigned long count,
                    void *data)
{
        /* get buffer size */
        procfs_buffer_size = count;
        if (procfs_buffer_size > PROCFS_MAX_SIZE ) {
                procfs_buffer_size = PROCFS_MAX_SIZE;
        }
        /* write data to the buffer */
        if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
				printk("ERROR: copy data from user failed\n");
                return -EFAULT;
        }
        return procfs_buffer_size;
}


static void config_efuse_data()
{
	/* read efuse data */
	addr = (volatile unsigned int*)IO_ADDRESS(NS2816_SCM_BASE+EFUSE_OFFSET+(EFUSE_SIZE-EFUSE_VALID_DATA_OFFSET)-EFUSE_DATA_SIZE);
	volatile unsigned int* ptr = NULL;
	unsigned int data[2] = {0};
	int i;
	for(i=0;i<2;i++){
		ptr = (volatile unsigned int*)(addr+i);
		data[i] = __raw_readl(ptr);
	}

	/* get exact value */
	u32 factorey_no = (data[0] >> 19) & 0x03;
	u32 ate_no = (data[0] >> 11) & 0xFF;
	u32 year = ((data[0] >> 4) & 0x7F) + 2000;
	u32 mon = data[0] & 0x0F;
	u32 day = (data[1] >> 27) & 0x1F; 
	u32 hour = (data[1] >> 22) & 0x1F;
	u32 min = (data[1] >> 16) & 0x3F;
	u32 sec = (data[1] >> 10) & 0x3F;
	u32 level = data[1] & 0x1F; 
	
	char factoryNo[100] = {0};
	sprintf(factoryNo, "factory no=%d; ", factorey_no);
	char ateNo[100] = {0};
	sprintf(ateNo, "ate no=%d; ", ate_no);
	char Year[100] = {0};
	sprintf(Year, "year=%d; ", year);
	char Mon[100] = {0};
	sprintf(Mon, "month=%d; ", mon);
	char Day[100] = {0};
	sprintf(Day, "day=%d; ", day);
	char Hour[100] = {0};
	sprintf(Hour, "hour=%d; ", hour);
	char Min[100] = {0};
	sprintf(Min, "minute=%d; ", min);
	char Sec[100] = {0};
	sprintf(Sec, "second=%d; ", sec);
	char Level[100] = {0};
	sprintf(Level, "Leakage level=%d;\n", level);

	char buffer[PROCFS_MAX_SIZE] = {0};	
	int size = 0;
	memcpy(buffer, factoryNo, strlen(factoryNo));
	size += strlen(factoryNo);
	memcpy(buffer+size, ateNo, strlen(ateNo));
	size += strlen(ateNo);
	memcpy(buffer+size, Year, strlen(Year));
	size += strlen(Year);
	memcpy(buffer+size, Mon, strlen(Mon));
	size += strlen(Mon);
	memcpy(buffer+size, Day, strlen(Day));
	size += strlen(Day);
	memcpy(buffer+size, Hour, strlen(Hour));
	size += strlen(Hour);
	memcpy(buffer+size, Min, strlen(Min));
	size += strlen(Min);
	memcpy(buffer+size, Sec, strlen(Sec));
	size += strlen(Sec);
	memcpy(buffer+size, Level, strlen(Level));
	size += strlen(Level);

	memcpy(procfs_buffer, buffer, size);
	procfs_buffer_size += size;
}

static int efuse_init(void)
{
	/* create the /proc file */
	Our_Proc_File = create_proc_entry(PROCFS_NAME, 0644, NULL);
	if (Our_Proc_File == NULL) {
			remove_proc_entry(PROCFS_NAME, NULL);
			printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
					 PROCFS_NAME);
			return -ENOMEM;
	}
	Our_Proc_File->read_proc   = procfile_read;
	Our_Proc_File->write_proc  = procfile_write;
	Our_Proc_File->mode        = S_IFREG | S_IRUGO;
	Our_Proc_File->size        = 37;

	config_efuse_data();

	return 0;
}



static void efuse_exit(void)
{
	remove_proc_entry(PROCFS_NAME, NULL);
	iounmap(addr);
}

module_init(efuse_init);
module_exit(efuse_exit);

MODULE_DESCRIPTION("NUSMART efuse driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nusmart Bai Nianfu");
