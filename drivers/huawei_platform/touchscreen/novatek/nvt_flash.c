
/*************************
  Create Device Node (Proc Entry)
**************************/

#include "nvt.h"

#define DEVICE_NAME "NVTflash"

extern struct i2c_client *nvt_client;
extern struct nvt_ts_data *ts;

ssize_t nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
	struct i2c_msg msgs[2];	
	char *str;
	int ret=-1;
	int retries = 0;
	file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
	str = file->private_data;
	ret=copy_from_user(str, buff, count);

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = str[1];
	msgs[0].buf   = &str[2];

	while(retries < 20){
		ret = i2c_transfer(ts->client->adapter, msgs, 1);
		if(ret == 1){
			break;
		}else{
			printk("write error %d\n", retries);
		}
		retries++;
	}
	return ret;
}

ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	struct i2c_msg msgs[2];	 
	char *str;
	int ret = -1;
	int retries = 0;
	file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
	str = file->private_data;
	if(copy_from_user(str, buff, count)){
		return -EFAULT;
	}

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = 1;
	msgs[0].buf   = &str[2];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = str[0];
	msgs[1].len   = str[1]-1;
	msgs[1].buf   = &str[3];

	while(retries < 20)
	{
		ret = i2c_transfer(ts->client->adapter, msgs, 2);
		if(ret == 2){
			break;
		}else{
			printk("read error %d\n", retries);
		}
		retries++;
	}
	ret=copy_to_user(buff, str, count);
	return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if(dev == NULL){
		return -ENOMEM;
	}
	rwlock_init(&dev->lock);
	file->private_data = dev;
	return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;
	
	if(dev){
		kfree(dev);
	}
	return 0;   
}

struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.write = nvt_flash_write,
	.read = nvt_flash_read,
};
int nvt_flash_init(void)
{		
	//int ret=0;
#if 0
	static struct proc_dir_entry *nvt_proc_entry;
  	nvt_proc_entry = create_proc_entry(DEVICE_NAME, 0666, NULL);
	if(nvt_proc_entry == NULL){
		printk("%s: couldn't create proc entry!\n", __func__);
		ret = -ENOMEM;
		return ret ;
	}else{
		printk("%s: create proc entry success!\n", __func__);
		nvt_proc_entry->proc_fops = &nvt_flash_fops;
	}
#endif
    proc_create(DEVICE_NAME, 0666, NULL, &nvt_flash_fops);
	printk("NVT_flash driver loaded\n");
	return 0;
}
