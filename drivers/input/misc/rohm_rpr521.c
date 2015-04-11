/*huawei kernel driver for rpr521*/
/*
 * rpr521.c - Linux kernel modules for ambient light + proximity sensor
 *

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/i2c/rohm_rpr521.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/wakelock.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

#include <misc/app_info.h>

#include <linux/debugfs.h>

/*************** Global Data ******************/
/* parameter for als calculation */
#define COEFFICIENT               (4)
//read these coefficient from dtsi
unsigned long *data0_coefficient;//[COEFFICIENT] = {7768, 4388, 2627, 1971};
unsigned long *data1_coefficient;//[COEFFICIENT] = {5066, 2315, 1106, 687};
unsigned long *judge_coefficient;//[COEFFICIENT] = {1032, 1605, 1904, 2864};

static int origin_prox = 822;
#define RPR521_DRV_NAME		"rpr521"
#define DRIVER_VERSION		"1.0.0"

#define RPR521_REG_LEN 0xf 

/* Register Value define : CONTROL */

#define SENSORS_I2C_SCL	908
#define SENSORS_I2C_SDA	907

#define RPR521_I2C_RETRY_COUNT		3 	/* Number of times to retry i2c */ 
#define RPR521_I2C_RETRY_TIMEOUT	1	/* Timeout between retry (miliseconds) */

#define RPR521_I2C_BYTE 0
#define RPR521_I2C_WORD 1
/*keep 400ms wakeup after the ps report the far or near state*/
#define PS_WAKEUP_TIME 400
/*it is like min_proximity_value in 8x12 and 8930,to adjust the dynamic proximity ditance*/
static int rpr521_ps_init_threshold = 960;
static u8  reg0_value = PS_ALS_SET_MODE_CONTROL;
u8 gain0 = 2;
u8 gain1 = 2;
u16 atime = 100;
/*delete the wakelock,so system can sleep when the device is in the calling*/
/*pls parameters,it is still different for every devices*/
static uint32_t rpr521_pwave_value = 100;
static uint32_t rpr521_pwindow_value = 200;
/*dynamic debug mask to control log print,you can echo value to rpr521_debug to control*/
static int rpr521_debug_mask= 0;  
module_param_named(rpr521_debug, rpr521_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(rpr521_pwindow, rpr521_pwindow_value, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(rpr521_pwave, rpr521_pwave_value, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define RPR521_ERR(x...) do {\
    if (rpr521_debug_mask >=0) \
        printk(KERN_ERR x);\
    } while (0)
/*KERNEL_HWFLOW is for radar using to control all the log of devices*/
#define RPR521_INFO(x...) do {\
    if (KERNEL_HWFLOW && (rpr521_debug_mask >=0)) \
        printk(KERN_ERR x);\
    } while (0)
#define RPR521_FLOW(x...) do {\
    if (KERNEL_HWFLOW && (rpr521_debug_mask >=1)) \
        printk(KERN_ERR x);\
    } while (0)

/*
 * Structs
 */
 struct ls_test_excep{
	int i2c_scl_val;		/* when i2c transfer err, read the gpio value*/
	int i2c_sda_val;
};
struct rpr521_data {
	struct i2c_client *client;
	/*to protect the i2c read and write operation*/
	struct mutex update_lock;
	struct work_struct	dwork;		/* for PS interrupt */
	struct work_struct	als_dwork;	/* for ALS polling */
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;

	/* regulator data */
	bool power_on;
	struct regulator *vdd;
	struct regulator *vio;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	/*sensor class for als and ps*/
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
	struct rpr521_platform_data *platform_data;
	/*for capture the i2c and other errors*/
	struct ls_test_excep ls_test_exception;

	int irq;
	/*hrtimer is removed from platform struct to here*/
	struct hrtimer timer;
	/*wake lock for not losing ps event reporting*/
	struct wake_lock ps_report_wk;
	unsigned int enable;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int control;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;
	/*to record the open or close state of als before suspend*/
	unsigned int enable_als_state;
	/*to record the close or far state of ps,0 is close,1 is far*/
	unsigned int ps_close_or_far;

	/* PS parameters */
	unsigned int ps_min_threshold; 	/*it is the min_proximity_value */
	unsigned int ps_detection;		/* 5 = near-to-far; 0 = far-to-near */
	unsigned int ps_data;/* to store PS data,it is pdata */

	/* ALS parameters */
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */
	unsigned int als_data;		/* to store ALS data from CH0 or CH1*/
	int als_prev_lux;		/* to store previous lux value */
	unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */
};

static struct sensors_classdev sensors_light_cdev = {
	.name = "rpr521-light",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "10000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static struct sensors_classdev sensors_proximity_cdev = {
	.name = "rpr521-proximity",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "1",
	.resolution = "1.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

/*
 * Global data
 */
/*delete two global data to avoid potencial risks*/
/* Proximity sensor use this work queue to report data */
static struct workqueue_struct *rpr521_workqueue = NULL;
/*changeable als gain and ADC time,we don't use*/

static int cur_pthreshold_h=0;
static int cur_pthreshold_l=0;

static int far_init=549;
static int near_init=550;

/*
 * Management functions,they are used to set registers in booting and enable
 */
/*init the register of device function for probe and every time the chip is powered on*/
static int rpr521_init_client(struct i2c_client *client);
/*
* if i2c transfer error, we check sda/scl value and regulator's value
*/
static void dump_i2c_exception_status(struct rpr521_data *data)
{
	/* print pm status and i2c gpio status*/
	struct ls_test_excep *test_exception = &data->ls_test_exception;
	int voltage_mv[2] = {0,0};
	/*the on or off of power,0 is off,1 is on*/
	int power_on[2]={1,1};

	if (data->vdd == NULL) {
		return;
	}

	if (data->vio == NULL) {
		return;
	}

	/* read i2c_sda i2c_scl gpio value*/
	mutex_lock(&data->update_lock);
	test_exception->i2c_scl_val = gpio_get_value(SENSORS_I2C_SCL);
	test_exception->i2c_sda_val = gpio_get_value(SENSORS_I2C_SDA);
	mutex_unlock(&data->update_lock);

	/* get regulator's status*/
	power_on[0] = regulator_is_enabled(data->vdd);
	if(power_on[0] < 0){
		RPR521_ERR("%s,line %d:regulator_is_enabled vdd failed",__func__,__LINE__);
	}
	power_on[1] = regulator_is_enabled(data->vio);
	if(power_on[1] < 0){
		RPR521_ERR("%s,line %d:regulator_is_enabled vio failed",__func__,__LINE__);
	}

	/* get regulator's value*/
	voltage_mv[0] = regulator_get_voltage(data->vdd)/1000;
	if(voltage_mv[0] < 0){
		RPR521_ERR("%s,line %d:regulator_get_voltage vdd failed",__func__,__LINE__);
	}

	voltage_mv[1] = regulator_get_voltage(data->vio)/1000;
	if(voltage_mv[1] < 0){
		RPR521_ERR("%s,line %d:regulator_get_voltage vio failed",__func__,__LINE__);
	}

	RPR521_INFO("%s,line %d:i2c_scl_val=%d,i2c_sda_val=%d,vdd = %d status %d, vddio=%d status %d",__func__,__LINE__,
				test_exception->i2c_scl_val,test_exception->i2c_sda_val,
				voltage_mv[0],power_on[0],voltage_mv[1],power_on[1]);
}

/*we use the unified the function for i2c write and read operation*/
static int rpr521_i2c_write(struct i2c_client*client, u8 reg, u16 value,bool flag)
{
	int err,loop;

	struct rpr521_data *data = i2c_get_clientdata(client);

	loop = RPR521_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_write_byte_data,1 is i2c_smbus_write_word_data*/
		if(flag == RPR521_I2C_BYTE)
		{
			err = i2c_smbus_write_byte_data(client, reg, (u8)value);
		}
		else if(flag == RPR521_I2C_WORD)
		{
			err = i2c_smbus_write_word_data(client, reg, value);
		}
		else
		{
			RPR521_ERR("%s,line %d:attention: i2c write wrong flag",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			mdelay(RPR521_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		RPR521_ERR("%s,line %d:attention:i2c write err = %d",__func__,__LINE__,err);
		dump_i2c_exception_status(data);
	}

	return err;
}

static int rpr521_i2c_read(struct i2c_client*client, u8 reg,bool flag)
{
	int err,loop;

	struct rpr521_data *data = i2c_get_clientdata(client);

	loop = RPR521_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_read_byte_data,1 is i2c_smbus_read_word_data*/
		if(flag == RPR521_I2C_BYTE)
		{
			err = i2c_smbus_read_byte_data(client, reg); 
		}
		else if(flag == RPR521_I2C_WORD)
		{
			err = i2c_smbus_read_word_data(client, reg);
		}
		else
		{
			RPR521_ERR("%s,line %d:attention: i2c read wrong flag",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			mdelay(RPR521_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		RPR521_ERR("%s,line %d:attention: i2c read err = %d,reg=0x%x",__func__,__LINE__,err,reg);
		dump_i2c_exception_status(data);
	}

	return err;
}

//grace modify in 2014.7.31 begin
static int rpr521_i2c_read_buf(struct i2c_client*client, u8 reg, u8 length, u8 *buf)
{
	int err,loop;
	u8 *values = buf;

	struct rpr521_data *data = i2c_get_clientdata(client);

	loop = RPR521_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		
		err = i2c_smbus_read_i2c_block_data(client, reg, length, values);
		
		mutex_unlock(&data->update_lock);
		if(err !=  length){
			loop--;
			mdelay(RPR521_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		RPR521_ERR("%s,line %d:attention: i2c read err = %d,reg=0x%x",__func__,__LINE__,err,reg);
		dump_i2c_exception_status(data);
	}

	return err;
}
//grace modify in 2014.7.31 end

/*
*	print the registers value with proper format
*/
static int dump_reg_buf(struct rpr521_data *data,char *buf, int size,int enable)
{
	int i=0;

	mutex_lock(&data->update_lock);

	if(enable)
		printk("[enable]");
	else
		printk("[disable]");
	printk(" reg_buf= ");
	for(i = 0;i < size; i++){
		printk("0x%2x  ",buf[i]);
	}
	mutex_unlock(&data->update_lock);

	printk("\n");
	return 0;
}
static int rpr521_regs_debug_print(struct rpr521_data *data,int enable)
{
	int i=0;
	char reg_buf[RPR521_REG_LEN];
	u8 reg = 0;
	struct i2c_client *client = data->client;

	/* read registers[0x0~0x1a] value*/
	for(i = 0; i < RPR521_REG_LEN; i++ )
	{
		reg = 0x40+i;
		reg_buf[i] = rpr521_i2c_read(client,reg,RPR521_I2C_BYTE);

		if(reg_buf[i] <0){
			RPR521_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg_buf[i] ;
		}
	}

	/* print the registers[0x0~0x1a] value in proper format*/
	dump_reg_buf(data,reg_buf,RPR521_REG_LEN,enable);

	return 0;
}



static int rpr521_set_enable(struct i2c_client *client, int enable)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client, REG_MODECONTROL, enable,RPR521_I2C_BYTE);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,enable = %d\n",__func__,__LINE__,enable);
		return ret;
	}

	data->enable = enable;
	RPR521_FLOW("%s,line %d:rpr521 enable = %d\n",__func__,__LINE__,enable);
	return ret;
}

static int rpr521_set_pilt(struct i2c_client *client, int threshold)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_PSTL, threshold,RPR521_I2C_WORD);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,threshold = %d\n",__func__,__LINE__,threshold);
		return ret;
	}

	data->pilt = threshold;
	RPR521_INFO("%s,line %d:set rpr521 pilt =%d\n", __func__, __LINE__,threshold);

	return ret;
}

static int rpr521_set_piht(struct i2c_client *client, int threshold)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_PSTH, threshold,RPR521_I2C_WORD);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,threshold = %d\n",__func__,__LINE__,threshold);
		return ret;
	}

	data->piht = threshold;
	RPR521_INFO("%s,line %d:set rpr521 piht =%d\n", __func__,__LINE__,threshold);
	return ret;
}

static int rpr521_set_pers(struct i2c_client *client, int pers)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_PERSISTENCE, pers,RPR521_I2C_BYTE);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,pers = %d\n",__func__,__LINE__,pers);
		return ret;
	}

	data->pers = pers;
	RPR521_FLOW("%s,line %d:rpr521 pers = %d\n",__func__,__LINE__,pers);
	return ret;
}

static int rpr521_set_config(struct i2c_client *client, int config)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_INTERRUPT, config,RPR521_I2C_BYTE);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,config = %d\n",__func__,__LINE__,config);
		return ret;
	}

	data->config = config;
	RPR521_FLOW("%s,line %d:rpr521 config = %d\n",__func__,__LINE__,config);
	return ret;
}

static int rpr521_set_control(struct i2c_client *client, int control)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_ALSPSCONTROL , control,RPR521_I2C_BYTE);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,control = %d\n",__func__,__LINE__,control);
		return ret;
	}

	data->control = control;
	RPR521_FLOW("%s,line %d:rpr521 control = %d\n",__func__,__LINE__,control);
	return ret;
}

static void rpr521_ps_report_event(struct i2c_client *client)
{
	struct rpr521_data *data = i2c_get_clientdata(client);

	int ret;
	unsigned char ps_info[3]; //grace modify in 2014.7.31
	
 //grace modify in 2014.7.31 begin
#if 0
	ret = rpr521_i2c_read(client, REG_PSDATA,RPR521_I2C_WORD);

	if( ret < 0 )
	{
	/* the number "200" is a value to make sure there is a valid value */
		data->ps_data = 200 ;
		RPR521_ERR("%s, line %d: pdate<0, reset to %d\n", __func__, __LINE__, data->ps_data);
	}else{
		data->ps_data = ret ;
	}
#else
        //here 3 means read register 43H¡¢44H¡¢45H one time
	ret = rpr521_i2c_read_buf(client, REG_PERSISTENCE, 3, ps_info);

	if( ret < 0 )
	{
	/* the number "200" is a value to make sure there is a valid value */
		data->ps_data = 200 ;
		RPR521_ERR("%s, line %d: pdate<0, reset to %d\n", __func__, __LINE__, data->ps_data);
	}else{
		data->ps_data = (ps_info[2] << 8) | ps_info[1];
	}
#endif
//grace modify in 2014.7.31 end

	/*in 400ms,system keeps in wakeup state to avoid the sleeling system lose the pls event*/
	wake_lock_timeout(&data->ps_report_wk, PS_WAKEUP_TIME);

	RPR521_FLOW("%s,line %d:rpr521 ps_data=%d\n",__func__,__LINE__,data->ps_data);
	RPR521_FLOW("%s,line %d:rpr521 ps_min_threshold=%d\n",__func__,__LINE__,data->ps_min_threshold);
	
#if 0 //grace modify in 2014.7.31
	if ((data->ps_data + rpr521_pwave_value) < (data->ps_min_threshold))
	{
		data->ps_min_threshold = data->ps_data + rpr521_pwave_value;
		ret = rpr521_i2c_write(client,REG_PSTL,data->ps_min_threshold,RPR521_I2C_WORD);
		ret += rpr521_i2c_write(client,REG_PSTH,data->ps_min_threshold + rpr521_pwindow_value,RPR521_I2C_WORD);
		if (ret < 0)
		{
			RPR521_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
			goto exit;
		}
		data->pilt = data->ps_min_threshold;
		data->piht = data->ps_min_threshold + rpr521_pwindow_value;
		RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
		{
			cur_pthreshold_h = data->piht;
			cur_pthreshold_l = data->pilt;
			RPR521_FLOW("%s,line %d:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, __LINE__, data->ps_data, data->pilt, data->piht);
		}
	}
#else
	if (ps_info[0] >> 6 == 0)
	{
		if ((data->ps_data + rpr521_pwave_value) < (data->ps_min_threshold))
		{
			data->ps_min_threshold = data->ps_data + rpr521_pwave_value;
			ret = rpr521_i2c_write(client,REG_PSTL,data->ps_min_threshold,RPR521_I2C_WORD);
			ret += rpr521_i2c_write(client,REG_PSTH,data->ps_min_threshold + rpr521_pwindow_value,RPR521_I2C_WORD);
			if (ret < 0)
			{
				RPR521_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
				goto exit;
			}
			data->pilt = data->ps_min_threshold;
			data->piht = data->ps_min_threshold + rpr521_pwindow_value;
			RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
			if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
			{
				cur_pthreshold_h = data->piht;
				cur_pthreshold_l = data->pilt;
				RPR521_FLOW("%s,line %d:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, __LINE__, data->ps_data, data->pilt, data->piht);
			}
		}
	}
#endif

	if (data->ps_data >= data->piht) {
		/* far-to-near detected */
		data->ps_detection = 1;

		/* FAR-to-NEAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
		input_sync(data->input_dev_ps);
		data->ps_close_or_far = 0;
		RPR521_INFO("%s,line %d:PROXIMITY close event, data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);

		ret = rpr521_i2c_write(client,REG_PSTL,data->ps_min_threshold,RPR521_I2C_WORD);
		ret += rpr521_i2c_write(client,REG_PSTH, REG_PSTH_MAX, RPR521_I2C_WORD);
		if (ret < 0){
			RPR521_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
			goto exit;
		}
		data->pilt = data->ps_min_threshold;
		data->piht = REG_PSTH_MAX;
		RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
		{
			cur_pthreshold_h = data->piht;
			cur_pthreshold_l = data->pilt;
			RPR521_FLOW("%s,line %d:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, __LINE__, data->ps_data, data->pilt, data->piht);
		}
	} else if ((data->ps_data <= data->pilt)) {
		/* near-to-far detected */
		data->ps_detection = 0;

		/* NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);
		input_sync(data->input_dev_ps);
		data->ps_close_or_far = 1;
		RPR521_INFO("%s,line %d:PROXIMITY far event, data->ps_data=%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);

		ret = rpr521_i2c_write(client,REG_PSTH,
				data->ps_min_threshold + rpr521_pwindow_value,RPR521_I2C_WORD); 
		ret += rpr521_i2c_write(client,REG_PSTL,0,RPR521_I2C_WORD); 
		if (ret < 0){
			RPR521_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
			goto exit;
		}

		data->piht = data->ps_min_threshold + rpr521_pwindow_value;
		data->pilt  = 0;
		RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
		{
			cur_pthreshold_h = data->piht;
			cur_pthreshold_l = data->pilt;
			RPR521_FLOW("%s:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, data->ps_data, data->pilt, data->piht);
		}
	}
	else{
		RPR521_ERR("%s,line %d:data->ps_data=%d,data->pilt = %d,data->piht=%d,wrong interrupts\n",__func__,__LINE__,data->ps_data, data->pilt,data->piht);
	}
	return ;
exit:
	/*if i2c error happens,we report far event*/
	if(data->ps_close_or_far)
	{
		return ;
	}
	else
	{
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);
		input_sync(data->input_dev_ps);
		data->ps_close_or_far = 1;
		RPR521_ERR("%s:i2c error happens, report far event, data->ps_data:%d\n", __func__,data->ps_data);
		return ;
	}
}


/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static int long_long_divider(long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile long long divier;
    volatile long      unit_sft;

    if ((data < 0) || (base_divier == 0)) {
        *answer   = 0;
        *overplus = 0;
        return (CALC_ERROR);
    }

    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while ((data > divier) && (divier > 0)) {
            unit_sft++;
            divier = divier << 1;
        }
        while ((data > base_divier) && (unit_sft > 0)) {
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }

    return (0);
}


/******************************************************************************
 * NAME       : calc_rohm_als_data
 * FUNCTION   : calculate illuminance data for rpr521
 * REMARKS    : final_data is 1000 times, which is defined as CUT_UNIT, of the actual lux value
 *****************************************************************************/
static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned short time) 
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (1000)
#define MAX_OUTRANGE    65535// (11357)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)

	int                result; //grace modified in 2014.4.2
	int                final_data;
	CALC_DATA          calc_data;
	CALC_ANS           calc_ans;
	unsigned long      calc_judge;
	unsigned char      set_case;
	unsigned long      div_answer;
	unsigned long long div_overplus;
	unsigned long long overplus;
	unsigned long      max_range;

	/* set the value of measured als data */
	calc_data.als_data0  = data0;
	calc_data.als_data1  = data1;
	calc_data.gain_data0 = gain0;

	/* set max range */
	if (calc_data.gain_data0 == 0) 
	{
		/* issue error value when gain is 0 */
		return (CALC_ERROR);
	}
	else
	{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}
	
	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE) 
	{
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	} 
	else 
	{
		/* get the value which is measured from power table */
		calc_data.als_time = time;
		if (calc_data.als_time == 0) 
		{
			/* issue error value when time is 0 */
			return (CALC_ERROR);
		}

		calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
		if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])) 
		{
			set_case = 0;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1]))
		{
			set_case = 1;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2])) 
		{
			set_case = 2;
		}
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3])) 
		{
			 set_case = 3;
		} 
		else
		{
			set_case = MAXSET_CASE;
		}
		calc_ans.positive = 0;
		if (set_case >= MAXSET_CASE) 
		{
			calc_ans.decimal = 0;	//which means that lux output is 0
		}
		else
		{
			calc_data.gain_data1 = gain1;
			if (calc_data.gain_data1 == 0) 
			{
				/* issue error value when gain is 0 */
				return (CALC_ERROR);
			}
			calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
			calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
			if(calc_data.data0 < calc_data.data1)	//In this case, data will be less than 0. As data is unsigned long long, it will become extremely big.
			{
				return (CALC_ERROR);
			}
			else
			{
				calc_data.data       = (calc_data.data0 - calc_data.data1);
			}
			calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;	//24 bit at max (128 * 128 * 100 * 10)
			if (calc_data.dev_unit == 0) 
			{
				/* issue error value when dev_unit is 0 */
				return (CALC_ERROR);
			}

			/* calculate a positive number */
			div_answer   = 0;
			div_overplus = 0;

			result = long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
      			if (result == CALC_ERROR)
      			{
        			 return (result);
      			}
			calc_ans.positive = div_answer;
			/* calculate a decimal number */
			calc_ans.decimal = 0;
			overplus         = div_overplus;
			if (calc_ans.positive < max_range)
			{
				if (overplus != 0)
				{
					overplus     = overplus << DECIMAL_BIT;
					div_answer   = 0;
					div_overplus = 0;

					result = long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
					if (result == CALC_ERROR)
      					{
         					return (result);
      					}
					calc_ans.decimal = div_answer;
				}
			}

			else
			{
				calc_ans.positive = max_range;
			}
		}
	}
	
	final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);
					
	return (final_data);

#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}


/* delete rpr521_reschedule_work, we use queue_work to replase queue_delayed_work, because flush_delayed_work
   may cause system stop work */
/* ALS polling routine */
static void rpr521_als_polling_work_handler(struct work_struct *work)
{
	struct rpr521_data *data = container_of(work, struct rpr521_data,als_dwork);
	struct i2c_client *client=data->client;
	int ch0data, ch1data, pdata;
	int luxValue=0;
	int ret;

	unsigned char lux_is_valid=1;

	ch0data = rpr521_i2c_read(client,
			REG_ALSDATA0,RPR521_I2C_WORD); 
	ch1data = rpr521_i2c_read(client,
			REG_ALSDATA1,RPR521_I2C_WORD);
	pdata = rpr521_i2c_read(client,
			REG_PSDATA,RPR521_I2C_WORD);
	if(ch0data < 0 || ch1data < 0 || pdata < 0)
	{
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
	}
	else
	{
		luxValue = calc_rohm_als_data(ch0data, ch1data, gain0, gain1, atime); 
	}

	//if (luxValue >= 0)
	if ((luxValue >= 0) && (luxValue!=CALC_ERROR)) //grace modify in 2014.7.31
	{

		luxValue = luxValue < RPR521_LUX_MAX? luxValue : RPR521_LUX_MAX;

		data->als_prev_lux = luxValue;
	}
	else
	{
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
	}

	/*
	 * check PS under sunlight
	 * PS was previously in far-to-near condition
	 */

	if ((data->ps_detection == 1) && (ch0data >  RPR521_SUNLIGHT_CHODATA))
	{
		/*
		 * need to inform input event as there will be no interrupt
		 * from the PS
		 */
		/* NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);
		input_sync(data->input_dev_ps);
		data->ps_close_or_far = 1;
		RPR521_INFO("%s:PS under sunlight input_report_abs report ABS_DISTANCE, far event, data->ps_data:%d\n", __func__,data->ps_data);

		ret = rpr521_i2c_write(client,REG_PSTH,
				data->ps_min_threshold + rpr521_pwindow_value,RPR521_I2C_WORD); 
		ret += rpr521_i2c_write(client,REG_PSTL,0,RPR521_I2C_WORD); 
		if (ret < 0)
			return ;

		data->piht = data->ps_min_threshold + rpr521_pwindow_value;
		data->piht = 0;
		RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
		{
			cur_pthreshold_h = data->piht;
			cur_pthreshold_l = data->pilt;
			RPR521_INFO("%s:PS under sunlight data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, data->ps_data, data->pilt, data->piht);
		}

		data->ps_detection = 0;	/* near-to-far detected */

		RPR521_FLOW("%s:  proximity light FAR is detected\n", __func__);
	}

	if (lux_is_valid) {
		/* report the lux level */
		input_report_abs(data->input_dev_als, ABS_MISC, luxValue);
		input_sync(data->input_dev_als);
		RPR521_FLOW("%s,line %d:rpr521 lux=%d\n",__func__,__LINE__,luxValue);
	}
	
 	RPR521_FLOW("%s: line:%d rpr521 light ch0data=%d, ch1data=%d, luxValue=%d, mode=%x\n", __func__, __LINE__, ch0data, ch1data,luxValue,rpr521_i2c_read(client,
			REG_ALSPSCONTROL,RPR521_I2C_BYTE)); //grace modify in 2014.7.31
	/* restart timer */
	/* start a work after 200ms */
	
	if (0 != hrtimer_start(&data->timer,
							ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL) )
	{
		RPR521_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
	}
}

/*****************************************************************
Parameters    :  timer
Return        :  HRTIMER_NORESTART
Description   :  hrtimer_start call back function,
				 use to report als data
*****************************************************************/
static enum hrtimer_restart rpr521_als_timer_func(struct hrtimer *timer)
{
	struct rpr521_data* data = container_of(timer,struct rpr521_data,timer); //wanghang modify
	queue_work(rpr521_workqueue, &data->als_dwork);  //wanghang modify
	return HRTIMER_NORESTART;
}

/* PS interrupt routine */

static void rpr521_work_handler(struct work_struct *work)
{
	struct rpr521_data *data = container_of(work, struct rpr521_data, dwork);
	struct i2c_client *client=data->client;
	int status;
	int ch0data;
	int control,mode,pers; 
	int judge_ch0; //grace modify in 2014.7.31

	status = rpr521_i2c_read(client, REG_INTERRUPT,RPR521_I2C_BYTE);
	
	RPR521_INFO("%s: line:%d rpr521 interrupt handler status =%x\n", __func__, __LINE__, status); 
	
	if(rpr521_debug_mask > 1)
	{
		control=rpr521_i2c_read(client, REG_ALSPSCONTROL,RPR521_I2C_BYTE);
		mode = rpr521_i2c_read(client, REG_MODECONTROL,RPR521_I2C_BYTE);
		pers= rpr521_i2c_read(client, REG_PERSISTENCE,RPR521_I2C_BYTE);

		RPR521_FLOW("%s,line %d:status = 0x%x\n",__func__,__LINE__,status);
		RPR521_FLOW("%s,line %d:control = 0x%x,mode=0x%x,pers=0x%x\n",__func__,__LINE__,control,mode,pers);
	}
	
	if ((status & PS_INT_MASK) == PS_INT_MASK) { 
		/* only PS is interrupted */
		RPR521_FLOW("%s,line %d:only PLS is detected.\n",__func__,__LINE__);
		/* check if this is triggered by background ambient noise */
		ch0data = rpr521_i2c_read(client,
				REG_ALSDATA0,RPR521_I2C_WORD); 
		RPR521_FLOW("%s,line %d:rpr521_ps_report_event, ch0data:%d\n", __func__,__LINE__,ch0data);

#if 0 //grace modify in 2014.7.31
		/*In the case of ambient light never turned on, ch0data=0*/
		if ((ch0data >= 0) && (ch0data < RPR521_SUNLIGHT_CHODATA))
		{
			rpr521_ps_report_event(client);
		}
		else if(ch0data > RPR521_SUNLIGHT_CHODATA)
		{
			if (data->ps_detection == 1){
				rpr521_ps_report_event(client);
			}
			else
				RPR521_INFO("%s: background ambient noise\n",__func__);
		}
		else
		{
			RPR521_ERR("%s,line%d: CH0DATA WRONG\n",__func__,__LINE__);
		}
#else
		mode = rpr521_i2c_read(client, REG_MODECONTROL,RPR521_I2C_BYTE); 
		judge_ch0 = mode & ALS_EN; //judge whether ALS is ON.
		if(!judge_ch0)
			ch0data = 0;
		/*In the case of ambient light never turned on, ch0data=0*/
		if ((ch0data >= 0) && (ch0data < RPR521_SUNLIGHT_CHODATA))
		{
			rpr521_ps_report_event(client);
		}
		else if(ch0data > RPR521_SUNLIGHT_CHODATA)
		{
			if (data->ps_detection == 1){
				rpr521_ps_report_event(client);
			}
			else
				RPR521_INFO("%s: background ambient noise\n",__func__);
		}
		else
		{
			RPR521_ERR("%s,line%d: CH0DATA WRONG\n",__func__,__LINE__);
		}
#endif

	} else{
		RPR521_ERR("%s,line %d:wrong interrupts,RPR521_STATUS_REG is 0X%x\n",__func__,__LINE__,status);
	}

	if (data->irq)
	{
		enable_irq(data->irq);
	}
}

/* assume this is ISR */
static irqreturn_t rpr521_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct rpr521_data *data = i2c_get_clientdata(client);

	disable_irq_nosync(data->irq);
	/* and ps data report function to workqueue */
	queue_work(rpr521_workqueue, &data->dwork);

	RPR521_INFO("%s: line:%d rpr521 interrupt\n", __func__, __LINE__); //grace modify in 2014.7.28

	return IRQ_HANDLED;
}

/*
 * IOCTL support
 */
static int rpr521_enable_als_sensor(struct i2c_client *client, int val)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	struct rpr521_platform_data *pdata = data->platform_data;
	int ret;
	/*Not in use power_value*/

	RPR521_FLOW("%s,line %d:enable als val=%d\n",__func__,__LINE__,val);
	/*Not in use power_value*/

	if (val == 1) {
		/* turn on light  sensor */
		if (data->enable_als_sensor == 0) {
			if(data->enable_ps_sensor == 0){

				/* Power on and initalize the device */
				if (pdata->power_on)
					pdata->power_on(true,data);

				ret = rpr521_init_client(client);
				if (ret) {
					RPR521_ERR("%s:line:%d,Failed to init rpr521\n", __func__, __LINE__);
					return ret;
				}
			}
			data->enable_als_sensor = 1;
			reg0_value = reg0_value|ALS_EN; 
			rpr521_set_enable(client, reg0_value);

			RPR521_INFO("%s: line:%d enable als sensor,reg0_value=%d\n", __func__, __LINE__, reg0_value);
			/* enable als sensor, start data report hrtimer */
			ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
			if (ret != 0) {
				RPR521_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
			}
		}
	} else {
		/*
		 * turn off light sensor
		 * what if the p sensor is active?
		 */
		 if(data->enable_als_sensor == 1)
		 {
			data->enable_als_sensor = 0;

			reg0_value = reg0_value&(~ALS_EN);
			rpr521_set_enable(client, reg0_value);

			RPR521_INFO("%s: line:%d,disable als sensor,reg0_value = 0x%x\n", __func__, __LINE__,reg0_value);
			/* disable als sensor, cancne data report hrtimer */
			hrtimer_cancel(&data->timer);
		 }

	}
	/* Vote off  regulators if both light and prox sensor are off */
	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0) &&(pdata->power_on)){
		pdata->power_on(false,data);
	}

	RPR521_FLOW("%s: line:%d,enable als sensor success\n", __func__, __LINE__);
	return 0;
}
static int rpr521_open_ps_sensor(struct rpr521_data *data, struct i2c_client *client)
{
	int ret = 0;
	/* turn on p sensor */
	if (data->enable_ps_sensor==0) {
		if(data->enable_als_sensor == 0){
			/* Power on and initalize the device */
			if (data->platform_data->power_on)
				data->platform_data->power_on(true,data);

			ret = rpr521_init_client(client);
			if (ret) {
				RPR521_ERR("%s:line:%d,Failed to init rpr521\n", __func__, __LINE__);
				return ret;
			}
		}

		data->enable_ps_sensor= 1;
		/*initialize the ps_min_threshold,to update data->piht and data->pilt*/
		data->ps_min_threshold = origin_prox;
		RPR521_FLOW("%s,line %d:change threshoid,data->ps_min_threshold =%d\n",__func__,__LINE__,data->ps_min_threshold);
		ret = rpr521_i2c_write(client,REG_PSTL,far_init,RPR521_I2C_WORD); 
		ret += rpr521_i2c_write(client,REG_PSTH,near_init,RPR521_I2C_WORD);
		if (ret < 0)
			return ret;
		RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);

		/*we use our own calibration algorithm,more details of the algorithm you can check rpr521_ps_report_event*/
		reg0_value = reg0_value |PS_EN;
		rpr521_set_enable(client, reg0_value);
		RPR521_INFO("%s: line:%d,enable pls sensor.reg0_value = 0x%x\n", __func__, __LINE__,reg0_value);
		/* 0 is close, 1 is far */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);
		input_sync(data->input_dev_ps);
		data->ps_close_or_far = 1;
		RPR521_INFO("%s,line %d:input_report_abs report ABS_DISTANCE, far event, data->ps_data:%d\n", __func__,__LINE__,data->ps_data);

		if (data->irq)
		{
			enable_irq(data->irq);
			/*set the property of pls irq,so the pls irq can wake up the sleeping system */
			irq_set_irq_wake(data->irq, 1);
		}
	}
	return ret;
}
static int rpr521_enable_ps_sensor(struct i2c_client *client,unsigned int val)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;
	/*Not in use power_value*/

	RPR521_FLOW("%s,line %d:val=%d\n",__func__,__LINE__,val);
	if ((val != 0) && (val != 1)) {
		RPR521_ERR("%s: invalid value=%d\n", __func__, val);
		return -EINVAL;
	}
	/*Not in use power_value*/
	if (val == 1) {
		ret = rpr521_open_ps_sensor(data, client);
		if(ret)
		{
			RPR521_ERR("%s,line %d:read power_value failed,open ps fail\n",__func__,__LINE__);
			return ret;
		}
	} else {
		/*
		 * turn off p sensor - kk 25 Apr 2011
		 * we can't turn off the entire sensor,
		 * the light sensor may be needed by HAL
		 */
		if (data->enable_ps_sensor==1) {

			data->enable_ps_sensor = 0;
			
			reg0_value = reg0_value &(~PS_EN);
			rpr521_set_enable(client, reg0_value);
			
			RPR521_INFO("%s: line:%d,disable pls sensor,reg0_value = 0x%x\n", __func__, __LINE__,reg0_value);
			if (data->irq)
			{
				/*when close the pls,make the wakeup property diabled*/
				irq_set_irq_wake(data->irq, 0);
				disable_irq(data->irq);
			}
		}
	}
	/* Vote off  regulators if both light and prox sensor are off */
	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0) &&(data->platform_data->power_on)){
		data->platform_data->power_on(false,data);
	}
	return 0;
}
/*
 * SysFS support
 */
 static int rpr521_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	static int als_enalbe_count=0;

	struct rpr521_data *data = container_of(sensors_cdev,struct rpr521_data, als_cdev);
	if ((enable != 0) && (enable != 1)) {
		RPR521_ERR("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}
	RPR521_FLOW("%s,line %d:rpr521 als enable=%d\n",__func__,__LINE__,enable);

	/*for debug and print registers value when enable/disable the als every time*/
	if(enable == 0)
	{
		if(rpr521_debug_mask >= 1){
		RPR521_FLOW("attention:before als_disable %d times\n", als_enalbe_count);
			rpr521_regs_debug_print(data,enable);
		}
		rpr521_enable_als_sensor(data->client, enable);

	}else{

		rpr521_enable_als_sensor(data->client, enable);

		if(rpr521_debug_mask >= 1){
		RPR521_FLOW("attention: after als_enable %d times\n",++als_enalbe_count);
			rpr521_regs_debug_print(data,enable);
		}
	 }
	 return ret;
}

static int rpr521_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct rpr521_data *data = container_of(sensors_cdev,
			struct rpr521_data, ps_cdev);
	if ((enable != 0) && (enable != 1)) {
		RPR521_ERR("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}
	RPR521_FLOW("%s,line %d:rpr521 enable ps value=(%d)\n",__func__,__LINE__, enable);
	return rpr521_enable_ps_sensor(data->client, enable);
}
/*use this function to reset the poll_delay time(ms),val is the time parameter*/
static int rpr521_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;
	//int atime_index;

	/* minimum 10ms */
	if (val < 10)
		val = 10;
	data->als_poll_delay = 200;
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->als_dwork);
	ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
	if (ret != 0) {
		RPR521_ERR("%s,line%d: hrtimer_start fail! nsec=%d\n", __func__, __LINE__,data->als_poll_delay);
		return ret;
	}
	return 0;
}

static int rpr521_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct rpr521_data *data = container_of(sensors_cdev,
			struct rpr521_data, als_cdev);
	rpr521_set_als_poll_delay(data->client, delay_msec);
	return 0;
}
static ssize_t rpr521_show_ch0data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ch0data;

	ch0data = rpr521_i2c_read(client,REG_ALSDATA0,RPR521_I2C_WORD);

	return snprintf(buf,32,"%d\n", ch0data);
}

static DEVICE_ATTR(ch0data, S_IRUGO, rpr521_show_ch0data, NULL);

static ssize_t rpr521_show_ch1data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct input_dev *input = to_input_dev(dev);
	struct rpr521_data *data = input_get_drvdata(input);
	int ch1data;

	ch1data = rpr521_i2c_read(data->client,REG_ALSDATA1,RPR521_I2C_WORD);

	return snprintf(buf,32, "%d\n", ch1data);
}

static DEVICE_ATTR(ch1data, S_IRUGO, rpr521_show_ch1data, NULL);

static ssize_t rpr521_show_pdata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int pdata;

	pdata = rpr521_i2c_read(client, REG_PSDATA,RPR521_I2C_WORD);
	if(pdata <0){
		RPR521_ERR("%s,line %d:read pdata failed\n",__func__,__LINE__);
	}

	return snprintf(buf,32, "%d\n", pdata);
}

static DEVICE_ATTR(pdata, S_IRUGO, rpr521_show_pdata, NULL);


/*
* set the register's value from userspace
* Usage: echo "0x08|0x12" > dump_reg
*			"reg_address|reg_value"
*/
static ssize_t rpr521_write_reg(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rpr521_data *data = i2c_get_clientdata(client);
	int val_len_max = 4;
	char* input_str =NULL;
	char reg_addr_str[10]={'\0'};
	char reg_val_str[10]={'\0'};
	long reg_addr,reg_val;
	int addr_lenth=0,value_lenth=0,buf_len=0,ret = -1;
	char* strtok=NULL;

	buf_len = strlen(buf);
	input_str = kzalloc(buf_len, GFP_KERNEL);
	if (!input_str)
	{
		pr_err("%s:kmalloc fail!\n",__func__);
		return -ENOMEM;
	}

	mutex_lock(&data->update_lock);
	snprintf(input_str, 10,"%s", buf);
	/*Split the string when encounter "|", for example "0x08|0x12" will be splited "0x18" "0x12" */
	strtok=strsep(&input_str, "|");
	if(strtok!=NULL)
	{
		addr_lenth = strlen(strtok);
		memcpy(reg_addr_str,strtok,((addr_lenth > (val_len_max))?(val_len_max):addr_lenth));
	}
	else
	{
		pr_err("%s: buf name Invalid:%s", __func__,buf);
		goto parse_fail_exit;
	}
	strtok=strsep(&input_str, "|");
	if(strtok!=NULL)
	{
		value_lenth = strlen(strtok);
		memcpy(reg_val_str,strtok,((value_lenth > (val_len_max))?(val_len_max):value_lenth));
	}
	else
	{
		pr_err("%s: buf value Invalid:%s", __func__,buf);
		goto parse_fail_exit;
	}

	/* transform string to long int */
	ret = kstrtol(reg_addr_str,16,&reg_addr);
	if(ret)
		goto parse_fail_exit;

	ret = kstrtol(reg_val_str,16,&reg_val);
	if(ret)
		goto parse_fail_exit;

	/* write the parsed value in the register*/
	ret = rpr521_i2c_write(client,(char)reg_addr,(char)reg_val,RPR521_I2C_BYTE);
	if (ret < 0)
		goto parse_fail_exit;
	return count;

parse_fail_exit:
	mutex_unlock(&data->update_lock);
	if (input_str)
		kfree(input_str);

	return ret;
}

/*
* show all registers' value to userspace
*/
static ssize_t rpr521_print_reg_buf(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i;
	char reg[RPR521_REG_LEN];
	int regs = 0;
	struct i2c_client *client = to_i2c_client(dev);

	/* read all register value and print to user*/
	for(i = 0; i < RPR521_REG_LEN; i++ )
	{
		regs = 0x40+i;
		reg[i] = rpr521_i2c_read(client,regs,RPR521_I2C_BYTE); 
		if(reg[i] <0){
			RPR521_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg[i] ;
		}
	}

	return snprintf(buf,512,"reg[0x40~0x48]=0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n"
				      "reg[0x49~0x4e]0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n",
			reg[0x0],reg[0x1],reg[0x2],reg[0x3],reg[0x4],reg[0x5],reg[0x6],reg[0x7],reg[0x8],
			reg[0x9],reg[0xa],reg[0xb],reg[0xc],reg[0xd],reg[0xe]); 
}

static DEVICE_ATTR(dump_reg ,S_IRUGO|S_IWUSR|S_IWGRP, rpr521_print_reg_buf, rpr521_write_reg);

static struct attribute *rpr521_attributes[] = {
	&dev_attr_ch0data.attr,
	&dev_attr_ch1data.attr,
	&dev_attr_pdata.attr,
	&dev_attr_dump_reg.attr,
	NULL
};

static const struct attribute_group rpr521_attr_group = {
	.attrs = rpr521_attributes,
};

/*
 * Initialization function
 */
static int rpr521_read_device_id(struct i2c_client *client)
{
	int id;
	int err;

	id = rpr521_i2c_read(client, REG_SYSTEMCONTROL,RPR521_I2C_BYTE);
	if (id == 0xa) {
		RPR521_INFO("%s: RPR521\n", __func__);
		err = app_info_set("LP-Sensor", "RPR521");
		if (err < 0)/*failed to add app_info*/
		{
		    RPR521_ERR("%s %d:failed to add app_info\n", __func__, __LINE__);
		}
	} else {
		RPR521_INFO("%s: Neither RPR521 \n", __func__);
		return -ENODEV;
	}
	return 0;
}
static int rpr521_init_client(struct i2c_client *client)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int err;

	err = rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_enable FAIL ",__func__,__LINE__);
		return err;
	}

	err = rpr521_set_config(client, PS_ALS_SET_INTR);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_config FAIL ",__func__,__LINE__);
		return err;
	}

	err = rpr521_set_control(client,PS_ALS_SET_ALSPS_CONTROL);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_control FAIL ",__func__,__LINE__);
		return err;
	}

	/* init threshold for proximity */
	err = rpr521_set_pilt(client, rpr521_ps_init_threshold-1);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_pilt FAIL ",__func__,__LINE__);
		return err;
	}

	err = rpr521_set_piht(client, rpr521_ps_init_threshold);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_piht FAIL ",__func__,__LINE__);
		return err;
	}

	data->ps_detection = 0; /* initial value = far*/


	/* 1 consecutive Interrupt persistence */
	err = rpr521_set_pers(client, PS_ALS_SET_INTR_PERSIST);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_pers FAIL ",__func__,__LINE__);
		return err;
	}

	/* sensor is in disabled mode but all the configurations are preset */
	return 0;
}
/*qualcom updated the regulator configure functions and we add them all*/
static int sensor_regulator_configure(struct rpr521_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				RPR521_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				RPR521_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			RPR521_ERR("%s,line%d:Regulator get failed vdd rc=%d\n",__func__,__LINE__, rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				RPR521_VDD_MIN_UV, RPR521_VDD_MAX_UV);
			if (rc) {
				RPR521_ERR("%s,line%d:Regulator set failed vdd rc=%d\n",__func__,__LINE__,rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			RPR521_ERR("%s,line%d:Regulator get failed vio rc=%d\n",__func__,__LINE__, rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				RPR521_VIO_MIN_UV, RPR521_VIO_MAX_UV);
			if (rc) {
				RPR521_ERR("%s,line%d:Regulator set failed vio rc=%d\n",__func__,__LINE__, rc);
				goto reg_vio_put;
			}
		}

	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, RPR521_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}
/*In suspend and resume function,we only control the als,leave pls alone*/
static int rpr521_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int rc;

	RPR521_FLOW("%s,line%d:RPR521 SUSPEND\n",__func__,__LINE__);
	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->als_dwork);
	/*
	* Save sensor state and disable them,
	* this is to ensure internal state flags are set correctly.
	* device will power off after both sensors are disabled.
	* P sensor will not be disabled because it  is a wakeup sensor.
	*/
	data->enable_als_state = data->enable_als_sensor;

	if(data->enable_als_sensor){
		rc = rpr521_enable_als_sensor(data->client, 0);
		if (rc){
			RPR521_ERR("%s,line%d:Disable light sensor fail! rc=%d\n",__func__,__LINE__, rc);
		}
	}

	return 0;
}

static int rpr521_resume(struct i2c_client *client)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret = 0;

	RPR521_FLOW("%s,line%d:RPR521 RESUME\n",__func__,__LINE__);
	if (data->enable_als_state) {
		ret = rpr521_enable_als_sensor(data->client, 1);
		if (ret){
			RPR521_ERR("%s,line%d:Disable light sensor fail! rc=%d\n",__func__,__LINE__, ret);
		}
	}

	return 0;
}
/*pamameter subfunction of probe to reduce the complexity of probe function*/
static int rpr521_sensorclass_init(struct rpr521_data *data,struct i2c_client* client)
{
	int err;
	/* Register to sensors class */
	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = rpr521_als_set_enable;
	data->als_cdev.sensors_poll_delay = rpr521_als_poll_delay;

	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = rpr521_ps_set_enable;
	data->ps_cdev.sensors_poll_delay = NULL;

	err = sensors_classdev_register(&client->dev, &data->als_cdev);
	if (err) {
		RPR521_ERR("%s: Unable to register to sensors class: %d\n",__func__, err);
		goto exit;
	}
	err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if (err) {
		RPR521_ERR("%s: Unable to register to sensors class: %d\n",__func__, err);
		goto exit_unregister_als_class;
	}
	goto exit;
exit_unregister_als_class:
	sensors_classdev_unregister(&data->als_cdev);
exit:
	return err;
}
static void rpr521_parameter_init(struct rpr521_data *data)
{
	/* Set the default parameters */
	
	rpr521_pwave_value= data->platform_data->pwave;
	rpr521_pwindow_value= data->platform_data->pwindow;

	data->enable = 0;	/* default mode is standard */
	data->ps_min_threshold = origin_prox;
	RPR521_FLOW("%s:set origin_prox to data->ps_min_threshold=%d\n", __func__, data->ps_min_threshold);
	data->enable_als_sensor = 0;	// default to 0
	data->enable_ps_sensor = 0;	// default to 0
	data->als_poll_delay = 200;	// default to 200ms
	data->als_prev_lux = 300;
}
/*input init subfunction of probe to reduce the complexity of probe function*/
static int rpr521_input_init(struct rpr521_data *data)
{
	int err = 0;
	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		RPR521_ERR("%s: Failed to allocate input device als\n", __func__);
		goto exit;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		input_free_device(data->input_dev_als);
		RPR521_ERR("%s: Failed to allocate input device ps\n", __func__);
		goto exit;
	}

	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 10000, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 5, 0, 0);

	data->input_dev_als->name = "light";
	data->input_dev_ps->name = "proximity";

	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		input_free_device(data->input_dev_als);
		input_free_device(data->input_dev_ps);
		RPR521_ERR("%s: Unable to register input device als: %s\n",
				__func__, data->input_dev_als->name);
		goto exit;
	}

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		RPR521_ERR("%s: Unable to register input device ps: %s\n",
				__func__, data->input_dev_ps->name);
		goto unregister_als;
	}
	goto exit;
unregister_als:
	input_unregister_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);
exit:
	return err;
}
/*irq request subfunction of probe to reduce the complexity of probe function*/
static int rpr521_irq_init(struct rpr521_data *data,struct i2c_client *client)
{
	int ret = 0;
	if (data->platform_data->irq_gpio)
	{
		ret = gpio_request(data->platform_data->irq_gpio,"rpr521_irq_gpio");
		if (ret)
		{
			RPR521_ERR("%s, line %d:unable to request gpio [%d]\n", __func__, __LINE__,data->platform_data->irq_gpio);
			return ret;
		}
		else
		{
			ret = gpio_direction_input(data->platform_data->irq_gpio);
			if(ret)
			{
				RPR521_ERR("%s, line %d: Failed to set gpio %d direction\n", __func__, __LINE__,data->platform_data->irq_gpio);
				return ret;
			}
		}
	}
	client->irq = gpio_to_irq(data->platform_data->irq_gpio);
	if (client->irq < 0) {
		ret = -EINVAL;
		RPR521_ERR("%s, line %d:gpio_to_irq FAIL! IRQ=%d\n", __func__, __LINE__,data->platform_data->irq_gpio);
		return ret;
	}
	data->irq = client->irq;
	if (client->irq)
	{
		if (request_irq(data->irq, rpr521_interrupt,IRQF_TRIGGER_FALLING|IRQF_ONESHOT|IRQF_NO_SUSPEND, RPR521_DRV_NAME, (void *)client) >= 0)
		{
			RPR521_FLOW("%s, line %d:Received IRQ!\n", __func__, __LINE__);
			disable_irq(data->irq);
		}
		else
		{
			RPR521_ERR("%s, line %d:Failed to request IRQ!\n", __func__, __LINE__);
			ret = -EINVAL;
			return ret;
		}
	}
	return ret;
}
static int sensor_regulator_power_on(struct rpr521_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			RPR521_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			RPR521_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			rc = regulator_enable(data->vdd);
			RPR521_ERR("%s:Regulator vio re-enabled rc=%d\n",__func__, rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			RPR521_ERR("%s:Regulator vdd enable failed rc=%d\n",__func__, rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			RPR521_ERR("%s:Regulator vio enable failed rc=%d\n", __func__,rc);
			rc = regulator_disable(data->vdd);
			return rc;
		}
	}
enable_delay:
	msleep(130);
	RPR521_FLOW("%s:Sensor regulator power on =%d\n",__func__, on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on,struct rpr521_data *data)
{
	int err = 0;

	if (data->power_on != on) {
		if (!IS_ERR_OR_NULL(data->pinctrl)) {
			if (on)
				/*after poweron,set the INT pin the default state*/
				err = pinctrl_select_state(data->pinctrl,
					data->pin_default);
			if (err)
				RPR521_ERR("%s,line%d:Can't select pinctrl state\n", __func__, __LINE__);
		}

		err = sensor_regulator_power_on(data, on);
		if (err)
			RPR521_ERR("%s,line%d:Can't configure regulator!\n", __func__, __LINE__);
		else
			data->power_on = on;
	}

	return err;
}
static int sensor_platform_hw_init(struct rpr521_data *data)
{
	int error;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		RPR521_ERR("%s,line %d:unable to configure regulator\n",__func__,__LINE__);
		return error;
	}

	return 0;
}

static void sensor_platform_hw_exit(struct rpr521_data *data)
{
	sensor_regulator_configure(data, false);

	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);
}
static int rpr521_pinctrl_init(struct rpr521_data *data)
{
	struct i2c_client *client = data->client;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		RPR521_ERR("%s,line %d:Failed to get pinctrl\n",__func__,__LINE__);
		return PTR_ERR(data->pinctrl);
	}
	/*we have not set the sleep state of INT pin*/
	data->pin_default =
		pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		RPR521_ERR("%s,line %d:Failed to look up default state\n",__func__,__LINE__);
		return PTR_ERR(data->pin_default);
	}

	return 0;
}

static unsigned long *create_and_get_array(struct device_node *dev_node,
		const char *name, int *size)
{
	const __be32 *values;
	unsigned long *val_array;
	int len;
	int sz;
	int rc;
	int i;

	values = of_get_property(dev_node, name, &len);
	if (values == NULL)
		return NULL;

	sz = len / sizeof(u32);
	pr_debug("%s: %s size:%d\n", __func__, name, sz);

	val_array = kzalloc(sz * sizeof(unsigned long), GFP_KERNEL);
	if (val_array == NULL) {
		rc = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < sz; i++)
		val_array[i] = (unsigned long)be32_to_cpup(values++);

	*size = sz;

	return val_array;

fail:
	return ERR_PTR(rc);
}


static unsigned long *create_and_get_als_judge_array(struct device_node *dev_node)
{
	unsigned long *judge;
	int size;
	int rc;

	judge = create_and_get_array(dev_node, "rpr521,judge_array", &size);
	if (IS_ERR_OR_NULL(judge))
		return (void *)judge;

	/* Check for valid judge size */
	if (size % COEFFICIENT) {
		rc = -EINVAL;
		goto fail_free_judge;
	}

	return judge;

fail_free_judge:
	kfree(judge);

	return ERR_PTR(rc);
}


static unsigned long *create_and_get_als_data1_array(struct device_node *dev_node)
{
	unsigned long *data1;
	int size;
	int rc;

	data1 = create_and_get_array(dev_node, "rpr521,data1_array", &size);
	if (IS_ERR_OR_NULL(data1))
		return (void *)data1;

	/* Check for valid data1 size */
	if (size % COEFFICIENT) {
		rc = -EINVAL;
		goto fail_free_data1;
	}

	return data1;

fail_free_data1:
	kfree(data1);

	return ERR_PTR(rc);
}


static unsigned long *create_and_get_als_data0_array(struct device_node *dev_node)
{
	unsigned long *data0;
	int size;
	int rc;

	data0 = create_and_get_array(dev_node, "rpr521,data0_array", &size);
	if (IS_ERR_OR_NULL(data0))
		return (void *)data0;

	/* Check for valid data0 size */
	if (size % COEFFICIENT) {
		rc = -EINVAL;
		goto fail_free_data0;
	}

	return data0;

fail_free_data0:
	kfree(data0);

	return ERR_PTR(rc);
}


static int sensor_parse_dt(struct device *dev,
		struct rpr521_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	unsigned int tmp;
	int rc = 0;

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;

	/* irq gpio */

	rc = of_get_named_gpio_flags(dev->of_node, "rpr521,irq-gpio", 0, NULL);
    	if (rc < 0)
    	{
        	RPR521_ERR("Unable to read irq gpio\n");
        	return rc;
    	}
	pdata->irq_gpio = rc;

	rc = of_property_read_u32(np, "rpr521,wave", &tmp);
	if (rc) {
		RPR521_ERR("Unable to read pwave_value\n");
		return rc;
	}
	pdata ->pwave= tmp;
	
	rc = of_property_read_u32(np, "rpr521,window", &tmp);
	if (rc) {
		RPR521_ERR("Unable to read pwindow_value\n");
		return rc;
	}
	pdata ->pwindow= tmp;

	//read data0 from dtsi for calculate als data
	data0_coefficient = create_and_get_als_data0_array(np);
	if (data0_coefficient == NULL)
	{
	        rc = -EINVAL;
		RPR521_ERR("Unable to read data0_array,  rc = %d \n",rc);
		return rc;
	}

	data1_coefficient = create_and_get_als_data1_array(np);
	if (data1_coefficient == NULL)
	{
	        rc = -EINVAL;
		RPR521_ERR("Unable to read data1_array,  rc = %d \n",rc);
		return rc;
	}

	judge_coefficient = create_and_get_als_judge_array(np);
	if (judge_coefficient == NULL)
	{
	        rc = -EINVAL;
		RPR521_ERR("Unable to read judge_array,  rc = %d \n",rc);
		return rc;
	}
	
	return 0;
}


/*
 * I2C init/probing/exit functions
 */
static struct i2c_driver rpr521_driver;
static int rpr521_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rpr521_data *data;
	struct rpr521_platform_data *pdata;
	int err = 0;

	RPR521_INFO("%s,line %d:PROBE START.\n",__func__,__LINE__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct rpr521_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			RPR521_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
			err =-ENOMEM;
			goto exit;
		}

		client->dev.platform_data = pdata;
		err = sensor_parse_dt(&client->dev, pdata);
		if (err) {
			RPR521_ERR("%s: sensor_parse_dt() err\n", __func__);
			goto exit_parse_dt;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			RPR521_ERR("%s,line %d:No platform data\n",__func__,__LINE__);
			err = -ENODEV;
			goto exit;
		}
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		RPR521_ERR("%s,line %d:Failed to i2c_check_functionality\n",__func__,__LINE__);
		err = -EIO;
		goto exit_parse_dt;
	}

	data = kzalloc(sizeof(struct rpr521_data), GFP_KERNEL);
	if (!data) {
		RPR521_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
		err = -ENOMEM;
		goto exit_parse_dt;
	}

	data->platform_data = pdata;
	data->client = client;

	/* h/w initialization */
	if (pdata->init)
		err = pdata->init(data);

	if (pdata->power_on)
		err = pdata->power_on(true,data);
	i2c_set_clientdata(client, data);
	rpr521_parameter_init(data);
	/* initialize pinctrl */
	err = rpr521_pinctrl_init(data);
	if (err) {
		RPR521_ERR("%s,line %d:Can't initialize pinctrl\n",__func__,__LINE__);
			goto exit_uninit;
	}
	err = pinctrl_select_state(data->pinctrl, data->pin_default);
	if (err) {
		RPR521_ERR("%s,line %d:Can't select pinctrl default state\n",__func__,__LINE__);
		goto exit_uninit;
	}

	mutex_init(&data->update_lock);

	INIT_WORK(&data->dwork, rpr521_work_handler);

	INIT_WORK(&data->als_dwork, rpr521_als_polling_work_handler);
	/* Initialize the RPR521 chip and judge who am i*/
	err=rpr521_read_device_id(client);
	if (err) {
		RPR521_ERR("%s: Failed to read rpr521\n", __func__);
		goto exit_uninit;
	}
	err = rpr521_init_client(client);
	if (err) {
		RPR521_ERR("%s: Failed to init rpr521\n", __func__);
		goto exit_uninit;
	}

	err = rpr521_input_init(data);
	if(err)
		goto exit_uninit;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &rpr521_attr_group);
	if (err)
		goto exit_unregister_dev_ps;

	wake_lock_init(&data->ps_report_wk, WAKE_LOCK_SUSPEND, "psensor_wakelock");

	err=rpr521_irq_init(data,client);
	if(err)
		goto exit_remove_sysfs_group;

	device_init_wakeup(&(client->dev), true);

	err = rpr521_sensorclass_init(data,client);
	if (err) {
		RPR521_ERR("%s: Unable to register to sensors class: %d\n",
	__func__, err);
		goto exit_free_irq;
	}

	rpr521_workqueue = create_workqueue("rpr521_work_queue");
	if (!rpr521_workqueue)
	{
		RPR521_ERR("%s: Create ps_workqueue fail.\n", __func__);
		goto exit_free_irq;
	}

	/* init hrtimer and call back function */
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = rpr521_als_timer_func;
	set_sensors_list(L_SENSOR + P_SENSOR);
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_APS);
	set_hw_dev_flag(DEV_I2C_L_SENSOR);
#endif
	err = set_sensor_input(PS, data->input_dev_ps->dev.kobj.name);
	if (err) {
		RPR521_ERR("%s set_sensor_input PS failed\n", __func__);
	}
	err = set_sensor_input(ALS, data->input_dev_als->dev.kobj.name);
	if (err) {
		RPR521_ERR("%s set_sensor_input ALS failed\n", __func__);
	}

	rpr521_regs_debug_print(data,1);

	//if (pdata->power_on)
		//err = pdata->power_on(false,data);
	RPR521_INFO("%s: Support ver. %s enabled\n", __func__, DRIVER_VERSION);

	return 0;
	/* delete this,use one work queue*/
exit_free_irq:
	free_irq(data->irq, client);
exit_remove_sysfs_group:
	wake_lock_destroy(&data->ps_report_wk);
	sysfs_remove_group(&client->dev.kobj, &rpr521_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
	input_unregister_device(data->input_dev_als);
exit_uninit:
	if (pdata->power_on)
		pdata->power_on(false,data);
	if (pdata->exit)
		pdata->exit(data);
	kfree(data);
exit_parse_dt:
	/*remove it,because devm_kzalloc will release the memory automatically
	if don't remove,there will be null pointer which lead to booting crush*/
exit:
	return err;
}

static int rpr521_remove(struct i2c_client *client)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	struct rpr521_platform_data *pdata = data->platform_data;

	/* Power down the device */
	rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);

	wake_lock_destroy(&data->ps_report_wk);
	sysfs_remove_group(&client->dev.kobj, &rpr521_attr_group);

	input_unregister_device(data->input_dev_ps);
	input_unregister_device(data->input_dev_als);

	free_irq(client->irq, data);
	hrtimer_cancel(&data->timer);
	if (pdata->power_on)
		pdata->power_on(false,data);

	if (pdata->exit)
		pdata->exit(data);

	kfree(data);

	return 0;
}

static const struct i2c_device_id rpr521_id[] = {
	{ "rpr521", 0 }, 
	{ }
};
MODULE_DEVICE_TABLE(i2c, rpr521_id);

static struct of_device_id rpr521_match_table[] = {
	{ .compatible = "rohm,rpr521",}, 
	{ },
};

static struct i2c_driver rpr521_driver = {
	.driver = {
		.name   = RPR521_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = rpr521_match_table,
	},
	.probe  = rpr521_probe,
	.remove = rpr521_remove,
	.suspend = rpr521_suspend,
	.resume = rpr521_resume,
	.id_table = rpr521_id,
};

static int __init rpr521_init(void)
{
	return i2c_add_driver(&rpr521_driver);
}

static void __exit rpr521_exit(void)
{
	/* destroy als and ps work queue */
	if (rpr521_workqueue) {
		destroy_workqueue(rpr521_workqueue);
		rpr521_workqueue = NULL;
	}

	i2c_del_driver(&rpr521_driver);
}

MODULE_DESCRIPTION("RPR521 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rpr521_init);
module_exit(rpr521_exit);
