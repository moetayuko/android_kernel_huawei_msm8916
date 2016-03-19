/* drivers/input/touchscreen/novatek/nvt.c
 *
 * Copyright (C) 2010 - 2014 Novatek, Inc.
 * 
 * History:
 * V1.0 : First Released
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
 
#include "nvt.h"
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

struct i2c_client *nvt_client;
struct nvt_ts_data *ts;
struct workqueue_struct *nvt_wq;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif 

extern int nvt_flash_init(void);
extern void nvt_sysfs_init(void);
extern void auto_update(void);
extern void nvt_procfs_create(void);

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM]={
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

 //I2C Read & Write
int nvt_i2c_read(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];

	while(retries < 5){
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2){
			break;
		}
		retries++;
	}
	return ret;	
}

void nvt_i2c_write (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = data;		
	
	while(retries < 5){
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1){
			break;
		}
		retries++;
	}
	return;
}

 //IC Reset using GPIO trigger 
void nvt_hw_reset(void)
{
	gpio_direction_output(ts->reset_gpio, 1);	
	msleep(20);
	gpio_direction_output(ts->reset_gpio, 0);	
	msleep(10);
	gpio_direction_output(ts->reset_gpio, 1);	
}

// Print Report Rate 
#if REPORT_RATE
int show_report_rate(void)
{
	struct timeval time_orgin;
	struct timeval time_now;
	int touch_count;
	if(touch_count==0){
		do_gettimeofday(&time_orgin);
	}
	do_gettimeofday(&time_now);
	if(time_now.tv_sec>time_orgin.tv_sec){
		do_gettimeofday(&time_orgin);
		return 1;
	}		
	else{
		return 0;		
	}
}
#endif

#if MT_PROTOCOL_B
static void mt_protocol_b(uint8_t* point_data,uint8_t touch_num)
{
	int i;
	int finger_cnt=0;
	unsigned int position = 0;	
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned char input_w = 0;
	unsigned char input_id = 0;
	unsigned char input_Prs = 0;
	
	for(i=0; i<touch_num; i++){
		position = 1 + 6*i;		
		input_id = (unsigned int)(point_data[position+0]>>3)-1;

		if((point_data[position]&0x07) == 0x03){       // finger up (break)
			input_mt_slot(ts->input_dev, input_id);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
		else if(((point_data[position]&0x07) == 0x01) || ((point_data[position]&0x07) == 0x02)){		//finger down (enter&moving)
			input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
			input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
			input_w = (unsigned int)(point_data[position+4])*5;
			input_Prs = (unsigned int)(point_data[position+5])*5;

			if(input_w > 255){
				input_w = 255;
			}
			if(input_Prs > 255){
				input_Prs = 255;
			}

		#if XY_REMAPPING
			nvt_xy_mapping(&input_x, &input_y);
		#endif

			if((input_x < 0) || (input_y < 0)){
				continue;
			}
			if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max)){
				continue;
			}

			input_x = TOUCH_MAX_WIDTH - input_x -1 ;
			input_y = TOUCH_MAX_HEIGHT -input_y -1;
			
			input_mt_slot(ts->input_dev, input_id);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			
			finger_cnt++;
		}
	}
	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt>0));	//all finger up
}
#else
static void mt_protocol_a(uint8_t* point_data,uint8_t touch_num)
{
	int i;
	int finger_cnt=0;
	unsigned int position = 0;	
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned char input_w = 0;
	unsigned char input_id = 0;
	unsigned char input_Prs = 0;

  	for(i=0; i<touch_num; i++){
		position = 1 + 6*i;
		input_id = (unsigned int)(point_data[position+0]>>3)-1;

		if((point_data[position]&0x07) == 0x03){		// finger up (break)
			continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
		}
		else if(((point_data[position]&0x07) == 0x01) || ((point_data[position]&0x07) == 0x02)){	//finger down (enter&moving)			
			input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
			input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
			input_w = (unsigned int)(point_data[position+4])*5;
			input_Prs = (unsigned int)(point_data[position+5])*5;

			if(input_w > 255){
				input_w = 255;
			}
			if(input_Prs > 255){
				input_Prs = 255;
			}
			
		#if XY_REMAPPING
			nvt_xy_mapping(&input_x, &input_y);
		#endif

			if((input_x < 0) || (input_y < 0)){
				continue;
			}
			if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max)){
				continue;
			}
			
			input_x = TOUCH_MAX_WIDTH - input_x -1 ;
			input_y = TOUCH_MAX_HEIGHT -input_y -1;

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_Prs);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_mt_sync(ts->input_dev);

			finger_cnt++;
		}
  	}	
	if(finger_cnt == 0)
	{	
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);			
		input_report_key(ts->input_dev, BTN_TOUCH, 0);		
		input_mt_sync(ts->input_dev);
    } 	
}
#endif

static void nvt_ts_work_func(struct work_struct *work)
{		
	int ret=-1;
	struct i2c_client *client = ts->client;
	uint8_t  point_data[(TOUCH_MAX_FINGER_NUM*6)+2+1]={0};
	
	ret = nvt_i2c_read(ts->client, I2C_FW_Address, point_data,  ts->max_touch_num*6+2+1);
	if(ret < 0){
		dev_err(&client->dev, "%s: nvt_i2c_read failed.\n", __func__ );
		goto xfer_error;
	}
	
#if MT_PROTOCOL_B
	mt_protocol_b(point_data,ts->max_touch_num);	
#else
	mt_protocol_a(point_data,ts->max_touch_num);	
#endif

#if TOUCH_KEY_NUM > 0
	if(point_data[ts->max_touch_num*6+1]==0xF8){
		for(i=0; i<TOUCH_KEY_NUM; i++){
			input_report_key(ts->input_dev, touch_key_array[i], ((point_data[ts->max_touch_num*6+2]>>i)&(0x01)));	
		}
	}else{
		for(i=0; i<TOUCH_KEY_NUM; i++){
			input_report_key(ts->input_dev, touch_key_array[i], 0);	
		}
	}	
#endif

	input_sync(ts->input_dev);

xfer_error:
	enable_irq(ts->client->irq);
}

static irqreturn_t nvt_ts_irq_handler(int irq, void *dev_id)
{	
#if REPORT_RATE
	int touch_c;
	if(show_report_rate()==1){
		printk("Report Rate = %d\n", touch_c);
		touch_c=0;
	}else{
		touch_c++;
	}
#endif

	disable_irq_nosync(ts->client->irq);
	queue_work(nvt_wq, &ts->work);
	
	return IRQ_HANDLED;
}

static int nv_hw_power_on(struct device *dev)
{
	char const *power_pin_vbus;
	
	int ret = 0;


	ret = of_property_read_string(dev->of_node,"novatek,vbus", &power_pin_vbus);
	if (ret) {
		dev_err(dev, "%s: Failed to get vbus ret:%d",
			__func__, ret);
		return -EINVAL;
	}

	ts->vbus_novatek = regulator_get(dev, power_pin_vbus);
	if (IS_ERR(ts->vbus_novatek)) {
		dev_err(dev,"%s: vbus_novatek regulator get fail\n", __func__);
		ret = -EINVAL;
		goto regulator_get_failed;
	}
	
	ret = regulator_set_voltage(ts->vbus_novatek,1800000,1800000);
	if (ret) {
		dev_err(dev, "%s: Failed to set vbus ret:%d",
			__func__, ret);
		goto regulator_set_failed;
	}
	
	ret = regulator_enable(ts->vbus_novatek);
	if (ret) {
		dev_err(dev, "%s: Failed to enable vbus ret:%d",
			__func__, ret);
       goto regulator_enable_failed;
	}

	return ret;
regulator_enable_failed:
	regulator_disable(ts->vbus_novatek);
regulator_set_failed:
	regulator_put(ts->vbus_novatek);
regulator_get_failed:
	
	return ret;

}

static int nv_request_gpios(struct device *dev)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, NULL);
	ret |= gpio_request(ts->irq_gpio, "novatek-irq");
	if (ret) {
		dev_err(dev, "%s: Failed to get irq gpio %d (code: %d)",
			__func__, ts->irq_gpio, ret);
		goto request_gpio_irq_falied;
	}
	
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, NULL);
	ret |= gpio_request(ts->reset_gpio, "novatek-reset");
	if (ret) {
		dev_err(dev, "%s: Failed to get reset gpio %d (code: %d)",
			__func__, ts->reset_gpio, ret);
		goto request_gpio_reset_falied;
	}
	return ret;
	
request_gpio_reset_falied:
	gpio_free(ts->irq_gpio);
request_gpio_irq_falied:
	return ret;
}

static int nv_release_gpios(void)
{
	gpio_free(ts->irq_gpio);
	gpio_free(ts->reset_gpio);
	return 0;
}

static int nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int retry = 0;
	uint8_t buf[7] = {0x00,};
	dev_info(&client->dev, "%s entry.......\n", __func__);

	//read probe flag
	if(touch_hw_data.read_touch_probe_flag){
		ret = touch_hw_data.read_touch_probe_flag();
		if(ret){
			dev_err(&client->dev, "%s:it's not the first touch driver! \n",__func__);
			return -EPERM;
		}
		else{
			dev_info(&client->dev, "%s:it's the first touch driver!line=%d\n",__func__,__LINE__);
		}
	}

	dev_info(&client->dev, "%s start.......\n", __func__);
	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if(ts == NULL){
		dev_err(&client->dev, "%s:TS malloc fail!\n",__func__);
		return -ENOMEM;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);

	//requst gpios
	ret = nv_request_gpios(&client->dev);
	if (ret) {
		dev_err(&client->dev, "%s: request gpio resource failed\n", __func__);
		goto err_request_gpio_failed;
	}
    
	//check i2c func
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	//power on
  	ret = nv_hw_power_on(&client->dev);
	if (ret) {
		dev_err(&client->dev, "nv_hw_power_on failed %d\n", ret);
		goto err_hw_power_on_failed;		
	}
	msleep(200);

	//check i2c read for 3 times till success
	
	for(retry=3; retry>=0; retry--){
		ret = nvt_i2c_read(client, I2C_FW_Address, buf, 5);
		dev_info(&client->dev, "buf[1]=%d, buf[2]=%d, buf[3]=%d, buf[4]=%d, buf[5]=%d\n",
			buf[1], buf[2], buf[3], buf[4], buf[5]);
		
		if(ret <= 0){
			dev_err(&client->dev, "i2c read test failed at retry=%d.\n", retry);
		}
		else	{
			dev_info(&client->dev,  "i2c read test succeed.\n");
			break;
		}
		if(retry == 0){
			goto err_i2c_failed;	
		}
	}
	
	
	//initial nvt work
	INIT_WORK(&ts->work, nvt_ts_work_func);

	//allocate input device
	ts->input_dev = input_allocate_device();
	if(ts->input_dev == NULL){
		ret = -ENOMEM;
		dev_err(&client->dev,  "allocate input device failed.\n");
		goto err_input_dev_alloc_failed;
	}

	ts->abs_x_max = TOUCH_MAX_WIDTH-1;
	ts->abs_y_max = TOUCH_MAX_HEIGHT-1;
	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif
	
	ts->int_trigger_type = INT_TRIGGER_TYPE;

#if XY_REMAPPING
	nvt_xy_mapping_getinfo();
#endif

	//set input device info
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num);
	input_set_abs_params(ts->input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_FINGER, 0, 0);
#endif	
	
	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);    //pressure = 255

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif	

#if TOUCH_KEY_NUM > 0
	for(retry = 0; retry < TOUCH_KEY_NUM; retry++){
		input_set_capability(ts->input_dev, EV_KEY,touch_key_array[retry]);	
	}
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0205;
	ts->input_dev->id.product = 0x0001;

	//register input device
	ret = input_register_device(ts->input_dev);
	if(ret){
		dev_err(&client->dev, "register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}
	ts->bad_data = 0;

	//set int-pin & request irq
	client->irq = gpio_to_irq(ts->irq_gpio);
	if(client->irq){
		dev_info(&client->dev, "int_trigger_type=%d\n",ts->int_trigger_type);
		ret = request_irq(client->irq, nvt_ts_irq_handler, ts->int_trigger_type, client->name, ts);
		if (ret != 0){
			dev_err(&client->dev, "request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		}
		else {	
			disable_irq(client->irq);
			dev_info(&client->dev, "request irq %d succeed.\n", client->irq);
		}
	}	
	enable_irq(client->irq);

	//set device node
	nvt_flash_init();
	
	//for test
	nvt_sysfs_init();

	//for print test info
	nvt_procfs_create();
	
	//check and auto update
	auto_update();
	
	//set touch flag. avoid to other tp driver probe repeatly! 
	if(touch_hw_data.set_touch_probe_flag){
		touch_hw_data.set_touch_probe_flag(1);
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nvt_ts_early_suspend;
	ts->early_suspend.resume = nvt_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	/* detect current device successful, set the flag as present */
	set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
#endif

	dev_info(&client->dev, "%s finished.\n", __func__);
	return 0;

err_int_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_i2c_failed:
	regulator_disable(ts->vbus_novatek);
	regulator_put(ts->vbus_novatek);
err_hw_power_on_failed:
err_check_functionality_failed:
	nv_release_gpios();
err_request_gpio_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

static int nvt_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	
	dev_notice(&client->dev, "removing driver...\n");
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	
	regulator_disable(ts->vbus_novatek);
	regulator_put(ts->vbus_novatek);
	
	kfree(ts);	
	return 0;
}

static int nvt_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{	
	dev_info(&client->dev, "%s\n", __func__);	
	return 0;
}

static int nvt_ts_resume(struct i2c_client *client)
{
	dev_info(&client->dev, "%s\n", __func__);
	nvt_hw_reset();
	enable_irq(client->irq);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nvt_ts_early_suspend(struct early_suspend *h)
{	
	nvt_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void nvt_ts_late_resume(struct early_suspend *h)
{	
	nvt_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id novatek_match_table[] = {
		{ .compatible = "novatek,Novatek-TS",},
		{ },
};
#endif

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.suspend	= nvt_ts_suspend,
	.resume		= nvt_ts_resume,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_I2C_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = novatek_match_table,
	},
};

static int __init nvt_driver_init(void)
{
	int ret;
	
	//add i2c driver
	printk("%s : entry.\n", __func__);
	ret = i2c_add_driver(&nvt_i2c_driver);
	if(ret){
		printk("%s : failed to add i2c driver.", __func__);
		goto err_driver;
	}

	//create workqueue
	nvt_wq = create_workqueue("nvt_wq");
	if(!nvt_wq){
		printk("%s : create workqueue failed.\n", __func__);
		return -ENOMEM;	
	}
	
	printk("%s : finished.\n", __func__);	

err_driver:
	return ret; 
}

static void __exit nvt_driver_exit(void)
{
	i2c_del_driver(&nvt_i2c_driver);
	i2c_unregister_device(nvt_client);
	
	if(nvt_wq)
		destroy_workqueue(nvt_wq);
}


module_init(nvt_driver_init);
module_exit(nvt_driver_exit);


MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
