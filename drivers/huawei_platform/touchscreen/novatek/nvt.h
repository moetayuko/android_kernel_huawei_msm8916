/* drivers/input/touchscreen/novatek/nvt.h
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
 *
 */
 
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#include <linux/unistd.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
//#include <linux/huawei_tp_adapter.h>
#include <huawei_platform/touchscreen/hw_tp_common.h>

#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>


//---INT trigger mode---
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_FALLING

//---I2C driver info.---
#define NVT_I2C_NAME "NVT-ts"
#define NVT_I2C_BUS_NUM 4
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x70

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"

//---Touch info.---
#define TOUCH_MAX_WIDTH 800
#define TOUCH_MAX_HEIGHT 1280
#define TOUCH_MAX_FINGER_NUM 5
#define TOUCH_KEY_NUM 0

//---Customerized func.---
#define FIRMWARE_NAME "novatek_fw.bin"
#define MT_PROTOCOL_B 0
#define REPORT_RATE 0
#define XY_REMAPPING 0

struct nvt_ts_data{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct work;
//add pwx
	struct regulator *vbus_novatek;
//end add pwx
	uint16_t addr;
	uint8_t bad_data;
	char phys[32];
	int retry;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int irq_gpio;
	int reset_gpio;
	int vcc_gpio;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t x_num;
	uint8_t y_num;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	uint8_t green_wake_mode;
};

struct nvt_flash_data{
	rwlock_t lock;
	unsigned char bufferIndex;
	unsigned int length;
	struct i2c_client *client;
};

#if XY_REMAPPING
extern void nvt_xy_mapping_getinfo(void);
#endif

#endif /* _LINUX_NVT_TOUCH_H */
