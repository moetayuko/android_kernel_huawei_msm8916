/**********************************************************
 * Filename:	dsm_pub.h
 *
 * Discription: Huawei device state monitor public head file
 *
 * Copyright: (C) 2014 huawei.
 *
 *
**********************************************************/

#ifndef _DSM_PUB_H
#define _DSM_PUB_H
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

extern int debug_output;

#define DSM_LOG_DEBUG(format, ...)				\
	do {						\
		if (debug_output)			\
			 printk("[DSM] "format,## __VA_ARGS__);\
	} while (0)


#define DSM_LOG_INFO(format, ...)		printk("[DSM] "format,## __VA_ARGS__)
#define DSM_LOG_ERR(format, ...)		printk("[DSM] "format,## __VA_ARGS__)

#define CLIENT_NAME_LEN		(32)

/*dsm error no define*/
#define DSM_ERR_NO_ERROR		(0)
#define DSM_ERR_I2C_TIMEOUT		(1)

/* charger and bms error mumbers*/
#define DSM_BMS_POWON_SOC_ERROR_NO				(10100)
#define DSM_BMS_NORMAL_SOC_ERROR_NO				(10101)
#define DSM_BMS_BQ_UPDATE_FIRMWARE_FAIL_NO			(10102)
#define DSM_CHARGER_NOT_CHARGE_ERROR_NO  			(10200)
#define DSM_CHARGER_OTG_OCP_ERROR_NO  				(10201)
#define DSM_CHARGER_CHG_OVP_ERROR_NO  				(10202)
#define DSM_CHARGER_BATT_PRES_ERROR_NO  			(10203)
#define DSM_CHARGER_SPMI_ABNORMAL_ERROR_NO  		(10204)
#define DSM_CHARGER_ONLINE_ABNORMAL_ERROR_NO  	(10205)
#define DSM_CHARGER_ADC_ABNORMAL_ERROR_NO  		(10206)
#define DSM_CHARGER_BQ_BOOST_FAULT_ERROR_NO  		(10207)
#define DSM_CHARGER_BQ_NORMAL_FAULT_ERROR_NO  	(10208)
#define DSM_CHARGER_BQ_I2C_ERROR_NO  				(10209)

/* touch panel error numbers */
#define DSM_TP_I2C_RW_ERROR_NO 				(20000)		// read or write i2c error
#define DSM_TP_FW_ERROR_NO 					(20001)		// fw has error
#define DSM_TP_DISTURB_ERROR_NO 			(20002)		// be disturbed by external condition
#define DSM_TP_IC_ERROR_NO 					(20003)		// ic has error
#define DSM_TP_BROKEN_ERROR_NO 				(20004)		// hardware broken
#define DSM_TP_ESD_ERROR_NO					(20005)		// esd check error
#define DSM_TP_F34_PDT_ERROR_NO 			(20006)		// fail to read pdt
#define DSM_TP_F54_PDT_ERROR_NO 			(20007)		// fail to read f54 pdt
#define DSM_TP_PDT_PROPS_ERROR_NO 			(20008)		// fail to read pdt props
#define DSM_TP_F34_READ_QUERIES_ERROR_NO 	(20009)		// fail to read f34 queries

/* LCD error numbers */
/* modify lcd error macro */
#define DSM_LCD_MIPI_ERROR_NO				(20100)
#define DSM_LCD_TE_TIME_OUT_ERROR_NO		(20101)
#define DSM_LCD_STATUS_ERROR_NO				(20102)
#define DSM_LCD_PWM_ERROR_NO				(20104)
#define DSM_LCD_BRIGHTNESS_ERROR_NO			(20105)
#define DSM_LCD_MDSS_DSI_ISR_ERROR_NO		(20106)
#define DSM_LCD_MDSS_MDP_ISR_ERROR_NO		(20107)
/* modify lcd error macro */
#define DSM_LCD_ESD_STATUS_ERROR_NO                    (20108)
#define DSM_LCD_ESD_REBOOT_ERROR_NO                    (20109)
#define DSM_LCD_POWER_STATUS_ERROR_NO                    (20110)
#define DSM_LCD_MDSS_UNDERRUN_ERROR_NO		(20111)
#define DSM_ERR_SDIO_RW_ERROR				(20300)
#define DSM_ERR_SENSORHUB_IPC_TIMEOUT		(20400)

#define CLIENT_NAME_GS_KX			"dsm_gs_kx023"
#define CLIENT_NAME_GS_LIS			"dsm_gs_lis3dh"
#define CLIENT_NAME_LPS_APDS		"dsm_lps_apds"
#define CLIENT_NAME_MS_AKM			"dsm_ms_akm09911"

#define DSM_GS_I2C_ERROR				(21100)
#define DSM_GS_DATA_ERROR				(21101)
#define DSM_LPS_I2C_ERROR				(21102)
#define DSM_MS_I2C_ERROR				(21103)
#define DSM_MS_DATA_ERROR				(21104)
#define DSM_SENSOR_BUF_MAX 				4096
#define DSM_SENSOR_BUF_COM				4096
#define DSM_AUDIO_ERROR_NUM                     (20200)
#define DSM_AUDIO_HANDSET_DECT_FAIL_ERROR_NO    (DSM_AUDIO_ERROR_NUM)
#define DSM_AUDIO_ADSP_SETUP_FAIL_ERROR_NO      (DSM_AUDIO_ERROR_NUM + 1)
#define DSM_AUDIO_CARD_LOAD_FAIL_ERROR_NO       (DSM_AUDIO_ERROR_NUM + 2)


#define DSM_CAMERA_ERROR							(20500)
#define DSM_CAMERA_SOF_ERR							(DSM_CAMERA_ERROR + 1)
#define DSM_CAMERA_I2C_ERR							(DSM_CAMERA_ERROR + 2)
#define DSM_CAMERA_CHIP_ID_NOT_MATCH				(DSM_CAMERA_ERROR + 3)
#define DSM_CAMERA_OTP_ERR							(DSM_CAMERA_ERROR + 4)
#define DSM_CAMERA_CPP_BUFF_ERR						(DSM_CAMERA_ERROR + 5)

struct dsm_client_ops{
	int (*poll_state) (void);
	int (*dump_func) (int type, void *buff, int size);
};

struct dsm_dev{
	const char *name;
	struct dsm_client_ops *fops;
	size_t buff_size;
};

/* one client */
struct dsm_client{
	void *driver_data;
	char client_name[CLIENT_NAME_LEN];
	int client_id;
	int error_no;
	unsigned long buff_flag;
	struct dsm_client_ops *cops;
	wait_queue_head_t waitq;
	size_t read_size;
	size_t used_size;
	size_t buff_size;
	u8 dump_buff[];
};

#ifdef CONFIG_HUAWEI_DSM
struct dsm_client *dsm_register_client (struct dsm_dev *dev);
void dsm_unregister_client (struct dsm_client *dsm_client,struct dsm_dev *dev);

int dsm_client_ocuppy(struct dsm_client *client);
int dsm_client_record(struct dsm_client *client, const char *fmt, ...);
int dsm_client_copy(struct dsm_client *client, void *src, int sz);
void dsm_client_notify(struct dsm_client *client, int error_no);
#else
static inline struct dsm_client *dsm_register_client (struct dsm_dev *dev)
{
	return NULL;
}
static inline int dsm_client_ocuppy(struct dsm_client *client)
{
	return 1;
}
static inline int dsm_client_record(struct dsm_client *client, const char *fmt, ...)
{
	return 0;
}
static inline int dsm_client_copy(struct dsm_client *client, void *src, int sz)
{
	return 0;
}
static inline void dsm_client_notify(struct dsm_client *client, int error_no)
{
	return;
}
#endif

#endif
