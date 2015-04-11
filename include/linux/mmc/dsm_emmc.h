#ifdef CONFIG_HUAWEI_EMMC_DSM
#ifndef LINUX_MMC_DSM_EMMC_H
#define LINUX_MMC_DSM_EMMC_H

#include <linux/dsm_pub.h>

/* define a 1024 size of array as buffer */
#define EMMC_DSM_BUFFER_SIZE  1024
#define MSG_MAX_SIZE 200
/* Error code, decimal[5]: 0 is input, 1 is output, 2 I&O
 * decimal[4:3]: 10 is for eMMC,
 * decimal[2:1]: for different error code.
 */
enum DSM_EMMC_ERR
{
	DSM_SYSTEM_W_ERR 		= 21001,
	DSM_EMMC_ERASE_ERR,
	DSM_EMMC_VDET_ERR,
	DSM_EMMC_SEND_CXD_ERR,
	DSM_EMMC_READ_ERR,
	DSM_EMMC_WRITE_ERR,
};


struct emmc_dsm_log {
	char emmc_dsm_log[EMMC_DSM_BUFFER_SIZE];
	spinlock_t lock;	/* mutex */
};

extern struct dsm_client *emmc_dclient;

/*buffer for transffering to device radar*/
extern struct emmc_dsm_log g_emmc_dsm_log;
extern int dsm_emmc_get_log(void *card, int code, char * err_msg);

/*Transfer the msg to device radar*/
#define DSM_EMMC_LOG(card, no, fmt, a...) \
	do { \
		char msg[MSG_MAX_SIZE]; \
		snprintf(msg, MSG_MAX_SIZE-1, fmt, ## a); \
		spin_lock(&g_emmc_dsm_log.lock); \
		if(dsm_emmc_get_log((card), (no), (msg))){ \
			if(!dsm_client_ocuppy(emmc_dclient)) { \
				dsm_client_copy(emmc_dclient,g_emmc_dsm_log.emmc_dsm_log, strlen(g_emmc_dsm_log.emmc_dsm_log) + 1); \
				dsm_client_notify(emmc_dclient, no); } \
		} \
		spin_unlock(&g_emmc_dsm_log.lock); \
	}while(0)

#endif /* LINUX_MMC_DSM_EMMC_H */
#endif /* CONFIG_HUAWEI_EMMC_DSM */
