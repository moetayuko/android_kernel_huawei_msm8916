

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_common_if"

#include <linux/hw_camera_common.h>
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

#define OV5648_SUNNY_P5V18G_RG_RATIO_TYPICAL 0x2eb
#define OV5648_SUNNY_P5V18G_BG_RATIO_TYPICAL 0x2dc

#define OV5648_SUNNY_P5V36D_RG_RATIO_TYPICAL 0x2e8
#define OV5648_SUNNY_P5V36D_BG_RATIO_TYPICAL 0x2dd

#define S5K4E1_LITEON_13P1_RG_RATIO_TYPICAL 0x2d0
#define S5K4E1_LITEON_13P1_BG_RATIO_TYPICAL 0x287

#define S5K4E1_FOXCONN_DC0301A_RG_RATIO_TYPICAL 0x34c
#define S5K4E1_FOXCONN_DC0301A_BG_RATIO_TYPICAL 0x2f3

#define OV8858_FOXCONN_RG_RATIO_TYPICAL 0x2FC
#define OV8858_FOXCONN_BG_RATIO_TYPICAL 0x2B5

#define OV5648_FOXCONN_KIW_RG_RATIO_TYPICAL     0x2da
#define OV5648_FOXCONN_KIW_BG_RATIO_TYPICAL     0x2c3

#define S5K4E1_SUNNY_KIW_RG_RATIO_TYPICAL       0x312
#define S5K4E1_SUNNY_KIW_BG_RATIO_TYPICAL       0x2DE

/* average of only two golden values(20150519) */
#define OV13850_OFILM_OHW8A04_RG_RATIO_TYPICAL  0x264  
#define OV13850_OFILM_OHW8A04_BG_RATIO_TYPICAL  0x24c

/* average of 5 golden values OK(20150519) */
#define OV13850_LITEON_193_RG_RATIO_TYPICAL     0x242
#define OV13850_LITEON_193_BG_RATIO_TYPICAL     0x233

/* Not used*/
#define IMX328_SUNNY_P13N10A_RG_RATIO_TYPICAL  0x01ED
#define IMX328_SUNNY_P13N10A_BG_RATIO_TYPICAL  0x0172

#define S5K4E1_SUNNY_132_RG_RATIO_TYPICAL            0x30c
#define S5K4E1_SUNNY_132_BG_RATIO_TYPICAL            0x2b1
#define OV5648_FOXCONN_132_RG_RATIO_TYPICAL          0x278
#define OV5648_FOXCONN_132_BG_RATIO_TYPICAL          0x2d2

#define OV13850_SUNNY_P13V01H_RG_RATIO_TYPICAL  0x248
#define OV13850_SUNNY_P13V01H_BG_RATIO_TYPICAL  0x257

#define OV13850_OFILM_OHWBA03_RG_RATIO_TYPICAL  0x248
#define OV13850_OFILM_OHWBA03_BG_RATIO_TYPICAL  0x257

#define OV5648_OFILM_OHW5F03_RG_RATIO_TYPICAL         0x02e8
#define OV5648_OFILM_OHW5F03_BG_RATIO_TYPICAL         0x0312
#define OV8858_FOXCONN_PAD_RG_RATIO_TYPICAL 0x2FC
#define OV8858_FOXCONN_PAD_BG_RATIO_TYPICAL 0x2B5
#define AR1335_SUNNY_F13M01M_RG_RATIO_TYPICAL 0x26D
#define AR1335_SUNNY_F13M01M_BG_RATIO_TYPICAL 0x267

struct otp_function_t otp_function_lists []=
{
	{
		"ov13850_sunny_p13v01h",
		ov13850_otp_func,
		OV13850_SUNNY_P13V01H_RG_RATIO_TYPICAL,
		OV13850_SUNNY_P13V01H_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov13850_ofilm_ohwba03",
		ov13850_otp_func,
		OV13850_OFILM_OHWBA03_RG_RATIO_TYPICAL,
		OV13850_OFILM_OHWBA03_BG_RATIO_TYPICAL,
		false,
	},
	{
		"imx328_sunny_p13n10a",
		imx328_sunny_p13n10a_otp_func,
		IMX328_SUNNY_P13N10A_RG_RATIO_TYPICAL,
		IMX328_SUNNY_P13N10A_BG_RATIO_TYPICAL,
		false,
	},
	{
		"s5k4e1_sunny_132",
		s5k4e1_otp_func,
		S5K4E1_SUNNY_132_RG_RATIO_TYPICAL,
		S5K4E1_SUNNY_132_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_foxconn_132",
		ov5648_otp_func,
		OV5648_FOXCONN_132_RG_RATIO_TYPICAL,
		OV5648_FOXCONN_132_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_ofilm_ohw5f03",
		ov5648_otp_func,
		OV5648_OFILM_OHW5F03_RG_RATIO_TYPICAL,
		OV5648_OFILM_OHW5F03_BG_RATIO_TYPICAL,
		false,
	},

    {
        "ov5648_sunny_p5v18g_pad",
        ov5648_sunny_p5v18g_otp_func,
        OV5648_SUNNY_P5V18G_RG_RATIO_TYPICAL,
        OV5648_SUNNY_P5V18G_BG_RATIO_TYPICAL,
        false,
    },
    {
        "s5k4e1_liteon_13p1_pad",
        s5k4e1_liteon_13p1_otp_func,
        S5K4E1_LITEON_13P1_RG_RATIO_TYPICAL,
        S5K4E1_LITEON_13P1_BG_RATIO_TYPICAL,
        false,
    },
    
    {
		"ov8858_foxconn_pad",  
		ov8858_foxconn_otp_func,
		OV8858_FOXCONN_PAD_RG_RATIO_TYPICAL,
		OV8858_FOXCONN_PAD_BG_RATIO_TYPICAL,
		false,
	},    
};
/*************************************************
  Function    : is_exist_otp_function
  Description: Detect the otp we support
  Calls:
  Called By  : msm_sensor_config
  Input       : s_ctrl
  Output     : index
  Return      : true describe the otp we support
                false describe the otp we don't support

*************************************************/
bool is_exist_otp_function( struct msm_sensor_ctrl_t *s_ctrl, int32_t *index)
{
	int32_t i = 0;

	for (i=0; i<(sizeof(otp_function_lists)/sizeof(otp_function_lists[0])); ++i)
	{
        if(strlen(s_ctrl->sensordata->sensor_name) != strlen(otp_function_lists[i].sensor_name))
            continue;
		if (0 == strncmp(s_ctrl->sensordata->sensor_name, otp_function_lists[i].sensor_name, strlen(s_ctrl->sensordata->sensor_name)))
		{
			*index = i;
			CMR_LOGI("is_exist_otp_function success i = %d\n", i);
			return true;
		}
	}
	return false;
}

