
/*************************
              XY_Remapping
**************************/

#include "nvt.h"

#if XY_REMAPPING

extern struct i2c_client *nvt_client;
extern struct nvt_ts_data *ts;
extern int nvt_i2c_read(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len);
extern void nvt_i2c_write (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len);

static int BoundaryFun_En = 0;			//_AT_	0x3E00;
static int XEdgeDist_L = 0;				//_AT_	0x3E01;
static int XCorrectionGradient_L = 0;	//_AT_	0x3E02;
static int XEdgeDist_R = 0;				//_AT_	0x3E03;
static int XCorrectionGradient_R = 0;	//_AT_	0x3E04;
static int YEdgeDist_U = 0;				//_AT_	0x3E05;
static int YCorrectionGradient_U = 0;	//_AT_	0x3E06;
static int YEdgeDist_D = 0;				//_AT_	0x3E07;
static int YCorrectionGradient_D = 0;	//_AT_	0x3E08;

static int OLM_Remapping_En = 0;		//_AT_	0x3E10;
static int X_VA_Res = 0;
//static int X_VA_Res_H = 0;				//_AT_	0x3E11;
//static int X_VA_Res_L = 0;				//_AT_	0x3E12;
static int Y_VA_Res = 0;
//static int Y_VA_Res_H = 0;				//_AT_	0x3E13;
//static int Y_VA_Res_L = 0;				//_AT_	0x3E14;
static signed char X_AdjX_R = 0;		//_AT_	0x3E15;
static signed char X_AdjX_L = 0;		//_AT_	0x3E16;
static signed char Y_AdjY_U = 0;		//_AT_	0x3E17;
static signed char Y_AdjY_D = 0;		//_AT_	0x3E18;

static int X_Res = 0;
//static int X_Res_H = 0;					//_AT_	0x3F7C;
//static int X_Res_L = 0;					//_AT_	0x3F7D;
static int Y_Res = 0;
//static int Y_Res_H = 0;					//_AT_	0x3F7E;
//static int Y_Res_L = 0;					//_AT_	0x3F7F;

void nvt_xy_mapping_getinfo(void)
{
	uint8_t I2C_Buf[16];

	//---get boundary info.---
	I2C_Buf[0]=0xFF;
	I2C_Buf[1]=0x3E;
	I2C_Buf[2]=0x00;
	nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);

	I2C_Buf[0] = 0;
	nvt_i2c_read(ts->client, I2C_FW_Address, I2C_Buf, 10);
	BoundaryFun_En = I2C_Buf[1];
	XEdgeDist_L = I2C_Buf[2];
	XCorrectionGradient_L = I2C_Buf[3];
	XEdgeDist_R = I2C_Buf[4];
	XCorrectionGradient_R = I2C_Buf[5];
	YEdgeDist_U = I2C_Buf[6];
	YCorrectionGradient_U = I2C_Buf[7];
	YEdgeDist_D = I2C_Buf[8];
	YCorrectionGradient_D = I2C_Buf[9];

	pr_info("%s: BoundaryFun_En=%d, XEdgeDist_L=%d, XCorrectionGradient_L=%d, XEdgeDist_R=%d, XCorrectionGradient_R=%d\n", __func__,
		BoundaryFun_En, XEdgeDist_L, XCorrectionGradient_L, XEdgeDist_R, XCorrectionGradient_R);
	pr_info("%s: YEdgeDist_U=%d, YCorrectionGradient_U=%d, YEdgeDist_D=%d, YCorrectionGradient_D=%d\n", __func__,
		YEdgeDist_U, YCorrectionGradient_U, YEdgeDist_D, YCorrectionGradient_D);


	//---get remapping info.---
	I2C_Buf[0]=0xFF;
	I2C_Buf[1]=0x3E;
	I2C_Buf[2]=0x10;
	nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);

	I2C_Buf[0] = 0;
	nvt_i2c_read(ts->client, I2C_FW_Address, I2C_Buf, 14);
	OLM_Remapping_En = I2C_Buf[1];
	X_VA_Res = I2C_Buf[2]*256 + I2C_Buf[3];
	Y_VA_Res = I2C_Buf[4]*256 + I2C_Buf[5];
	X_AdjX_R = I2C_Buf[6];
	X_AdjX_L = I2C_Buf[7];
	Y_AdjY_U = I2C_Buf[8];
	Y_AdjY_D = I2C_Buf[9];

	pr_info("%s: OLM_Remapping_En=%d, X_VA_Res=%d, Y_VA_Res=%d\n", __func__,
		OLM_Remapping_En, X_VA_Res, Y_VA_Res);
	pr_info("%s: X_AdjX_R=%+d, X_AdjX_L=%+d, Y_AdjY_U=%+d, Y_AdjY_D=%+d\n", __func__,
		X_AdjX_R, X_AdjX_L, Y_AdjY_U, Y_AdjY_D);


	//---get xy res.---
	I2C_Buf[0]=0xFF;
	I2C_Buf[1]=0x3F;
	I2C_Buf[2]=0x00;
	nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);

	I2C_Buf[0] = 0x7C;
	nvt_i2c_read(ts->client, I2C_FW_Address, I2C_Buf, 5);
	X_Res = I2C_Buf[1]*256 + I2C_Buf[2];
	Y_Res = I2C_Buf[3]*256 + I2C_Buf[4];

	pr_info("%s: X_Res=%d, Y_Res=%d\n", __func__,
		X_Res, Y_Res);	
}

void nvt_xy_mapping(unsigned int *xp, unsigned int *yp)
{
	s32 x = *xp;
	s32 y = *yp;

	if(BoundaryFun_En == 1)
	{
		//Boundary mapping X
		if(x < XEdgeDist_L)
		{
			x = x - (((XEdgeDist_L - x)*XCorrectionGradient_L)/16);
			if(x < 0)
				x = 0;
		}
		else if(x > (X_Res-1-XEdgeDist_R))
		{
			x = x + (((x-(X_Res-1-XEdgeDist_R))* XCorrectionGradient_R)/16);
			if(x > (X_Res-1))
				x = (X_Res-1);
		}
		//Boundary mapping Y
		if(y < YEdgeDist_U)
		{
			y = y - (((YEdgeDist_U - y)*YCorrectionGradient_U)/16);
			if(y < 0)
				y = 0;
		}
		else if(y > (Y_Res-1-YEdgeDist_D))
		{
			y = y + (((y-(Y_Res-1-YEdgeDist_D))* YCorrectionGradient_D)/16);
			if(y > (Y_Res-1))
				y = (Y_Res-1);
		}
	}

	if(OLM_Remapping_En == 1)
	{
		//Normal AA remaping X
		x = ((x*TOUCH_MAX_WIDTH)/X_Res); 	
		//Normal AA remaping Y
		y = ((y*TOUCH_MAX_HEIGHT)/Y_Res);		
	}
	else if(OLM_Remapping_En == 2)
	{
		//TOD AA remapping X
		if(x > (X_Res/2))
			x = ((TOUCH_MAX_WIDTH/2) + (((x-(X_Res/2))*TOUCH_MAX_WIDTH) / (X_VA_Res+X_AdjX_R)));
		else
			x = ((TOUCH_MAX_WIDTH/2) - ((((X_Res/2)-x)*TOUCH_MAX_WIDTH) / (X_VA_Res+X_AdjX_L)));
   				
		//TOD AA remapping Y
		if(y > (Y_Res/2))
			y = ((TOUCH_MAX_HEIGHT/2) + (((y-(Y_Res/2))*TOUCH_MAX_HEIGHT) / (Y_VA_Res+Y_AdjY_D)));
		else
			y = ((TOUCH_MAX_HEIGHT/2) - ((((Y_Res/2)-y)*TOUCH_MAX_HEIGHT) / (Y_VA_Res+Y_AdjY_U)));
	}

	*xp = (unsigned int)x;
	*yp = (unsigned int)y;
}
#endif

