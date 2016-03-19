
/*******************
  Auto Update FW in Probe
********************/

#include "nvt.h"
#ifdef CONFIG_HUAWEI_KERNEL
#include <misc/app_info.h>
#endif
#define FW_DATA_LENGTH	32768
static char touch_info[50] = {0};
char firmware_name[64];
const struct firmware *fw_entry = NULL;
struct delayed_work nvt_fw_updatew_wq;

extern struct nvt_ts_data *ts;

extern void nvt_hw_reset(void);
extern int nvt_i2c_read(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len);
extern void nvt_i2c_write (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len);


int get_fw_ver(void)
{
	int fw_ver = 0;
	uint8_t I2C_Buf[16];
	memset(I2C_Buf,0,sizeof(uint8_t)*16);

	I2C_Buf[0]=0xFF;
	I2C_Buf[1]=0x3F;
	I2C_Buf[2]=0x00;
	nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);
	msleep(10);

	I2C_Buf[0] = 0x78;
	nvt_i2c_read(ts->client, 0x01, I2C_Buf, 2);
	dev_info(&ts->client->dev, "IC FW Ver = %d\n", I2C_Buf[1]);

	fw_ver = (int)I2C_Buf[1];
	return fw_ver;
}

int check_fw_ver(u8 *fw_data)
{
	int fw_ver = 0;
	
	fw_ver = get_fw_ver();
	dev_info(&ts->client->dev, "Bin FW Ver = %d\n", *(fw_data + 0x7F00));
	if(fw_ver >= *(fw_data + 0x7F00)){
		return 1;
	}else{
		return 0;
	}
}

int check_checksum(u8 *fw_data)
{
	uint8_t I2C_Buf[64];
	uint8_t buf2[64];
	int i, j, k, Retry_Counter=0;
	int addr=0;
	uint8_t addrH, addrL;
	unsigned short RD_Filechksum, WR_Filechksum;

	WR_Filechksum = 0;

	memset(I2C_Buf,0,sizeof(uint8_t)*64);
	memset(buf2,0,sizeof(uint8_t)*64);
	
	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x5A;
	nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 2);

	msleep(1000);

	I2C_Buf[0]=0xFF;
	I2C_Buf[1]=0x3F;
	I2C_Buf[2]=0xE8;
	nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0xEA;
	nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 2);

	for(i=0;i<FW_DATA_LENGTH/128;i++){
		for(j=0;j<16;j++){
			unsigned char tmp=0;
			addrH = addr>>8;
			addrL = addr&0xFF;
			for(k=0;k<8;k++){
				tmp+= *(fw_data + (i*128+j*8+k));
			}
			tmp = tmp+addrH+addrL+8;
			tmp = (255-tmp)+1;
			WR_Filechksum+=tmp;
			addr+=8;
		}
	}
	msleep(800);
	do{
		msleep(10);
		I2C_Buf[0]=0xFF;
		I2C_Buf[1]=0x3F;
		I2C_Buf[2]=0xF8;
		nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);

		buf2[0]=0x00;
		buf2[1]=0x00;
		buf2[2]=0x00;
		buf2[3]=0x00;
		nvt_i2c_read(ts->client, I2C_FW_Address, buf2, 4);

		Retry_Counter++;
		msleep(10);

	}while((Retry_Counter<20)&& (buf2[1]!=0xAA));

	if(buf2[1]==0xAA){
		RD_Filechksum=(buf2[2]<<8)+buf2[3];
		if(RD_Filechksum==WR_Filechksum){
			return 1;	// checksum match
		}else{
			return 0;	// checksum not match
		}
	}else{
		return -1;	// read checksum failed
	}
}


uint8_t init_update(void)
{
	uint8_t I2C_Buf[16];
	memset(I2C_Buf,0,sizeof(uint8_t)*16);

	// initial BootLoader
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 2);
	msleep(2);
  
	// Initiate Flash Block
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 2);
	msleep(20);

	// Read status
	I2C_Buf[0] = 0x00;
	nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2);
	return I2C_Buf[1];
}

uint8_t erase_flash(void)
{
	int i = 0;
	unsigned int Row_Address = 0;
	uint8_t I2C_Buf[16];	
	memset(I2C_Buf,0,sizeof(uint8_t)*16);	
	
	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x66;
	I2C_Buf[2]=0x00;
	I2C_Buf[3]=0x0E;
	I2C_Buf[4]=0x01;
	I2C_Buf[5]=0xB4;
	I2C_Buf[6]=0x3D;
	nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 7);

	while(1){
		msleep(1);
		nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2);
		if(I2C_Buf[1]==0xAA)  break;
	}

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x66;
	I2C_Buf[2]=0x00;
	I2C_Buf[3]=0x0F;
	I2C_Buf[4]=0x01;
	I2C_Buf[5]=0xEF;
	I2C_Buf[6]=0x01;
	nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 7);

	while(1){
		msleep(1);
		nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2);
		if(I2C_Buf[1]==0xAA)  break;
	}
	
	for (i = 0 ; i < FW_DATA_LENGTH/4096 ; i++){		// 32K = 8 times
		Row_Address = i * 4096; 															
		// Erase Flash	
		I2C_Buf [0] = 0x00;
		I2C_Buf [1] = 0x33;
		I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );	// Address High Byte  
		I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);	// Address Low Byte 					
		nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 4);
		msleep(15);	// Delay 15 ms 
			  
		// Read Erase status
		nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2); 

		// check status        
		if (I2C_Buf[1] != 0xAA){
			return I2C_Buf[1];
		}			
	}
	return I2C_Buf[1];
}

uint8_t write_ic(u8 *fw_data)
{
	int i,j;
	unsigned int Flash_Address = 0;
	uint8_t I2C_Buf[16];	
	uint8_t CheckSum[16];	// 128/8 = 16 times ;
	memset(I2C_Buf,0,sizeof(uint8_t)*16);
	memset(CheckSum,0,sizeof(uint8_t)*16);

	for(j=0;j<FW_DATA_LENGTH/128;j++){
	    	Flash_Address=(j)*128;
	   	for (i = 0 ; i < 16 ; i++, Flash_Address += 8){	// 128/8 = 16 times for One Row program
	   	
			// write bin data to IC
			I2C_Buf[0] = 0x00;
			I2C_Buf[1] = 0x55;	//Flash write command
			I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
			I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
			I2C_Buf[4] = 0x08;	//Flash write length (byte)
			I2C_Buf[6] = *(fw_data + Flash_Address + 0);	//Binary data 1
			I2C_Buf[7] = *(fw_data + Flash_Address + 1);	//Binary data 2
			I2C_Buf[8] = *(fw_data + Flash_Address + 2);	//Binary data 3
			I2C_Buf[9] = *(fw_data + Flash_Address + 3);	//Binary data 4
			I2C_Buf[10] = *(fw_data + Flash_Address + 4);   //Binary data 5
			I2C_Buf[11] = *(fw_data + Flash_Address + 5);	//Binary data 6
			I2C_Buf[12] = *(fw_data + Flash_Address + 6);	//Binary data 7
			I2C_Buf[13] = *(fw_data + Flash_Address + 7);	//Binary data 8

			CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
				          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
			    	      I2C_Buf[13]) + 1;

			// Load check sum to I2C Buffer
			I2C_Buf[5] = CheckSum[i];
			nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 14);
	   	}
		msleep(10);
		
		// Read status
		I2C_Buf[0] = 0x00;
		while(1){
			nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2);
			if(I2C_Buf[1]==0xAA){
				break;
			}else{
				return  I2C_Buf[1];
			}
		}
	}
	return I2C_Buf[1];
}

int  update_firmware(u8 *fw_data)
{
	
	int ret;
	uint8_t status = 0;
	struct i2c_client *client = ts->client;

	//step1:init
	status = init_update();
	if (status != 0xAA){
		dev_err(&client->dev, "Program: init get status(0x%2X) error.",status);
		return status;
	}
	dev_info(&client->dev, "Program: init get status(0x%2X) success.",status);

	//step2:erase
	status = erase_flash();    
	if (status != 0xAA){
		dev_err(&client->dev, "Program: erase(0x%2X) error", status);
			return status;
	}			
	dev_info(&client->dev, "Program: erase(0x%2X) success", status);

	//step3:write
	dev_info(&client->dev, "Program: write begin, please wait...");
	status = write_ic(fw_data);   
	if (status != 0xAA){
		dev_err(&client->dev, "Program: write_ic(0x%2X) error", status);
			return status;
	}			
	dev_info(&client->dev, "Program: write_ic(0x%2X) success", status);

	// step4:verify  
	dev_info(&client->dev, "Program: Verify begin, please wait...");
	ret=check_checksum(fw_data);
	if(ret==1){
		dev_info(&client->dev, "Program: Verify Pass!");
	}else if(ret==0){
		dev_info(&client->dev, "Program: Verify NG!");
	}else if(ret==-1){
		dev_info(&client->dev, "Program: Verify FW not return!");
	}

	dev_info(&client->dev, "Program: END");
	return ret;
}

int  update_firmware_request(void)
{
	struct i2c_client *client = ts->client;	
	int ret = 0;

	snprintf(firmware_name, sizeof(firmware_name), FIRMWARE_NAME);

	dev_info(&client->dev, "firmware_name = %s\n", firmware_name);
	ret = request_firmware(&fw_entry, firmware_name, &client->dev);
	if (ret) {
		dev_err(&client->dev, "Firmware %s not available, code = %d\n",
			firmware_name, ret);
		return  ret;
	}

	dev_info(&client->dev, "Got firmware, size: %d.\n", (int)fw_entry->size);
	dev_info(&client->dev, "fw_entry.data[0x7F00]: %d.\n", *(fw_entry->data + 0x7F00));
	
	return ret;
}

void update_firmware_release(void)
{
	if (fw_entry){
		release_firmware(fw_entry);
	}
	fw_entry = NULL;
}

void set_app_info(void)
{
	int ret = -1;
	int fw_ver = get_fw_ver();

	snprintf(touch_info,sizeof(touch_info),"NOVATEK-000-%x",fw_ver);
	ret = app_info_set("touch_panel", touch_info);
	if (ret < 0) {
		printk("nvt app_info_set failed\n");
	}
	return;
}

void ckeck_and_update_firmware(struct work_struct *nvt_fw_updatew_wq)
{
	struct i2c_client *client = ts->client;
	int ret = 0;

	ret = update_firmware_request();
	if(ret < 0){
		dev_info(&client->dev, "update firmware request failed.\n");
		return ;
	}

	ret = check_checksum((u8 *)fw_entry->data);	
	if(ret==-1){	// read fw checksum failed	
		dev_info(&client->dev, "checksum not match,need upgrd.\n");
		update_firmware((u8 *)fw_entry->data);
	}
	// (fw checksum not match) && (bin fw version > ic fw version)
	else if(ret==0&&(check_fw_ver((u8 *)fw_entry->data)==0))	{
		dev_info(&client->dev, "bin firmware version is higher ,need upgrd.\n");
		update_firmware((u8 *)fw_entry->data);
	}
	else {
		dev_info(&client->dev, "firmware is same,ver:0x%x.\n", *((u8 *)fw_entry->data + 0x7F00));
		dev_info(&client->dev, "firmware need not to upgrd.\n");
	}

	nvt_hw_reset();
	msleep(200);
	
	update_firmware_release();

	set_app_info();
}

void auto_update(void)
{
	INIT_DELAYED_WORK(&nvt_fw_updatew_wq, ckeck_and_update_firmware);
	schedule_delayed_work(&nvt_fw_updatew_wq, msecs_to_jiffies(2000));
}


