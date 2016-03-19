
/*******************
  Nvt Touchpanel Test
********************/
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "nvt.h"
#include "nvt_test.h"

#define MAXBUF 2000
#define RAW_DATA_SIZE 1024
#define FW_DATA_LENGTH	32768
#define FW_BUF  40960
#define MANUAL_UPDATE_FIRMWARE_ENABLE

int xnum;
int ynum;
int iaccum;
int fcf;
int fcc;
mapping_t mappingtable[45][35];
unsigned char gridcolor_flag[40][40];
unsigned char record_result[40*40];	
int mutual_goldenratio[40][40];
int mutual_data[40*40];
int cm_data[40*40];

void nvt_sw_reset(void)
{
	uint8_t buffer[2]={0x00,0x5A};
	nvt_i2c_write(ts->client, I2C_HW_Address, buffer, 2);	
}

void nvt_sw_reset_idle(void)
{
	uint8_t buffer[2]={0x00,0xA5};
	nvt_i2c_write(ts->client, I2C_HW_Address, buffer, 2);	
}

uint8_t nvt_read_firmware_version(void)
{
	uint8_t buffer[16]={0};
	buffer[0]=0x78;
	nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 5);
	msleep(50);
	xnum = buffer[3];
	ynum = buffer[4];
	return buffer[1];
}

static ssize_t nvt_sysfs_reset_touch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	nvt_sw_reset();
	msleep(500);	
	return snprintf(buf, PAGE_SIZE, "reset touch ic!\n");
}


int gpioshort_allcheck(uint8_t *buffer)
{
	unsigned short timeoutcnt1;
	timeoutcnt1 = 0;
short_allcheck:

	buffer[0]=0xFF;
	buffer[1]=0x3F;
	buffer[2]=0xE8;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);	
	msleep(1);
	
	buffer[0]=0x00;
	buffer[1]=0xC5;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);	
	msleep(100);

	while(1){
		buffer[0]=0xFF;
		buffer[1]=0x3F;
		buffer[2]=0xE9;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);	
		msleep(1);

		buffer[0]=0x00;
		buffer[1]=0x00;		
		nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 8);
		if(buffer[1]==0xBB){
			break;
		}
		msleep(1);
		
		timeoutcnt1++;
		if(timeoutcnt1 > 3){
			return 1;
		}else{
			goto short_allcheck;
		}		
	}	
	return 0;
}

char* gpio2ain2txrx(char *fail,unsigned char gpio)
{
	unsigned char  ii, jj;
	int record_ain=0;
	if(AIN[gpio] == -1){
		if(!record_ain){
			sprintf(fail, "UnusedPin, ");
		}else{
			sprintf(fail,"UnusedPin%d[AIN%d], ", (AIN[gpio]-32), (gpio));
		}
	}else if(AIN[gpio]>=32){
		if(!record_ain){
			sprintf(fail,"rx%d, ", (AIN[gpio]-32));
		}else{
			sprintf(fail,"rx%d[AIN%d], ", (AIN[gpio]-32), (gpio));
		}
		for(jj =0; jj< ynum;jj++){
			for(ii =0; ii < xnum; ii++){
				if(mappingtable[jj][ii].rx == (AIN[gpio]-32)){
					gridcolor_flag[ii][jj] = PUROPLE;
				}
			}
		}		
	}else{
		if(!record_ain){
			sprintf(fail,"tx%d, ", (AIN[gpio]));
		}else{
			sprintf(fail,"tx%d[AIN%d], ", (AIN[gpio]), (gpio));
		}
		for(jj =0; jj< ynum;jj++){
			for(ii =0; ii < xnum; ii++){
				if(mappingtable[jj][ii].tx == (AIN[gpio])){
					gridcolor_flag[ii][jj] = PUROPLE;
				}
			}
		}
	}	
	return fail;
}


int enter_test_mode(void)
{
	int timeoutcnt0=0, timeoutcnt1=0;
	uint8_t buffer[16]={0};
lenter_test_mode:
	buffer[0]=0xFF;
	buffer[1]=0x3F;
	buffer[2]=0xE8;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
	msleep(1);

	buffer[0]=0x00;
	buffer[1]=0xCC;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);
	msleep(500);
	
	while(1)
	{
		buffer[0]=0xFF;
		buffer[1]=0x3F;
		buffer[2]=0xE9;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
		msleep(1);

		buffer[0]=0x00;
		nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 2);
		if(buffer[1]==0xBB){
			break;
		}else {
			msleep(1);
			timeoutcnt0++;
			if(timeoutcnt0 > 100){
				timeoutcnt1++;
				timeoutcnt0=0;
				if(timeoutcnt1 > 3){
					return 1;
				}else{
					goto lenter_test_mode;
				}
			}
		}
	}
	return 0;
}

void leave_test_mode(void)
{	
	uint8_t buffer[16]={0};
	buffer[0]=0xFF;
	buffer[1]=0x3F;
	buffer[2]=0xE9;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
	msleep(1);

	buffer[0]=0x00;
	buffer[1]=0xAA;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);
	msleep(1);
}
	
void get_tp_cm_parameter(void)
{
	uint8_t buffer[32]={0};
	const int cf_table[8] = { 222,  327,  428,  533,  632,  737, 838, 943};
	const int cc_table[32] = { 30,  57,  82,  109,  136,  163, 188, 215,
								    237,  264,  289,  316,  343,  370 , 395, 422,
								    856,  883,  908,  935,  962,  989,1014,1041,
								   1063, 109 , 1115, 1142, 1169, 1196,1221,1248};
	//Reset IC
	nvt_hw_reset();
	msleep(500);

	buffer[0]=0xFF;
	buffer[1]=0x3F;
	buffer[2]=0xB7;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
	msleep(1);

	buffer[0]=0x00;
	nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 25);
               
	iaccum =  buffer[1];
	fcf = cf_table[buffer[9]& 0x07];
	fcc = cc_table[((buffer[9]& 0xF8)>>4) |(((buffer[9]& 0xF8)& 0x08) <<1)];

	leave_test_mode();
}

int check_fw_status(void)
{
	uint8_t buffer[16]={0};
	int i;
	for(i=0;i<100;i++){
		msleep(1);
		buffer[0]=0xFF;
		buffer[1]=0x3D;
		buffer[2]=0xFB;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
		msleep(1);

		buffer[0]=0x00;
		nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 2);

		if(buffer[1]==0xAA){
			break;
		}
		msleep(10);
	}

	if(i==100){
		return -1;
	}else{
		return 0;
	}
}

void readraw_nt11205(void)
{
	unsigned int startAddr;
	int i, j, k, m;
	int bytecount, sec_ct, Residual_1, sec_ct2, Residual_2, temp_cnt, offsetAddr;
	uint8_t buffer[64]={0};
	int *diff_temp = NULL;
	diff_temp=(int *)kzalloc(sizeof(int)*5000,GFP_KERNEL);
	temp_cnt=0;
	bytecount=xnum*ynum*2;
	sec_ct=bytecount/244;
	Residual_1=bytecount%244;
	sec_ct2=Residual_1/61;
	Residual_2=Residual_1%61;
	startAddr=0x0800;
	
	for(m=0;m<sec_ct;m++){
		offsetAddr=0;
		buffer[0]=0xFF;
		buffer[1]=startAddr>>8;
		buffer[2]=startAddr&0xFF;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
		msleep(1);

		for(k=0;k<4;k++){
			buffer[0]=offsetAddr&0xFF;
			nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 62);
			offsetAddr+=61;
			for(i=0;i<61;i++){
				diff_temp[temp_cnt++]=(int)buffer[1+i];
			}
		}
		startAddr+=offsetAddr;
	}
	
	if(Residual_1>0){
		offsetAddr=0;
		buffer[0]=0xFF;
		buffer[1]=startAddr>>8;
		buffer[2]=startAddr&0xFF;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
		msleep(1);

		for(k=0;k<sec_ct2;k++){
			buffer[0]=offsetAddr&0xFF;
			nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 62);
			offsetAddr+=61;
			for(i=0;i<61;i++){
				diff_temp[temp_cnt++]=(int)buffer[1+i];
			}
		}
		startAddr+=offsetAddr;
		if(Residual_2>0){
			offsetAddr=0;
			buffer[0]=0xFF;
			buffer[1]=startAddr>>8;
			buffer[2]=startAddr&0xFF;
			nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
			msleep(1);

			buffer[0]=offsetAddr&0xFF;
			nvt_i2c_read(ts->client, I2C_FW_Address, buffer, Residual_2+1);
			for(i=0;i<Residual_2;i++){
				diff_temp[temp_cnt++]=(int)buffer[1+i];
			}
		}
	}

	for(j = 0; j<ynum; j++){
		for(i=0; i< xnum; i++){
			mutual_data[j*xnum+i] = (unsigned short)((diff_temp[2*(j*xnum+i)]<<8)|diff_temp[2*(j*xnum+i)+1]);
			printk("%d, ", mutual_data[j*xnum+i]);
		}
		printk("\n");
	}
	printk("\n");	
	kfree(diff_temp);
}

void rawdata_to_cm(void)
{
	int i,j;
	int kk=0;
	int RepeatCnt = 0;
	int temp1;	
	uint8_t buffer[16]={0};
	unsigned short *rawdata = NULL;
	rawdata = (unsigned short *)kzalloc(sizeof(unsigned short )*1000,GFP_KERNEL);
againgetdata:

		//Reset IC
		nvt_hw_reset();
		msleep(500);

		//SendCmdToGetrawdata();
		buffer[0]=0xFF;
		buffer[1]=0x3F;
		buffer[2]=0xE8;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
		msleep(1);			
		buffer[0]=0x00;
		buffer[1]=0xCF;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);
		msleep(500);	

		buffer[0]=0xFF;
		buffer[1]=0x3D;
		buffer[2]=0xFC;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
		msleep(1);			
		buffer[0]=0x00;
		buffer[1]=0xAA;
		buffer[2]=0x5A;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
		msleep(500);

		//Backup data at last time
		for(j = 0 ; j< ynum; j++){
			for(i = 0; i < xnum; i++){
				rawdata[j*xnum + i] = mutual_data[j*xnum + i];
			}
		}
		
		//Get data 
		if(check_fw_status()==-1){
			kfree(rawdata);
			return;
		}

		//get Rawdata		
		readraw_nt11205();

		kk =0;
		for(j = 0 ; j< ynum; j++) {
			for(i = 0; i < xnum; i++){
				if(abs(rawdata[j*xnum + i] - mutual_data[j*xnum + i] >1000)){
					kk++;
					if(kk > 2){
						RepeatCnt++;
						goto againgetdata;
					}
				}
			}
		}
		//\C2\u09a8CM
		temp1=(int)(((2*(long int)fcc*10/3)*10000/fcf));
		printk("iaccum=%d\n", iaccum);
		printk("temp1=%d\n", temp1);
		printk("fcc=%d\n", fcc);
		printk("fcf=%d\n", fcf);		
		printk("cm_data\n");
		
		for(j = 0; j<ynum; j++){
			for(i=0; i< xnum; i++){
				cm_data[xnum*j +i]= (int)((((((((mutual_data[xnum*j +i]/((iaccum+1)/2))-1024)*24)+temp1)/2)*3/10)*fcf/100));
				printk("%d, ", cm_data[xnum*j +i]);
			}
			printk("\n");
		}
		printk("\n");

		kfree(rawdata);
}

int test_gratio_and_normal(void)
{
	int i,j,k;
	long int tmpvalue;
	long int maxsum=0;
	int maxnum=0, maxindex=0;
	int max = -9999999; 
	int min =  9999999;
	int offset;	
	int data;	
	int statisticsstep=0;

	int *statistics_num = NULL;
	long int *statistics_sum = NULL;
	statistics_num = (int *)kzalloc(sizeof(int)*MAXBUF,GFP_KERNEL);
	statistics_sum = (long int *)kzalloc(sizeof(long int)*MAXBUF,GFP_KERNEL);
		

	//1. (Testing_CM - Golden_CM ) / Testing_CM
	for(j = 0 ;j < ynum; j++){
		for(i=0 ; i < xnum; i++){
            		data = cm_data[j*xnum + i];
            		if(data == 0){
				data = 1;
            		}
			mutual_goldenratio[j][i] = data - Mutual_AVG[j*xnum + i];
			mutual_goldenratio[j][i] /=  data ;	
		}
	}	

	// 2. mutual_goldenratio*10000 
	for(j = 0 ; j < ynum; j++){
		for(i = 0 ; i < xnum; i++){
			mutual_goldenratio[j][i] *=  10000 ;					
		}
	}
	
	// 3. Calculate statisticsstep
	for(j = 0 ;j < ynum; j++){
		for(i=0 ; i < xnum; i++){				
			//mutual_goldenratio[j][i] *=  10000 ;
			if(max < (int)mutual_goldenratio[j][i]){
				max  =  (int)mutual_goldenratio[j][i];
			}
			if(min > (int)mutual_goldenratio[j][i]){
				min = (int)mutual_goldenratio[j][i];
			}
		}
	}
	
	offset = 0;
	if(min < 0) {
		offset =  0 - min ;
		offset += statisticsstep;		
		for(j =0 ; j < ynum ; j++){
			for(i = 0 ; i < xnum; i++){
				mutual_goldenratio[j][i] += offset;
			}
		}				
		max += offset;
	}
	 statisticsstep = max /  MAXBUF;
	statisticsstep+=1;
	if(statisticsstep < 0) {
		kfree(statistics_num);
		kfree(statistics_sum);
		return 1;
	}
             
	// 4. Start Statistics and Average
	for(j = 0 ;  j < ynum ; j++){
		for(i =0; i < xnum; i++){
			tmpvalue = (long int)mutual_goldenratio[j][i];
			tmpvalue /= statisticsstep;
			statistics_num[tmpvalue] += 2;
			statistics_sum[tmpvalue] += (2* (long int)mutual_goldenratio[j][i]);

			if((tmpvalue + 1) <  MAXBUF){
				statistics_num[tmpvalue+1] += 1;
				statistics_sum[tmpvalue+1] += (long int)mutual_goldenratio[j][i] ;
			}
			if((tmpvalue - 1) <  MAXBUF){
				statistics_num[tmpvalue - 1] += 1;
				statistics_sum[tmpvalue - 1] += (long int)mutual_goldenratio[j][i];	
			}
		}
	}
	
	//Find out max Statistics
	maxnum =0;
	for(k = 0 ; k <  MAXBUF ;k++){		
		if(maxnum < statistics_num[k]){
			maxsum = statistics_sum[k];
			maxnum = statistics_num[k];	
			maxindex = k;
		}			
	}
	
	//Caluate Statistics Average
	if(maxsum > 0){
		tmpvalue =(long int)(statistics_sum[maxindex]  /(long int)statistics_num[maxindex])*2;
		if((maxindex+1) < (MAXBUF)) {
			tmpvalue+= (long int)(statistics_sum[maxindex+1]/(long int)statistics_num[maxindex+1]); 
		}
		if((maxindex-1) >= 0 ){
			tmpvalue+= (long int)(statistics_sum[maxindex-1]/(long int)statistics_num[maxindex-1]);
		}

		if((maxindex+1) < (MAXBUF) &&( (maxindex-1) >=0)) {
			tmpvalue /=4;
		}else{
			tmpvalue /=3;
		}
	}else{
		statistics_sum[0] = 0;
		statistics_num[0] = 0;
		for(j =0; j <ynum; j++){
			for(i = 0 ; i < xnum; i++){
				statistics_sum[0] += (long int)mutual_goldenratio[j][i];
				statistics_num[0]++;
			}
		}
		tmpvalue = statistics_sum[0] / statistics_num[0];		
	}

	tmpvalue -= offset;
	for(j =0 ;j < ynum ; j++){
		for(i=0; i < xnum;  i++){
			mutual_goldenratio[j][i] -= offset;
			mutual_goldenratio[j][i] =  mutual_goldenratio[j][i] - tmpvalue;
			mutual_goldenratio[j][i] =  mutual_goldenratio[j][i] / 10000;
		}
	}
	kfree(statistics_num);
	kfree(statistics_sum);
	return 0;
}

static ssize_t nvt_sysfs_sensor_short_test(char *buf)
{
	unsigned char port;	
	unsigned char fail_ain;
	unsigned char test_pin;
	unsigned char gpio_shortlist[48];
	unsigned char gpio_shortcnt=0;
	char fail[64] = { 0 };
	uint8_t buffer[16]={0};
	int i,j;
	int ret=0;
	int len = 0;
	char *gpiomsg = NULL;
	gpiomsg = (char *)kzalloc(sizeof(char)*2048,GFP_KERNEL);
	memset(gpio_shortlist, 0xff, sizeof(gpio_shortlist));  
	gpio_shortcnt = 0;

	printk("\nshort test:\n");
	//Get XY channel number
	nvt_read_firmware_version();

	//Reset IC
	nvt_hw_reset();
	msleep(500);
	
	//Set Delay
	buffer[0]=0xFF;
	buffer[1]=0x3F;
	buffer[2]=0xF4;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
	msleep(1);

	buffer[0]=0x00;
	buffer[1]=255;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);
	msleep(1);

	//All AINs Check in  ShortTest originally 
	if(gpioshort_allcheck(buffer) >0){
		sprintf(gpiomsg,"FW is Timeout!");
		len = snprintf(buf, PAGE_SIZE, "Short Test ,FAIL\n");
		kfree(gpiomsg);
		return 1;
	}
	for(i =0 ; i< 6; i++){
		if(buffer[i+ 2] > 0){
		    port = buffer[i+2];
			for(j = 0; j < 8; j++){
				test_pin = (0x01  << j);
				if((port & test_pin) >0){
					fail_ain = (i* 8) + j;
					if(AIN[fail_ain]==-1){
						continue;
					}else {
						gpio_shortlist[gpio_shortcnt] = fail_ain;						
						gpio_shortcnt++;
					}				
				}			
			}
		}		
	}

	//Get Result 
	ret =0;
	
	//gpiomsg = "{";
	sprintf(gpiomsg, "{");
	for(j = 0 ; j < gpio_shortcnt; j++){
		i = gpio_shortlist[j];
		if(i >= 0xff){
			continue;
		}else if(AIN[i]==-1){
			continue;
		}
		memset(fail, 0, sizeof(fail));
		gpio2ain2txrx(fail, i);
		sprintf(gpiomsg,"%s%s", gpiomsg, fail);
		ret++;
	}

	if(ret <= 1){
		sprintf(gpiomsg,"%sGND,},", gpiomsg);
	}else{
		sprintf(gpiomsg,"%s},", gpiomsg);
	}

	nvt_hw_reset();	
	if(ret > 0){
		len = snprintf(buf, PAGE_SIZE, "Short Test ,FAIL\n");
		printk("Short Test is FAIL=>%s\n", gpiomsg);
	}else{
		len = snprintf(buf, PAGE_SIZE, "Short Test ,PASS\n");
		printk("Short Test ,PASS\n");
	}
	kfree(gpiomsg);
	return len;
}

static ssize_t nvt_sysfs_sensor_short_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return nvt_sysfs_sensor_short_test(buf);
}

static ssize_t nvt_sysfs_sensor_open_test(char *buf)
{
	int i,j;
	int kk=0;
	printk("\nopen test:\n");
	//Get XY channel number
	nvt_read_firmware_version();
	
	//Reset IC
	nvt_hw_reset();
	msleep(500);

	// Init record_result array
	for(j =0 ; j < ynum ; j++){
		for(i = 0; i <xnum; i++){
			if(gridcolor_flag[i][j]  != WHITE){
				record_result[j*xnum + i] =  0x80;
			}else {
				record_result[j*xnum + i] =  0x00;
			}
		}
	}

	// Get rawdata and To CM 
	get_tp_cm_parameter();
	rawdata_to_cm();
	for(j = 0; j<ynum; j++){
		for(i=0; i< xnum; i++){
			cm_data[xnum*j +i]-= FPC_CM[xnum*j +i];
		}
	}
	for(j = 0; j < ynum ;  j++){
		for(i = 0 ; i<xnum ; i++){
			if(cm_data[j*xnum + i] == 0){
				 cm_data[j*xnum + i] = 1;
			}else if(cm_data[j*xnum + i] < 0){		
				 cm_data[j*xnum + 1] = 1;
			}
		}
	}

	//  Check abs low boundary
	for(j =0 ; j < ynum ; j++){
		for(i = 0; i <xnum; i++){
			kk = (int)((Mutual_AVG[j*xnum+i]*(1000-ITOLERANCE_S))/1000);
			if(cm_data[j*xnum + i]  < kk ){
				record_result[j*xnum + i] |= 1;
			}
			kk = (int)((Mutual_AVG[j*xnum+i]*(1000+IPOSTIVE_TOLERANCE))/1000);
			if(cm_data[j*xnum + i]  > kk )	{
				record_result[j*xnum + i] |= 1;
			}        
		}
	}

	// (Testing_CM - Golden_CM ) / Testing_CM
	test_gratio_and_normal();

	// Check Golden Ratio Test
	for(j=0;j<ynum;j++){	
		for(i=0; i<xnum;i++){
			if(mutual_goldenratio[j][i]*1000 < IDIFF_LIMITG){
				record_result[j*xnum+ i]|=  0x02;
			}
		}
	}

	// Record Test Result
	for(j=0;j<ynum;j++){
		for(i=0; i<xnum;i++){
			kk = 0;
			if((record_result[j*xnum+ i] & 0x01) > 0){
				kk++;
			}
			if((record_result[j*xnum+ i] & 0x02) > 0){
				kk++;
			}
		}
	}

	nvt_hw_reset();
	if(kk >=1){ 
		return snprintf(buf, PAGE_SIZE, "Self Open Test ,FAIL\n");					
	}else{ 
		return snprintf(buf, PAGE_SIZE, "Self Open Test ,PASS\n");					
	}
}

static ssize_t nvt_sysfs_sensor_open_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return nvt_sysfs_sensor_open_test(buf);
}

static ssize_t nvt_sysfs_sensor_uniformity_test(char *buf)
{
	int i,j;
	int retry=0;
	int Testmin, Testmax;
	uint8_t buffer[16]={0};

	printk("\nuniformity test:\n");
	get_tp_cm_parameter();

	buffer[0]=0xFF;
	buffer[1]=0x3D;
	buffer[2]=0xFC;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
	msleep(1);

	buffer[0]=0x00;
	buffer[1]=0xAA;
	buffer[2]=0x5B;
	nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
	msleep(1);

	retry=0;
	while(retry<3){
		buffer[0]=0xFF;
		buffer[1]=0x3D;
		buffer[2]=0xFB;
		nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
		msleep(1);

		buffer[0]=0x00;
		nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 2);
		if(buffer[1]==0xAA){
			break;
		}else {
			msleep(1);
			retry++;
		}
	}
	if(retry>=3){
		return snprintf(buf, PAGE_SIZE, "Self Uniformity Test ,FAIL\n");
	}else{
		readraw_nt11205();
	}

	Testmin=0xFFFF;
	Testmax=0;
	for(j = 0; j<ynum; j++){
		for(i=0; i< xnum; i++){
			if(mutual_data[xnum*j +i]>Testmax){
				Testmax=mutual_data[xnum*j +i];
			}
			if(mutual_data[xnum*j +i]<Testmin){
				Testmin=mutual_data[xnum*j +i];
			}
		}
	}

	nvt_hw_reset();
	if(Testmin*1000 < Testmax*IUNIFORMITY){ 
		return snprintf(buf, PAGE_SIZE, "Self Uniformity Test ,FAIL\n");
	}else{ 
		return snprintf(buf, PAGE_SIZE, "Self Uniformity Test ,PASS\n");
	}
}

static ssize_t nvt_sysfs_sensor_uniformity_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return nvt_sysfs_sensor_uniformity_test(buf);
}

static ssize_t nvt_sysfs_sensor_abslimit_test(char *buf)
{
	int maxCm = 0; 
	int minCm = 65535;	
	int i,j;
	printk("\nabslimit test:\n");
	//Get XY channel number
	nvt_read_firmware_version();
	
	//Reset IC
	nvt_hw_reset();
	msleep(500);

	// Init record_result array
	for(j =0 ; j < ynum ; j++){
		for(i = 0; i <xnum; i++){
			if(gridcolor_flag[i][j]  != WHITE){
				record_result[j*xnum + i] =  0x80;
			}else {
			      record_result[j*xnum + i] =  0x00;
		       }
	      }
	}
		
	// Get rawdata and To CM 
	get_tp_cm_parameter();
	rawdata_to_cm();
	for(j = 0; j<ynum; j++){
		for(i=0; i< xnum; i++){
			cm_data[xnum*j +i]-= FPC_CM[xnum*j +i];
			printk("%d, ", cm_data[xnum*j +i]);
		}
		printk("\n");
	}
	printk("\n");

	for(i=0; i<xnum*ynum; i++){
		if(cm_data[i] > maxCm){
			maxCm = cm_data[i];
		}else if(cm_data[i] < minCm){
			minCm = cm_data[i];
		}
	}

	printk("maxCm: %d\n", maxCm);
	printk("minCm: %d\n", minCm);
	printk("IABSLIMIT_MAX: %d\n", IABSLIMIT_MAX);
	printk("IABSLIMIT_MIN %d\n", IABSLIMIT_MIN);
	  
	nvt_hw_reset();	
	
	if(maxCm > IABSLIMIT_MAX){
		return snprintf(buf, PAGE_SIZE, "Self ABSLimit Test ,FAIL\n");
	}
	
	if(minCm < IABSLIMIT_MIN){
		return snprintf(buf, PAGE_SIZE, "Self ABSLimit Test ,FAIL\n");
	}else{
		return snprintf(buf, PAGE_SIZE, "Self ABSLimit Test ,PASS\n");
	}
}
static ssize_t nvt_sysfs_sensor_abslimit_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return nvt_sysfs_sensor_abslimit_test(buf);
}

static ssize_t nvt_sysfs_mmi_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char tmp_buf[64] = { 0 };
	char mmi_result_buf[256] = { 0 };
	int len = 0;

	//short_test
	memset(tmp_buf, 0, sizeof(tmp_buf));
	len = nvt_sysfs_sensor_short_test(tmp_buf);
	strncat(mmi_result_buf, tmp_buf, len );
	msleep(100);
	
	//open test
	memset(tmp_buf, 0, sizeof(tmp_buf));
	len = nvt_sysfs_sensor_open_test(tmp_buf);
	strncat(mmi_result_buf, tmp_buf, len);
	msleep(100);
	
	//abslimit test
	memset(tmp_buf, 0, sizeof(tmp_buf));
	len = nvt_sysfs_sensor_abslimit_test(tmp_buf);
	strncat(mmi_result_buf, tmp_buf, len);
	msleep(100);
	
	//uniformity test
	memset(tmp_buf, 0, sizeof(tmp_buf));
	len = nvt_sysfs_sensor_uniformity_test(tmp_buf);
	strncat(mmi_result_buf, tmp_buf, len);
	
	return snprintf(buf, PAGE_SIZE, mmi_result_buf);
}

#ifdef  MANUAL_UPDATE_FIRMWARE_ENABLE

static int nvt_file_open_firmware(u8 *file_path,u8 *databuf,
                int *file_size)
{
    struct file *filp = NULL;
    struct inode *inode = NULL;
    unsigned int file_len = 0;
    mm_segment_t oldfs;
    int retval = 0;

    if(file_path == NULL || databuf == NULL){
        printk("%s,path || buf is NULL.\n",__func__);
        return -EINVAL;
    }

    printk("%s,path = %s.\n",__func__,file_path);

    // open file
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    filp = filp_open(file_path, O_RDONLY, S_IRUSR);
    if (IS_ERR(filp)){
        printk("%s: open %s error.\n", __func__,file_path);
        retval = -EIO;
        goto err;
    }

    if (filp->f_op == NULL) {
        printk("%s: File Operation Method Error\n", __func__);
        retval = -EINVAL;
        goto exit;
    }

    inode = filp->f_path.dentry->d_inode;
    if (inode == NULL) {
        printk("%s: Get inode from filp failed\n", __func__);
        retval = -EINVAL;
        goto exit;
    }

    //Get file size
    file_len = i_size_read(inode->i_mapping->host);
    if (file_len == 0){
        printk("%s: file size error,file_len = %d\n", __func__,file_len);
        retval = -EINVAL;
        goto exit;
    }

    // read image data to kernel */
    if (filp->f_op->read(filp, databuf, file_len, &filp->f_pos) != file_len) {
        printk("%s: file read error.\n", __func__);
        retval = -EINVAL;
        goto exit;
    }

    *file_size = file_len;

exit:
    filp_close(filp, NULL);
err:
    set_fs(oldfs);
    return retval;
}


static ssize_t update_firmware_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = -1;
	int size = 0;
	char *path = "/sdcard/dload/novatek_fw.bin" ;
	void *ext_fw_data = NULL;
	ext_fw_data = kzalloc(FW_BUF, GFP_KERNEL);
	if (ext_fw_data == NULL) {
		printk("%s:malloc fw_data failed\n", __func__);
		return -1;
	}

	ret = nvt_file_open_firmware(path,(u8 *)ext_fw_data,&size);
	if(ret != 0 || ((size > FW_DATA_LENGTH)||(size <= 0))){
		printk("%s: open firmware error\n", __func__);
		goto open_fw_error;
	}else{
		printk("fw_data size %d\n",size);
	}

	ret = update_firmware((u8 *)ext_fw_data);
	if(ret==1){
		ret = snprintf(buf, PAGE_SIZE, "update firmware Pass!\n");
	}else if(ret==0){
		ret = snprintf(buf, PAGE_SIZE, "update firmware NG!\n");
	}else if(ret==-1){
		ret = snprintf(buf, PAGE_SIZE, "verify FW not return!\n");
	}

	nvt_hw_reset();
	msleep(200);

open_fw_error:	
	kfree(ext_fw_data);
	ext_fw_data = NULL;

	return ret;
}
#endif

static ssize_t firmware_version_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{	
	uint8_t version = nvt_read_firmware_version();
	return snprintf(buf, PAGE_SIZE, "0x%02X\n", version);
}

static ssize_t firmware_tptype_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", 
			"NOVATEK-000");
}


/* SYSFS : Device Attributes */
static DEVICE_ATTR(firmware_version,             0444,          firmware_version_show,                             NULL);
static DEVICE_ATTR(reset_touch,				S_IRUGO,	nvt_sysfs_reset_touch_show,				NULL);
static DEVICE_ATTR(sensor_short_test,		S_IRUGO,	nvt_sysfs_sensor_short_test_show,		NULL);
static DEVICE_ATTR(sensor_open_test,		S_IRUGO,	nvt_sysfs_sensor_open_test_show,		NULL);
static DEVICE_ATTR(sensor_uniformity_test,	S_IRUGO,	nvt_sysfs_sensor_uniformity_test_show,	NULL);
static DEVICE_ATTR(sensor_abslimit_test,		S_IRUGO,	nvt_sysfs_sensor_abslimit_test_show,		NULL);
static DEVICE_ATTR(mmi_test_result,		      S_IRUGO,	nvt_sysfs_mmi_test_show,		             NULL);
#ifdef MANUAL_UPDATE_FIRMWARE_ENABLE
static DEVICE_ATTR(manual_update_firmware,  0444, update_firmware_show,NULL);
#endif
static DEVICE_ATTR(firmware_tptype, 0444, firmware_tptype_show,NULL);

static struct attribute *nvt_ts_attributes[] = {
	&dev_attr_reset_touch.attr,
	&dev_attr_sensor_short_test.attr,
	&dev_attr_sensor_open_test.attr,
	&dev_attr_sensor_uniformity_test.attr,
	&dev_attr_sensor_abslimit_test.attr,
	&dev_attr_mmi_test_result.attr,
	&dev_attr_firmware_version.attr,
#ifdef MANUAL_UPDATE_FIRMWARE_ENABLE
	&dev_attr_manual_update_firmware.attr, 
#endif
	&dev_attr_firmware_tptype.attr,
	NULL
};

static struct attribute_group nvt_ts_attribute_group = {
    .attrs = nvt_ts_attributes
};

void nvt_sysfs_init(void)
{
	int ret=0;
	struct kobject *kobject_ts = NULL;

	kobject_ts = kobject_create_and_add("touchscreen", NULL);
	if (!kobject_ts) {
		printk("create kobjetct error!\n");
		return ;
	}

	ret = sysfs_create_group(kobject_ts, &nvt_ts_attribute_group);
	if (ret){
		printk("%s() - ERROR: sysfs_create_group() failed: %d\n", __func__, ret);
	}else{
		printk("%s() - sysfs_create_group() succeeded.\n", __func__);
	}
}

static void nvt_mutual_data_print(struct seq_file *m)
{
	int i,j;
	for(j = 0; j<ynum; j++){
		for(i=0; i< xnum; i++){
			seq_printf(m,"%6d ", mutual_data[j*xnum+i]);
		}
		seq_printf(m,"\n");
	}
	seq_printf(m,"\n");
}

static void nvt_cm_data_print(struct seq_file *m)
{
	int i , j;
	int temp1;
	temp1=(int)(((2*(long int)fcc*10/3)*10000/fcf));
	for(j = 0; j<ynum; j++){
		for(i=0; i< xnum; i++){
			cm_data[xnum*j +i]= (int)((((((((mutual_data[xnum*j +i]/((iaccum+1)/2))-1024)*24)+temp1)/2)*3/10)*fcf/100));
			seq_printf(m,"%6d ",  cm_data[xnum*j +i]);
		}
		seq_printf(m,"\n");
	}
	seq_printf(m,"\n");
}


static int rawdata_proc_show(struct seq_file *m, void *v)
{
	int i, j;
	char tmp_buf[64] = { 0 };

	if(m->size <= RAW_DATA_SIZE*50) {
		m->count = m->size;
		return 0;
	}

	seq_printf(m, "*************touch data*************\n");

	//short test
	seq_printf(m,"short test:\n");
	nvt_sysfs_sensor_short_test(tmp_buf);
	seq_printf(m,tmp_buf);
	seq_printf(m,"\n");

	//open test
	msleep(30);
	seq_printf(m,"open test:\n");
	memset(tmp_buf, 0, sizeof(tmp_buf));
	nvt_sysfs_sensor_open_test(tmp_buf);
	seq_printf(m,tmp_buf);
	nvt_mutual_data_print(m);
	nvt_cm_data_print(m);

	//abslimit test
	msleep(30);
	seq_printf(m,"abslimit test:\n");
	memset(tmp_buf, 0, sizeof(tmp_buf));
	nvt_sysfs_sensor_abslimit_test(tmp_buf);
	seq_printf(m,tmp_buf);
	nvt_mutual_data_print(m);
	nvt_cm_data_print(m);
	for(j = 0; j<ynum; j++){
		for(i=0; i< xnum; i++){
			cm_data[xnum*j +i]-= FPC_CM[xnum*j +i];
			seq_printf(m,"%6d ", cm_data[xnum*j +i]);
		}
		seq_printf(m,"\n");
	}
	seq_printf(m,"\n");

	//uniformity test
	msleep(30);
	seq_printf(m,"uniformity test:\n");
	memset(tmp_buf, 0, sizeof(tmp_buf));
	nvt_sysfs_sensor_uniformity_test(tmp_buf);
	seq_printf(m,tmp_buf);
	nvt_mutual_data_print(m);

	return 0;
}

static int nvt_rawdata_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rawdata_proc_show, NULL);
}

static const struct file_operations nvt_rawdata_proc_fops = {
	.open		= nvt_rawdata_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void nvt_procfs_create(void)
{
	if (!proc_mkdir("touchscreen", NULL))
		return ;
	 proc_create("touchscreen/test_info", S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, NULL, &nvt_rawdata_proc_fops);
	return ;
}

