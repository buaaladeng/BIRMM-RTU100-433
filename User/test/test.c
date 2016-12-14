

/***************************本程序为初始化设备********************************/
#include "test.h"
#include "time.h"
#include "SensorInit.h"
#include  "DS2780.h"
#include  "AiderProtocol.h"
extern struct liquid_set DeviceConfig;

extern unsigned char   Usart3_send_buff[300];    //3G模块发送数组
uint8_t SetTime[6]=SETTIME;
//(1)配置采集相关信息，分别为采集周期、上传周期、重传次数、低浓度阈值、高浓度阈值
void write_collect_parameter_to_flash(void)      //利用该函数可进行燃气智能终端采集配置   配置3G版本
{  
	uint8_t collect_period[2]={00,1};     //采集周期
  uint8_t send_out[2]={00,1};           //上传周期
  uint8_t retry_num = 3;                //重传次数
	uint8_t first_collect[2]={00,00};     //第一次采集时间
	uint8_t collect_num = 0;              //已采集数量
  uint8_t WorkNum[2]={00,00};         //已工作时间
 
	
	union Hex_Float low_alarm;
	union Hex_Float high_alarm;
   low_alarm.Data_Float =25.0;
   high_alarm.Data_Float =50.0;
	
  //PowerON_Flash(); 
	DataWrite_To_Flash(0,2,0,collect_period,2);  
	DataWrite_To_Flash(0,3,0,send_out,2);
  DataWrite_To_Flash(0,7,0,low_alarm.Data_Hex ,4);
	DataWrite_To_Flash(0,8,0,first_collect,2);
	DataWrite_To_Flash(0,9,0,&collect_num,1);
	DataWrite_To_Flash(0,10,0,&(retry_num),1);
	DataWrite_To_Flash(0,11,0,high_alarm.Data_Hex ,4);
  DataWrite_To_Flash(0,12,0, WorkNum ,2);
	
}

//(2)配置3G模块信息, 报警电路号码、IP端口号、PORT端口号
void write_3G_parameter_to_flash(void) 
{
	char    phone_num[16]={"861064617178004"};             //报警电话号码
	char    ip[16]={"119.254.103.80"};                     //网络服务IP地址
	char    port[6]={"2017"};                              //网络服务端口号
 
////////////////////将参数写入FLASH中////////////////////////////
  DataWrite_To_Flash(0,4,0,phone_num,strlen(phone_num));
	DataWrite_To_Flash(0,5,0,ip,strlen(ip));
	DataWrite_To_Flash(0,6,0,port,strlen(port));
}

//(3)配置433模块信息
void write_433_parameter_to_flash(void) 
{
	
	
}
//(4)配置时钟信息
void Set_Time(void)
{
//	time_t rawtime;
//  struct tm * timeinfo;
//  //rawtime= time((time_t*)NULL);
//  timeinfo = localtime (&rawtime);
//	DeviceConfig.Time_Year = timeinfo->tm_year;
//	DeviceConfig.Time_Mon  = timeinfo->tm_mon;
//	DeviceConfig.Time_Mday = timeinfo->tm_mday;
//	DeviceConfig.Time_Hour = timeinfo->tm_hour;
//	DeviceConfig.Time_Min  = timeinfo->tm_min;
//	DeviceConfig.Time_Sec  = timeinfo->tm_sec;
	
	DeviceConfig.Time_Year = SetTime[0];
	DeviceConfig.Time_Mon  = SetTime[1];
	DeviceConfig.Time_Mday = SetTime[2];
	DeviceConfig.Time_Hour = SetTime[3];
	DeviceConfig.Time_Min  = SetTime[4];
	DeviceConfig.Time_Sec  = SetTime[5];
	
	
	
	Time_Auto_Regulate(&DeviceConfig);
}

void Set_DS2780(void)
{
	Set_register_ds2780();    //掉电后对库仑计重新初始化
	set_ACR(1000);           //掉电后对库仑计重新初始化
	DS2780_CapacityInit();    //掉电后重新写电池容量
}


void Test_SendToServ(u8* pDeviceID)
{
//模拟进行报警上传
	struct TagStruct AlarmTAG,OIDTAG[3],ResponsTAG[3];
	int i;
	struct SenserData pGetData;
	pGetData.Ch4Data.Data_Float=30.0;
	pGetData.CollectTime = 120;
	
	AlarmTAG.OID_Command=(DeviceConfig.CollectPeriod<<11)+pGetData.CollectTime  +((0xC0 + REPORTDATA )<<24);
	AlarmTAG.OID_Command=ntohl(AlarmTAG.OID_Command);
	AlarmTAG.Width=4;
	for(i=0;i<AlarmTAG.Width;i++)
	AlarmTAG.Value[i]=pGetData.Ch4Data.Data_Hex[i];                                 //组成报警的TAG
	SendDataToServ(TRAPREQUEST,&AlarmTAG,1,Usart3_send_buff, pDeviceID);                //433模块主动上传数据
  Delay_ms(2000);
	
//模拟只上传OID
	ResponsTAG[0].OID_Command = SYSTERM_DATA;
	ResponsTAG[0].Width=0;
	ResponsTAG[1].OID_Command = CLT1_ITRL1;
	ResponsTAG[1].Width=0;
	ResponsTAG[2].OID_Command = UPLOAD_CYCLE;
	ResponsTAG[2].Width=0;
	SendDataToServ(GETRESPONSE,ResponsTAG,3,Usart3_send_buff, pDeviceID); 
	 Delay_ms(2000);
	 
//模拟上传接收下发配置TAG
	OIDTAG[0].OID_Command = SYSTERM_DATA;
	OIDTAG[0].Width=3;
	OIDTAG[0].Value [0]=16;
	OIDTAG[0].Value [1]=12;
	OIDTAG[0].Value [2]=14;
	
	OIDTAG[1].OID_Command = CLT1_ITRL1;
	OIDTAG[1].Width=2;
	OIDTAG[1].Value [0]=0;
	OIDTAG[1].Value [1]=0x3c;
	
	OIDTAG[2].OID_Command = UPLOAD_CYCLE;
	OIDTAG[2].Width=2;
	OIDTAG[2].Value [0]=0;
	OIDTAG[2].Value [1]=0xa;
	SendDataToServ(GETRESPONSE,OIDTAG,3,Usart3_send_buff, pDeviceID); 
	Delay_ms(2000);
	
	
	
	
}



