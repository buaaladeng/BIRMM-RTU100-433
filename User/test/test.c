

/***************************本程序为初始化设备********************************/
#include "test.h"
#include "time.h"
#include "SensorInit.h"
#include  "DS2780.h"
extern struct liquid_set DeviceConfig;

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




