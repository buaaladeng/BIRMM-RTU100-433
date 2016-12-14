#ifndef _GPRS_H_
#define _GPRS_H_
#include "stm32f10x.h"
#include "AiderProtocol.h"


#define DEBUG_WORK 1   //打印功能，应用于正常显示


///////////////////////////////////////////////////////////////////////////////
//声明结构体
struct SMS_Config_RegPara 
{
	
	char     CurrentPhoneNum[16];      //当前通信手机号码，字符串格式
	char     AlarmPhoneNum[16];   //短信配置液位报警号码，字符串格式
	char     ServerIP[16];     //短信配置服务器IP，字符串格式
	char     ServerPort[6];    //短信配置服务器端口号，字符串格式
  
	union   Hex_Float LowAlarmLevel;
	union   Hex_Float HighAlarmLevel;
		
	
	uint8_t  CollectPeriod_Byte[2];    //数据采集间隔，使用数组存储，以方便Flash读写
	uint8_t  SendCount_Byte[2];        //一天上传液位数据次数，使用数组存储，以方便Flash读写  上传周期
	uint8_t  CollectStartTime_Byte[2];     //开始采集时间
	u8       CollectNum;                  //已经完成的采集数量
	uint8_t  RetryNum;                    //重传次数
	uint8_t  WorkNum_Byte[2];             //工作次数
	
	uint8_t  SensorDataInquireFlag;    //短信查询当前传感器数据标志变量
	uint8_t  DeviceSetInquireFlag;     //短信查询当前 传感器配置信息标志变量
	
};

//声明函数

void  mput(char *str);
void  mput_mix(char *str,int length);
char* Find_String(char *Source, char *Object);
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object);

void  USART_DataBlock_Send(USART_TypeDef *USART_PORT,char *SendUartBuf,u16 SendLength);   //批量向串口发送数据

void  ConfigData_Init(struct liquid_set* Para);

void  ParameterConfig_SX1278(char* pRecevCommand);  //配置433模块
void  ParameterInquire_SX1278(u8 InquireCommand);    //查询433配置信息
u16 Receive_Data_Analysis(u8* pDeviceID, u16 sNodeAddress);                    //分析433串口接收的数据

void Treaty_Data_Analysis(u8* pTreatyBuff, u16* pFlag, u8* pDeviceID, u16 NodeAddress);	
uint16_t char_to_int(char* pSetPara);               //处理短信下发的配置，主要将字符转化为整数
float    char_to_float(char* pSetPara);               //处理短信下发的配置，主要将字符转化为小数
             
void mput_mix_sx1278(char *str,int length);           //用于433模块发送数据，数据发送完成以后必须拉高SET引脚，否则模块经常接收不到数据
#endif

