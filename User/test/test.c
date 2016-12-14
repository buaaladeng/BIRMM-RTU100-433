

/***************************������Ϊ��ʼ���豸********************************/
#include "test.h"
#include "time.h"
#include "SensorInit.h"
#include  "DS2780.h"
#include  "AiderProtocol.h"
extern struct liquid_set DeviceConfig;

extern unsigned char   Usart3_send_buff[300];    //3Gģ�鷢������
uint8_t SetTime[6]=SETTIME;
//(1)���òɼ������Ϣ���ֱ�Ϊ�ɼ����ڡ��ϴ����ڡ��ش���������Ũ����ֵ����Ũ����ֵ
void write_collect_parameter_to_flash(void)      //���øú����ɽ���ȼ�������ն˲ɼ�����   ����3G�汾
{  
	uint8_t collect_period[2]={00,1};     //�ɼ�����
  uint8_t send_out[2]={00,1};           //�ϴ�����
  uint8_t retry_num = 3;                //�ش�����
	uint8_t first_collect[2]={00,00};     //��һ�βɼ�ʱ��
	uint8_t collect_num = 0;              //�Ѳɼ�����
  uint8_t WorkNum[2]={00,00};         //�ѹ���ʱ��
 
	
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

//(2)����3Gģ����Ϣ, ������·���롢IP�˿ںš�PORT�˿ں�
void write_3G_parameter_to_flash(void) 
{
	char    phone_num[16]={"861064617178004"};             //�����绰����
	char    ip[16]={"119.254.103.80"};                     //�������IP��ַ
	char    port[6]={"2017"};                              //�������˿ں�
 
////////////////////������д��FLASH��////////////////////////////
  DataWrite_To_Flash(0,4,0,phone_num,strlen(phone_num));
	DataWrite_To_Flash(0,5,0,ip,strlen(ip));
	DataWrite_To_Flash(0,6,0,port,strlen(port));
}

//(3)����433ģ����Ϣ
void write_433_parameter_to_flash(void) 
{
	
	
}
//(4)����ʱ����Ϣ
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
	Set_register_ds2780();    //�����Կ��ؼ����³�ʼ��
	set_ACR(1000);           //�����Կ��ؼ����³�ʼ��
	DS2780_CapacityInit();    //���������д�������
}


void Test_SendToServ(u8* pDeviceID)
{
//ģ����б����ϴ�
	struct TagStruct AlarmTAG,OIDTAG[3],ResponsTAG[3];
	int i;
	struct SenserData pGetData;
	pGetData.Ch4Data.Data_Float=30.0;
	pGetData.CollectTime = 120;
	
	AlarmTAG.OID_Command=(DeviceConfig.CollectPeriod<<11)+pGetData.CollectTime  +((0xC0 + REPORTDATA )<<24);
	AlarmTAG.OID_Command=ntohl(AlarmTAG.OID_Command);
	AlarmTAG.Width=4;
	for(i=0;i<AlarmTAG.Width;i++)
	AlarmTAG.Value[i]=pGetData.Ch4Data.Data_Hex[i];                                 //��ɱ�����TAG
	SendDataToServ(TRAPREQUEST,&AlarmTAG,1,Usart3_send_buff, pDeviceID);                //433ģ�������ϴ�����
  Delay_ms(2000);
	
//ģ��ֻ�ϴ�OID
	ResponsTAG[0].OID_Command = SYSTERM_DATA;
	ResponsTAG[0].Width=0;
	ResponsTAG[1].OID_Command = CLT1_ITRL1;
	ResponsTAG[1].Width=0;
	ResponsTAG[2].OID_Command = UPLOAD_CYCLE;
	ResponsTAG[2].Width=0;
	SendDataToServ(GETRESPONSE,ResponsTAG,3,Usart3_send_buff, pDeviceID); 
	 Delay_ms(2000);
	 
//ģ���ϴ������·�����TAG
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



