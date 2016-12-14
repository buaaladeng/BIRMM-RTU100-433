
// File Name: gprs.c
#include "string.h"
#include "gprs.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "AiderProtocol.h"
#include "bsp_rtc.h"
#include "433_Wiminet.h"
#include "common.h"
#include "math.h"
#include "SPI_Flash.h"
#include "DS2780.h"
#include "API-Platform.h"
#include "test.h"
#include "SensorInit.h"


/***********************************变量声明与定义***************************************************/ 
struct    DataFrame ClientSetRequest;
char      Usart3_recev_buff[300]={'\0'};     //USART3接收缓存
uint16_t  Usart3_recev_count=0;              //USART3接收计数器

extern unsigned char Usart3_send_buff[300];
extern struct    SMS_Config_RegPara   ConfigData;     //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入FLASH

extern struct    liquid_set  DeviceConfig;   //液位计配置信息结构体
extern uint8_t   DataCollectCount;           //数据采集计数器
extern uint8_t   LiquidDataSend_Flag;
extern unsigned char   Usart2_send_buff[SENDBUFF_SIZE];    //433发送数据组
extern uint8_t   DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];
extern uint8_t   DMA_UART3_RECEV_FLAG ;      //USART3 DMA接收标志变量

extern vu8       Uart4_rev_comflag;          //RS485串口接收完成标志变量
extern  u8       Uart4_rev_buff[100];
extern  vu8      Uart4_rev_count;            //RS485串口接收计数器

/***********************************函数申明***************************************************/ 
extern unsigned char  DMA_UART3_RecevDetect(unsigned char RecevFlag,u8* pDeviceID, u16 sNodeAddress);     //USART3接收数据监测与数据解析

/*******************************************************************************
* Function Name  : USART_DataBlock_Send
* Description    : 批量向串口发送数据
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_DataBlock_Send(USART_TypeDef* USART_PORT,char* SendUartBuf,u16 SendLength)    //批量向串口发送数据
{
    u16 i;
        
    for(i=0;i<SendLength;i++)
    {
        USART_SendData(USART_PORT, *(SendUartBuf+i));
        while (USART_GetFlagStatus(USART_PORT, USART_FLAG_TC) == RESET);
    } 
}

void mput_mix(char *str,int length)
{
	int i;                                       //测试使用
	
	printf("length:%d\r\n",length);             //测试使用
	//printf("\r\n");
	for(i=0;i<length;i++)                                            //输出测试
	printf("--%x--",str[i]);

  GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433模块的SET引脚拉低，切换至发送模式
	Delay_ms(100);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433模块的EN引脚拉低，切换至发送模式
	Delay_ms(500);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	USART_DataBlock_Send(USART2,str,length);
//	USART_DataBlock_Send(USART2,"\r\n",2);
	//USART_DataBlock_Send(USART1,str,length);
	//USART_DataBlock_Send(USART1,"\r\n",2);
	Delay_ms(500);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);                    //433模块的EN引脚拉高， 切换至接收模式
	GPIO_SetBits(GPIOC,GPIO_Pin_5);                    //433模块的SET引脚拉高，切换至接收模式
	Delay_ms(100);
	//USART_ClearFlag(USART2,USART_FLAG_TC);
	//USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); 
}

void mput_mix_sx1278(char *str,int length)           //用于433模块发送数据，数据发送完成以后必须拉高SET引脚，否则模块经常接收不到数据
{
	//printf("length:%d\r\n",length);                    //测试使用
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433模块SET管脚拉低，切换到高速发送模式
	Delay_ms(200);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433模块EN管脚拉低，切换到高速发送模式
	Delay_ms(500);
	USART_DataBlock_Send(USART2,str,length);
//	USART_DataBlock_Send(USART2,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,length);
//	USART_DataBlock_Send(USART1,"\r\n",2);
	Delay_ms(800);                                     
	GPIO_SetBits(GPIOC,GPIO_Pin_5);                    //433模块SET管脚拉高，切换到接收模式
	Delay_ms(100);
//	USART_ClearFlag(USART2,USART_FLAG_TC);
}


void mput(char* str)
{
	printf("length:%d\r\n",strlen(str));     //测试使用
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433模块的SET引脚拉低，切换至发送模式
	Delay_ms(500);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433模块的EN引脚拉低，切换至发送模式
	Delay_ms(500);
//	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //向USART3发送数据前，先打开USART3接收空闲中断，便于监测数据接收完成
	USART_DataBlock_Send(USART2,str,strlen(str));
	USART_DataBlock_Send(USART2,"\r\n",2);
	USART_DataBlock_Send(USART1,str,strlen(str));
	USART_DataBlock_Send(USART1,"\r\n",2);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);                    //433模块的EN引脚拉高， 切换至接收模式
	GPIO_SetBits(GPIOC,GPIO_Pin_5);                    //433模块的SET引脚拉高，切换至接收模式
}

/*******************************************************************************
* Function Name  : char* Find_String(char* Source, char* Object)
* Description    : 在目标字符串中发现一个指定的字符串
* Input          : 
* Output         : 
* Return         : 如果找到，则返回目标字符串在源字符串中的首地址
*******************************************************************************/
char* Find_String(char* Source, char* Object)
{
	char*   Ptemp1 = Source;
	char*   Ptemp2 = Object;
	short   Length_Source =0;
	short   Length_Object =0;
	short   i=0,j=0;
	short   count=0;
	
	
	Length_Source = strlen(Source);
	Length_Object = strlen(Object);

	if(Length_Source < Length_Object)
	{
     return NULL;
  }
  
	else if(Length_Source == Length_Object)
	{
		 if((Length_Source==0)&&(Length_Object==0))
		 {
			  return NULL;
     }  
		 else  
		 { 
			  for(i=0;i<Length_Source;i++)
		    {
				   if(Ptemp1[i] != Ptemp2[i])
					 return NULL;
        }
				return Ptemp1;
     }	  
  }
	else 
	{
		 if(Length_Object == 0)
		 {
			  return NULL;
     }  
		 else  
		 {  count = Length_Source - Length_Object + 1;
			  for(i=0;i<count;i++)
		    {  for(j=0;j<Length_Object;j++)
					 {
               if(Ptemp1[i+j] != Ptemp2[j])
						   break;
           }
					 if(j==Length_Object)
					 return  &Ptemp1[i]; 
				 
        }
				return NULL;
     }	  
  }
}
/***********函数功能：在特定序列中发现一个指定的序列****************/
/***********如果找到，则返回目标序列在源序列中的首地址**************/
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object)  
{
	char*   Ptemp1 = Source;
	char*   Ptemp2 = Object;
	short   i=0,j=0;
	short   count=0;
	
	if((Length_Source < 0)||(Length_Object < 0))
	{
     return NULL;
  }
  if(Length_Source < Length_Object)
	{
     return NULL;
  }
  
	else if(Length_Source == Length_Object)
	{
		 if((Length_Source==0)&&(Length_Object==0))
		 {
			  return NULL;
     }  
		 else  
		 { 
			  for(i=0;i<Length_Source;i++)
		    {
				   if(Ptemp1[i] != Ptemp2[i])
					 return NULL;
        }
				return Ptemp1;
     }	  
  }
	else 
	{
		 if(Length_Object == 0)
		 {
			  return NULL;
     }  
		 else  
		 {  
			  count = Length_Source - Length_Object + 1;
			  for(i=0;i<count;i++)
		    {  for(j=0;j<Length_Object;j++)
					 {
               if(Ptemp1[i+j] != Ptemp2[j])
						   break;
           }
					 if(j==Length_Object)
					 {
							return  &Ptemp1[i]; 
					 }
				 
        }
				return NULL;
     }	  
  }
}


/**************************************字符转化为浮点数******************************************/
/**************************************用于计算报警阈值******************************************/
float  char_to_float(char* pSetPara)
{
	char CharTemp[10]={0x00};
	int i,j;
	int IntegerPart=0;
	int DecimalPart=0;
	int Counter=0;
	float number=0.0;
	for(j=0;j<strlen(pSetPara);j++)                       //调用函数已经做了防溢出处理,strlen(pSetPara)<=5
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[j] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>2)
			 {
          printf("\r\nInput ERROR!!\r\n");
          return 0;
       } 
			
			 for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
				 
         IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //计算报警阈值整数部分
       
				 
       }
			 
			 Counter =strlen(CharTemp)+1; //跳过小数点
			 memset(CharTemp,0x00,sizeof(CharTemp));
			 for(i=0,j=Counter;j<strlen(pSetPara);j++,i++)                     
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[i] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>2)
			 {
          printf("\r\nInput Alarm Threshold ERROR!!\r\n");
          return 0;
       }
       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
         if(strlen(CharTemp)==1)
				 {
           DecimalPart =(CharTemp[i-1]-'0')*10;
					 break;
         }
				 else
				 {
           DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));  //计算报警阈值小数部分
         }
       }
	
	     number=DecimalPart*0.01 + IntegerPart;
			 return(number);
}


/**************************************字符转化为整数******************************************/
uint16_t char_to_int(char* pSetPara)
{
	char CharTemp[10]={0x00};
	int i,j;
  int length;
	int IntegerPart=0;


	for(j=0;j<strlen(pSetPara);j++)                       //调用函数已经做了防溢出处理,strlen(pSetPara)<=5
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[j] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 
			 if(strlen(CharTemp)>5)
			 {
          printf("\r\nInput  ERROR!!\r\n");
          return 0;
       } 
			 
			 
			 length=strlen(CharTemp);
	  
			 for(i=0;i<length;i++)
			 printf("----the data is %c -----",CharTemp[i]);
			 
       
			 
			 IntegerPart = IntegerPart + (CharTemp[length-1]-'0');
				
			 if(length>1)			 
			 for(i=1; i<strlen(CharTemp); i++)
			 {
				
				IntegerPart =IntegerPart + (CharTemp[length-1-i]-'0')*(int)(pow(10,i));  //
					
       }

	return (IntegerPart);
}


/*******************************************************************************
* Function Name  : Receive_DataAnalysis
* Description    : 进行数据的第一层解析
* Input          : None
* Output         : None      
* Return         : None
*******************************************************************************/
u16 Receive_Data_Analysis(u8* pDeviceID, u16 sNodeAddress)	
{
	 u16               Recev_Flag2 =0;                   //接收数据正确性标志变量
	 char*             pRecevBuff =NULL; 

   char   RecevFromCollector[2]={0xA3,SOFTVERSION};       //接收服务器发送的数据标志序列
   char   RecevFromConfig   [4]={0xA3,0xA3,0xA3,0xA3};       //接收参数配置器数据标志序列

   u8     PayloadLen =0;
   u16    CrcVerify =0x0000;
   u16    CrcRecev  =0x0000;

   //RecevFromCollector[2] = sNodeAddress>>8;             //节点地址高字节
   //RecevFromCollector[3]  = sNodeAddress&0xff;           //节点地址低字节
#if DEBUG_TEST
   printf("\r\n接收数据解析!!\r\n");          //测试使用
#endif

 ////////////////////////////////////检测是否为自身配置433模块命令//////////////////////////////////////////////////////
   pRecevBuff = Find_SpecialString(Usart3_recev_buff, RecevFromConfig, sizeof(Usart3_recev_buff), sizeof(RecevFromConfig));  //检查是否收到参数配置器应答
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-1-9)))  //防止指针越界          
   {
		  printf("\r\nReceive 433 Config Command from Srvies!\r\n"); //测试使用
	 	  if(pRecevBuff[4] ==0x00)
			{
         ParameterConfig_SX1278(pRecevBuff+5);
      }
			else if(pRecevBuff[4] ==0x01)
			{
         ParameterInquire_SX1278(pRecevBuff[5]);
      }
			else
			{
					#if DEBUG_TEST	 
					printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
					#endif
      }
			pRecevBuff =NULL;
			return 0;
	 }
/////////////////////////////////////检测是否为上位机下发采集参数/////////////////////////////////////////////////
   pRecevBuff = Find_SpecialString(Usart3_recev_buff,RecevFromCollector,sizeof(Usart3_recev_buff),sizeof(RecevFromCollector));  //检查有无收到主站回复
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-1-9)))  //防止指针越界          
	 {	
#if DEBUG_TEST	
      printf("\r\n接收上位机数据解析!!\r\n");          //测试使用 		
#endif		 
			
			Treaty_Data_Analysis(((u8*)pRecevBuff), &Recev_Flag2, pDeviceID, sNodeAddress);  //解析接收数据
			return (Recev_Flag2);
	 
 }
#if DEBUG_TEST	
	 printf("\r\n接收数据解析完成!!\r\n");                 //调试使用
#endif
	 return 0;
}

/*******************************************************************************
* Function Name  : XX
* Description    : 433远程配置参数命令解析
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  ParameterConfig_SX1278(char* pRecevCommand)
{
   u8   RecevCommandArry[5]={0x00};
	
	 u8   UseFlag   =0;    //配置参数用途标志变量，也用于指示配置参数的时效性，参数为0时立即生效，否则作为备份参数
	 u8   Opcode    =0;    //命令码字段，用于指示命令的操作对象
	 u8   DataLenth =0;    //参数长度
	 u16  TempLong  =0;
	 u8   TempShort =0;
	 u8   ConfigReply[12] ={0xAA,0xAA,0xAF,0xFF,0xAF,0xEE,0x00,0xAA,0xAA,0xAA,0xAA,0xAA}; //数组中0xAA字段为待定字段，含义分别是（从左向右）：
     
                       //节点地址高字节，节点地址低字节，参数用途，命令码，配置参数长度，配置参数第1字节，配置参数第2字节（可能不存在）
   memcpy(RecevCommandArry, pRecevCommand, sizeof(RecevCommandArry));
   UseFlag   =RecevCommandArry[0];
	 Opcode    =RecevCommandArry[1];
	 DataLenth =RecevCommandArry[2];
   TempLong  =GetNodeID (); //获取节点ID
   ConfigReply[0] =TempLong>>8;
   ConfigReply[1] =TempLong &0xFF;
	 ConfigReply[7] =UseFlag ;
   ConfigReply[8] =Opcode;
   Delay_ms(3000);
	 if(UseFlag ==0)
	 {
     switch(Opcode )
		 {
        case 0x09:                    //配置433模块串口波特率参数，谨慎修改!!!
				{
           if(DataLenth ==2)
					 {
//						 SetNodeSerialPort(RecevCommandArry[3], RecevCommandArry[4]);     //设置433模块串口波特率和校验类型
//						 PowerOFF_GPRS();                                           //重启433模块，使配置生效        
//						 Delay_ms(5000);
//						 PowerON_GPRS();
//						 TempLong =GetNodeSerialPortConfig();                      //读取433模块串口配置参数
//						 ConfigReply[9] =2;
//						 ConfigReply[10] =TempLong >>8;
//					   ConfigReply[11] =TempLong &0xFF;
//						 mput_mix_sx1278((char*)ConfigReply, 12);                  //参数配置应答帧
//						 //参数存入Flash，有待完善
						 #if DEBUG_TEST	 
						 printf("\r\nSerial Port Parameters Config not allowable !!\r\n"); //测试使用
						 #endif
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x0B:                                                    //配置433模块载波频率
				{
           if(DataLenth ==2)
					 {
						 TempLong =(RecevCommandArry[3]<<8) +RecevCommandArry[4];      //获得配置频率参数
					
						 ConfigReply[9] =2;
						 ConfigReply[10] =TempLong>>8;
					   ConfigReply[11] =TempLong &0xFF;
						 mput_mix((char*)ConfigReply, 12);                //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);
						 mput_mix((char*)ConfigReply, 12);                //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功,多发一次为了解决首次发送数据丢失BUG
						 Delay_ms(1000);
						 SetNodeCentralFrequency(TempLong);                      //设置433模块载波中心频率
						 TempLong =GetNodeCentralFrequency();                    //读取433模块载波频率参数
	           #if DEBUG_TEST	 
					   printf("\r\nCentral Frequency :%d MHz\r\n",(TempLong+1));   //测试使用
						 #endif
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x0D:                                                    //配置433模块扩频因子参数
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //获得配置扩频因子参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                  //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);	
						 mput_mix((char*)ConfigReply, 11);                  //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);							 
						 SetNodeFrequencyExpandFactor(TempShort);                  //设置433模块扩频因子
						 TempShort  =GetNodeFrequencyExpandFactor();               //获取433模块扩频因子参数
			
						 #if DEBUG_TEST	 
					   printf("\r\nNode Frequency Expand Factor(Index Code):%d\r\n",TempShort); //测试使用
						 #endif
						//参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x0F:                                                        //配置433模块扩频带宽参数
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                              //获得配置扩频带宽参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                     //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);			
						 mput_mix((char*)ConfigReply, 11);                     //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);
						 SetNodeFrequencyExpandBandwidth(TempShort);                  //设置433模块扩频带宽
						 TempShort  =GetNodeFrequencyExpandBandwidth();               //获取433模块扩频带宽参数
	           #if DEBUG_TEST	 
					   printf("\r\nNode Frequency Expand Bandwidth(Index Code):%d\r\n",TempShort); //测试使用
						 #endif
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x11:                                                     //配置433模块工作模式参数，谨慎修改!!!
				{
           if(DataLenth ==1)
					 {
//						 TempShort =RecevCommandArry[3];                           //获得配置工作模式参数
//						 SetNodeWorkMode(TempShort);                               //设置433模块工作模式
//						 TempShort  =GetNodeWorkMode();                            //获取433模块工作模式参数
//						 ConfigReply[9] =1;
//						 ConfigReply[10] =TempShort ;   
//						 mput_mix_sx1278((char*)ConfigReply, 11);                  //参数配置应答帧
//					 //参数存入Flash，有待完善
						 #if DEBUG_TEST	 
						 printf("\r\nNode Work Mode Config not allowable !!\r\n"); //测试使用
						 #endif
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x13:                                                     //配置433模块客户ID
				{
					 if(DataLenth ==2)
					 {
						 TempLong =(RecevCommandArry[3]<<8) +RecevCommandArry[4];  //获得配置客户ID参数
						 ConfigReply[0] =TempLong>>8;                              //重新更新节点ID字段
             ConfigReply[1] =TempLong &0xFF;                           //重新更新节点ID字段
						 ConfigReply[9] =2;
						 ConfigReply[10] =TempLong>>8;
					   ConfigReply[11] =TempLong &0xFF;
						 mput_mix((char*)ConfigReply, 12);                  //参数配置应答帧
						 Delay_ms(2000);
						 mput_mix((char*)ConfigReply, 12);                  //参数配置应答帧
						 Delay_ms(1000);
	
						 SetNodeID(TempLong);                                      //设置433模块客户ID
						 TempLong =GetNodeID();                                    //读取433模块客户ID参数				 
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Node ID is:%x\r\n",TempLong); //测试使用
						 #endif
						 	//参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x15:                                                        //配置433模块网络ID
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                              //获得配置网络ID参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                     //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);	
						 mput_mix((char*)ConfigReply, 11);                     //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);	
						 
						 SetNodeNetworkID(TempShort);                              //设置433模块网络ID
						 TempShort  = GetNetworkID();                              //获取433模块网络ID参数
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Network ID is:%x\r\n",TempShort);  //测试使用
						 #endif
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }	
				case 0x17:                                                     //配置433模块发射功率等级
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //获得配置发射功率等级参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);
						 mput_mix((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);							 
						 
						 SetNodeSendPowerGrade(TempShort);                         //设置433模块发射功率等级
						 TempShort =GetNodeSendPowerGrade();                       //获取433模块发射功率等级参数
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Send Power Grade is:%x\r\n",TempShort);  //测试使用
						 #endif				 
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }	
				case 0x19:                                                     //配置433模块呼吸周期
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //获得配置呼吸周期参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);	
						 mput_mix((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);	
						 
						 SetNodeBreathPeriod(TempShort);                           //设置433模块呼吸周期
						 TempShort =GetNodeBreathPeriod();                         //获取433模块呼吸周期参数
             #if DEBUG_TEST	 
					   printf("\r\nNode Breath Period is:%x\r\n",TempShort);  //测试使用
						 #endif	
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }	
				case 0x1B:                                                     //配置433模块呼吸时间
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //获得配置呼吸时间参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);	
						 mput_mix((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);	
						 
						 SetNodeBreathTime(TempShort);                             //设置433模块呼吸时间
						 TempShort =GetNodeBreathTime();                           //获取433模块呼吸时间参数
						 #if DEBUG_TEST	 
					   printf("\r\nNode Wake Time is:%x\r\n",TempShort);         //测试使用
						 #endif
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }	
				default:
				{
					 #if DEBUG_TEST	
           printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
					 #endif
					 break;
        }
     }
   }
	 else if(UseFlag ==1)
	 {
		 //参数存入Flash，不立即更新模块参数,待完善
	 }
	 else
	 {
     //不做处理 ,待完善
   }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 433远程查询参数命令解析
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  ParameterInquire_SX1278(u8 InquireCommand)
{

   ;//待完善

}

/*******************************************************************************
* Function Name  : ConfigData_Init
* Description    : 初始化一些配置参数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ConfigData_Init(struct liquid_set* Para)
{
  char     CharArry[16] ={0x00};
  uint8_t  length =0;
  uint16_t Temp=0;
  uint32_t LTemp=0;
	float    fTemp=0;
	int i;
#ifdef TEST_DEBUG
  printf("\r\n参数配置信息：\r\n");  //测试使用
#endif
  BKP_TamperPinCmd(DISABLE);                   //




/******************************************************************************************/
//(2)采集周期
	DataRead_From_Flash(0,2,0, ConfigData.CollectPeriod_Byte,2); //从Flash中读取液位计采集间隔
  Temp =ConfigData.CollectPeriod_Byte[0]*256 + ConfigData.CollectPeriod_Byte[1];
  //printf("------------%d------------",Temp);
  if((Temp>0)&&(Temp<=60))                    //可配置1~60分钟进行数据采集间隔
	{
		Para->CollectPeriod =  Temp;
	}
	else
	{
    Para->CollectPeriod =60;           //每隔15分钟采集一次数据。
  }
		printf("\r\n**************采集周期-->> %d ***********\r\n",Para->CollectPeriod);       //输出测试
	
//(3)上报周期	
  DataRead_From_Flash(0,3,0, ConfigData.SendCount_Byte ,2);             //从Flash中读取上传周期
  Temp =ConfigData.SendCount_Byte[0]*256 + ConfigData.SendCount_Byte[1];
	
	//printf("------------%d------------",Temp);
  if((Temp>0)&&(Temp<=1440))  
	{
		Para->SendCount =  Temp;
	}
	else
	{    
    Para->SendCount =60;              //1小时发送一次数据
  }	
	
  printf("\r\n**************上报周期-->> %d *********** \r\n",Para->SendCount );   //输出测试


//(7)低浓度报警阈值
	DataRead_From_Flash(0,7,0, ConfigData.LowAlarmLevel .Data_Hex ,4);   //从Flash中读取预设报警阈值

	fTemp = ConfigData.LowAlarmLevel .Data_Float ;
	
  //printf("\r\n---read low level ---%f----\r\n",fTemp );                       //打印FLASH读取的原始数据
   	
                               

		 if( (0.0<fTemp ) && ( fTemp <25.0) )
     Para->LowAlarmLevel  .Data_Float  =ConfigData.LowAlarmLevel .Data_Float  ;      

	else                                                     
	{ 
		 Para->LowAlarmLevel .Data_Float  =25.0;                        //读取数据无效时，将报警阈值设为25.0，当报警阈值为0时，不会触发报警事件
  }
	
	printf("\r\n**************低报警浓度-->> %0.2f ************\r\n", Para->LowAlarmLevel  .Data_Float );   //输出测试
	
// （8）开始采集时间
	DataRead_From_Flash(0,8,0,ConfigData.CollectStartTime_Byte ,2); //从Flash中读取第一次采集时间

	//Temp =  char_to_int(CharArry);
	
  Temp =ConfigData.CollectStartTime_Byte [0]*256 + ConfigData.CollectStartTime_Byte [1];
	
	//printf("\r\n---read CollectStartTime ---%d----\r\n",Temp );                       //打印FLASH读取的原始数据
  if((Temp>=0)&&(Temp<=1440))  
	{
		Para->CollectStartTime  =  Temp;
	}
	else
	{
    Para->CollectStartTime  =0;           //第一次采集时间为0点钟
  }

	printf("\r\n**************开始采集时间-->> %d ***********\r\n", Para->CollectStartTime );   //输出测试
	
	
//(9)已采集数量
	DataRead_From_Flash(0,9,0,    &(ConfigData.CollectNum)   ,1); //从Flash中读取当前采集数量
	
//	printf("\r\n---read retry number ---%s----\r\n",(u8*)ConfigData.CollectNum_Byte );                       //打印FLASH读取的原始数据
  //Temp =ConfigData.CollectNum_Byte [0]*256 + ConfigData.CollectNum_Byte [1];
	Temp = ConfigData.CollectNum;
	//printf("\r\n---read CollectNum ---%d----\r\n",Temp );                       //打印FLASH读取的原始数据
//  if((0 <=ConfigData.CollectNum )&&(ConfigData.CollectNum <= MAX_COLLECTNUM))  
//	{
//		if(ConfigData.CollectNum <=( Para->SendCount /Para->CollectPeriod ))
//		{ Para->CollectNum  =  ConfigData.CollectNum; }
//		else
//		{ Para->CollectNum =( Para->SendCount /Para->CollectPeriod );}
//	}
//	else
//	{
//    Para->CollectNum  = ( Para->SendCount /Para->CollectPeriod ) ;           //当前采集的传感器数据量 
//  }
	
		printf("\r\n**************已采集数量-->> %d ***********\r\n", Temp );   //输出测试
//(10)重传次数
	DataRead_From_Flash(0,10,0, &(ConfigData.RetryNum ) ,1); //从Flash中读取重传次数
  
	//printf("\r\n---read retry number ---%d----\r\n",ConfigData.RetryNum );                       //打印FLASH读取的原始数据
	
	Temp =ConfigData.RetryNum ;
  if((Temp>0)&&(Temp<=10))  
	{
		Para->RetryNum  =  Temp;
	}
	else
	{
    Para->RetryNum  =3;           //默认为3
  }
	printf("\r\n**************重传次数-->> %d ***********\r\n", Para->RetryNum );   //输出测试
	

//(11)高浓度报警阈值
	DataRead_From_Flash(0,11,0, ConfigData.HighAlarmLevel .Data_Hex  ,4);    //从Flash中读取高浓度报警
  
	//printf("\r\n---read high alarm level ---%f----\r\n",ConfigData.HighAlarmLevel .Data_Float );                       //打印FLASH读取的原始数据
	
	fTemp =ConfigData.HighAlarmLevel .Data_Float  ;
  if((fTemp>0.0) && (fTemp < 50.0) && (fTemp > ConfigData.LowAlarmLevel .Data_Float ))  
	{
		Para->HighAlarmLevel .Data_Float   =  fTemp;
	}
	else
	{
    	Para->HighAlarmLevel .Data_Float  =  50.0;           //报警高浓度阈值默认为50%
  }
	printf("\r\n**************高报警浓度-->> %0.2f ***********\r\n", 	Para->HighAlarmLevel .Data_Float );   //输出测试
	
//(12)设备已工作次数
	DataRead_From_Flash(0,12,0, (ConfigData.WorkNum_Byte),4);    //从Flash中读取已工作次数

	Temp=ConfigData.WorkNum_Byte [0]*256+ConfigData.WorkNum_Byte [1];
  Para->WorkNum  = Temp; 
	printf("\r\n**************工作次数-->> %d ***********\r\n", 	Para->WorkNum);   //输出测试
//(13)电池电量
	printf("\r\n**************电池电量-->> %d%% ***********\r\n", 	Para->BatteryCapacity=DS2780_Test());   //输出测试
//	Para->Time_Sec  =0x00;
//	Para->Time_Min  =0x00;
//	Para->Time_Hour =0x00;
//	Para->Time_Mday =0x00;
//	Para->Time_Mon  =0x00;
//	Para->Time_Year =0x00;
//  Para->BatteryCapacity =0x64;    //电池电量，暂定为100%
	Para->MessageSetFlag  =0;       
}



/**********************************************END******************************************/










