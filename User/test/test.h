#include "stm32f10x.h"
#include "SPI_Flash.h"
#include "AiderProtocol.h" 
#include "common.h"
#include "bsp_rtc.h"

#include "gprs.h"
#include "bsp_SysTick.h"
#include "modbus.h"
#include "string.h"
#include "433_Wiminet.h"
#include "API-Platform.h"

void write_collect_parameter_to_flash(void);      //进行采集信息的配置
void write_3G_parameter_to_flash(void);          //进行3G模块的配置
void write_433_parameter_to_flash(void);         //进行433模块进行配置
void Set_Time(void);                             //进行设备时间的初始化
void Set_DS2780(void);
void Test_SendToServ(u8* pDeviceID);             //测试发送函数

