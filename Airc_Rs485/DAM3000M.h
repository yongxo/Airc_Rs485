#ifndef  _ART_DAM3000M_SERIAL_
#define _ART_DAM3000M_SERIAL_

#include <windows.h>

// ********************* 开关量输出的参数结构 ****************************
typedef struct _DAM3000M_PARA_DO			// 数字量输出参数
{
	BYTE DO0;       	// 0通道
	BYTE DO1;       	// 1通道
	BYTE DO2;       	// 2通道
	BYTE DO3;       	// 3通道
	BYTE DO4;       	// 4通道
	BYTE DO5;       	// 5通道
	BYTE DO6;       	// 6通道
	BYTE DO7;       	// 7通道
	BYTE DO8;			// 8通道
	BYTE DO9;			// 9通道
	BYTE DO10;      	// 10通道
	BYTE DO11;      	// 11通道
	BYTE DO12;      	// 12通道
	BYTE DO13;			// 13通道
	BYTE DO14;			// 14通道
	BYTE DO15;			// 15通道
} DAM3000M_PARA_DO, *PDAM3000M_PARA_DO;


// ********************* 开关量输入的参数结构 *******************************
typedef struct  _DAM3000M_PARA_DI		// 数字量输入参数(1为高电平)
{
	BYTE DI0;			// 0通道
	BYTE DI1;			// 1通道
	BYTE DI2;			// 2通道
	BYTE DI3;			// 3通道
	BYTE DI4;			// 4通道
	BYTE DI5;			// 5通道
	BYTE DI6;			// 6通道
	BYTE DI7;			// 7通道
	BYTE DI8;			// 8通道
	BYTE DI9;			// 9通道
	BYTE DI10;			// 10通道
	BYTE DI11;			// 11通道
	BYTE DI12;			// 12通道
	BYTE DI13;			// 13通道
	BYTE DI14;			// 14通道
	BYTE DI15;			// 15通道
} DAM3000M_PARA_DI, *PDAM3000M_PARA_DI;

typedef struct  _DAM3000M_PARA_LATCH      // 数字量锁存参数(1为锁存)
{
	BYTE Latch0;		// 0通道
	BYTE Latch1;        // 1通道
	BYTE Latch2;        // 2通道
	BYTE Latch3;        // 3通道
	BYTE Latch4;        // 4通道
	BYTE Latch5;        // 5通道
	BYTE Latch6;        // 6通道
	BYTE Latch7;        // 7通道
	BYTE Latch8;        // 8通道
	BYTE Latch9;        // 9通道
	BYTE Latch10;       // 10通道
	BYTE Latch11;       // 11通道
	BYTE Latch12;       // 12通道
	BYTE Latch13;       // 13通道
	BYTE Latch14;       // 14通道
	BYTE Latch15;       // 15通道
} DAM3000M_PARA_LATCH, *PDAM3000M_PARA_LATCH;


// ****************** 模拟量输入通道配置结构体 *************************
typedef struct _DAM3000M_ADCHANNEL_ARRAY
{
	BYTE bChannel0;		// 1，有效；0，无效
	BYTE bChannel1;
	BYTE bChannel2;
	BYTE bChannel3;
	BYTE bChannel4;
	BYTE bChannel5;
	BYTE bChannel6;
	BYTE bChannel7;
}DAM3000M_ADCHANNEL_ARRAY, *PDAM3000M_ADCHANNEL_ARRAY;


// ****************** 计数器参数配置结构体 ******************************
typedef struct _DAM3000M_PARA_CNT			// 基于各通道的计数器参数结构体
{
	LONG WorkMode;			// 计数器/频率工作模式
	LONG FreqBuildTime;		// 测频器建立时间, 单位: s
	LONG InputMode;			// 计数器/频率输入方式	0: 非隔离	1: 隔离
	ULONG InitVal;			// 计数器初始值
	ULONG MaxVal;			// 计数器最大值
	LONG GateSts;			// 门槛值状态(计数模式)
} DAM3000M_PARA_CNT, *PDAM3000M_PARA_CNT;

typedef struct _DAM3000M_CNT_ALARM
{
	LONG AlarmMode;			// 报警方式	
	LONG EnableAlarm0;		// 0通道报警使能
	LONG EnableAlarm1;		// 1通道报警使能
	ULONG Alarm0Val;		// 0通道报警值
	ULONG Alarm1Val;		// 1通道报警值
	ULONG Alarm0HiHiVal;	// 0通道上上限(Hi-Hi)报警值, 报警方式1有效
} DAM3000M_CNT_ALARM, *PDAM3000M_CNT_ALARM;

typedef struct _DAM3000M_PARA_FILTER		// 用于计数器滤波的参数结构体
{
	LONG TrigLevelHigh;		// 触发高电平(非隔离输入)
	LONG TrigLevelLow;		// 触发低电平(非隔离输入)
	LONG MinWidthHigh;		// 高电平最小输入信号宽度
	LONG MinWidthLow;		// 低电平最小输入信号宽度
	LONG bEnableFilter;		// 使能滤波
} DAM3000M_PARA_FILTER, *PDAM3000M_PARA_FILTER;
//	LONG DisplayChannel;	// 设置显示通道		0：0通道计数/频率，1：1通道计数/频率


typedef struct _DAM3000M_CNT_STATUS			// 计数器硬件参数状态结构体
{
	LONG WorkMode;			// 计数器/频率工作模式*
	LONG FreqBuildTime;		// 测频器建立时间, 单位: s*
	LONG InputMode;			// 计数器/频率输入方式	0: 非隔离	1: 隔离*
	LONG bCNTSts;			// 计数/频率器的状态(起停状态)*
	LONG FilterSts;			// 计数器的滤波状态*
	LONG MinWidthHigh;		// 高电平最小输入信号宽度*
	LONG MinWidthLow;		// 低电平最小输入信号宽度*
	LONG TrigLevelHigh;		// 触发高电平(非隔离输入)*
	LONG TrigLevelLow;		// 触发低电平(非隔离输入)*
	LONG GateSts;			// 门槛值设置状态(计数模式)*
	ULONG MaxVal;			// 计数器最大值*
	ULONG InitVal;			// 计数器初始值*
	LONG bOverflowSts;		// 计数器溢出状态*
	LONG AlarmMode;			// 计数器报警方式*
	LONG EnableAlarm0;		// 计数器0报警使能状态*
	LONG EnableAlarm1;		// 计数器1报警使能状态*
	ULONG Alarm0Val;		// 0通道报警值*
	ULONG Alarm1Val;		// 1通道报警值*
	ULONG Alarm1HiHiVal;	// 报警方式1上上限(Hi-Hi)报警值*
	LONG bDO0;				// DO0*
	LONG bDO1;				// DO1*
} DAM3000M_CNT_STATUS, *PDAM3000M_CNT_STATUS;


// ****************** 设备基本信息的结构体 ******************************
typedef struct _DAM3000M_DEVICE_INFO
{
	LONG    DeviceType;		// 模块类型 
	LONG    TypeSuffix;		// 类型后缀
	LONG	ModusType;		// M
	LONG	VesionID;		// 版本号(2字节)
	LONG	DeviceID;		// 模块ID号(SetDeviceInfo时，为设备的新ID)
	LONG	BaudRate;		// 波特率
	LONG	bParity;		// 有无校验
} DAM3000M_DEVICE_INFO, *PDAM3000M_DEVICE_INFO;

// 模拟量输入类型(电压类型) 供DAM3000M_SetModeAD函数中的lMode参数使用
#define DAM3000M_VOLT_N15_P15					0x01 //  -15～+15mV
#define DAM3000M_VOLT_N50_P50					0x02 //  -50～+50mV
#define DAM3000M_VOLT_N100_P100					0x03 // -100～+100mV
#define DAM3000M_VOLT_N150_P150					0x04 // -150～+150mV
#define DAM3000M_VOLT_N500_P500					0x05 // -500～+500mV
#define DAM3000M_VOLT_N1_P1						0x06 //   -1～+1V
#define DAM3000M_VOLT_N25_P25					0x07 // -2.5～+2.5V
#define DAM3000M_VOLT_N5_P5						0x08 //   -5～+5V
#define DAM3000M_VOLT_N10_P10					0x09 //  -10～+10V
#define DAM3000M_VOLT_N0_P5						0x0D //    0～+5V
#define DAM3000M_VOLT_N0_P10					0x0E //    0～+10V
#define DAM3000M_VOLT_N0_P25					0x0F //    0～+2.5V

// 模拟量输入类型(电流类型) 供DAM3000M_SetModeAD函数中的lMode参数使用
#define DAM3000M_CUR_N0_P10						0x00 //   0～10mA
#define DAM3000M_CUR_N20_P20					0x0A // -20～+20mA
#define DAM3000M_CUR_N0_P20						0x0B //   0～20mA
#define DAM3000M_CUR_N4_P20						0x0C //   4～20mA

// 模拟量输入类型(热电偶类型) 供DAM3000M_SetModeAD函数中的lMode参数使用
#define DAM3000M_TMC_J							0x10 // J型热电偶   0～1200℃
#define DAM3000M_TMC_K							0x11 // K型热电偶   0～1300℃
#define DAM3000M_TMC_T							0x12 // T型热电偶 -200～400℃
#define DAM3000M_TMC_E							0x13 // E型热电偶   0～1000℃
#define DAM3000M_TMC_R							0x14 // R型热电偶 500～1700℃
#define DAM3000M_TMC_S							0x15 // S型热电偶 500～1768℃
#define DAM3000M_TMC_B							0x16 // B型热电偶 500～1800℃
#define DAM3000M_TMC_N							0x17 // N型热电偶   0～1300℃
#define DAM3000M_TMC_C							0x18 // C型热电偶   0～2090℃
#define DAM3000M_TMC_WRE						0x19 // 钨铼5-钨铼26 0～2310℃

// 模拟量输入类型(热电阻类型) 供DAM3000M_SetModeAD函数中的lMode参数使用
#define DAM3000M_RTD_PT100_385_N200_P850		0x20 // Pt100(385)热电阻 -200℃～850℃
#define DAM3000M_RTD_PT100_385_N100_P100		0x21 // Pt100(385)热电阻 -100℃～100℃
#define DAM3000M_RTD_PT100_385_N0_P100			0x22 // Pt100(385)热电阻    0℃～100℃
#define DAM3000M_RTD_PT100_385_N0_P200			0x23 // Pt100(385)热电阻    0℃～200℃
#define DAM3000M_RTD_PT100_385_N0_P600			0x24 // Pt100(385)热电阻    0℃～600℃
#define DAM3000M_RTD_PT100_3916_N200_P850		0x25 // Pt100(3916)热电阻-200℃～850℃
#define DAM3000M_RTD_PT100_3916_N100_P100		0x26 // Pt100(3916)热电阻-100℃～100℃
#define DAM3000M_RTD_PT100_3916_N0_P100			0x27 // Pt100(3916)热电阻   0℃～100℃
#define DAM3000M_RTD_PT100_3916_N0_P200			0x28 // Pt100(3916)热电阻   0℃～200℃
#define DAM3000M_RTD_PT100_3916_N0_P600			0x29 // Pt100(3916)热电阻   0℃～600℃
#define DAM3000M_RTD_PT1000						0x30 // Pt1000热电阻     -200℃～850℃
#define DAM3000M_RTD_CU50						0x40 // Cu50热电阻        -50℃～150℃
#define DAM3000M_RTD_CU100						0x41 // Cu100热电阻       -50℃～150℃
#define DAM3000M_RTD_BA1						0x42 // BA1热电阻        -200℃～650℃
#define DAM3000M_RTD_BA2						0x43 // BA2热电阻        -200℃～650℃
#define DAM3000M_RTD_G53						0x44 // G53热电阻         -50℃～150℃
#define DAM3000M_RTD_Ni50						0x45 // Ni50热电阻        100℃
#define DAM3000M_RTD_Ni508						0x46 // Ni508热电阻         0℃～100℃
#define DAM3000M_RTD_Ni1000						0x47 // Ni1000热电阻      -60℃～160℃

// 模块量输出斜率类型	供DAM3000M_SetModeDA函数中的参数 lType 使用
#define DAM3000M_SLOPE_IMMEDIATE				0x00 // Immediate
#define DAM3000M_SLOPE_POINT125					0x01 // 0.125 mA/S
#define DAM3000M_SLOPE_POINT25					0x02 // 0.25  mA/S
#define DAM3000M_SLOPE_POINT5					0x03 // 0.5  mA/S
#define DAM3000M_SLOPE_1						0x04 // 1.0  mA/S
#define DAM3000M_SLOPE_2						0x05 // 2.0  mA/S
#define DAM3000M_SLOPE_4						0x06 // 4.0  mA/S
#define DAM3000M_SLOPE_8						0x07 // 8.0  mA/S
#define DAM3000M_SLOPE_16						0x08 // 16.0  mA/S
#define DAM3000M_SLOPE_32						0x09 // 32.0  mA/S
#define DAM3000M_SLOPE_64						0x0A // 64.0  mA/S
#define DAM3000M_SLOPE_128						0x0B // 128.0  mA/S
#define DAM3000M_SLOPE_256						0x0C // 256.0  mA/S
#define DAM3000M_SLOPE_512						0x0D // 512.0  mA/S
#define DAM3000M_SLOPE_1024						0x0E // 1024.0  mA/S
#define DAM3000M_SLOPE_2048						0x0F // 2048.0  mA/S

// DI计数方式 供DAM3000M_SetDeviceMode函数中的lMode参数使用
#define DAM3000M_DI_MODE_DI						0x00 // DI方式
#define DAM3000M_DI_MODE_COUNT					0x01 // 计数方式
#define DAM3000M_DI_MODE_LATCH					0x02 // 锁存方式

// DI计数方式 供DAM3000M_SetDeviceMode函数中的lEdgeMode参数使用
#define DAM3000M_DIR_FALLING					0x00 // 下降沿
#define DAM3000M_DIR_RISING						0x01 // 上升沿

//########################## 计数器 ###################################
// 模块的工作模式 供DAM3000M_SetDevWorkMode函数中的lMode参数使用
#define DAM3000M_WORKMODE_CNT					0x00 // 计数器
#define DAM3000M_WORKMODE_FREQ					0x01 // 频率器

// 计数器/频率的输入方式 供DAM3000M_PARA_CNT结构体中的lInputMode参数使用
#define DAM3000M_UNISOLATED						0x00 // 非隔离
#define DAM3000M_ISOLATED						0x01 // 隔离

// 门槛值状态 供DAM3000M_PARA_CNT结构体中的GateSts参数使用
#define DAM3000M_GATE_LOW						0x00 // 门槛值为低电平
#define DAM3000M_GATE_HIGH						0x01 // 门槛值为高电平
#define DAM3000M_GATE_NULL						0x02 // 门槛值无效

// 报警方式 供DAM3000M_CNT_ALARM结构体中的AlarmMode参数使用
#define CNT_ALARM_MODE0							0x00 // 报警方式0	0通道-1通道上限
#define CNT_ALARM_MODE1							0x01 // 报警方式1	0通道上限 / 上上限

// 报警方式0使能 供DAM3000M_CNT_ALARM结构体中的EnableAlarm0 和 EnableAlarm1参数使用
#define CNT_ALAMODE0_DISABLE					0x00 // 报警方式0禁止报警
#define CNT_ALAMODE0_ENABLE						0x01 // 报警方式0允许报警

// 报警方式1使能 供DAM3000M_CNT_ALARM结构体中的EnableAlarm0参数使用
#define CNT_ALAMODE1_DISABLE					0x00 // 报警方式1 计数器0 禁止报警
#define CNT_ALAMODE1_INSTANT					0x01 // 报警方式1 计数器0 瞬间报警允许
#define CNT_ALAMODE1_LATCH						0x02 // 报警方式1 计数器0 闭锁报警允许

// 滤波状态使能 供DAM3000M_PARA_FILTER结构体中的bEnableFilter参数使用
#define DAM3000M_FILTER_DISABLE					0x00 // 禁止滤波
#define DAM3000M_FILTER_ENABLE					0x01 // 允许滤波
//-----------------------------------------------------
#define DAM3000_BAUD_1200						0x00
#define DAM3000_BAUD_2400						0x01
#define DAM3000_BAUD_4800						0x02
#define DAM3000_BAUD_9600						0x03
#define DAM3000_BAUD_19200						0x04
#define DAM3000_BAUD_38400						0x05
#define DAM3000_BAUD_57600						0x06
#define DAM3000_BAUD_115200						0x07

#define DAM3000_DEFAULT_TIMEOUT                 -1

//########################## 电量 ###################################
#define DAM3000M_GET_I_RMS						0x00 // 获得电流有效值
#define DAM3000M_GET_V_RMS						0x01 // 获得电压有效值
#define DAM3000M_GET_POWER						0x02 // 获得有功功率
#define DAM3000M_GET_VAR						0x03 // 获得无功功率
#define DAM3000M_GET_VA							0x04 // 获得视在功率
#define DAM3000M_GET_WATTHR						0x05 // 获得正相有功电度
#define DAM3000M_GET_RWATTHR					0x06 // 获得反相有功电度
#define DAM3000M_GET_VARHR						0x07 // 获得正相无功电度
#define DAM3000M_GET_RVARHR						0x08 // 获得反相无功电度
#define DAM3000M_GET_PF							0x09 // 获得功率因数
#define DAM3000M_GET_FREQ						0x0A // 获得输入信号频率
#define DAM3000M_GET_VAWATTHR					0x0B // 获得电度

// 串口号(以此类推) 供DAM3000M_CreateDevice使用，可根据自身需要扩充
#define DAM3000M_COM1							0x01 // COM1
#define DAM3000M_COM2							0x02 // COM2
#define DAM3000M_COM3							0x03 // COM3
#define DAM3000M_COM4							0x04 // COM4
#define DAM3000M_COM5							0x05 // COM5

// 波特率选择 供DAM3000M_SetDeviceInfo和DAM3000M_GetDeviceInfo中的PDAM3000M_DEVICE_INFO使用
#define DAM3000M_BAUD_1200						0x00
#define DAM3000M_BAUD_2400						0x01
#define DAM3000M_BAUD_4800						0x02
#define DAM3000M_BAUD_9600						0x03
#define DAM3000M_BAUD_19200						0x04
#define DAM3000M_BAUD_38400						0x05
#define DAM3000M_BAUD_57600						0x06
#define DAM3000M_BAUD_115200					0x07

#define DAM3000M_DEFAULT_TIMEOUT                 -1

// 驱动函数接口
#ifndef _ART_DRIVER_
#define DEVAPI __declspec(dllimport)
#else
#define DEVAPI __declspec(dllexport)
#endif

#ifdef __cplusplus
extern "C" {
#endif
//####################### 设备对象管理函数 #################################	
	HANDLE DEVAPI FAR PASCAL DAM3000M_CreateDevice(LONG lPortNum);		// 创建设备对象

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_InitDevice(						// 初始与模块之间的通信参数
									HANDLE	hDevice,					// 设备对象句柄
									LONG    lBaud,						// 波特率
									BOOL    bCheck = TRUE,				// Modbus协议下其值必须为TRUE，即必须有交验
									LONG    lTimeOut = DAM3000M_DEFAULT_TIMEOUT);	// 超时时间，主要用于接收数据，如果为-1 则使用默认超时时间

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_ReleaseDevice(HANDLE hDevice);		// 释放设备对象 

//################################### 模块信息取得/修改函数 ############################
	BOOL DEVAPI FAR PASCAL	DAM3000M_GetDeviceInfo(						// 读取模块信息(类型、地址、波特率、校验)
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PDAM3000M_DEVICE_INFO pInfo);		// 设备信息

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetDeviceInfo(						// 修改模块信息(地址、波特率、校验)
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									DAM3000M_DEVICE_INFO& Info);		// 设备信息
	
	BOOL DEVAPI	FAR	PASCAL  DAM3000M_GetDevLastError(					// 获得最后一个错误
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID);					// 模块地址
		
	//####################### AD数据读取函数 ###################################	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_ReadDeviceAD(						// 读取AD模拟量输入 
									HANDLE	hDevice,					// 设备对象句柄	
									LONG	lDeviceID,					// 模块地址
									LONG	lpADBuffer[],				// 接收AD数据的用户缓冲区 注意:lpADBuffer最好大于等于lLastChannel - lFirstChannel +1
									LONG    lBufferSize,        		// 数组lpADBuffer[]的大小
									LONG	lFirstChannel = 0,			// 首通道
									LONG	lLastChannel = 0);			// 末通道
		
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetModeAD(							// 获得模拟量输入模式 
									HANDLE	hDevice,					// 设备对象句柄	
									LONG	lDeviceID,					// 模块地址
									PLONG	lpMode,						// AD模式
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetModeAD(							// 设置AD输入模式 
									HANDLE	hDevice,					// 设备对象句柄	
									LONG	lDeviceID,					// 模块地址
									LONG	lMode,						// AD模式 
									LONG	lChannel = 0);				// 通道号
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetGroundingAD(					// 获得通道接地模式(只对可软件配置单/双端输入模块有效)
									HANDLE	hDevice,					// 设备对象句柄	
									LONG	lDeviceID,					// 模块地址
									PLONG	lpGrounding,				// AD通道接地模式
									LONG	lChannel = 0);				// 通道号
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetGroundingAD(					// 设置通道接地模式(只对可软件配置单/双端输入模块有效)
									HANDLE	hDevice,					// 设备对象句柄	
									LONG	lDeviceID,					// 模块地址
									LONG	lGrounding,					// AD通道接地模式
									LONG	lChannel = 0);				// 通道号
	
	BOOL DEVAPI FAR PASCAL	DAM3000M_GetLowLimitVal(					// 获得模拟量输入报警下限值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 模块地址
									LONG	lLowLimit[],				// 下限报警值
									LONG    lFirstChannel = 0,			// 首通道号
									LONG	lLastChannel = 0);			// 末通道号

	BOOL DEVAPI FAR PASCAL DAM3000M_GetHighLimitVal(					// 获得模拟量输入报警上限值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 模块地址
									LONG	lHighLimit[],				// 上限报警值
									LONG    lFirstChannel = 0,			// 首通道号
									LONG	lLastChannel = 0);			// 末通道号

	BOOL DEVAPI FAR PASCAL DAM3000M_SetLowLimitVal(						// 设置下限报警值
									HANDLE	hDevice,
									LONG	lDeviceID,					// 模块地址
									LONG	lLowVal[],					// 下限报警值
									LONG    lFirstChannel = 0,			// 首通道号
									LONG	lLastChannel = 0);			// 末通道号
		
	BOOL DEVAPI FAR PASCAL DAM3000M_SetHighLimitVal(					// 设置上限报警值
									HANDLE	hDevice,
									LONG	lDeviceID,					// 模块地址
									LONG	lHighVal[],					// 上限报警值
									LONG    lFirstChannel = 0,			// 首通道号
									LONG	lLastChannel = 0);			// 末通道号
		
	BOOL DEVAPI FAR PASCAL  DAM3000M_GetAlarmPulse(						// 获得报警的电平
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 模块地址
									PLONG	lpAlarmVal,					// 报警电平,0:低电平, 1:高电平
									LONG	lChannel = 0);				// 通道号
	
	BOOL DEVAPI FAR PASCAL  DAM3000M_SetAlarmPulse(						// 设置模拟量输入报警电平
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 模块地址
									LONG	lAlarmVal,					// 报警电平,0:低电平, 1:高电平	
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetAlarmSts(						// 获得报警状态
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 模块地址
									PLONG	lpAlarmSts,					// 报警状态
									LONG	lChannel = 0);				// 读取通道

	//####################### DA数据读取函数 ###################################
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetDeviceDAVal(					// 回读DA输出值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PLONG	lpDAValue,          		// DA输出当前值
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_WriteDeviceDA(						// 设定单通道DA
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lDAData,					// DA输出值
									LONG	lChannel = 0);      		// 通道号
	
	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetOutPutRangeDA(					// 读取模拟量输出量程
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PLONG	lpRange,					// 输出量程
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetOutPutRangeDA(					// 设置模拟量输出量程
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lRange,						// 输出量程
									LONG	lChannel = 0);				// 通道号
	
	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetPowerOnValueDA(					// 获得DA上电值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PLONG	lpPowerOnVal,       		// 上电值
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetPowerOnValueDA(					// 设置DA上电值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lPowerOnVal,				// 上电值
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetSafeValueDA(					// 获得DA安全值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PLONG	lpSafeVal,					// 安全值
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetSafeValueDA(					// 设置DA安全值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lSafeVal,					// 安全值
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetSlopeDA(						// 读模拟量输出斜率
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PLONG	lpSlopeType,				// 输出斜率类型
									LONG	lChannel = 0);				// 通道号
	
	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetSlopeDA(						// 修改模拟量输出斜率
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lSlopeType,					// 输出斜率类型
									LONG	lChannel = 0);				// 通道号

	//####################### DI输入输出操作函数 ##############################
	BOOL DEVAPI FAR PASCAL	DAM3000M_GetModeDI(							// 读取数字量输入的工作模式
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lMode[],					// 输入的工作模式,0:DI模式,1:计数方式,2:锁存方式
									LONG	lEdgeMode[],				// 边沿方式,0:低电平,1:高电平
									LONG	lFirstChannel = 0,			// 首通道
									LONG	lLastChannel = 0);			// 末通道

	BOOL DEVAPI FAR PASCAL	DAM3000M_SetModeDI(							// 设置数字量输入的工作模式
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lMode,						// 输入的工作模式,0:DI模式,1:计数方式,2:锁存方式
									LONG	lEdgeMode,					// 边沿方式,0:低电平,1:高电平
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetDeviceDI(						// 读取开关量输入 
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PDAM3000M_PARA_DI pDIPara);			// DI值
		
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_StartDeviceDI(						// 启动DI计数
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lChannel = 0);				// 通道号
		
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_StopDeviceDI(						// 停止DI计数
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lChannel = 0);				// 通道号
		
	BOOL DEVAPI FAR PASCAL  DAM3000M_GetCNTDI(							// 读取DI计数器值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PLONG	lpCounterValue,     		// 范围(0~65535)
									LONG	lFirstChannel = 0,			// 首通道
									LONG	lLastChannel = 0);			// 末通道

	BOOL DEVAPI	FAR PASCAL  DAM3000M_SetCNTDI(							// 设置DI计数器初始值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lInitValue,					// 计数初值
									LONG	lChannel = 0);				// 通道号
	 
	BOOL DEVAPI FAR PASCAL  DAM3000M_StartLatch(						// 启动锁存
									HANDLE	 hDevice,					// 设备句柄
									LONG	 lDeviceID,					// 设备地址
									LONG	 lChannel = 0);				// 通道号
	
	BOOL DEVAPI FAR PASCAL  DAM3000M_StopLatch(							// 停止锁存
									HANDLE	 hDevice,					// 设备句柄
									LONG	 lDeviceID,					// 设备地址
									LONG	 lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR PASCAL  DAM3000M_GetLatchStatus(					// 读锁存状态
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lLatchType,					// 锁存类型，分为上升沿和下降沿锁存
									PDAM3000M_PARA_LATCH pLatchStatus);	// 锁存状态

	BOOL DEVAPI	FAR PASCAL  DAM3000M_ClearCNTVal(						// 清除计数值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR PASCAL  DAM3000M_ClearLatchStatus(					// 清除锁存状态
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lChannel = 0);				// 通道号
			
	//####################### DO数字量输出函数 ###################################	
	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetDeviceDO(						// 回读开关量输出
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PDAM3000M_PARA_DO pDOPara);			// 当前DO输出值

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetDeviceDO(						// 设置DO开关量输出值 
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									BYTE	byDOSts[],					// 设置DO输出值
									LONG    lFirstChannel,				// 首通道号
									LONG	lLastChannel);				// 末通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetPowerOnValueDO(					// 获取DO上电初始值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PDAM3000M_PARA_DO pPowerOnPara);	// 上电值 

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetPowerOnValueDO(					// 设置DO上电初始值 
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									DAM3000M_PARA_DO& PowerOnPara);		// 上电值

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetSafeValueDO(					// 读DO安全值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PDAM3000M_PARA_DO pDOSafePara);		// 安全值

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetSafeValueDO(					// 设置安全值
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									DAM3000M_PARA_DO& DOSafePara);		// 安全值

	//#############################  计数器  ###################################
	BOOL DEVAPI FAR PASCAL DAM3000M_SetCounterMode(						// 对各个计数器进行参数设置
									HANDLE   hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									PDAM3000M_PARA_CNT pCNTPara,		// 基于各通道的计数器参数
									LONG	 lChannel = 0);				// 通道号

	BOOL DEVAPI FAR PASCAL DAM3000M_InitCounterAlarm(					// 初始化报警的工作模式
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									PDAM3000M_CNT_ALARM pCNTAlarm);		// 报警参数设置

	BOOL DEVAPI FAR PASCAL DAM3000M_SetCounterAlarmMode(				// 设置计数器报警方式
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									LONG	 lAlarmMode);				// 报警方式

	BOOL DEVAPI FAR PASCAL DAM3000M_GetCounterSts(						// 获得计数器设备硬件参数状态
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									PDAM3000M_CNT_STATUS pStsCNT,		// 返回值
									LONG	 lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_StartCounter(						// 启动计数器计数
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									LONG	 lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_StopCounter(						// 停止计数器计数
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									LONG	 lChannel = 0);				// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetCounterCurVal(					// 取得计数器当前值
									HANDLE	 hDevice,					// 设备对象句柄
									LONG     lDeviceID,					// 设备地址
									PULONG   pulCNTVal,					// 计数值
									LONG     lChannel = 0);				// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetFreqCurVal(						// 取得频率器当前值
									HANDLE   hDevice,					// 设备对象句柄
									LONG     lDeviceID,					// 设备地址
									PULONG   pulFreqVal,				// 频率值
									LONG     lChannel = 0);				// 通道号

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_ResetCounter(						// 计数器复位
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									LONG	 lChannel = 0);				// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_InitCounterFilter(					// 初始化滤波
									HANDLE   hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									PDAM3000M_PARA_FILTER pFilter,		// 滤波参数
									LONG	 lChannel = 0);				// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_EnableFilter(						// 使能滤波状态
									HANDLE   hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									BOOL	 bEnable,					// 使能滤波
									LONG	 lChannel = 0);				// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetCounterAlarmSts(				// 获得DO及报警状态
									HANDLE   hDevice,					// 设备对象句柄
									LONG     lDeviceID,					// 设备地址
									PLONG    plEnableAlarm,				// 计数器报警状态
									PLONG    pbDO,						// DO
									LONG	 lChannel = 0);				// 通道号
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetCounterDO(						// 设置DO
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									BYTE	byDOSts[],					// DO
									LONG    lFirstChannel,				// 首通道号
									LONG	lLastChannel);				// 末通道号
										
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_ClearAlarmSts(						// 清报警方式1报警输出
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID);					// 设备地址
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetLEDCounterCH(					// 取得计数器LED显示通道
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									PLONG	plChannel);					// 通道号
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetLEDCounterCH(					// 设置计数器LED显示通道
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 设备地址
									LONG	lChannel);					// 通道号

	//#############################  电量模块  ###################################
	BOOL DEVAPI FAR PASCAL  DAM3000M_GetEnergyVal(						// 获得电量值
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									LONG	 lValue[],					// 电量值
									LONG	 lAanlogType,				// 模拟量类型
									LONG	 lFirstChannel = 0,			// 首通道
									LONG	 lLastChannel = 0);			// 末通道

	BOOL DEVAPI FAR PASCAL  DAM3000M_ClrEnergyReg(						// 清能量寄存器
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									LONG	 lChannel = 0);				// 通道

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetEnergyPerLSB(					// 获得能量单位
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									PLONG	 lpEnergyPerLSB);			// 能量单位

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetEnergyPerLSB(					// 设置能量单位
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									LONG	 lEnergyPerLSB);			// 能量单位

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetInputRange(						// 获得输入量程
									HANDLE	 hDevice,					// 设备对象句柄
									LONG	 lDeviceID,					// 设备地址
									PLONG	 lpInputRangeV,				// 电压输入量程
									PLONG	 lpInputRangeI);			// 电流输入量程

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetInputRange(						// 设置输入量程
									HANDLE	hDevice,					// 设备对象句
									LONG	lDeviceID,					// 设备地址
									LONG	lInputRangeV,				// 电压输入量程
									LONG	lInputRangeI);				// 电流输入量程

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetEvrmTemp(						// 获得环境温度
									HANDLE	hDevice,					// 设备对象句
									LONG	lDeviceID,					// 设备地址
									PLONG	lpEvrmTemp,					// 温度
									LONG	lChannel = 0);				// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetEvrmHum(						// 获得环境湿度
									HANDLE	hDevice,					// 设备对象句
									LONG	lDeviceID,					// 设备地址
									PLONG	lpEvrmHum,					// 湿度		
									LONG	lChannel = 0);				// 通道号


	//##################################### 看门狗 ################################
	BOOL DEVAPI FAR PASCAL  DAM3000M_HostIsOK(HANDLE hDevice);			// 下位机无返回信息

	BOOL DEVAPI FAR PASCAL  DAM3000M_EnableWatchdog(					// 打开主看门狗(先设置超时间隔，再使能看门狗)
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID);					// 模块地址

	BOOL DEVAPI FAR PASCAL  DAM3000M_CloseWatchdog(						// 禁止看门狗工作
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID);					// 模块地址

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetWatchdogStatus(					// 读取主看门狗的状态(S = 1, Host is down; S = 0 OK)
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 模块地址
									PLONG	lpWatchdogStatus);			// 看门狗状态

	BOOL DEVAPI FAR PASCAL  DAM3000M_ResetWatchdogStatus(				// Func: 复位主看门狗的状态(S = 0)
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID);					// 模块地址

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetWatchdogTimeoutVal(				// 取得看门狗设置的时间间隔
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 模块地址
									PLONG	lpInterval);				// 时间间隔

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetWatchdogTimeoutVal(				// 设置看门狗设置的时间间隔
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 模块地址
									LONG	lInterval);					// 时间间隔

	// ##################################### DIGIT LED 设置函数 #################################
	BOOL DEVAPI FAR PASCAL  DAM3000M_GetDLedMode(						// 获得显示模式请求
									HANDLE hDevice,						// 设备对象句柄
									LONG   lDeviceID,					// 模块地址
									PLONG  lpDispMode);					// 显示模式 0x00：温度格式,0x01:欧姆值

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetDLedMode(						// 修改显示模式请求
									HANDLE hDevice,						// 设备对象句柄
									LONG   lDeviceID,					// 模块地址
									LONG   lDispMode);					// 显示模式 0x01：温度格式,0x02:欧姆值

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetDLedDispChannel(				// 获得LED显示通道号
									HANDLE hDevice,						// 设备对象句柄
									LONG   lDeviceID,					// 模块地址
									PLONG  lpChannel);					// 通道号,lpChannel = 0xff:主机控制显示

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetDLedDispChannel(				// 设置LED显示通道号
									HANDLE hDevice,						// 设备对象句柄
									LONG   lDeviceID,					// 模块地址
									LONG   lChannel = 0);				// 通道号,lpChannel = 0xff:主机控制显示
		
	BOOL DEVAPI FAR PASCAL  DAM3000M_SetDLedValue(						// 主机控制显示值
									HANDLE hDevice,						// 设备对象句柄
									LONG   lDeviceID,					// 模块地址
									LPCTSTR strWriteBuf,				// 显示的字符串
									LONG   llength);					// 数据长度


//####################### 输入输出任意二进制字符 ###########################
	int DEVAPI	FAR	PASCAL	DAM3000M_WriteDeviceChar(					// 直接写设备
									HANDLE	hDevice,					// 设备对象句柄
									char*	strWriteBuf,				// 写的数据
									long	llength,					// 数据长度
									long	timeout = 100);				// 超时范围(mS)
	
	int	DEVAPI	FAR	PASCAL	DAM3000M_ReadDeviceChar(					// 直接读设备
									HANDLE	hDevice,					// 设备对象句柄
									char*	strReadBuf,					// 读取的数据
									long	llength,					// 数据长度
									long	timeout = 100);				// 超时范围(mS)
											   

//###########################　模块信息确认函数  #################################
	BOOL DEVAPI	FAR	PASCAL	InitCheckInfo(
									HANDLE	hDevice, 
									LONG	lDeviceID, 
									DWORD	dwNum);

	BOOL DEVAPI	FAR	PASCAL	ReadCheckInfo(
									HANDLE	hDevice, 
									LONG	lDeviceID, 
									LONG	lIndex, 
									BYTE&	byEncrypt);

//####################################### 辅助函数 ####################################
	BOOL DEVAPI FAR PASCAL  DAM3000M_AdjustSlopeVal(					// 微调当前补偿斜率
									HANDLE  hDevice,					// 设备对象句柄
									LONG    lDeviceID,   				// 模块地址
									LONG    lAdjustVal,					// 微调值
									LONG    lChannel);					// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_StoreSlopeVal(						// 设置当前值为输出补偿斜率
									HANDLE  hDevice,					// 设备对象句柄
									LONG    lDeviceID,   				// 模块地址
									LONG    lChannel);					// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetFaultSlopeVal(					// 设定补偿斜率为默认值
									HANDLE  hDevice,					// 设备对象句柄
									LONG    lDeviceID,   				// 模块地址
									LONG    lChannel);					// 通道号	0xFF为所有通道恢复

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetZeroRepair(						// 设置零点偏移补偿
									HANDLE	hDevice,					// 设备对象句柄
									LONG	lDeviceID,					// 模块地址
									LONG	lZeroRepair,				// 零点值
									LONG	lChannel);					// 通道号

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetDevTestMode(					// 设置模块进入测试模式
									HANDLE  hDevice,					// 设备对象句柄
									LONG    lDeviceID);					// 模块地址

	BOOL DEVAPI FAR PASCAL  DAM3000M_ResetModule(						// 模块软复位
									HANDLE  hDevice,					// 设备对象句柄
									LONG    lDeviceID); 				// 模块地址

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetEnvironmentTemp(				// 取得环境温度(为取热电偶值作准备)
									HANDLE	hDevice,
									LONG	lDeviceID,
									PFLOAT	lpETemprt);

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetAdjustTemp(				// 取得环境温度(为取热电偶值作准备)
									HANDLE	hDevice,
									LONG	lDeviceID,
									BYTE	lETemprt);

	BOOL DEVAPI	FAR	PASCAL  DAM3000M_GetDevLastError(					// 错误应答命令
									HANDLE	hDevice, 
									LONG	lDeviceID);


#ifdef __cplusplus
}
#endif

//#######################################################################
#ifndef _ART_DRIVER_
	#pragma comment(lib, "DAM3000M.lib")
	#pragma message("======== Welcome to use our art company product!")
	#pragma message("======== Automatically linking with DAM3000M.dll...")
	#pragma message("======== Successfully linked with DAM3000M.dll")
#endif

#endif // ifndef _ART_DAM3000M_SERIAL_
