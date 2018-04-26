#ifndef  _ART_DAM3000M_SERIAL_
#define _ART_DAM3000M_SERIAL_

#include <windows.h>

// ********************* ����������Ĳ����ṹ ****************************
typedef struct _DAM3000M_PARA_DO			// �������������
{
	BYTE DO0;       	// 0ͨ��
	BYTE DO1;       	// 1ͨ��
	BYTE DO2;       	// 2ͨ��
	BYTE DO3;       	// 3ͨ��
	BYTE DO4;       	// 4ͨ��
	BYTE DO5;       	// 5ͨ��
	BYTE DO6;       	// 6ͨ��
	BYTE DO7;       	// 7ͨ��
	BYTE DO8;			// 8ͨ��
	BYTE DO9;			// 9ͨ��
	BYTE DO10;      	// 10ͨ��
	BYTE DO11;      	// 11ͨ��
	BYTE DO12;      	// 12ͨ��
	BYTE DO13;			// 13ͨ��
	BYTE DO14;			// 14ͨ��
	BYTE DO15;			// 15ͨ��
} DAM3000M_PARA_DO, *PDAM3000M_PARA_DO;


// ********************* ����������Ĳ����ṹ *******************************
typedef struct  _DAM3000M_PARA_DI		// �������������(1Ϊ�ߵ�ƽ)
{
	BYTE DI0;			// 0ͨ��
	BYTE DI1;			// 1ͨ��
	BYTE DI2;			// 2ͨ��
	BYTE DI3;			// 3ͨ��
	BYTE DI4;			// 4ͨ��
	BYTE DI5;			// 5ͨ��
	BYTE DI6;			// 6ͨ��
	BYTE DI7;			// 7ͨ��
	BYTE DI8;			// 8ͨ��
	BYTE DI9;			// 9ͨ��
	BYTE DI10;			// 10ͨ��
	BYTE DI11;			// 11ͨ��
	BYTE DI12;			// 12ͨ��
	BYTE DI13;			// 13ͨ��
	BYTE DI14;			// 14ͨ��
	BYTE DI15;			// 15ͨ��
} DAM3000M_PARA_DI, *PDAM3000M_PARA_DI;

typedef struct  _DAM3000M_PARA_LATCH      // �������������(1Ϊ����)
{
	BYTE Latch0;		// 0ͨ��
	BYTE Latch1;        // 1ͨ��
	BYTE Latch2;        // 2ͨ��
	BYTE Latch3;        // 3ͨ��
	BYTE Latch4;        // 4ͨ��
	BYTE Latch5;        // 5ͨ��
	BYTE Latch6;        // 6ͨ��
	BYTE Latch7;        // 7ͨ��
	BYTE Latch8;        // 8ͨ��
	BYTE Latch9;        // 9ͨ��
	BYTE Latch10;       // 10ͨ��
	BYTE Latch11;       // 11ͨ��
	BYTE Latch12;       // 12ͨ��
	BYTE Latch13;       // 13ͨ��
	BYTE Latch14;       // 14ͨ��
	BYTE Latch15;       // 15ͨ��
} DAM3000M_PARA_LATCH, *PDAM3000M_PARA_LATCH;


// ****************** ģ��������ͨ�����ýṹ�� *************************
typedef struct _DAM3000M_ADCHANNEL_ARRAY
{
	BYTE bChannel0;		// 1����Ч��0����Ч
	BYTE bChannel1;
	BYTE bChannel2;
	BYTE bChannel3;
	BYTE bChannel4;
	BYTE bChannel5;
	BYTE bChannel6;
	BYTE bChannel7;
}DAM3000M_ADCHANNEL_ARRAY, *PDAM3000M_ADCHANNEL_ARRAY;


// ****************** �������������ýṹ�� ******************************
typedef struct _DAM3000M_PARA_CNT			// ���ڸ�ͨ���ļ����������ṹ��
{
	LONG WorkMode;			// ������/Ƶ�ʹ���ģʽ
	LONG FreqBuildTime;		// ��Ƶ������ʱ��, ��λ: s
	LONG InputMode;			// ������/Ƶ�����뷽ʽ	0: �Ǹ���	1: ����
	ULONG InitVal;			// ��������ʼֵ
	ULONG MaxVal;			// ���������ֵ
	LONG GateSts;			// �ż�ֵ״̬(����ģʽ)
} DAM3000M_PARA_CNT, *PDAM3000M_PARA_CNT;

typedef struct _DAM3000M_CNT_ALARM
{
	LONG AlarmMode;			// ������ʽ	
	LONG EnableAlarm0;		// 0ͨ������ʹ��
	LONG EnableAlarm1;		// 1ͨ������ʹ��
	ULONG Alarm0Val;		// 0ͨ������ֵ
	ULONG Alarm1Val;		// 1ͨ������ֵ
	ULONG Alarm0HiHiVal;	// 0ͨ��������(Hi-Hi)����ֵ, ������ʽ1��Ч
} DAM3000M_CNT_ALARM, *PDAM3000M_CNT_ALARM;

typedef struct _DAM3000M_PARA_FILTER		// ���ڼ������˲��Ĳ����ṹ��
{
	LONG TrigLevelHigh;		// �����ߵ�ƽ(�Ǹ�������)
	LONG TrigLevelLow;		// �����͵�ƽ(�Ǹ�������)
	LONG MinWidthHigh;		// �ߵ�ƽ��С�����źſ��
	LONG MinWidthLow;		// �͵�ƽ��С�����źſ��
	LONG bEnableFilter;		// ʹ���˲�
} DAM3000M_PARA_FILTER, *PDAM3000M_PARA_FILTER;
//	LONG DisplayChannel;	// ������ʾͨ��		0��0ͨ������/Ƶ�ʣ�1��1ͨ������/Ƶ��


typedef struct _DAM3000M_CNT_STATUS			// ������Ӳ������״̬�ṹ��
{
	LONG WorkMode;			// ������/Ƶ�ʹ���ģʽ*
	LONG FreqBuildTime;		// ��Ƶ������ʱ��, ��λ: s*
	LONG InputMode;			// ������/Ƶ�����뷽ʽ	0: �Ǹ���	1: ����*
	LONG bCNTSts;			// ����/Ƶ������״̬(��ͣ״̬)*
	LONG FilterSts;			// ���������˲�״̬*
	LONG MinWidthHigh;		// �ߵ�ƽ��С�����źſ��*
	LONG MinWidthLow;		// �͵�ƽ��С�����źſ��*
	LONG TrigLevelHigh;		// �����ߵ�ƽ(�Ǹ�������)*
	LONG TrigLevelLow;		// �����͵�ƽ(�Ǹ�������)*
	LONG GateSts;			// �ż�ֵ����״̬(����ģʽ)*
	ULONG MaxVal;			// ���������ֵ*
	ULONG InitVal;			// ��������ʼֵ*
	LONG bOverflowSts;		// ���������״̬*
	LONG AlarmMode;			// ������������ʽ*
	LONG EnableAlarm0;		// ������0����ʹ��״̬*
	LONG EnableAlarm1;		// ������1����ʹ��״̬*
	ULONG Alarm0Val;		// 0ͨ������ֵ*
	ULONG Alarm1Val;		// 1ͨ������ֵ*
	ULONG Alarm1HiHiVal;	// ������ʽ1������(Hi-Hi)����ֵ*
	LONG bDO0;				// DO0*
	LONG bDO1;				// DO1*
} DAM3000M_CNT_STATUS, *PDAM3000M_CNT_STATUS;


// ****************** �豸������Ϣ�Ľṹ�� ******************************
typedef struct _DAM3000M_DEVICE_INFO
{
	LONG    DeviceType;		// ģ������ 
	LONG    TypeSuffix;		// ���ͺ�׺
	LONG	ModusType;		// M
	LONG	VesionID;		// �汾��(2�ֽ�)
	LONG	DeviceID;		// ģ��ID��(SetDeviceInfoʱ��Ϊ�豸����ID)
	LONG	BaudRate;		// ������
	LONG	bParity;		// ����У��
} DAM3000M_DEVICE_INFO, *PDAM3000M_DEVICE_INFO;

// ģ������������(��ѹ����) ��DAM3000M_SetModeAD�����е�lMode����ʹ��
#define DAM3000M_VOLT_N15_P15					0x01 //  -15��+15mV
#define DAM3000M_VOLT_N50_P50					0x02 //  -50��+50mV
#define DAM3000M_VOLT_N100_P100					0x03 // -100��+100mV
#define DAM3000M_VOLT_N150_P150					0x04 // -150��+150mV
#define DAM3000M_VOLT_N500_P500					0x05 // -500��+500mV
#define DAM3000M_VOLT_N1_P1						0x06 //   -1��+1V
#define DAM3000M_VOLT_N25_P25					0x07 // -2.5��+2.5V
#define DAM3000M_VOLT_N5_P5						0x08 //   -5��+5V
#define DAM3000M_VOLT_N10_P10					0x09 //  -10��+10V
#define DAM3000M_VOLT_N0_P5						0x0D //    0��+5V
#define DAM3000M_VOLT_N0_P10					0x0E //    0��+10V
#define DAM3000M_VOLT_N0_P25					0x0F //    0��+2.5V

// ģ������������(��������) ��DAM3000M_SetModeAD�����е�lMode����ʹ��
#define DAM3000M_CUR_N0_P10						0x00 //   0��10mA
#define DAM3000M_CUR_N20_P20					0x0A // -20��+20mA
#define DAM3000M_CUR_N0_P20						0x0B //   0��20mA
#define DAM3000M_CUR_N4_P20						0x0C //   4��20mA

// ģ������������(�ȵ�ż����) ��DAM3000M_SetModeAD�����е�lMode����ʹ��
#define DAM3000M_TMC_J							0x10 // J���ȵ�ż   0��1200��
#define DAM3000M_TMC_K							0x11 // K���ȵ�ż   0��1300��
#define DAM3000M_TMC_T							0x12 // T���ȵ�ż -200��400��
#define DAM3000M_TMC_E							0x13 // E���ȵ�ż   0��1000��
#define DAM3000M_TMC_R							0x14 // R���ȵ�ż 500��1700��
#define DAM3000M_TMC_S							0x15 // S���ȵ�ż 500��1768��
#define DAM3000M_TMC_B							0x16 // B���ȵ�ż 500��1800��
#define DAM3000M_TMC_N							0x17 // N���ȵ�ż   0��1300��
#define DAM3000M_TMC_C							0x18 // C���ȵ�ż   0��2090��
#define DAM3000M_TMC_WRE						0x19 // ���5-���26 0��2310��

// ģ������������(�ȵ�������) ��DAM3000M_SetModeAD�����е�lMode����ʹ��
#define DAM3000M_RTD_PT100_385_N200_P850		0x20 // Pt100(385)�ȵ��� -200�桫850��
#define DAM3000M_RTD_PT100_385_N100_P100		0x21 // Pt100(385)�ȵ��� -100�桫100��
#define DAM3000M_RTD_PT100_385_N0_P100			0x22 // Pt100(385)�ȵ���    0�桫100��
#define DAM3000M_RTD_PT100_385_N0_P200			0x23 // Pt100(385)�ȵ���    0�桫200��
#define DAM3000M_RTD_PT100_385_N0_P600			0x24 // Pt100(385)�ȵ���    0�桫600��
#define DAM3000M_RTD_PT100_3916_N200_P850		0x25 // Pt100(3916)�ȵ���-200�桫850��
#define DAM3000M_RTD_PT100_3916_N100_P100		0x26 // Pt100(3916)�ȵ���-100�桫100��
#define DAM3000M_RTD_PT100_3916_N0_P100			0x27 // Pt100(3916)�ȵ���   0�桫100��
#define DAM3000M_RTD_PT100_3916_N0_P200			0x28 // Pt100(3916)�ȵ���   0�桫200��
#define DAM3000M_RTD_PT100_3916_N0_P600			0x29 // Pt100(3916)�ȵ���   0�桫600��
#define DAM3000M_RTD_PT1000						0x30 // Pt1000�ȵ���     -200�桫850��
#define DAM3000M_RTD_CU50						0x40 // Cu50�ȵ���        -50�桫150��
#define DAM3000M_RTD_CU100						0x41 // Cu100�ȵ���       -50�桫150��
#define DAM3000M_RTD_BA1						0x42 // BA1�ȵ���        -200�桫650��
#define DAM3000M_RTD_BA2						0x43 // BA2�ȵ���        -200�桫650��
#define DAM3000M_RTD_G53						0x44 // G53�ȵ���         -50�桫150��
#define DAM3000M_RTD_Ni50						0x45 // Ni50�ȵ���        100��
#define DAM3000M_RTD_Ni508						0x46 // Ni508�ȵ���         0�桫100��
#define DAM3000M_RTD_Ni1000						0x47 // Ni1000�ȵ���      -60�桫160��

// ģ�������б������	��DAM3000M_SetModeDA�����еĲ��� lType ʹ��
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

// DI������ʽ ��DAM3000M_SetDeviceMode�����е�lMode����ʹ��
#define DAM3000M_DI_MODE_DI						0x00 // DI��ʽ
#define DAM3000M_DI_MODE_COUNT					0x01 // ������ʽ
#define DAM3000M_DI_MODE_LATCH					0x02 // ���淽ʽ

// DI������ʽ ��DAM3000M_SetDeviceMode�����е�lEdgeMode����ʹ��
#define DAM3000M_DIR_FALLING					0x00 // �½���
#define DAM3000M_DIR_RISING						0x01 // ������

//########################## ������ ###################################
// ģ��Ĺ���ģʽ ��DAM3000M_SetDevWorkMode�����е�lMode����ʹ��
#define DAM3000M_WORKMODE_CNT					0x00 // ������
#define DAM3000M_WORKMODE_FREQ					0x01 // Ƶ����

// ������/Ƶ�ʵ����뷽ʽ ��DAM3000M_PARA_CNT�ṹ���е�lInputMode����ʹ��
#define DAM3000M_UNISOLATED						0x00 // �Ǹ���
#define DAM3000M_ISOLATED						0x01 // ����

// �ż�ֵ״̬ ��DAM3000M_PARA_CNT�ṹ���е�GateSts����ʹ��
#define DAM3000M_GATE_LOW						0x00 // �ż�ֵΪ�͵�ƽ
#define DAM3000M_GATE_HIGH						0x01 // �ż�ֵΪ�ߵ�ƽ
#define DAM3000M_GATE_NULL						0x02 // �ż�ֵ��Ч

// ������ʽ ��DAM3000M_CNT_ALARM�ṹ���е�AlarmMode����ʹ��
#define CNT_ALARM_MODE0							0x00 // ������ʽ0	0ͨ��-1ͨ������
#define CNT_ALARM_MODE1							0x01 // ������ʽ1	0ͨ������ / ������

// ������ʽ0ʹ�� ��DAM3000M_CNT_ALARM�ṹ���е�EnableAlarm0 �� EnableAlarm1����ʹ��
#define CNT_ALAMODE0_DISABLE					0x00 // ������ʽ0��ֹ����
#define CNT_ALAMODE0_ENABLE						0x01 // ������ʽ0������

// ������ʽ1ʹ�� ��DAM3000M_CNT_ALARM�ṹ���е�EnableAlarm0����ʹ��
#define CNT_ALAMODE1_DISABLE					0x00 // ������ʽ1 ������0 ��ֹ����
#define CNT_ALAMODE1_INSTANT					0x01 // ������ʽ1 ������0 ˲�䱨������
#define CNT_ALAMODE1_LATCH						0x02 // ������ʽ1 ������0 ������������

// �˲�״̬ʹ�� ��DAM3000M_PARA_FILTER�ṹ���е�bEnableFilter����ʹ��
#define DAM3000M_FILTER_DISABLE					0x00 // ��ֹ�˲�
#define DAM3000M_FILTER_ENABLE					0x01 // �����˲�
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

//########################## ���� ###################################
#define DAM3000M_GET_I_RMS						0x00 // ��õ�����Чֵ
#define DAM3000M_GET_V_RMS						0x01 // ��õ�ѹ��Чֵ
#define DAM3000M_GET_POWER						0x02 // ����й�����
#define DAM3000M_GET_VAR						0x03 // ����޹�����
#define DAM3000M_GET_VA							0x04 // ������ڹ���
#define DAM3000M_GET_WATTHR						0x05 // ��������й����
#define DAM3000M_GET_RWATTHR					0x06 // ��÷����й����
#define DAM3000M_GET_VARHR						0x07 // ��������޹����
#define DAM3000M_GET_RVARHR						0x08 // ��÷����޹����
#define DAM3000M_GET_PF							0x09 // ��ù�������
#define DAM3000M_GET_FREQ						0x0A // ��������ź�Ƶ��
#define DAM3000M_GET_VAWATTHR					0x0B // ��õ��

// ���ں�(�Դ�����) ��DAM3000M_CreateDeviceʹ�ã��ɸ���������Ҫ����
#define DAM3000M_COM1							0x01 // COM1
#define DAM3000M_COM2							0x02 // COM2
#define DAM3000M_COM3							0x03 // COM3
#define DAM3000M_COM4							0x04 // COM4
#define DAM3000M_COM5							0x05 // COM5

// ������ѡ�� ��DAM3000M_SetDeviceInfo��DAM3000M_GetDeviceInfo�е�PDAM3000M_DEVICE_INFOʹ��
#define DAM3000M_BAUD_1200						0x00
#define DAM3000M_BAUD_2400						0x01
#define DAM3000M_BAUD_4800						0x02
#define DAM3000M_BAUD_9600						0x03
#define DAM3000M_BAUD_19200						0x04
#define DAM3000M_BAUD_38400						0x05
#define DAM3000M_BAUD_57600						0x06
#define DAM3000M_BAUD_115200					0x07

#define DAM3000M_DEFAULT_TIMEOUT                 -1

// ���������ӿ�
#ifndef _ART_DRIVER_
#define DEVAPI __declspec(dllimport)
#else
#define DEVAPI __declspec(dllexport)
#endif

#ifdef __cplusplus
extern "C" {
#endif
//####################### �豸��������� #################################	
	HANDLE DEVAPI FAR PASCAL DAM3000M_CreateDevice(LONG lPortNum);		// �����豸����

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_InitDevice(						// ��ʼ��ģ��֮���ͨ�Ų���
									HANDLE	hDevice,					// �豸������
									LONG    lBaud,						// ������
									BOOL    bCheck = TRUE,				// ModbusЭ������ֵ����ΪTRUE���������н���
									LONG    lTimeOut = DAM3000M_DEFAULT_TIMEOUT);	// ��ʱʱ�䣬��Ҫ���ڽ������ݣ����Ϊ-1 ��ʹ��Ĭ�ϳ�ʱʱ��

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_ReleaseDevice(HANDLE hDevice);		// �ͷ��豸���� 

//################################### ģ����Ϣȡ��/�޸ĺ��� ############################
	BOOL DEVAPI FAR PASCAL	DAM3000M_GetDeviceInfo(						// ��ȡģ����Ϣ(���͡���ַ�������ʡ�У��)
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PDAM3000M_DEVICE_INFO pInfo);		// �豸��Ϣ

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetDeviceInfo(						// �޸�ģ����Ϣ(��ַ�������ʡ�У��)
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									DAM3000M_DEVICE_INFO& Info);		// �豸��Ϣ
	
	BOOL DEVAPI	FAR	PASCAL  DAM3000M_GetDevLastError(					// ������һ������
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID);					// ģ���ַ
		
	//####################### AD���ݶ�ȡ���� ###################################	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_ReadDeviceAD(						// ��ȡADģ�������� 
									HANDLE	hDevice,					// �豸������	
									LONG	lDeviceID,					// ģ���ַ
									LONG	lpADBuffer[],				// ����AD���ݵ��û������� ע��:lpADBuffer��ô��ڵ���lLastChannel - lFirstChannel +1
									LONG    lBufferSize,        		// ����lpADBuffer[]�Ĵ�С
									LONG	lFirstChannel = 0,			// ��ͨ��
									LONG	lLastChannel = 0);			// ĩͨ��
		
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetModeAD(							// ���ģ��������ģʽ 
									HANDLE	hDevice,					// �豸������	
									LONG	lDeviceID,					// ģ���ַ
									PLONG	lpMode,						// ADģʽ
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetModeAD(							// ����AD����ģʽ 
									HANDLE	hDevice,					// �豸������	
									LONG	lDeviceID,					// ģ���ַ
									LONG	lMode,						// ADģʽ 
									LONG	lChannel = 0);				// ͨ����
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetGroundingAD(					// ���ͨ���ӵ�ģʽ(ֻ�Կ�������õ�/˫������ģ����Ч)
									HANDLE	hDevice,					// �豸������	
									LONG	lDeviceID,					// ģ���ַ
									PLONG	lpGrounding,				// ADͨ���ӵ�ģʽ
									LONG	lChannel = 0);				// ͨ����
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetGroundingAD(					// ����ͨ���ӵ�ģʽ(ֻ�Կ�������õ�/˫������ģ����Ч)
									HANDLE	hDevice,					// �豸������	
									LONG	lDeviceID,					// ģ���ַ
									LONG	lGrounding,					// ADͨ���ӵ�ģʽ
									LONG	lChannel = 0);				// ͨ����
	
	BOOL DEVAPI FAR PASCAL	DAM3000M_GetLowLimitVal(					// ���ģ�������뱨������ֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// ģ���ַ
									LONG	lLowLimit[],				// ���ޱ���ֵ
									LONG    lFirstChannel = 0,			// ��ͨ����
									LONG	lLastChannel = 0);			// ĩͨ����

	BOOL DEVAPI FAR PASCAL DAM3000M_GetHighLimitVal(					// ���ģ�������뱨������ֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// ģ���ַ
									LONG	lHighLimit[],				// ���ޱ���ֵ
									LONG    lFirstChannel = 0,			// ��ͨ����
									LONG	lLastChannel = 0);			// ĩͨ����

	BOOL DEVAPI FAR PASCAL DAM3000M_SetLowLimitVal(						// �������ޱ���ֵ
									HANDLE	hDevice,
									LONG	lDeviceID,					// ģ���ַ
									LONG	lLowVal[],					// ���ޱ���ֵ
									LONG    lFirstChannel = 0,			// ��ͨ����
									LONG	lLastChannel = 0);			// ĩͨ����
		
	BOOL DEVAPI FAR PASCAL DAM3000M_SetHighLimitVal(					// �������ޱ���ֵ
									HANDLE	hDevice,
									LONG	lDeviceID,					// ģ���ַ
									LONG	lHighVal[],					// ���ޱ���ֵ
									LONG    lFirstChannel = 0,			// ��ͨ����
									LONG	lLastChannel = 0);			// ĩͨ����
		
	BOOL DEVAPI FAR PASCAL  DAM3000M_GetAlarmPulse(						// ��ñ����ĵ�ƽ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// ģ���ַ
									PLONG	lpAlarmVal,					// ������ƽ,0:�͵�ƽ, 1:�ߵ�ƽ
									LONG	lChannel = 0);				// ͨ����
	
	BOOL DEVAPI FAR PASCAL  DAM3000M_SetAlarmPulse(						// ����ģ�������뱨����ƽ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// ģ���ַ
									LONG	lAlarmVal,					// ������ƽ,0:�͵�ƽ, 1:�ߵ�ƽ	
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetAlarmSts(						// ��ñ���״̬
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// ģ���ַ
									PLONG	lpAlarmSts,					// ����״̬
									LONG	lChannel = 0);				// ��ȡͨ��

	//####################### DA���ݶ�ȡ���� ###################################
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetDeviceDAVal(					// �ض�DA���ֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PLONG	lpDAValue,          		// DA�����ǰֵ
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_WriteDeviceDA(						// �趨��ͨ��DA
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lDAData,					// DA���ֵ
									LONG	lChannel = 0);      		// ͨ����
	
	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetOutPutRangeDA(					// ��ȡģ�����������
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PLONG	lpRange,					// �������
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetOutPutRangeDA(					// ����ģ�����������
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lRange,						// �������
									LONG	lChannel = 0);				// ͨ����
	
	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetPowerOnValueDA(					// ���DA�ϵ�ֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PLONG	lpPowerOnVal,       		// �ϵ�ֵ
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetPowerOnValueDA(					// ����DA�ϵ�ֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lPowerOnVal,				// �ϵ�ֵ
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetSafeValueDA(					// ���DA��ȫֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PLONG	lpSafeVal,					// ��ȫֵ
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetSafeValueDA(					// ����DA��ȫֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lSafeVal,					// ��ȫֵ
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetSlopeDA(						// ��ģ�������б��
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PLONG	lpSlopeType,				// ���б������
									LONG	lChannel = 0);				// ͨ����
	
	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetSlopeDA(						// �޸�ģ�������б��
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lSlopeType,					// ���б������
									LONG	lChannel = 0);				// ͨ����

	//####################### DI��������������� ##############################
	BOOL DEVAPI FAR PASCAL	DAM3000M_GetModeDI(							// ��ȡ����������Ĺ���ģʽ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lMode[],					// ����Ĺ���ģʽ,0:DIģʽ,1:������ʽ,2:���淽ʽ
									LONG	lEdgeMode[],				// ���ط�ʽ,0:�͵�ƽ,1:�ߵ�ƽ
									LONG	lFirstChannel = 0,			// ��ͨ��
									LONG	lLastChannel = 0);			// ĩͨ��

	BOOL DEVAPI FAR PASCAL	DAM3000M_SetModeDI(							// ��������������Ĺ���ģʽ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lMode,						// ����Ĺ���ģʽ,0:DIģʽ,1:������ʽ,2:���淽ʽ
									LONG	lEdgeMode,					// ���ط�ʽ,0:�͵�ƽ,1:�ߵ�ƽ
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetDeviceDI(						// ��ȡ���������� 
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PDAM3000M_PARA_DI pDIPara);			// DIֵ
		
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_StartDeviceDI(						// ����DI����
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lChannel = 0);				// ͨ����
		
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_StopDeviceDI(						// ֹͣDI����
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lChannel = 0);				// ͨ����
		
	BOOL DEVAPI FAR PASCAL  DAM3000M_GetCNTDI(							// ��ȡDI������ֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PLONG	lpCounterValue,     		// ��Χ(0~65535)
									LONG	lFirstChannel = 0,			// ��ͨ��
									LONG	lLastChannel = 0);			// ĩͨ��

	BOOL DEVAPI	FAR PASCAL  DAM3000M_SetCNTDI(							// ����DI��������ʼֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lInitValue,					// ������ֵ
									LONG	lChannel = 0);				// ͨ����
	 
	BOOL DEVAPI FAR PASCAL  DAM3000M_StartLatch(						// ��������
									HANDLE	 hDevice,					// �豸���
									LONG	 lDeviceID,					// �豸��ַ
									LONG	 lChannel = 0);				// ͨ����
	
	BOOL DEVAPI FAR PASCAL  DAM3000M_StopLatch(							// ֹͣ����
									HANDLE	 hDevice,					// �豸���
									LONG	 lDeviceID,					// �豸��ַ
									LONG	 lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR PASCAL  DAM3000M_GetLatchStatus(					// ������״̬
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lLatchType,					// �������ͣ���Ϊ�����غ��½�������
									PDAM3000M_PARA_LATCH pLatchStatus);	// ����״̬

	BOOL DEVAPI	FAR PASCAL  DAM3000M_ClearCNTVal(						// �������ֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR PASCAL  DAM3000M_ClearLatchStatus(					// �������״̬
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lChannel = 0);				// ͨ����
			
	//####################### DO������������� ###################################	
	BOOL DEVAPI	FAR PASCAL	DAM3000M_GetDeviceDO(						// �ض����������
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PDAM3000M_PARA_DO pDOPara);			// ��ǰDO���ֵ

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetDeviceDO(						// ����DO���������ֵ 
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									BYTE	byDOSts[],					// ����DO���ֵ
									LONG    lFirstChannel,				// ��ͨ����
									LONG	lLastChannel);				// ĩͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetPowerOnValueDO(					// ��ȡDO�ϵ��ʼֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PDAM3000M_PARA_DO pPowerOnPara);	// �ϵ�ֵ 

	BOOL DEVAPI	FAR PASCAL	DAM3000M_SetPowerOnValueDO(					// ����DO�ϵ��ʼֵ 
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									DAM3000M_PARA_DO& PowerOnPara);		// �ϵ�ֵ

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetSafeValueDO(					// ��DO��ȫֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PDAM3000M_PARA_DO pDOSafePara);		// ��ȫֵ

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetSafeValueDO(					// ���ð�ȫֵ
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									DAM3000M_PARA_DO& DOSafePara);		// ��ȫֵ

	//#############################  ������  ###################################
	BOOL DEVAPI FAR PASCAL DAM3000M_SetCounterMode(						// �Ը������������в�������
									HANDLE   hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									PDAM3000M_PARA_CNT pCNTPara,		// ���ڸ�ͨ���ļ���������
									LONG	 lChannel = 0);				// ͨ����

	BOOL DEVAPI FAR PASCAL DAM3000M_InitCounterAlarm(					// ��ʼ�������Ĺ���ģʽ
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									PDAM3000M_CNT_ALARM pCNTAlarm);		// ������������

	BOOL DEVAPI FAR PASCAL DAM3000M_SetCounterAlarmMode(				// ���ü�����������ʽ
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									LONG	 lAlarmMode);				// ������ʽ

	BOOL DEVAPI FAR PASCAL DAM3000M_GetCounterSts(						// ��ü������豸Ӳ������״̬
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									PDAM3000M_CNT_STATUS pStsCNT,		// ����ֵ
									LONG	 lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_StartCounter(						// ��������������
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									LONG	 lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_StopCounter(						// ֹͣ����������
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									LONG	 lChannel = 0);				// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetCounterCurVal(					// ȡ�ü�������ǰֵ
									HANDLE	 hDevice,					// �豸������
									LONG     lDeviceID,					// �豸��ַ
									PULONG   pulCNTVal,					// ����ֵ
									LONG     lChannel = 0);				// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetFreqCurVal(						// ȡ��Ƶ������ǰֵ
									HANDLE   hDevice,					// �豸������
									LONG     lDeviceID,					// �豸��ַ
									PULONG   pulFreqVal,				// Ƶ��ֵ
									LONG     lChannel = 0);				// ͨ����

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_ResetCounter(						// ��������λ
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									LONG	 lChannel = 0);				// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_InitCounterFilter(					// ��ʼ���˲�
									HANDLE   hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									PDAM3000M_PARA_FILTER pFilter,		// �˲�����
									LONG	 lChannel = 0);				// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_EnableFilter(						// ʹ���˲�״̬
									HANDLE   hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									BOOL	 bEnable,					// ʹ���˲�
									LONG	 lChannel = 0);				// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetCounterAlarmSts(				// ���DO������״̬
									HANDLE   hDevice,					// �豸������
									LONG     lDeviceID,					// �豸��ַ
									PLONG    plEnableAlarm,				// ����������״̬
									PLONG    pbDO,						// DO
									LONG	 lChannel = 0);				// ͨ����
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetCounterDO(						// ����DO
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									BYTE	byDOSts[],					// DO
									LONG    lFirstChannel,				// ��ͨ����
									LONG	lLastChannel);				// ĩͨ����
										
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_ClearAlarmSts(						// �屨����ʽ1�������
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID);					// �豸��ַ
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetLEDCounterCH(					// ȡ�ü�����LED��ʾͨ��
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									PLONG	plChannel);					// ͨ����
	
	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetLEDCounterCH(					// ���ü�����LED��ʾͨ��
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// �豸��ַ
									LONG	lChannel);					// ͨ����

	//#############################  ����ģ��  ###################################
	BOOL DEVAPI FAR PASCAL  DAM3000M_GetEnergyVal(						// ��õ���ֵ
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									LONG	 lValue[],					// ����ֵ
									LONG	 lAanlogType,				// ģ��������
									LONG	 lFirstChannel = 0,			// ��ͨ��
									LONG	 lLastChannel = 0);			// ĩͨ��

	BOOL DEVAPI FAR PASCAL  DAM3000M_ClrEnergyReg(						// �������Ĵ���
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									LONG	 lChannel = 0);				// ͨ��

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetEnergyPerLSB(					// ���������λ
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									PLONG	 lpEnergyPerLSB);			// ������λ

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetEnergyPerLSB(					// ����������λ
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									LONG	 lEnergyPerLSB);			// ������λ

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetInputRange(						// �����������
									HANDLE	 hDevice,					// �豸������
									LONG	 lDeviceID,					// �豸��ַ
									PLONG	 lpInputRangeV,				// ��ѹ��������
									PLONG	 lpInputRangeI);			// ������������

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetInputRange(						// ������������
									HANDLE	hDevice,					// �豸�����
									LONG	lDeviceID,					// �豸��ַ
									LONG	lInputRangeV,				// ��ѹ��������
									LONG	lInputRangeI);				// ������������

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetEvrmTemp(						// ��û����¶�
									HANDLE	hDevice,					// �豸�����
									LONG	lDeviceID,					// �豸��ַ
									PLONG	lpEvrmTemp,					// �¶�
									LONG	lChannel = 0);				// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetEvrmHum(						// ��û���ʪ��
									HANDLE	hDevice,					// �豸�����
									LONG	lDeviceID,					// �豸��ַ
									PLONG	lpEvrmHum,					// ʪ��		
									LONG	lChannel = 0);				// ͨ����


	//##################################### ���Ź� ################################
	BOOL DEVAPI FAR PASCAL  DAM3000M_HostIsOK(HANDLE hDevice);			// ��λ���޷�����Ϣ

	BOOL DEVAPI FAR PASCAL  DAM3000M_EnableWatchdog(					// �������Ź�(�����ó�ʱ�������ʹ�ܿ��Ź�)
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID);					// ģ���ַ

	BOOL DEVAPI FAR PASCAL  DAM3000M_CloseWatchdog(						// ��ֹ���Ź�����
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID);					// ģ���ַ

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetWatchdogStatus(					// ��ȡ�����Ź���״̬(S = 1, Host is down; S = 0 OK)
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// ģ���ַ
									PLONG	lpWatchdogStatus);			// ���Ź�״̬

	BOOL DEVAPI FAR PASCAL  DAM3000M_ResetWatchdogStatus(				// Func: ��λ�����Ź���״̬(S = 0)
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID);					// ģ���ַ

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetWatchdogTimeoutVal(				// ȡ�ÿ��Ź����õ�ʱ����
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// ģ���ַ
									PLONG	lpInterval);				// ʱ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetWatchdogTimeoutVal(				// ���ÿ��Ź����õ�ʱ����
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// ģ���ַ
									LONG	lInterval);					// ʱ����

	// ##################################### DIGIT LED ���ú��� #################################
	BOOL DEVAPI FAR PASCAL  DAM3000M_GetDLedMode(						// �����ʾģʽ����
									HANDLE hDevice,						// �豸������
									LONG   lDeviceID,					// ģ���ַ
									PLONG  lpDispMode);					// ��ʾģʽ 0x00���¶ȸ�ʽ,0x01:ŷķֵ

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetDLedMode(						// �޸���ʾģʽ����
									HANDLE hDevice,						// �豸������
									LONG   lDeviceID,					// ģ���ַ
									LONG   lDispMode);					// ��ʾģʽ 0x01���¶ȸ�ʽ,0x02:ŷķֵ

	BOOL DEVAPI FAR PASCAL  DAM3000M_GetDLedDispChannel(				// ���LED��ʾͨ����
									HANDLE hDevice,						// �豸������
									LONG   lDeviceID,					// ģ���ַ
									PLONG  lpChannel);					// ͨ����,lpChannel = 0xff:����������ʾ

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetDLedDispChannel(				// ����LED��ʾͨ����
									HANDLE hDevice,						// �豸������
									LONG   lDeviceID,					// ģ���ַ
									LONG   lChannel = 0);				// ͨ����,lpChannel = 0xff:����������ʾ
		
	BOOL DEVAPI FAR PASCAL  DAM3000M_SetDLedValue(						// ����������ʾֵ
									HANDLE hDevice,						// �豸������
									LONG   lDeviceID,					// ģ���ַ
									LPCTSTR strWriteBuf,				// ��ʾ���ַ���
									LONG   llength);					// ���ݳ���


//####################### �����������������ַ� ###########################
	int DEVAPI	FAR	PASCAL	DAM3000M_WriteDeviceChar(					// ֱ��д�豸
									HANDLE	hDevice,					// �豸������
									char*	strWriteBuf,				// д������
									long	llength,					// ���ݳ���
									long	timeout = 100);				// ��ʱ��Χ(mS)
	
	int	DEVAPI	FAR	PASCAL	DAM3000M_ReadDeviceChar(					// ֱ�Ӷ��豸
									HANDLE	hDevice,					// �豸������
									char*	strReadBuf,					// ��ȡ������
									long	llength,					// ���ݳ���
									long	timeout = 100);				// ��ʱ��Χ(mS)
											   

//###########################��ģ����Ϣȷ�Ϻ���  #################################
	BOOL DEVAPI	FAR	PASCAL	InitCheckInfo(
									HANDLE	hDevice, 
									LONG	lDeviceID, 
									DWORD	dwNum);

	BOOL DEVAPI	FAR	PASCAL	ReadCheckInfo(
									HANDLE	hDevice, 
									LONG	lDeviceID, 
									LONG	lIndex, 
									BYTE&	byEncrypt);

//####################################### �������� ####################################
	BOOL DEVAPI FAR PASCAL  DAM3000M_AdjustSlopeVal(					// ΢����ǰ����б��
									HANDLE  hDevice,					// �豸������
									LONG    lDeviceID,   				// ģ���ַ
									LONG    lAdjustVal,					// ΢��ֵ
									LONG    lChannel);					// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_StoreSlopeVal(						// ���õ�ǰֵΪ�������б��
									HANDLE  hDevice,					// �豸������
									LONG    lDeviceID,   				// ģ���ַ
									LONG    lChannel);					// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetFaultSlopeVal(					// �趨����б��ΪĬ��ֵ
									HANDLE  hDevice,					// �豸������
									LONG    lDeviceID,   				// ģ���ַ
									LONG    lChannel);					// ͨ����	0xFFΪ����ͨ���ָ�

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetZeroRepair(						// �������ƫ�Ʋ���
									HANDLE	hDevice,					// �豸������
									LONG	lDeviceID,					// ģ���ַ
									LONG	lZeroRepair,				// ���ֵ
									LONG	lChannel);					// ͨ����

	BOOL DEVAPI FAR PASCAL  DAM3000M_SetDevTestMode(					// ����ģ��������ģʽ
									HANDLE  hDevice,					// �豸������
									LONG    lDeviceID);					// ģ���ַ

	BOOL DEVAPI FAR PASCAL  DAM3000M_ResetModule(						// ģ����λ
									HANDLE  hDevice,					// �豸������
									LONG    lDeviceID); 				// ģ���ַ

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_GetEnvironmentTemp(				// ȡ�û����¶�(Ϊȡ�ȵ�żֵ��׼��)
									HANDLE	hDevice,
									LONG	lDeviceID,
									PFLOAT	lpETemprt);

	BOOL DEVAPI	FAR	PASCAL	DAM3000M_SetAdjustTemp(				// ȡ�û����¶�(Ϊȡ�ȵ�żֵ��׼��)
									HANDLE	hDevice,
									LONG	lDeviceID,
									BYTE	lETemprt);

	BOOL DEVAPI	FAR	PASCAL  DAM3000M_GetDevLastError(					// ����Ӧ������
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
