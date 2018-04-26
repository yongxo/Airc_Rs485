#include "DAM3000M.h"

#define DAM3046_AI 6

#define GET_DATA 0x42
#define GET_STATE 0x43
#define GET_WARN 0x44
#define SET_ON 0x45  //开机
#define SET_OFF 0x46		//关机
#define GET_PARA 0x47
#define SET_PARA 0x49    //参数
#define GET_RUNTIME 0x80
#define GET_TIMES 0x81
#define GET_VERSION   0x4F
#define GET_ADDR 0x50

//空调设置参数命令表
#define AIRC_SET_CMD_CP	 (0x80) //制冷设定点
#define AIRC_SET_CMD_SENS (0x81)// 灵敏度
#define AIRC_SET_CMD_HP  (0x82)//加热设定点
#define AIRC_SET_CMD_HS  (0x83)  //加热灵敏度
#define AIRC_SET_CMD_ITHP (0x84) //内高温报警点
#define AIRC_SET_CMD_ITLP (0x85) //内低温报警点
#define AIRC_SET_CMD_OTHP (0x86)//外高温报警点
#define AIRC_SET_CMD_OTLP (0x87)//外低温报警点
#define AIRC_SET_CMD_IWP (0x88)//内风机停止点
#define AIRC_SET_CMD_ESTP (0x89)//紧急停止点
#define AIRC_SET_CMD_OWP (0x8a)//外风机温度设定点
#define AIRC_SET_CMD_OWUP (0x8b)//外风机上限
#define AIRC_SET_CMD_OWLP  (0x8c)//外风机下限
#define AIRC_SET_CMD_OWSP1 (0x8d)//外风机调速点
#define AIRC_SET_CMD_OWSP1UP (0x8e)//外风机调速上偏差
#define AIRC_SET_CMD_OWSP1DOWN (0x8f)//外风机调速下偏差
#define AIRC_SET_CMD_OWSP1H (0x90)//外风机调速上限
#define AIRC_SET_CMD_OWSP1L (0x91) //外风机调速下限
#define AIRC_SET_CMD_OWSP2 (0x92)//外风机调速点
#define AIRC_SET_CMD_OWSP2UP (0x93)//外风机调速上偏差
#define AIRC_SET_CMD_OWSP2DOWN (0x94)//外风机调速下偏差
#define AIRC_SET_CMD_OWSP2H (0x95) //外风机调速上限
#define AIRC_SET_CMD_OWSP2L (0x96) //外风机调速下限


struct  Airc_CONYC
{
	UINT  in_temp;			//机柜温度
	UINT  out_temp;			//机柜外温度
	UINT  cab_hnmidity;		//机柜内湿度
	UINT  Ele_flow;		//负载电流
	UINT  ele_voltage;		//交流电压
	UINT  DC_voltage;		//直流电压
};

struct Airc_Param   //用于描述空调工作参数
{
	BYTE	cold_point; //制冷设定点
	BYTE	sensitivity; // 灵敏度
	BYTE	heat_point; //加热设定点
	BYTE	heat_sen;  //加热灵敏度
	BYTE	inhigh_temp;//内高温报警点
	BYTE	inlow_temp;//内低温报警点
	BYTE	outhigh_temp;//外高温报警点
	BYTE	outlow_temp;//外低温报警点
	BYTE	in_windstop;//内风机停止点
	BYTE	emerg_stop;//紧急停止点
	BYTE	out_temp;//外风机温度设定点
	BYTE	out_uplimit;//外风机上限
	BYTE	out_downlimit;//外风机下限
	BYTE	out_wind1Turn;//外风机1调速点
	BYTE	out_wind1UP;//外风机1调速上偏差
	BYTE	out_wind1down;//外风机1调速下偏差
	BYTE	out_W1uplimit; //外风机1调速上限
	BYTE	out_W1downlimit; //外风机1调速下限
	BYTE	out_wind2Turn;//外风机2调速点
	BYTE	out_wind2UP;//外风机2调速上偏差
	BYTE	out_wind2down;//外风机2调速下偏差
	BYTE	out_W2uplimit; //外风机2调速上限
	BYTE	out_W2downlimit; //外风机2调速下限
	BYTE	user;//用户自定义
};

struct Airc_Status   //用于描述空调各部分工作状态
{
	BYTE  dev_state;//设备状态
	BYTE  in_wind;//内风机
	BYTE	press_mch;//压缩机
	BYTE	ele_hot; //电加热
	BYTE	out_wind;//外风机
	BYTE	out_wind1;//外部风机1
	BYTE	out_wind2;//外部风机2
	BYTE   user_define;//有户定义

};
struct Airc_Warn  //用于描述空调各部分报警信息
{
	BYTE  in_temp; //机柜内高温告警
	BYTE	low_temp;// 机柜内低温告警
	BYTE	basehigh_temp; //基站内高温告警
	BYTE 	baselow_temp; //基站内低温告警
	BYTE 	cab_temp;  //机柜内温度传感器失效
	BYTE	base_temp; //基站内温度传感器失效
	BYTE	cab_hnmid;  //机柜内温度传感器失效
	BYTE	press_mach;//压缩机告警
	BYTE	door_ban; //门禁告警
	BYTE	shake; //振动
	BYTE	press_mach_bug; //压缩机故障
	BYTE	heat_bug;//加电器故障
	BYTE	water_flood;//水淹告警
	BYTE	smoke; //烟雾告警
	BYTE	thunder_eer; //防雷失效告警
	BYTE	prevent_cold;//盘管防冻保护告警
	BYTE	AC_pass_voltage;  //交流过压告警
	BYTE	AC_lock_voltage;	//交流欠压告警
	BYTE	stop_ele; //市停电告警
	BYTE	in_wind_bug;//内风机故障
	BYTE	in_wind1_bug;//内风机1故障
	BYTE 	in_wind2_bug;//内风机2故障
	BYTE	outair_temp; //排气温度过高
	BYTE	scroll_temp_effic;   //盘管温度传感器失效
	BYTE	outair_temp_effic; //排气温度传感器失效
	BYTE	EEPROM;  // EEPROM故障告警
	BYTE	usr;//保留
};

struct devrun_times  //设备运行次数
{
	long 	dev_state; //设备状态
	long	in_wind; //内风机
	long	press_mach;//压缩机
	long	heat_hot; //电加热
	long	out_wind;//外风机	
	long	out_wind1;//外风机
	long	out_wind2;//外风机
	long	user_times;
};

#define AIRC_INVALID_CMD (0x0)
#define AIRC_SET_CMD_CNT (23)
struct AIRC_SET_CMD
{
	char cmd_id;
	char cmd_data;
};

extern struct AIRC_SET_CMD airc_set_cmd_list[AIRC_SET_CMD_CNT];

class _declspec(dllexport) Air_Con_Ctrl
{
public:
	void SetAirc_param(Airc_Param param);
	/**********************************************************************************
	Function 		: GetAirc_conYC
	Description		: 获取空调遥测信息
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetAirc_conYC(Airc_CONYC*);
/**********************************************************************************
	Function 		: GetAirc_param
	Description		: 获取空调参数
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetAirc_param(Airc_Param*);
/**********************************************************************************
	Function 		: GetAirc_status
	Description		: 获取空调运行状态
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetAirc_status(Airc_Status*);
/**********************************************************************************
	Function 		: GetAirc_warn
	Description		: 获取空调报警信息
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetAirc_warn(Airc_Warn*);
/**********************************************************************************
	Function 		: GetDevrun_times
	Description		: 获取空调运行时间信息
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetDevrun_times(devrun_times*);
	/**********************************************************************************
	Function 		: GetDevrun_time
	Description		: 获取空调运行次数信息
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetDevrun_time(devrun_times*);
	/**********************************************************************************
	Function 		: Start_Airc
	Description		: 启动空调事务处理模块
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	bool Start_Airc(unsigned int comrate);
	/**********************************************************************************
	Function 		: Stop_Airc
	Description		: 停止空调事务处理模块
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	bool Stop_Airc();

	static  DWORD WINAPI TemThread(LPVOID lparam);

	void CloseThread();             //关闭线程
public:
	TCHAR   m_szWarn[7][32];

	int     m_nWarnNum;
	//线程退出事件
	HANDLE m_hCloseEvent;
	//线程句柄
	HANDLE m_hThread;
	//线程ID标识
	DWORD m_dwThreadID;


public:
	HANDLE m_hDevice;
	BOOL Tem_start(LONG nPortNum,LONG lBaud);
	BOOL ReadDeviceAD( // 读取AD模拟量输入 
		LONG	lDeviceID,			// 模块地址
		LONG	lpADBuffer[],		// 接收AD数据的用户缓冲区 注意:lpADBuffer最好大于等于lLastChannel - lFirstChannel +1
		LONG    lBufferSize,        // 数组lpADBuffer[]的大小
		LONG	lFirstChannel,	// 首通道
		LONG	lLastChannel); // 末通道
	void Tem_stop();
	LONG m_ADBuffer[DAM3046_AI];
	LONG m_lADMode;
	LONG m_lDeviceID;		// 设备地址
	DAM3000M_DEVICE_INFO m_Info;
	BOOL temflag;
};