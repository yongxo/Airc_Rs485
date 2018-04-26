#include "StdAfx.h"
#include "Airc_Thread.h"
#include "Airc_dev.h"

//设置空调参数宏定义
#define AIRC_PORT _T("COM1:")
#define AIRC_BAUT 9600
#define AIRC_PARITY NOPARITY
#define AIRC_DBIT 8
#define AIRC_SBIT ONESTOPBIT
#define AIRC_GET_CMD_CNT 8



Airc_protocol m_aircprotocol;
Airc_dev m_aircdev;
void CALLBACK TimerProc(HWND hWnd,UINT nMsg,UINT nTimerid,DWORD dwTime); 
struct AIRC_SET_CMD airc_set_cmd_list[AIRC_SET_CMD_CNT];
static unsigned char airc_get_cmd_list[AIRC_GET_CMD_CNT];
DWORD WINAPI ThreadProc(LPVOID lpParameter);


Airc_Thread::Airc_Thread(void)
{

}

Airc_Thread::~Airc_Thread(void)
{

}

Airc_protocol::Airc_protocol(void)
{
	m_varsion = 0;
	m_success = 0;
	m_length = 0;
	//m_sendFrame =(BYTE*) malloc(100 * sizeof(BYTE));
	//m_recvFrame =(BYTE*) malloc(100 * sizeof(BYTE));
	//m_tempFrame =(BYTE*) malloc(100 * sizeof(BYTE));
	m_sendFrame = sendBuf;
	m_recvFrame = recvBuf;

	
}

Airc_protocol::~Airc_protocol(void)
{
    free(m_sendFrame);
	free(m_recvFrame);
	free(m_tempFrame);
}

bool Airc_protocol::FrameToBuffer()
{

	return true;
}

bool Airc_protocol::BufferToFrame()
{

	return true;
}

UINT Airc_protocol::Airc_ctrl(int Cmd, void *p)
{
	UINT flag = -1;
	switch (Cmd)
	{
	case 0x42:
		SetFrame(0x42, 0x01, 0x0, 0x0 ,0);
		flag = 0;
		break;
	case 0x43:
		SetFrame(0x43, 0x01, 0x0, 0x0 ,0);
		flag = 0;
		break;
	case 0x44:
		SetFrame(0x44, 0x01, 0x0, 0x0 ,0);
		flag = 0;
		break;
	case 0x45:
		SetFrame(0x45, 0x01, 0x01, 0x0, 0x02);  //遥控开机
		flag = 0;
		break;
	case 0x46:
		SetFrame(0x45, 0x02, 0x02, 0x0, 0x02);  //遥控关机
		flag = 0;
		break;	
	case 0x47:
		SetFrame(0x47, 0x01, 0x0, 0x0 ,0);  //遥控关机
		flag = 0;
		break;
	case 0x49:
		{BYTE *data =(BYTE*)p;
		if (0x80<= data[0] && data[0]<= 0x96)
		{
			SetParaData(data[0], data[1]);
		}
		flag = 0;
		break;}
	case 0x80:
		SetFrame(0x80, 0x01, 0x0, 0x0, 0x0);  
		flag = 0;
		break;
	case 0x81:
		SetFrame(0x81, 0x01, 0x0, 0x0, 0x0);  
		flag = 0;
		break;
	case 0x4F:
		SetFrame(0x4F, 0x01, 0x0, 0x0, 0x0);  
		flag = 0;
		break;
	case 0x50:
		SetFrame(0x50, 0x01, 0x0, 0x0, 0x0);  
		flag = 0;
		break;
	}
	if (flag)
	{
		return flag ;
	}
	else
	{
		return flag;
	}	
}

UINT Airc_protocol::SetParaData(UCHAR type, CHAR para)
{
	UINT flag = -1;
	CHAR SetPoint = -5; //制冷设定点
	CHAR codeOn = 40;   //制冷开启点
	CHAR Uplimti = 0;   //下限速
	CHAR DownLimti = 0; //上限速
	CHAR OutPoint = 0;//外机调速点
	switch (type)
	{
		case 0x80:
			if (18 <= para && para <= 40)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); //设定参数
				flag = 0;
			}		
			break;
		case 0x81:
			if (1 <= para && para <= 10)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); //设定参数
				flag = 0;
			}		
			break;
		case 0x82:
			if (  5 <= para && para <= SetPoint)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); 
				flag = 0;
			}
			break;
		case 0x83:
			if (1 <= para && para <= 0x10)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); //设定参数
				flag = 0;
			}
			break;
		case 0x84:
			if ((0x28 == para) || (codeOn <= para && para <= 0x70))
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);  
				flag = 0;
			}
			break;
		case 0x85:
			if (-15<= para && para <= 0x10)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);  
				flag = 0;
			}
			break;
		case 0x86:
			if (0x28 <= para && para <= 0x70)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); 
				flag = 0;
			}
			break;
		case 0x87:
			if (-15 <= para && para <= 0x10)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);  
				flag = 0;
			}
			break;
		case 0x88:
			if (0 <= para && para <= codeOn)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);  
				flag = 0;
			}
			break;
		case 0x89:
			if ((0x28 == para) || (codeOn<= para && para <= 0x70))
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);  
				flag = 0;
			}
			break;
		case 0x8A:
			if ((0x30 <= para) || para <= 0x60)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); 
				flag = 0;
			}
			break;
		case 0x8B:
			if ((DownLimti <= para) && para <= 0x100)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);
				flag = 0;
			}
			break;
		case 0x8C:
			if ((0x30 <= para) && para <= Uplimti)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);
				flag = 0;
			}
			break;
		case 0x8D:
			if ((0x18 <= para) && para <= codeOn)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); 
				flag = 0;
			}
			break;
		case 0x8E:
			if ((0x01 <= para) && para <= 0x10)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); 
				flag = 0;
			}
			break;
		case 0x8F:
			if (0x0 <= para && para <= 0x10)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); 
				flag = 0;
			}
			break;
		case 0x90:
			if (DownLimti <= para && para <= 0x100)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); 
				flag = 0;
			}
			break;
		case 0x91:
			if (0x30 <= para || para <= Uplimti)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);
				flag = 0;
			}
			break;
		case 0x92:
			if (OutPoint <= para || para <= 0x40)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); 
				flag = 0;
			}
			break;
		case 0x93:
			if (0x1 <= para || para <= 0x10)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);  
				flag = 0;
			}
			break;
		case 0x94:
			if (0x0 <= para || para <= 0x10)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);  
				flag = 0;
			}
			break;
		case 0x95:
			if (DownLimti <= para || para <= 0x100)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);  
				flag = 0;
			}
			break;
		case 0x96:
			if (0x30 <= para && para <= Uplimti)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04);  
				flag = 0;
			}
			break;
	}
	return flag;
}

void Airc_protocol::SetFrame(BYTE CID2, BYTE addr, BYTE type, UINT para, unsigned short lenid)     //设置帧内容
{	
	int i = 0;
//	m_varsion = 0;
	m_success = 0;
	m_length = 0;
	m_sendFrame[i++] = /*m_cmdata.SOI = */0x7e;
	m_cmdata.VER = 0x21;
	m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.VER));
	m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.VER));
	m_cmdata.ADR = addr;  //设备地址描述 1-254 
	m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.ADR));
	m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.ADR));
	m_cmdata.CID1 = 0x60;
	m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.CID1));
	m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.CID1));
	m_cmdata.CID2 = CID2;
	m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.CID2));
	m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.CID2));	
	if (lenid != 0)
	{
		m_cmdata.LENGTH = GetLength(lenid);
		m_sendFrame[i++] = AscToHex(GetHigh4((m_cmdata.LENGTH&0xFF00)>>8));
		m_sendFrame[i++] = AscToHex(GetLow4((m_cmdata.LENGTH& 0xFF00)>>8));
		m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.LENGTH& 0x00FF));
		m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.LENGTH& 0x00FF));
		m_cmdata.LENID = lenid;
		UINT temp =0;
		switch (type)  //类型1 开 2 关 3 参数设定
		{
		case 0x01:
			m_cmdata.INFO = 0x10;
			break;
		case 0x02:
			m_cmdata.INFO =0x1F;
			break;
		default:
			m_cmdata.INFO = type;
		}
		if ((type == 0x01) || (type == 0x02))
		{			
			m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.INFO));
			m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.INFO));
		}
		else
		{
			m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.INFO));
			m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.INFO));
			m_sendFrame[i++] = AscToHex(GetHigh4(para));
			m_sendFrame[i++] = AscToHex(GetLow4(para));
		}		
	}
	else
	{
		m_cmdata.LENID = 0;
		m_cmdata.LENGTH = 0;
		m_cmdata.INFO = 0;	
		m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.LENGTH));
		m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.LENGTH));
		
		m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.INFO));
		m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.INFO));
	}	
	m_cmdata.CHKSUM = VerifyCheck(m_sendFrame , i-1);
	m_sendFrame[i++] = AscToHex(GetHigh4((m_cmdata.CHKSUM & 0xFF00)>>8));//(m_cmdata.CHKSUM & 0x00FF)
	m_sendFrame[i++] = AscToHex(GetLow4((m_cmdata.CHKSUM & 0xFF00)>>8));
	m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.CHKSUM & 0x00FF));
	m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.CHKSUM & 0x00FF));
	m_cmdata.EOI = 0x0D;
	m_sendFrame[i++] = m_cmdata.EOI;
	m_length = i;
	m_flag = CID2;
}

BYTE Airc_protocol::GetHigh4(BYTE high)
{
	high= high & 0xF0;
	high = high>>4;
	return high;
}

BYTE Airc_protocol::GetLow4(BYTE low)
{
	low = low & 0x0F;
	return low;
}

//获取LENGTH字段值
unsigned short  Airc_protocol::GetLength(unsigned short lenid = 0)
{
	unsigned short len = lenid;
	unsigned short val = 0;
	unsigned short temp = 0;	
	for (int i = 0; i< 3; i++)
	{
		temp = len & 0x000F;
		val += temp;
		len = len>>4;		
	}
	val =val % 0x000F;
	val =((~val) & 0x000F) + 0x01;
	val = ( val << 12) | lenid;	
	return val;		
}

UINT Airc_protocol::GetFrameLen()
{
	return m_length;
}

BYTE Airc_protocol::divdeFrame(BYTE * buf, UINT length)  // 接收到数据处理
{
	//memset(m_recvFrame, 0, 100);
	UINT chktemp = VerifyCheck(buf, length - 6);//取长度做校验，长度需要减掉4个校验+结束字节。
	UINT conlen = convert(buf, length);
	UINT verify =( m_recvFrame[conlen-3]<<8)+ m_recvFrame[conlen-2];
	if (0x7e != m_recvFrame[0] || chktemp != verify)
	{
		return -1;
	}
	BYTE state = RTNstatus(m_recvFrame[4]);
	if (state != 0x0)
	{
		return m_RTNstatus;
	}	
	UINT lenth = (m_recvFrame[5]<<8) + m_recvFrame[6];
	UINT lenID = (lenth & 0x0FFF)/2;	
	switch (m_flag)
	{
	case 0x42:
#if 0//协议文本错误，调整---xuliang
		m_conyc.DC_voltage = m_recvFrame[7]*255 + m_recvFrame[8];			
		m_conyc.ele_voltage= m_recvFrame[9]*255 + m_recvFrame[10];		//交流电压
		m_conyc.Ele_flow = m_recvFrame[11]*255 + m_recvFrame[12];		//负载电流
		m_conyc.cab_hnmidity = m_recvFrame[13]*255 + m_recvFrame[14];	//机柜内湿度
		m_conyc.out_temp = m_recvFrame[15]*255 + m_recvFrame[16];		//机柜外温度
		m_conyc.in_temp = m_recvFrame[17]*255 + m_recvFrame[18];//机柜温度
#endif
#if 0 // 售后称除柜内温度外，其余几项均没有检测。
		m_conyc.DC_voltage = (m_recvFrame[7]*256 + m_recvFrame[8])/10;			
		m_conyc.ele_voltage= (m_recvFrame[9]*256 + m_recvFrame[10])/10;		//交流电压
		m_conyc.Ele_flow = (m_recvFrame[11]*256 + m_recvFrame[12])/10;		//负载电流
		m_conyc.cab_hnmidity = m_recvFrame[13]*256 + m_recvFrame[14];	//机柜内湿度
		m_conyc.out_temp = m_recvFrame[15]*256 + m_recvFrame[16];		//机柜外温度
		m_conyc.in_temp = m_recvFrame[17]*256 + m_recvFrame[18];//机柜温度
#endif
		m_conyc.in_temp = (m_recvFrame[7]*256 + m_recvFrame[8])/10;
		break;
	case 0x43:
		m_airc_status.dev_state = m_recvFrame[7];
		m_airc_status.dev_state = m_recvFrame[8];//设备状态
		m_airc_status.in_wind= m_recvFrame[9];//内风机
		m_airc_status.press_mch= m_recvFrame[10];//压缩机
		m_airc_status.ele_hot= m_recvFrame[11]; //电加热
		m_airc_status.out_wind= m_recvFrame[12];//外风机
		m_airc_status.out_wind1= m_recvFrame[13];//外部风机1
	//	m_airc_status.out_wind2= m_recvFrame[14];//外部风机2
	//	m_airc_status.user_define= m_recvFrame[15];//有户定义
		break;
	case 0x44:
		m_airc_warn.in_temp = m_recvFrame[7]; //机柜内高温告警
		m_airc_warn.low_temp= m_recvFrame[8];// 机柜内低温告警
		m_airc_warn.basehigh_temp = m_recvFrame[9]; //基站内高温告警
		m_airc_warn.baselow_temp= m_recvFrame[10]; //基站内低温告警
		m_airc_warn.cab_temp= m_recvFrame[11];  //机柜内温度传感器失效
		m_airc_warn.base_temp = m_recvFrame[12]; //基站内温度传感器失效
		m_airc_warn.cab_hnmid = m_recvFrame[13];  //机柜内温度传感器失效
		m_airc_warn.press_mach = m_recvFrame[14];//压缩机告警
		m_airc_warn.door_ban = m_recvFrame[15]; //门禁告警
		m_airc_warn.shake = m_recvFrame[16]; //振动
		m_airc_warn.press_mach_bug = m_recvFrame[17]; //压缩机故障
		m_airc_warn.heat_bug = m_recvFrame[18];//加电器故障
		m_airc_warn.water_flood = m_recvFrame[19];//水淹告警
		m_airc_warn.smoke = m_recvFrame[20]; //烟雾告警
		m_airc_warn.thunder_eer = m_recvFrame[21]; //防雷失效告警
		m_airc_warn.prevent_cold = m_recvFrame[22];//盘管防冻保护告警
		m_airc_warn.AC_pass_voltage = m_recvFrame[23];  //交流过压告警
		m_airc_warn.AC_lock_voltage = m_recvFrame[24];	//交流欠压告警
		m_airc_warn.stop_ele = m_recvFrame[25]; //市停电告警
		m_airc_warn.in_wind_bug = m_recvFrame[26];//内风机故障
		m_airc_warn.in_wind1_bug = m_recvFrame[27];//内风机1故障
		m_airc_warn.in_wind2_bug = m_recvFrame[28];//内风机2故障
		m_airc_warn.outair_temp = m_recvFrame[29]; //排气温度过高
		m_airc_warn.scroll_temp_effic = m_recvFrame[30];   //盘管温度传感器失效
		m_airc_warn.outair_temp_effic = m_recvFrame[31]; //排气温度传感器失效
		m_airc_warn.EEPROM = m_recvFrame[32];  // EEPROM故障告警
		break;
	case 0x45:
			m_success = 1;  //设置命令成功！
		break;
	case 0x47:
		m_airc_param.cold_point = m_recvFrame[7]; //制冷设定点
		m_airc_param.sensitivity = m_recvFrame[8]; // 灵敏度
		m_airc_param.heat_point = m_recvFrame[9]; //加热设定点
		m_airc_param.heat_sen = m_recvFrame[10];  //加热灵敏度
		m_airc_param.inhigh_temp = m_recvFrame[11];//内高温报警点
		m_airc_param.inlow_temp = m_recvFrame[12];//内低温报警点
		m_airc_param.outhigh_temp = m_recvFrame[13];//外高温报警点
		m_airc_param.outlow_temp = m_recvFrame[14];//外低温报警点
		m_airc_param.in_windstop = m_recvFrame[15];//内风机停止点
		m_airc_param.emerg_stop = m_recvFrame[16];//紧急停止点
		m_airc_param.out_temp = m_recvFrame[17];//外风机温度设定点
		m_airc_param.out_uplimit = m_recvFrame[18];//外风机上限
		m_airc_param.out_downlimit = m_recvFrame[19];//外风机下限
		m_airc_param.out_wind1Turn = m_recvFrame[20];//外风机1调速点
		m_airc_param.out_wind1UP = m_recvFrame[21];//外风机1调速上偏差
		m_airc_param.out_wind1down = m_recvFrame[22];//外风机1调速下偏差
		m_airc_param.out_W1uplimit = m_recvFrame[23]; //外风机1调速上限
		m_airc_param.out_W1downlimit = m_recvFrame[24]; //外风机1调速下限
		m_airc_param.out_wind2Turn = m_recvFrame[25];//外风机2调速点
		m_airc_param.out_wind2UP = m_recvFrame[26];//外风机2调速上偏差
		m_airc_param.out_wind2down = m_recvFrame[27];//外风机2调速下偏差
		m_airc_param.out_W2uplimit = m_recvFrame[28]; //外风机2调速上限
		m_airc_param.out_W2downlimit = m_recvFrame[29]; //外风机2调速下限
		//m_airc_param.user;//用户自定义
		break;
	case 0x49:
		m_success = 1;
		break;
	case 0x80:
		m_dev_time.dev_state = (m_recvFrame[7]<<24) + (m_recvFrame[8]<<16) + (m_recvFrame[9]<<8) + m_recvFrame[10]; //设备状态
		m_dev_time.in_wind = (m_recvFrame[11]<<24) + (m_recvFrame[12]<<16) + (m_recvFrame[13]<<8) + m_recvFrame[14]; //内风机
		m_dev_time.press_mach = (m_recvFrame[15]<<24) + (m_recvFrame[16]<<16) + (m_recvFrame[17]<<8) + m_recvFrame[18];//压缩机
		m_dev_time.heat_hot= (m_recvFrame[19]<<24) + (m_recvFrame[20]<<16) + (m_recvFrame[21]<<8) + m_recvFrame[22]; //电加热
		m_dev_time.out_wind = (m_recvFrame[23]<<24) + (m_recvFrame[24]<<16) + (m_recvFrame[25]<<8) + m_recvFrame[26];//外风机	
		m_dev_time.out_wind1 = (m_recvFrame[27]<<24) + (m_recvFrame[28]<<16) + (m_recvFrame[29]<<8) + m_recvFrame[30];//外风机
		m_dev_time.out_wind2 = (m_recvFrame[31]<<24) + (m_recvFrame[32]<<16) + (m_recvFrame[33]<<8) + m_recvFrame[34];//外风机
		//m_dev_time.user_times = m_recvFrame[7]<<24 + m_recvFrame[8]<<16 + m_recvFrame[9]<<8 + m_recvFrame[10];
		break;
	case 0x81:
		m_dev_times.dev_state = (m_recvFrame[7]<<24) + (m_recvFrame[8]<<16) + (m_recvFrame[9]<<8) + m_recvFrame[10]; //设备状态
		m_dev_times.in_wind = (m_recvFrame[11]<<24) + (m_recvFrame[12]<<16) + (m_recvFrame[13]<<8) + m_recvFrame[14]; //内风机
		m_dev_times.press_mach = (m_recvFrame[15]<<24) + (m_recvFrame[16]<<16) + (m_recvFrame[17]<<8) + m_recvFrame[18];//压缩机
		m_dev_times.heat_hot= (m_recvFrame[19]<<24) + (m_recvFrame[20]<<16) + (m_recvFrame[21]<<8) + m_recvFrame[22]; //电加热
		m_dev_times.out_wind = (m_recvFrame[23]<<24) + (m_recvFrame[24]<<16) + (m_recvFrame[25]<<8) + m_recvFrame[26];//外风机	
		m_dev_times.out_wind1 = (m_recvFrame[27]<<24) + (m_recvFrame[28]<<16) + (m_recvFrame[29]<<8) + m_recvFrame[30];//外风机
		m_dev_times.out_wind2 = (m_recvFrame[31]<<24) + (m_recvFrame[32]<<16) + (m_recvFrame[33]<<8) + m_recvFrame[34];//外风机
		break;
	case 0x4F:
		m_varsion = m_recvFrame[1];
		break;
	case 0x50:
		m_address = m_recvFrame[2];
		break;
	}
	return 0;
}

UINT Airc_protocol::convert(BYTE *sbuf, UINT length)
{
	UINT i, j = 0;
	m_recvFrame[j++] = sbuf[0];
	for (  i = 1; i < length; i+=2 )
	{
		m_recvFrame[j++] = SetHigh4(HexToAsc(sbuf[i])) + SetLow4(HexToAsc(sbuf[i+1]));
	}
	return j; 
}

BYTE Airc_protocol::SetHigh4(BYTE high)
{
	high = high << 4;
	return high;		
}

BYTE Airc_protocol::SetLow4(BYTE low)
{
      low = low & 0x0F; 
	  return low;
}
Airc_DataPack Airc_protocol::GetDataPack()
{
	return m_cmdata;
}

BYTE* Airc_protocol::GetSendFrame()
{
	return m_sendFrame;
}

unsigned short Airc_protocol::VerifyCheck(BYTE *buf, UINT length)   //检查验证码
{
	UINT check = 0;
	for (UINT i = 1; i <= length; i++)
	{
		check += buf[i];
	}	
	check = check % 65536;
	check= (~check) + 1;
	return check;
}

BYTE Airc_protocol::RTNstatus(byte rtn)      //判断PTN是否正确
{	
	switch (rtn)
	{
	case 0x0:
		m_RTNstatus = 0x00;
		break;
	case 0x01:
		m_RTNstatus = 0x01;
		break;
	case 0x02:
		m_RTNstatus = 0x02;
		break;
	case 0x03:
		m_RTNstatus = 0x03;
		break;
	case 0x04:	
		m_RTNstatus = 0x04;	
		break;
	case 0x05:
		m_RTNstatus = 0x05;
		break;
	case 0x06:
		m_RTNstatus = 0x06;
		break;
	default:
		m_RTNstatus = 0xEF;
	}
	return m_RTNstatus;
}

BYTE Airc_protocol::GetStatus()       //判断一个设备当前状态
{
	return m_RTNstatus;
}
//函 数 名：AscToHex()
//功能描述：把ASCII转换为16进制
unsigned char Airc_protocol::AscToHex(unsigned char aHex)
{
    if((aHex>=0)&&(aHex<=9))
        aHex += 0x30;
    else if((aHex>=10)&&(aHex<=15))//A-F
        aHex += 0x37;
    else aHex = 0xff;
    return aHex;
}
//函 数 名：HexToAsc()
//功能描述：把16进制转换为ASCII
unsigned char Airc_protocol::HexToAsc(unsigned char aChar)
{
    if((aChar>=0x30)&&(aChar<=0x39))
        aChar -= 0x30;
    else if((aChar>=0x41)&&(aChar<=0x46))//大写字母
        aChar -= 0x37;
    else if((aChar>=0x61)&&(aChar<=0x66))//小写字母
        aChar -= 0x57;
    else aChar = 0xff;
    return aChar; 
} 


DWORD WINAPI ThreadProc(LPVOID lpParameter)
{
	time_t curr_time,last_time,curr_time_1,last_time_1;

	Airc_protocol *pThis = &m_aircprotocol;
	int i,cnt,j;
	UINT ret;
	
	//获取时间
	curr_time=time(NULL);
	last_time=time(NULL);
	last_time_1=time(NULL);
	while(1)
	{
		//检测是否有设置命令，设置命令应该立刻执行
		for(i=0,j=0;i<AIRC_SET_CMD_CNT;)
		{
			//获取时间
			curr_time=time(NULL);
			last_time=time(NULL);
			//存在设置命令，需要执行
			if(airc_set_cmd_list[i].cmd_id != AIRC_INVALID_CMD)
			{
				//组成发送序列
				ret = pThis->Airc_ctrl(SET_PARA,(struct AIRC_SET_CMD *)&airc_set_cmd_list[i]);
				if(ret == 0)
				{			
					if(m_aircdev.CommSend(pThis->m_sendFrame,pThis->m_length))
					{
						//发送成功后开始接收
						cnt = 0;
						while(1)
						{
							curr_time=time(NULL);
							if(difftime(curr_time,last_time)>3)
							{
								//本次通信失败
								break;
							}
							m_aircdev.CommRecv(pThis->m_recvFrame+cnt,1);
							if(*(pThis->m_recvFrame) == 0x7E)
							{
								if(*(pThis->m_recvFrame+cnt) != 0x0D)
								{
									cnt++;
								}
								else
								{
									//接收完毕
									if(pThis->divdeFrame(pThis->m_recvFrame,(cnt+1)) == -1)
									{
										j++;
										if(j>2)
										{
											//通信故障
											break;
										}
									}
									else
									{
										//清空当前命令列表
										airc_set_cmd_list[i].cmd_id = AIRC_INVALID_CMD;
										memset(pThis->m_recvFrame,0,AIRC_RECV_SIZE);
										i++;
										break;
									}
								}
							}
						}
					}
				}
			}
			else
			{
				i++;
			}
		}

		curr_time_1=time(NULL);
		//每隔10s，进行一次查询空调数据操作
		if(difftime(curr_time_1,last_time_1)>10)
		{
			last_time_1=time(NULL);
			//循环发送获取参数的命令
			for(i=0,j=0;i<AIRC_GET_CMD_CNT;)
			{		
				curr_time=time(NULL);
				last_time=time(NULL);

				{
					//组成发送序列
					ret = pThis->Airc_ctrl(airc_get_cmd_list[i],NULL);
					if(ret == 0)
					{			
						if(m_aircdev.CommSend(pThis->m_sendFrame,pThis->m_length))
						{
							//发送成功后开始接收
							cnt = 0;
							while(1)
							{
								curr_time=time(NULL);
								if(difftime(curr_time,last_time)>3)
								{
									//本次通信失败
									break;
								}
								m_aircdev.CommRecv(pThis->m_recvFrame+cnt,1);
								if(*(pThis->m_recvFrame) == 0x7E)
								{
									if(*(pThis->m_recvFrame+cnt) != 0x0D)
									{
										cnt++;
									}
									else
									{
										//接收完毕
										if(pThis->divdeFrame(pThis->m_recvFrame,(cnt+1)) == -1)
										{
											j++;
											if(j>2)
											{
												//通信故障
												break;
											}
										}
										else
										{
											//清空当前命令列表
											memset(pThis->m_recvFrame,0,AIRC_RECV_SIZE);
											i++;
											break;
										}
									}
								}
							}
						}
					}
				}
			}
		}
		//线程停止条件
		if(WaitForSingleObject(m_aircdev.m_StartSendvent, 0) == WAIT_OBJECT_0)
		{
			CloseHandle(m_aircdev.m_StartSendvent);
			m_aircdev.m_StartSendvent = NULL;
			CloseHandle(m_aircprotocol.m_Airc_Thread);
		    m_aircprotocol.m_Airc_Thread = NULL;
			break;
		}
	}
	return 0;
}


void Air_Con_Ctrl::GetAirc_conYC(Airc_CONYC* pdata)
{
	if(pdata != NULL)
	{
		*pdata = m_conyc;
	}
}

void Air_Con_Ctrl::SetAirc_param(Airc_Param param)
{
	airc_set_cmd_list[0].cmd_id = 0x80;
	airc_set_cmd_list[0].cmd_data = param.cold_point;

	airc_set_cmd_list[1].cmd_id = 0x81;
	airc_set_cmd_list[1].cmd_data = param.sensitivity;

	airc_set_cmd_list[2].cmd_id = 0x82;
	airc_set_cmd_list[2].cmd_data = param.heat_point;

	airc_set_cmd_list[3].cmd_id = 0x83;
	airc_set_cmd_list[3].cmd_data = param.heat_sen;

	airc_set_cmd_list[4].cmd_id = 0x84;
	airc_set_cmd_list[4].cmd_data = param.inhigh_temp;

	airc_set_cmd_list[5].cmd_id = 0x85;
	airc_set_cmd_list[5].cmd_data = param.inlow_temp;

	airc_set_cmd_list[6].cmd_id = 0x86;
	airc_set_cmd_list[6].cmd_data = param.outhigh_temp;

	airc_set_cmd_list[7].cmd_id = 0x87;
	airc_set_cmd_list[7].cmd_data = param.outlow_temp;

	airc_set_cmd_list[8].cmd_id = 0x88;
	airc_set_cmd_list[8].cmd_data = param.in_windstop;

	airc_set_cmd_list[9].cmd_id = 0x89;
	airc_set_cmd_list[9].cmd_data = param.emerg_stop;

	airc_set_cmd_list[10].cmd_id = 0x8a;
	airc_set_cmd_list[10].cmd_data = param.out_temp;

	airc_set_cmd_list[11].cmd_id = 0x8b;
	airc_set_cmd_list[11].cmd_data = param.out_uplimit;

	airc_set_cmd_list[12].cmd_id = 0x8c;
	airc_set_cmd_list[12].cmd_data = param.out_downlimit;

	airc_set_cmd_list[13].cmd_id = 0x8d;
	airc_set_cmd_list[13].cmd_data = param.out_wind1Turn;

	airc_set_cmd_list[14].cmd_id = 0x8e;
	airc_set_cmd_list[14].cmd_data = param.out_wind1UP;

	airc_set_cmd_list[15].cmd_id = 0x8f;
	airc_set_cmd_list[15].cmd_data = param.out_wind1down;

	airc_set_cmd_list[16].cmd_id = 0x90;
	airc_set_cmd_list[16].cmd_data = param.out_W1uplimit;

	airc_set_cmd_list[17].cmd_id = 0x91;
	airc_set_cmd_list[17].cmd_data = param.out_W1downlimit;

	airc_set_cmd_list[18].cmd_id = 0x92;
	airc_set_cmd_list[18].cmd_data = param.out_wind2Turn;

	airc_set_cmd_list[19].cmd_id = 0x93;
	airc_set_cmd_list[19].cmd_data = param.out_wind2UP;

	airc_set_cmd_list[20].cmd_id = 0x94;
	airc_set_cmd_list[20].cmd_data = param.out_wind2down;

	airc_set_cmd_list[21].cmd_id = 0x95;
	airc_set_cmd_list[21].cmd_data = param.out_W2uplimit;

	airc_set_cmd_list[22].cmd_id = 0x96;
	airc_set_cmd_list[22].cmd_data = param.out_W2downlimit;
}

void Air_Con_Ctrl::GetAirc_param(Airc_Param* pdata)
{
	if(pdata != NULL)
	{
		*pdata = m_airc_param;
	}
}

void Air_Con_Ctrl::GetAirc_status(Airc_Status* pdata)
{
	if(pdata != NULL)
	{
		*pdata = m_airc_status;
	}
}

void Air_Con_Ctrl::GetAirc_warn(Airc_Warn* pdata)
{
	if(pdata != NULL)
	{
		*pdata = m_airc_warn;
	}
}

void Air_Con_Ctrl::GetDevrun_times(devrun_times* pdata)
{
	if(pdata != NULL)
	{
		*pdata = m_dev_times;
	}
}

void Air_Con_Ctrl::GetDevrun_time(devrun_times* pdata)
{
	if(pdata != NULL)
	{
		*pdata = m_dev_time;
	}
}

DWORD Air_Con_Ctrl::TemThread(LPVOID lparam)
{
	Air_Con_Ctrl* pthis = (Air_Con_Ctrl*)lparam;
	int i;
	Airc_Warn m_warn;
	Airc_Param m_param;
	TCHAR m_sztemp[7][32];

	while(true)
	{
		i = 0;
		memset(m_sztemp,0x00,7*32);
		pthis->GetAirc_warn(&m_warn);
		if (m_warn.in_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"机柜内高温告警");
			i++;
		}

		if (m_warn.low_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"机柜内低温告警");
			i++;
		}

		if (m_warn.basehigh_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"基站内高温告警");
			i++;
		}

		if (m_warn.baselow_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"基站内低温告警");
			i++;
		}

		if (m_warn.cab_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"机柜内温度传感器失效");
			i++;
		}

		if (m_warn.base_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"基站内温度传感器失效");
			i++;
		}

		if (m_warn.cab_hnmid == 0xf0)
		{
			wsprintf(m_sztemp[i],L"机柜内温度传感器失效");
			i++;
		}

		if (i<6 && m_warn.press_mach == 0xf0)
		{
			wsprintf(m_sztemp[i],L"压缩机告警");
			i++;
		}

		if (i<6 && m_warn.door_ban == 0xf0)
		{
			wsprintf(m_sztemp[i],L"门禁告警");
			i++;
		}

		if (i<6 && m_warn.shake == 0xf0)
		{
			wsprintf(m_sztemp[i],L"振动告警");
			i++;
		}

		if (i<6 && m_warn.press_mach_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"压缩机故障");
			i++;
		}

		if (i<6 && m_warn.heat_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"加电器故障");
			i++;
		}

		if (i<6 && m_warn.water_flood == 0xf0)
		{
			wsprintf(m_sztemp[i],L"水淹告警");
			i++;
		}

		if (i<6 && m_warn.smoke == 0xf0)
		{
			wsprintf(m_sztemp[i],L"烟雾告警");
			i++;
		}

		if (i<6 && m_warn.thunder_eer == 0xf0)
		{
			wsprintf(m_sztemp[i],L"防雷失效告警");
			i++;
		}

		if (i<6 && m_warn.prevent_cold == 0xf0)
		{
			wsprintf(m_sztemp[i],L"盘管防冻保护告警");
			i++;
		}

		if (i<6 && m_warn.AC_pass_voltage == 0xf0)
		{
			wsprintf(m_sztemp[i],L"交流过压告警");
			i++;
		}

		if (i<6 && m_warn.AC_lock_voltage == 0xf0)
		{
			wsprintf(m_sztemp[i],L"交流欠压告警");
			i++;
		}

		if (i<6 && m_warn.stop_ele == 0xf0)
		{
			wsprintf(m_sztemp[i],L"市停电告警");
			i++;
		}

		if (i<6 && m_warn.in_wind_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"内风机故障");
			i++;
		}

		if (i<6 && m_warn.in_wind1_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"内风机1故障");
			i++;
		}

		if (i<6 && m_warn.in_wind2_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"内风机2故障");
			i++;
		}

		if (i<6 && m_warn.outair_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"排气温度过高");
			i++;
		}

		if (i<6 && m_warn.scroll_temp_effic == 0xf0)
		{
			wsprintf(m_sztemp[i],L"盘管温度传感器失效");
			i++;
		}

		if (i<6 && m_warn.outair_temp_effic == 0xf0)
		{
			wsprintf(m_sztemp[i],L"排气温度传感器失效");
			i++;
		}

		if (i<6 && m_warn.EEPROM == 0xf0)
		{
			wsprintf(m_sztemp[i],L"EEPROM故障告警");
			i++;
		}

		if (i<6 && pthis->temflag)
		{
			if (pthis->ReadDeviceAD(pthis->m_lDeviceID,pthis->m_ADBuffer,DAM3046_AI,0,5))
			{
				pthis->GetAirc_param(&m_param);
				for (int j=0;j<6;j++)
				{
					if ((i<6) && (m_param.inhigh_temp<(pthis->m_ADBuffer[i])))
					{
						wsprintf(m_sztemp[i],L"仓内温度采集点%d过温",i);
						i++;
					}

					if ((i<6) && (m_param.inlow_temp>(pthis->m_ADBuffer[i])))
					{
						wsprintf(m_sztemp[i],L"仓内温度采集点%d欠温",i);
						i++;
					}
				}
				
			}
		}

		pthis->m_nWarnNum = i;
		memcpy((pthis->m_szWarn),m_sztemp,7*32);
		//如果收到读线程退出信号，则退出线程
		if (WaitForSingleObject(pthis->m_hCloseEvent,500) == WAIT_OBJECT_0)
		{
			break;
		}
	}

	return 0;
}

void Air_Con_Ctrl::CloseThread()
{
	SetEvent(m_hCloseEvent);

	//等待10秒，如果读线程没有退出，则强制退出
	if (WaitForSingleObject(m_hThread,500) == WAIT_TIMEOUT)
	{
		TerminateThread(m_hThread,0);
	}
	m_hThread = NULL;
}

BOOL Air_Con_Ctrl ::Tem_start(LONG nPortNum,LONG lBaud)
{
	BOOL ret;
	LONG i;
	m_hDevice = DAM3000M_CreateDevice(nPortNum);
	if(m_hDevice == NULL)
	{
		return false;
	}

	ret = DAM3000M_InitDevice(m_hDevice,lBaud,TRUE,500);
	if (!ret)
	{
		return ret;
	}

	for (i=0;i<255;i++)
	{
		ret = DAM3000M_GetDeviceInfo(m_hDevice,i,&m_Info);
		if (ret)
		{
			m_lDeviceID = i;
			break;
		}
	}

	return ret;

}

void Air_Con_Ctrl::Tem_stop()
{
	DAM3000M_ReleaseDevice(m_hDevice);
}

BOOL Air_Con_Ctrl::ReadDeviceAD(LONG lDeviceID, LONG lpADBuffer[], LONG lBufferSize, LONG lFirstChannel, LONG lLastChannel)
{
	return DAM3000M_ReadDeviceAD(m_hDevice,lDeviceID,lpADBuffer,lBufferSize,lFirstChannel,lLastChannel);
}

bool Air_Con_Ctrl::Start_Airc(unsigned int comrate)
{
	BOOL ret = FALSE;
	temflag = FALSE;
	int i;

	memset(m_ADBuffer,0,sizeof(m_ADBuffer));
	m_lADMode=0;
	m_lDeviceID=0;		// 设备地址
	memset(&m_Info,0,sizeof(DAM3000M_DEVICE_INFO));
	m_hDevice = NULL;
	
	//初始化变量
	m_aircdev.m_StartSendvent = NULL;
	//初始化命令列表
	memset(airc_set_cmd_list,0,sizeof(airc_set_cmd_list));
	for(i=0;i<3;i++)
	{
		airc_get_cmd_list[i] = GET_DATA+i;
	}
	airc_get_cmd_list[3] = GET_PARA;
	for(i=0;i<4;i++)
	{
		airc_get_cmd_list[i+4] = GET_RUNTIME+i;
	}
	airc_get_cmd_list[6] = GET_VERSION;
	airc_get_cmd_list[7] = GET_ADDR;

	//temflag = Tem_start(DAM3000M_COM1,DAM3000_BAUD_9600);

	//打开串口
	ret = m_aircdev.OnOpenCom(AIRC_PORT,comrate,AIRC_DBIT,AIRC_SBIT,AIRC_PARITY);
	if(ret == TRUE)
	{

		m_hCloseEvent = CreateEvent(NULL,TRUE,FALSE,NULL);
		m_hThread = CreateThread(NULL,0,TemThread,this,0,&m_dwThreadID);
		//创建事件
		m_aircdev.m_StartSendvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		if(m_aircdev.m_StartSendvent != NULL)
		{
			//创建时间中断
			m_aircprotocol.m_Airc_Thread =  CreateThread(NULL, 0, ThreadProc, this, 0, NULL);
			if(m_aircprotocol.m_Airc_Thread != NULL)
			{
				return true;
			}
			else
			{
				CloseHandle(m_aircdev.m_StartSendvent);
                m_aircdev.m_StartSendvent =NULL;
			}
		}
	}
	return false;
}

bool Air_Con_Ctrl::Stop_Airc(void)
{
	if(m_aircprotocol.m_Airc_Thread != NULL)
	{
		SetEvent(m_aircdev.m_StartSendvent);
		
		WaitForSingleObject(m_aircprotocol.m_Airc_Thread, 1500);
		CloseHandle(m_aircprotocol.m_Airc_Thread);
		m_aircprotocol.m_Airc_Thread = NULL;
	}

	if (m_aircdev.m_StartSendvent != NULL)
	{
		CloseHandle(m_aircdev.m_StartSendvent);
		m_aircdev.m_StartSendvent = NULL;
	}

	CloseThread();

	//Tem_stop();
	
	return true;
}

time_t time(time_t *inTT)
{
	SYSTEMTIME sysTimeStruct;
	FILETIME fTime;
	ULARGE_INTEGER int64time;
	time_t locTT = 0;

	if(inTT == NULL)
	{
		inTT = &locTT;
	}

	GetSystemTime(&sysTimeStruct);
	if(SystemTimeToFileTime(&sysTimeStruct, &fTime))
	{
		memcpy( &int64time, &fTime, sizeof(FILETIME));
		/* Subtract the value for 1970-01-01 00:00 (UTC) */
		int64time.QuadPart -= 0x19db1ded53e8000;
		/* Convert to seconds. */
		int64time.QuadPart /= 10000000;
		*inTT = (time_t)int64time.QuadPart;
	}
	return *inTT;
}

