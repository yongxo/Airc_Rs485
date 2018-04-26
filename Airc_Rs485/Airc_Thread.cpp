#include "StdAfx.h"
#include "Airc_Thread.h"
#include "Airc_dev.h"

//���ÿյ������궨��
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
		SetFrame(0x45, 0x01, 0x01, 0x0, 0x02);  //ң�ؿ���
		flag = 0;
		break;
	case 0x46:
		SetFrame(0x45, 0x02, 0x02, 0x0, 0x02);  //ң�عػ�
		flag = 0;
		break;	
	case 0x47:
		SetFrame(0x47, 0x01, 0x0, 0x0 ,0);  //ң�عػ�
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
	CHAR SetPoint = -5; //�����趨��
	CHAR codeOn = 40;   //���俪����
	CHAR Uplimti = 0;   //������
	CHAR DownLimti = 0; //������
	CHAR OutPoint = 0;//������ٵ�
	switch (type)
	{
		case 0x80:
			if (18 <= para && para <= 40)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); //�趨����
				flag = 0;
			}		
			break;
		case 0x81:
			if (1 <= para && para <= 10)
			{
				SetFrame(0x49, 0x01, 0x80, para, 0x04); //�趨����
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
				SetFrame(0x49, 0x01, 0x80, para, 0x04); //�趨����
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

void Airc_protocol::SetFrame(BYTE CID2, BYTE addr, BYTE type, UINT para, unsigned short lenid)     //����֡����
{	
	int i = 0;
//	m_varsion = 0;
	m_success = 0;
	m_length = 0;
	m_sendFrame[i++] = /*m_cmdata.SOI = */0x7e;
	m_cmdata.VER = 0x21;
	m_sendFrame[i++] = AscToHex(GetHigh4(m_cmdata.VER));
	m_sendFrame[i++] = AscToHex(GetLow4(m_cmdata.VER));
	m_cmdata.ADR = addr;  //�豸��ַ���� 1-254 
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
		switch (type)  //����1 �� 2 �� 3 �����趨
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

//��ȡLENGTH�ֶ�ֵ
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

BYTE Airc_protocol::divdeFrame(BYTE * buf, UINT length)  // ���յ����ݴ���
{
	//memset(m_recvFrame, 0, 100);
	UINT chktemp = VerifyCheck(buf, length - 6);//ȡ������У�飬������Ҫ����4��У��+�����ֽڡ�
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
#if 0//Э���ı����󣬵���---xuliang
		m_conyc.DC_voltage = m_recvFrame[7]*255 + m_recvFrame[8];			
		m_conyc.ele_voltage= m_recvFrame[9]*255 + m_recvFrame[10];		//������ѹ
		m_conyc.Ele_flow = m_recvFrame[11]*255 + m_recvFrame[12];		//���ص���
		m_conyc.cab_hnmidity = m_recvFrame[13]*255 + m_recvFrame[14];	//������ʪ��
		m_conyc.out_temp = m_recvFrame[15]*255 + m_recvFrame[16];		//�������¶�
		m_conyc.in_temp = m_recvFrame[17]*255 + m_recvFrame[18];//�����¶�
#endif
#if 0 // �ۺ�Ƴ������¶��⣬���༸���û�м�⡣
		m_conyc.DC_voltage = (m_recvFrame[7]*256 + m_recvFrame[8])/10;			
		m_conyc.ele_voltage= (m_recvFrame[9]*256 + m_recvFrame[10])/10;		//������ѹ
		m_conyc.Ele_flow = (m_recvFrame[11]*256 + m_recvFrame[12])/10;		//���ص���
		m_conyc.cab_hnmidity = m_recvFrame[13]*256 + m_recvFrame[14];	//������ʪ��
		m_conyc.out_temp = m_recvFrame[15]*256 + m_recvFrame[16];		//�������¶�
		m_conyc.in_temp = m_recvFrame[17]*256 + m_recvFrame[18];//�����¶�
#endif
		m_conyc.in_temp = (m_recvFrame[7]*256 + m_recvFrame[8])/10;
		break;
	case 0x43:
		m_airc_status.dev_state = m_recvFrame[7];
		m_airc_status.dev_state = m_recvFrame[8];//�豸״̬
		m_airc_status.in_wind= m_recvFrame[9];//�ڷ��
		m_airc_status.press_mch= m_recvFrame[10];//ѹ����
		m_airc_status.ele_hot= m_recvFrame[11]; //�����
		m_airc_status.out_wind= m_recvFrame[12];//����
		m_airc_status.out_wind1= m_recvFrame[13];//�ⲿ���1
	//	m_airc_status.out_wind2= m_recvFrame[14];//�ⲿ���2
	//	m_airc_status.user_define= m_recvFrame[15];//�л�����
		break;
	case 0x44:
		m_airc_warn.in_temp = m_recvFrame[7]; //�����ڸ��¸澯
		m_airc_warn.low_temp= m_recvFrame[8];// �����ڵ��¸澯
		m_airc_warn.basehigh_temp = m_recvFrame[9]; //��վ�ڸ��¸澯
		m_airc_warn.baselow_temp= m_recvFrame[10]; //��վ�ڵ��¸澯
		m_airc_warn.cab_temp= m_recvFrame[11];  //�������¶ȴ�����ʧЧ
		m_airc_warn.base_temp = m_recvFrame[12]; //��վ���¶ȴ�����ʧЧ
		m_airc_warn.cab_hnmid = m_recvFrame[13];  //�������¶ȴ�����ʧЧ
		m_airc_warn.press_mach = m_recvFrame[14];//ѹ�����澯
		m_airc_warn.door_ban = m_recvFrame[15]; //�Ž��澯
		m_airc_warn.shake = m_recvFrame[16]; //��
		m_airc_warn.press_mach_bug = m_recvFrame[17]; //ѹ��������
		m_airc_warn.heat_bug = m_recvFrame[18];//�ӵ�������
		m_airc_warn.water_flood = m_recvFrame[19];//ˮ�͸澯
		m_airc_warn.smoke = m_recvFrame[20]; //����澯
		m_airc_warn.thunder_eer = m_recvFrame[21]; //����ʧЧ�澯
		m_airc_warn.prevent_cold = m_recvFrame[22];//�̹ܷ��������澯
		m_airc_warn.AC_pass_voltage = m_recvFrame[23];  //������ѹ�澯
		m_airc_warn.AC_lock_voltage = m_recvFrame[24];	//����Ƿѹ�澯
		m_airc_warn.stop_ele = m_recvFrame[25]; //��ͣ��澯
		m_airc_warn.in_wind_bug = m_recvFrame[26];//�ڷ������
		m_airc_warn.in_wind1_bug = m_recvFrame[27];//�ڷ��1����
		m_airc_warn.in_wind2_bug = m_recvFrame[28];//�ڷ��2����
		m_airc_warn.outair_temp = m_recvFrame[29]; //�����¶ȹ���
		m_airc_warn.scroll_temp_effic = m_recvFrame[30];   //�̹��¶ȴ�����ʧЧ
		m_airc_warn.outair_temp_effic = m_recvFrame[31]; //�����¶ȴ�����ʧЧ
		m_airc_warn.EEPROM = m_recvFrame[32];  // EEPROM���ϸ澯
		break;
	case 0x45:
			m_success = 1;  //��������ɹ���
		break;
	case 0x47:
		m_airc_param.cold_point = m_recvFrame[7]; //�����趨��
		m_airc_param.sensitivity = m_recvFrame[8]; // ������
		m_airc_param.heat_point = m_recvFrame[9]; //�����趨��
		m_airc_param.heat_sen = m_recvFrame[10];  //����������
		m_airc_param.inhigh_temp = m_recvFrame[11];//�ڸ��±�����
		m_airc_param.inlow_temp = m_recvFrame[12];//�ڵ��±�����
		m_airc_param.outhigh_temp = m_recvFrame[13];//����±�����
		m_airc_param.outlow_temp = m_recvFrame[14];//����±�����
		m_airc_param.in_windstop = m_recvFrame[15];//�ڷ��ֹͣ��
		m_airc_param.emerg_stop = m_recvFrame[16];//����ֹͣ��
		m_airc_param.out_temp = m_recvFrame[17];//�����¶��趨��
		m_airc_param.out_uplimit = m_recvFrame[18];//��������
		m_airc_param.out_downlimit = m_recvFrame[19];//��������
		m_airc_param.out_wind1Turn = m_recvFrame[20];//����1���ٵ�
		m_airc_param.out_wind1UP = m_recvFrame[21];//����1������ƫ��
		m_airc_param.out_wind1down = m_recvFrame[22];//����1������ƫ��
		m_airc_param.out_W1uplimit = m_recvFrame[23]; //����1��������
		m_airc_param.out_W1downlimit = m_recvFrame[24]; //����1��������
		m_airc_param.out_wind2Turn = m_recvFrame[25];//����2���ٵ�
		m_airc_param.out_wind2UP = m_recvFrame[26];//����2������ƫ��
		m_airc_param.out_wind2down = m_recvFrame[27];//����2������ƫ��
		m_airc_param.out_W2uplimit = m_recvFrame[28]; //����2��������
		m_airc_param.out_W2downlimit = m_recvFrame[29]; //����2��������
		//m_airc_param.user;//�û��Զ���
		break;
	case 0x49:
		m_success = 1;
		break;
	case 0x80:
		m_dev_time.dev_state = (m_recvFrame[7]<<24) + (m_recvFrame[8]<<16) + (m_recvFrame[9]<<8) + m_recvFrame[10]; //�豸״̬
		m_dev_time.in_wind = (m_recvFrame[11]<<24) + (m_recvFrame[12]<<16) + (m_recvFrame[13]<<8) + m_recvFrame[14]; //�ڷ��
		m_dev_time.press_mach = (m_recvFrame[15]<<24) + (m_recvFrame[16]<<16) + (m_recvFrame[17]<<8) + m_recvFrame[18];//ѹ����
		m_dev_time.heat_hot= (m_recvFrame[19]<<24) + (m_recvFrame[20]<<16) + (m_recvFrame[21]<<8) + m_recvFrame[22]; //�����
		m_dev_time.out_wind = (m_recvFrame[23]<<24) + (m_recvFrame[24]<<16) + (m_recvFrame[25]<<8) + m_recvFrame[26];//����	
		m_dev_time.out_wind1 = (m_recvFrame[27]<<24) + (m_recvFrame[28]<<16) + (m_recvFrame[29]<<8) + m_recvFrame[30];//����
		m_dev_time.out_wind2 = (m_recvFrame[31]<<24) + (m_recvFrame[32]<<16) + (m_recvFrame[33]<<8) + m_recvFrame[34];//����
		//m_dev_time.user_times = m_recvFrame[7]<<24 + m_recvFrame[8]<<16 + m_recvFrame[9]<<8 + m_recvFrame[10];
		break;
	case 0x81:
		m_dev_times.dev_state = (m_recvFrame[7]<<24) + (m_recvFrame[8]<<16) + (m_recvFrame[9]<<8) + m_recvFrame[10]; //�豸״̬
		m_dev_times.in_wind = (m_recvFrame[11]<<24) + (m_recvFrame[12]<<16) + (m_recvFrame[13]<<8) + m_recvFrame[14]; //�ڷ��
		m_dev_times.press_mach = (m_recvFrame[15]<<24) + (m_recvFrame[16]<<16) + (m_recvFrame[17]<<8) + m_recvFrame[18];//ѹ����
		m_dev_times.heat_hot= (m_recvFrame[19]<<24) + (m_recvFrame[20]<<16) + (m_recvFrame[21]<<8) + m_recvFrame[22]; //�����
		m_dev_times.out_wind = (m_recvFrame[23]<<24) + (m_recvFrame[24]<<16) + (m_recvFrame[25]<<8) + m_recvFrame[26];//����	
		m_dev_times.out_wind1 = (m_recvFrame[27]<<24) + (m_recvFrame[28]<<16) + (m_recvFrame[29]<<8) + m_recvFrame[30];//����
		m_dev_times.out_wind2 = (m_recvFrame[31]<<24) + (m_recvFrame[32]<<16) + (m_recvFrame[33]<<8) + m_recvFrame[34];//����
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

unsigned short Airc_protocol::VerifyCheck(BYTE *buf, UINT length)   //�����֤��
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

BYTE Airc_protocol::RTNstatus(byte rtn)      //�ж�PTN�Ƿ���ȷ
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

BYTE Airc_protocol::GetStatus()       //�ж�һ���豸��ǰ״̬
{
	return m_RTNstatus;
}
//�� �� ����AscToHex()
//������������ASCIIת��Ϊ16����
unsigned char Airc_protocol::AscToHex(unsigned char aHex)
{
    if((aHex>=0)&&(aHex<=9))
        aHex += 0x30;
    else if((aHex>=10)&&(aHex<=15))//A-F
        aHex += 0x37;
    else aHex = 0xff;
    return aHex;
}
//�� �� ����HexToAsc()
//������������16����ת��ΪASCII
unsigned char Airc_protocol::HexToAsc(unsigned char aChar)
{
    if((aChar>=0x30)&&(aChar<=0x39))
        aChar -= 0x30;
    else if((aChar>=0x41)&&(aChar<=0x46))//��д��ĸ
        aChar -= 0x37;
    else if((aChar>=0x61)&&(aChar<=0x66))//Сд��ĸ
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
	
	//��ȡʱ��
	curr_time=time(NULL);
	last_time=time(NULL);
	last_time_1=time(NULL);
	while(1)
	{
		//����Ƿ������������������Ӧ������ִ��
		for(i=0,j=0;i<AIRC_SET_CMD_CNT;)
		{
			//��ȡʱ��
			curr_time=time(NULL);
			last_time=time(NULL);
			//�������������Ҫִ��
			if(airc_set_cmd_list[i].cmd_id != AIRC_INVALID_CMD)
			{
				//��ɷ�������
				ret = pThis->Airc_ctrl(SET_PARA,(struct AIRC_SET_CMD *)&airc_set_cmd_list[i]);
				if(ret == 0)
				{			
					if(m_aircdev.CommSend(pThis->m_sendFrame,pThis->m_length))
					{
						//���ͳɹ���ʼ����
						cnt = 0;
						while(1)
						{
							curr_time=time(NULL);
							if(difftime(curr_time,last_time)>3)
							{
								//����ͨ��ʧ��
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
									//�������
									if(pThis->divdeFrame(pThis->m_recvFrame,(cnt+1)) == -1)
									{
										j++;
										if(j>2)
										{
											//ͨ�Ź���
											break;
										}
									}
									else
									{
										//��յ�ǰ�����б�
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
		//ÿ��10s������һ�β�ѯ�յ����ݲ���
		if(difftime(curr_time_1,last_time_1)>10)
		{
			last_time_1=time(NULL);
			//ѭ�����ͻ�ȡ����������
			for(i=0,j=0;i<AIRC_GET_CMD_CNT;)
			{		
				curr_time=time(NULL);
				last_time=time(NULL);

				{
					//��ɷ�������
					ret = pThis->Airc_ctrl(airc_get_cmd_list[i],NULL);
					if(ret == 0)
					{			
						if(m_aircdev.CommSend(pThis->m_sendFrame,pThis->m_length))
						{
							//���ͳɹ���ʼ����
							cnt = 0;
							while(1)
							{
								curr_time=time(NULL);
								if(difftime(curr_time,last_time)>3)
								{
									//����ͨ��ʧ��
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
										//�������
										if(pThis->divdeFrame(pThis->m_recvFrame,(cnt+1)) == -1)
										{
											j++;
											if(j>2)
											{
												//ͨ�Ź���
												break;
											}
										}
										else
										{
											//��յ�ǰ�����б�
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
		//�߳�ֹͣ����
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
			wsprintf(m_sztemp[i],L"�����ڸ��¸澯");
			i++;
		}

		if (m_warn.low_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�����ڵ��¸澯");
			i++;
		}

		if (m_warn.basehigh_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"��վ�ڸ��¸澯");
			i++;
		}

		if (m_warn.baselow_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"��վ�ڵ��¸澯");
			i++;
		}

		if (m_warn.cab_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�������¶ȴ�����ʧЧ");
			i++;
		}

		if (m_warn.base_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"��վ���¶ȴ�����ʧЧ");
			i++;
		}

		if (m_warn.cab_hnmid == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�������¶ȴ�����ʧЧ");
			i++;
		}

		if (i<6 && m_warn.press_mach == 0xf0)
		{
			wsprintf(m_sztemp[i],L"ѹ�����澯");
			i++;
		}

		if (i<6 && m_warn.door_ban == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�Ž��澯");
			i++;
		}

		if (i<6 && m_warn.shake == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�񶯸澯");
			i++;
		}

		if (i<6 && m_warn.press_mach_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"ѹ��������");
			i++;
		}

		if (i<6 && m_warn.heat_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�ӵ�������");
			i++;
		}

		if (i<6 && m_warn.water_flood == 0xf0)
		{
			wsprintf(m_sztemp[i],L"ˮ�͸澯");
			i++;
		}

		if (i<6 && m_warn.smoke == 0xf0)
		{
			wsprintf(m_sztemp[i],L"����澯");
			i++;
		}

		if (i<6 && m_warn.thunder_eer == 0xf0)
		{
			wsprintf(m_sztemp[i],L"����ʧЧ�澯");
			i++;
		}

		if (i<6 && m_warn.prevent_cold == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�̹ܷ��������澯");
			i++;
		}

		if (i<6 && m_warn.AC_pass_voltage == 0xf0)
		{
			wsprintf(m_sztemp[i],L"������ѹ�澯");
			i++;
		}

		if (i<6 && m_warn.AC_lock_voltage == 0xf0)
		{
			wsprintf(m_sztemp[i],L"����Ƿѹ�澯");
			i++;
		}

		if (i<6 && m_warn.stop_ele == 0xf0)
		{
			wsprintf(m_sztemp[i],L"��ͣ��澯");
			i++;
		}

		if (i<6 && m_warn.in_wind_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�ڷ������");
			i++;
		}

		if (i<6 && m_warn.in_wind1_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�ڷ��1����");
			i++;
		}

		if (i<6 && m_warn.in_wind2_bug == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�ڷ��2����");
			i++;
		}

		if (i<6 && m_warn.outair_temp == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�����¶ȹ���");
			i++;
		}

		if (i<6 && m_warn.scroll_temp_effic == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�̹��¶ȴ�����ʧЧ");
			i++;
		}

		if (i<6 && m_warn.outair_temp_effic == 0xf0)
		{
			wsprintf(m_sztemp[i],L"�����¶ȴ�����ʧЧ");
			i++;
		}

		if (i<6 && m_warn.EEPROM == 0xf0)
		{
			wsprintf(m_sztemp[i],L"EEPROM���ϸ澯");
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
						wsprintf(m_sztemp[i],L"�����¶Ȳɼ���%d����",i);
						i++;
					}

					if ((i<6) && (m_param.inlow_temp>(pthis->m_ADBuffer[i])))
					{
						wsprintf(m_sztemp[i],L"�����¶Ȳɼ���%dǷ��",i);
						i++;
					}
				}
				
			}
		}

		pthis->m_nWarnNum = i;
		memcpy((pthis->m_szWarn),m_sztemp,7*32);
		//����յ����߳��˳��źţ����˳��߳�
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

	//�ȴ�10�룬������߳�û���˳�����ǿ���˳�
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
	m_lDeviceID=0;		// �豸��ַ
	memset(&m_Info,0,sizeof(DAM3000M_DEVICE_INFO));
	m_hDevice = NULL;
	
	//��ʼ������
	m_aircdev.m_StartSendvent = NULL;
	//��ʼ�������б�
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

	//�򿪴���
	ret = m_aircdev.OnOpenCom(AIRC_PORT,comrate,AIRC_DBIT,AIRC_SBIT,AIRC_PARITY);
	if(ret == TRUE)
	{

		m_hCloseEvent = CreateEvent(NULL,TRUE,FALSE,NULL);
		m_hThread = CreateThread(NULL,0,TemThread,this,0,&m_dwThreadID);
		//�����¼�
		m_aircdev.m_StartSendvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		if(m_aircdev.m_StartSendvent != NULL)
		{
			//����ʱ���ж�
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

