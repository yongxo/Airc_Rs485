#include "DAM3000M.h"

#define DAM3046_AI 6

#define GET_DATA 0x42
#define GET_STATE 0x43
#define GET_WARN 0x44
#define SET_ON 0x45  //����
#define SET_OFF 0x46		//�ػ�
#define GET_PARA 0x47
#define SET_PARA 0x49    //����
#define GET_RUNTIME 0x80
#define GET_TIMES 0x81
#define GET_VERSION   0x4F
#define GET_ADDR 0x50

//�յ����ò��������
#define AIRC_SET_CMD_CP	 (0x80) //�����趨��
#define AIRC_SET_CMD_SENS (0x81)// ������
#define AIRC_SET_CMD_HP  (0x82)//�����趨��
#define AIRC_SET_CMD_HS  (0x83)  //����������
#define AIRC_SET_CMD_ITHP (0x84) //�ڸ��±�����
#define AIRC_SET_CMD_ITLP (0x85) //�ڵ��±�����
#define AIRC_SET_CMD_OTHP (0x86)//����±�����
#define AIRC_SET_CMD_OTLP (0x87)//����±�����
#define AIRC_SET_CMD_IWP (0x88)//�ڷ��ֹͣ��
#define AIRC_SET_CMD_ESTP (0x89)//����ֹͣ��
#define AIRC_SET_CMD_OWP (0x8a)//�����¶��趨��
#define AIRC_SET_CMD_OWUP (0x8b)//��������
#define AIRC_SET_CMD_OWLP  (0x8c)//��������
#define AIRC_SET_CMD_OWSP1 (0x8d)//�������ٵ�
#define AIRC_SET_CMD_OWSP1UP (0x8e)//����������ƫ��
#define AIRC_SET_CMD_OWSP1DOWN (0x8f)//����������ƫ��
#define AIRC_SET_CMD_OWSP1H (0x90)//������������
#define AIRC_SET_CMD_OWSP1L (0x91) //������������
#define AIRC_SET_CMD_OWSP2 (0x92)//�������ٵ�
#define AIRC_SET_CMD_OWSP2UP (0x93)//����������ƫ��
#define AIRC_SET_CMD_OWSP2DOWN (0x94)//����������ƫ��
#define AIRC_SET_CMD_OWSP2H (0x95) //������������
#define AIRC_SET_CMD_OWSP2L (0x96) //������������


struct  Airc_CONYC
{
	UINT  in_temp;			//�����¶�
	UINT  out_temp;			//�������¶�
	UINT  cab_hnmidity;		//������ʪ��
	UINT  Ele_flow;		//���ص���
	UINT  ele_voltage;		//������ѹ
	UINT  DC_voltage;		//ֱ����ѹ
};

struct Airc_Param   //���������յ���������
{
	BYTE	cold_point; //�����趨��
	BYTE	sensitivity; // ������
	BYTE	heat_point; //�����趨��
	BYTE	heat_sen;  //����������
	BYTE	inhigh_temp;//�ڸ��±�����
	BYTE	inlow_temp;//�ڵ��±�����
	BYTE	outhigh_temp;//����±�����
	BYTE	outlow_temp;//����±�����
	BYTE	in_windstop;//�ڷ��ֹͣ��
	BYTE	emerg_stop;//����ֹͣ��
	BYTE	out_temp;//�����¶��趨��
	BYTE	out_uplimit;//��������
	BYTE	out_downlimit;//��������
	BYTE	out_wind1Turn;//����1���ٵ�
	BYTE	out_wind1UP;//����1������ƫ��
	BYTE	out_wind1down;//����1������ƫ��
	BYTE	out_W1uplimit; //����1��������
	BYTE	out_W1downlimit; //����1��������
	BYTE	out_wind2Turn;//����2���ٵ�
	BYTE	out_wind2UP;//����2������ƫ��
	BYTE	out_wind2down;//����2������ƫ��
	BYTE	out_W2uplimit; //����2��������
	BYTE	out_W2downlimit; //����2��������
	BYTE	user;//�û��Զ���
};

struct Airc_Status   //���������յ������ֹ���״̬
{
	BYTE  dev_state;//�豸״̬
	BYTE  in_wind;//�ڷ��
	BYTE	press_mch;//ѹ����
	BYTE	ele_hot; //�����
	BYTE	out_wind;//����
	BYTE	out_wind1;//�ⲿ���1
	BYTE	out_wind2;//�ⲿ���2
	BYTE   user_define;//�л�����

};
struct Airc_Warn  //���������յ������ֱ�����Ϣ
{
	BYTE  in_temp; //�����ڸ��¸澯
	BYTE	low_temp;// �����ڵ��¸澯
	BYTE	basehigh_temp; //��վ�ڸ��¸澯
	BYTE 	baselow_temp; //��վ�ڵ��¸澯
	BYTE 	cab_temp;  //�������¶ȴ�����ʧЧ
	BYTE	base_temp; //��վ���¶ȴ�����ʧЧ
	BYTE	cab_hnmid;  //�������¶ȴ�����ʧЧ
	BYTE	press_mach;//ѹ�����澯
	BYTE	door_ban; //�Ž��澯
	BYTE	shake; //��
	BYTE	press_mach_bug; //ѹ��������
	BYTE	heat_bug;//�ӵ�������
	BYTE	water_flood;//ˮ�͸澯
	BYTE	smoke; //����澯
	BYTE	thunder_eer; //����ʧЧ�澯
	BYTE	prevent_cold;//�̹ܷ��������澯
	BYTE	AC_pass_voltage;  //������ѹ�澯
	BYTE	AC_lock_voltage;	//����Ƿѹ�澯
	BYTE	stop_ele; //��ͣ��澯
	BYTE	in_wind_bug;//�ڷ������
	BYTE	in_wind1_bug;//�ڷ��1����
	BYTE 	in_wind2_bug;//�ڷ��2����
	BYTE	outair_temp; //�����¶ȹ���
	BYTE	scroll_temp_effic;   //�̹��¶ȴ�����ʧЧ
	BYTE	outair_temp_effic; //�����¶ȴ�����ʧЧ
	BYTE	EEPROM;  // EEPROM���ϸ澯
	BYTE	usr;//����
};

struct devrun_times  //�豸���д���
{
	long 	dev_state; //�豸״̬
	long	in_wind; //�ڷ��
	long	press_mach;//ѹ����
	long	heat_hot; //�����
	long	out_wind;//����	
	long	out_wind1;//����
	long	out_wind2;//����
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
	Description		: ��ȡ�յ�ң����Ϣ
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetAirc_conYC(Airc_CONYC*);
/**********************************************************************************
	Function 		: GetAirc_param
	Description		: ��ȡ�յ�����
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetAirc_param(Airc_Param*);
/**********************************************************************************
	Function 		: GetAirc_status
	Description		: ��ȡ�յ�����״̬
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetAirc_status(Airc_Status*);
/**********************************************************************************
	Function 		: GetAirc_warn
	Description		: ��ȡ�յ�������Ϣ
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetAirc_warn(Airc_Warn*);
/**********************************************************************************
	Function 		: GetDevrun_times
	Description		: ��ȡ�յ�����ʱ����Ϣ
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetDevrun_times(devrun_times*);
	/**********************************************************************************
	Function 		: GetDevrun_time
	Description		: ��ȡ�յ����д�����Ϣ
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	void GetDevrun_time(devrun_times*);
	/**********************************************************************************
	Function 		: Start_Airc
	Description		: �����յ�������ģ��
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	bool Start_Airc(unsigned int comrate);
	/**********************************************************************************
	Function 		: Stop_Airc
	Description		: ֹͣ�յ�������ģ��
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	bool Stop_Airc();

	static  DWORD WINAPI TemThread(LPVOID lparam);

	void CloseThread();             //�ر��߳�
public:
	TCHAR   m_szWarn[7][32];

	int     m_nWarnNum;
	//�߳��˳��¼�
	HANDLE m_hCloseEvent;
	//�߳̾��
	HANDLE m_hThread;
	//�߳�ID��ʶ
	DWORD m_dwThreadID;


public:
	HANDLE m_hDevice;
	BOOL Tem_start(LONG nPortNum,LONG lBaud);
	BOOL ReadDeviceAD( // ��ȡADģ�������� 
		LONG	lDeviceID,			// ģ���ַ
		LONG	lpADBuffer[],		// ����AD���ݵ��û������� ע��:lpADBuffer��ô��ڵ���lLastChannel - lFirstChannel +1
		LONG    lBufferSize,        // ����lpADBuffer[]�Ĵ�С
		LONG	lFirstChannel,	// ��ͨ��
		LONG	lLastChannel); // ĩͨ��
	void Tem_stop();
	LONG m_ADBuffer[DAM3046_AI];
	LONG m_lADMode;
	LONG m_lDeviceID;		// �豸��ַ
	DAM3000M_DEVICE_INFO m_Info;
	BOOL temflag;
};