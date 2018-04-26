#pragma once

#include "Air_Conditioner_Ctrl.h"


struct Airc_DataPack   //���������յ�RS485Э���ʽ
{
	BYTE SOI;
	BYTE VER;
	BYTE ADR;
	BYTE CID1;
	BYTE CID2;
	unsigned int LENGTH;
	unsigned int LENID;
	UINT INFO;
	unsigned int CHKSUM;
	BYTE EOI;	
};

struct CMD_TYPE   //Ԥ��
{
	BYTE COMMAND_GROUP;
	BYTE COMMAND_ID;
	BYTE COMMAND_TYPE;  //10H �յ�����  1F�յ��ػ�
	BYTE COMMAND_TIME;
};
class Airc_Thread
{
public:
	Airc_Thread(void);
	virtual ~Airc_Thread(void);
};

class Airc_protocol
{
public:
	Airc_protocol(void);
	virtual ~Airc_protocol(void);
	HANDLE	m_Airc_Thread;
	HANDLE m_hStopEvt;

public:
	bool FrameToBuffer();   //Ԥ�� ֡ת�����ַ���
	bool BufferToFrame();
	/**********************************************************************************
	Function 		: Airc_ctrl
	Description		: ִ�з���ĳһcommand
	Parameter		: 
		[IN]	 Cmd	 ������Ϣ
		[IN]	 p		���ò�������
	return			:  0 �ɹ�   -1ʧ��
	*************************************************************************************/
	UINT Airc_ctrl(int Cmd, void *p=NULL); 
	/**********************************************************************************
	Function 		: SetParaData
	Description		: ��Ҫ������������
	Parameter		: 
		[IN]	 type	 ������Ϣ
		[IN]	 para	 ������ֵ
	return			:  0 �ɹ��� -1ʧ��
	*************************************************************************************/
	UINT SetParaData(UCHAR type, CHAR para);
	/**********************************************************************************
	Function 		: SetFrame
	Description		: ���ý�Ҫ���͵�����֡��������һ����֡����
	Parameter		: 
		[IN]	 CID2	 ������Ϣ
		[IN]	 addr	 �豸��ַ
		[IN]	 type	 �������� ȡֵ 1������2�ػ� ���� ���ò���
		[IN]	 para	 type=��������Ҫ���ô���ֵ
		[IN]	 lenid	 ���ȱ�ʾ��ASCII����
	return			: 
	*************************************************************************************/
	void SetFrame(BYTE CID2, BYTE addr, BYTE type = 0, UINT para = 0, unsigned short lenid = 0);
	/**********************************************************************************
	Function 		: divdeFrame
	Description		: ���յ���������������ж���֤�����Ϣ�Ƿ���ȷ�����⴦����Ϣ
	Parameter		: 
		[IN]	  * buf	 ����ָ��
		[IN]	 length	 ���ݳ���
	return			: 0  �ɹ�   ���� ʧ��
	*************************************************************************************/
	BYTE divdeFrame(BYTE * buf, UINT length);  
	/**********************************************************************************
	Function 		: VerifyCheck
	Description		: ����У����
	Parameter		: 
		[IN]	  * buf	 ����ָ��
		[IN]	 length	 ���ݳ���
	return			: ��֡��У��
	*************************************************************************************/
	unsigned short VerifyCheck(BYTE *buf, UINT length);			   
	/**********************************************************************************
	Function 		: RTNstatus
	Description		: ������Ӧ֡����
	Parameter		: 
		[IN]	  rtn	 ��Ӧ����
	return			: m_RTNstatus ��ֵ
	*************************************************************************************/
	BYTE RTNstatus(BYTE rtn);				
	/**********************************************************************************
	Function 		: GetStatus
	Description		: ��ȡ������Ӧ״̬
	Parameter		: 
		[IN]	 
	return			: m_RTNstatus 
	*************************************************************************************/
	BYTE GetStatus();							
	/**********************************************************************************
	Function 		: GetLength
	Description		: ����֡��LENGHT�ֶ�
	Parameter		: 
		[IN]	  lenid	 ASCII����
	return			: LENGHTֵ
	*************************************************************************************/
	unsigned short GetLength(unsigned short lenid); //��ȡ֡�����ֶ�

	Airc_DataPack GetDataPack();   //��ȡ������
	/**********************************************************************************
	Function 		: GetLow4
	Description		: ��ȡ��4λ��ֵ
	Parameter		: 
		[IN]	  low	 ֵ
	return			: ��4λֵ
	*************************************************************************************/
	BYTE GetLow4(BYTE low);        
	BYTE GetHigh4(BYTE high);
	BYTE SetHigh4(BYTE high);
	BYTE SetLow4(BYTE low);
	/**********************************************************************************
	Function 		: AscToHex
	Description		: AsciiתHEx
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	unsigned char AscToHex(unsigned char aHex);  
	unsigned char HexToAsc(unsigned char aChar); //HexתAscii
	UINT convert(BYTE *sbuf, UINT length);
	BYTE* GetSendFrame();
	/**********************************************************************************
	Function 		: GetFrameLen
	Description		: ��ȡ��Ҫ���͵�֡�ĳ���
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
    UINT  GetFrameLen();

	BYTE *m_tempFrame;
	BYTE *m_sendFrame;
	BYTE *m_recvFrame;
	HANDLE m_comm;  //ͨ�ž��
	BYTE m_RTNstatus;
//	BYTE m_comtype;
	BYTE m_flag;
	UINT m_length;
	UINT m_success;
	BYTE m_varsion;
	BYTE m_address;

};
	//�յ�����֡�ṹ
	Airc_DataPack m_cmdata;
	//�յ�ң����Ϣ
	Airc_CONYC m_conyc;
	//�յ�������Ϣ
	Airc_Param m_airc_param;
	//�յ�����״̬��Ϣ
	Airc_Status	m_airc_status;
	//�յ�������Ϣ
	Airc_Warn m_airc_warn;
	//�յ���������
	devrun_times m_dev_times;
	//�յ�����ʱ��
	devrun_times m_dev_time;
