#pragma once

class Airc_dev
{
public:
	Airc_dev(void);
	virtual ~Airc_dev(void);
protected:
	DCB dcb;						/* ���ڲ����ṹ�� */
	HANDLE m_hComm;					/* ���ڲ������ */
	//HANDLE m_ExitThreadEvent;		/* ���ڽ����߳��˳��¼� */
	
	BOOL ClosePort(void);			/* �رմ��� */

	// �򿪴��� 
	BOOL OpenPort(LPCTSTR Port, int BaudRate, int DataBits, int StopBits, int Parity);

	// ���ڽ����߳�
	//static DWORD CommRecvTread(LPVOID lparam);
public:
	// ���ڷ���
	BOOL CommSend(BYTE* send_buf,int datalen);
	BOOL CommRecv(BYTE* recv_buf,int datalen);
    BOOL OnOpenCom(LPCTSTR Port, int BaudRate, int DataBits, int StopBits, int Parity);
    void OnCloseCom();
	// ���ڽ������ݳɹ��ص�����
	//typedef void (CALLBACK *ONCOMMRECV)(CWnd* pWnd, char *buf, int buflen);
	//static void CALLBACK OnCommRecv(CWnd* pWnd, char *buf, int buflen);

	HANDLE m_StartSendvent;		    /* ���ڿ�ʼ�����¼� */
	

};

#define AIRC_RECV_SIZE 200
#define AIRC_SEND_SIZE 50

extern	BYTE recvBuf[AIRC_RECV_SIZE];
extern	BYTE sendBuf[AIRC_SEND_SIZE];