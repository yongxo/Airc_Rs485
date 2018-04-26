#include "StdAfx.h"
#include "Airc_dev.h"

Airc_dev::Airc_dev(void)
{
}

Airc_dev::~Airc_dev(void)
{
}

BYTE recvBuf[200];
BYTE sendBuf[50];

/*******************************************************************************************
��������: Airc_dev::ClosePort
��    ��: �رմ���
�������: ��
�������: ��
��    ��: FALSE: ʧ��;    TRUE: �ɹ�
********************************************************************************************/
BOOL Airc_dev::ClosePort(void)
{
	if(m_hComm != INVALID_HANDLE_VALUE)
	{
		SetCommMask(m_hComm, 0);		
		PurgeComm(m_hComm, PURGE_TXCLEAR | PURGE_RXCLEAR);	/* �����/������ */
		CloseHandle(m_hComm);								/* �رմ��ڲ������ */
		m_hComm = INVALID_HANDLE_VALUE;
		return TRUE;
	}

	return FALSE;
}

/*******************************************************************************************
��������: Airc_dev::OpenPort
��    ��: �򿪴���
�������: LPCTSTR Port: ������,��"COM0:","COM1:"
		  int BaudRate: ������
		  int DataBits: ����λ, ȡֵΪ7��8
		  int StopBits: ֹͣλ
		  int Parity  : ��żУ��λ
�������: ��
��    ��: FALSE: ʧ��;    TRUE: �ɹ�
********************************************************************************************/
BOOL Airc_dev::OpenPort(LPCTSTR Port, int BaudRate, int DataBits, int StopBits, int Parity)
{
	COMMTIMEOUTS CommTimeOuts;

	// �򿪴���
	m_hComm = CreateFile(Port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	if(m_hComm == INVALID_HANDLE_VALUE)
	{
		//MessageBox(_T("�޷��򿪶˿ڻ�˿��Ѵ�!�����Ƿ��ѱ�ռ��."));
		return FALSE;
	}

	GetCommState(m_hComm, &dcb);						/* ��ȡ���ڵ�DCB */
	dcb.BaudRate = BaudRate;			
	dcb.ByteSize = DataBits;
	dcb.Parity = Parity;
	dcb.StopBits = StopBits;	
	dcb.fParity = FALSE;								/* ��ֹ��żУ�� */
	dcb.fBinary = TRUE;
	dcb.fDtrControl = 0;								/* ��ֹ�������� */
	dcb.fRtsControl = 0;
	dcb.fOutX = 0;
	dcb.fInX = 0;
	dcb.fTXContinueOnXoff = 0;
	
	//����״̬����
	SetCommMask(m_hComm, EV_RXCHAR);					/* �����¼�:���յ�һ���ַ� */	
	SetupComm(m_hComm, 16384, 16384);					/* ���ý����뷢�͵Ļ�������С,wince��Ϊ�յ��� */
	if(!SetCommState(m_hComm, &dcb))					/* ���ô��ڵ�DCB */
	{
		//MessageBox(_T("�޷�����ǰ�������ö˿ڣ��������!"));
		ClosePort();
		return FALSE;
	}
		
	//���ó�ʱ����
	GetCommTimeouts(m_hComm, &CommTimeOuts);		
	CommTimeOuts.ReadIntervalTimeout = 100;				/* �����ַ������ʱ���� */
	CommTimeOuts.ReadTotalTimeoutMultiplier = 1;		
	CommTimeOuts.ReadTotalTimeoutConstant = 100;		/* �������ܳ�ʱ���� */
	CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
	CommTimeOuts.WriteTotalTimeoutConstant = 0;		
	if(!SetCommTimeouts(m_hComm, &CommTimeOuts))
	{
		//MessageBox(_T("�޷����ó�ʱ����!"));
		ClosePort();
		return FALSE;
	}
		
	PurgeComm(m_hComm, PURGE_TXCLEAR | PURGE_RXCLEAR);	 /* �����/�������� */		
	return TRUE;		
}

/*******************************************************************************************
��������: Airc_dev::CommRecv
��    ��: ���ڽ����߳�
�������: LPVOID lparam: �̲߳���,�����߳�ʱ����
�������: ��
��    ��: 0: �߳��˳�, ����ֵû���⺬��
********************************************************************************************/
BOOL Airc_dev::CommRecv(BYTE * recv_buf, int datalen)
{
	DWORD dwLength;
	
#if 0
	Airc_dev *pDlg = (Airc_dev*)lparam;

	while(TRUE)
	{																/* �ȴ��߳��˳��¼� */
		if (WaitForSingleObject(pDlg->m_ExitThreadEvent, 0) == WAIT_OBJECT_0)
			break;	

		if (pDlg->m_hComm != INVALID_HANDLE_VALUE)
		{															/* �Ӵ��ڶ�ȡ���� */
			BOOL fReadState = ReadFile(pDlg->m_hComm, recvBuf, 1024, &dwLength, NULL);
			if(!fReadState)
			{
				//MessageBox(_T("�޷��Ӵ��ڶ�ȡ����!"));
			}
			else
			{
				if(dwLength != 0)
					OnCommRecv(NULL, recvBuf, dwLength);			/* ���ճɹ����ûص����� */
			}
		}
	}		
#endif
	BOOL fReadState;
	if(m_hComm !=INVALID_HANDLE_VALUE)
	{
		fReadState = ReadFile(m_hComm, recv_buf, datalen, &dwLength, NULL);
	}
	if(!fReadState)
	{
		//MessageBox(_T("�޷��Ӵ��ڶ�ȡ����!"));
		return FALSE;
	}
	else
	{
		return TRUE;
	}

}


/*******************************************************************************************
��������: Airc_dev::CommSend
��    ��: ���ڷ��ͺ���
�������: char* send_buf: ���ͻ������׵�ַ
          int datalen: �������ݳ���
�������: ��
��    ��: ���ͳɹ�
          ����ʧ��
********************************************************************************************/
BOOL Airc_dev::CommSend(BYTE* send_buf,int datalen)
{
	DWORD dwactlen;

	if((m_hComm == INVALID_HANDLE_VALUE)||(send_buf == NULL))
	{
		//MessageBox(_T("����δ��!"));
		return FALSE;
	}

	WriteFile(m_hComm, send_buf, datalen, &dwactlen, NULL);	 /* �Ӵ��ڷ������� */
	return TRUE;
}

/*******************************************************************************************
��������: Airc_dev::OnOpenCom
��    ��: "�򿪶˿�" 
�������: ��
�������: ��
��    ��: ��
********************************************************************************************/
BOOL Airc_dev::OnOpenCom(LPCTSTR Port, int BaudRate, int DataBits, int StopBits, int Parity) 
{
	DWORD IDThread;
	//HANDLE hRecvThread;												/* �����߳̾�� */

	BOOL ret = OpenPort(Port, BaudRate, DataBits, StopBits, Parity);	/* �򿪴��� */
	if (ret == FALSE)
		return FALSE;

	//m_ExitThreadEvent = CreateEvent(NULL, TRUE, FALSE, NULL);		/* �������ڽ����߳��˳��¼�*/

	#if 0
	// �������ڽ����߳�
	hRecvThread = CreateThread(0, 0, CommRecvTread, this, 0, &IDThread);
	if (hRecvThread == NULL) 
	{
		return FALSE;
	}
	#endif
	return TRUE;
}


/*******************************************************************************************
��������: Airc_dev::OnCloseCom
��    ��: "�رն˿�" 
�������: ��
�������: ��
��    ��: ��
********************************************************************************************/
void Airc_dev::OnCloseCom() 
{
	#if 0
	if (m_ExitThreadEvent != NULL)
	{
		SetEvent(m_ExitThreadEvent);					/* ֪ͨ�߳��˳� */
		Sleep(1000);
		CloseHandle(m_ExitThreadEvent);
		m_ExitThreadEvent = NULL;
	}
	#endif
	ClosePort();
}