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
函数名称: Airc_dev::ClosePort
描    述: 关闭串口
输入参数: 无
输出参数: 无
返    回: FALSE: 失败;    TRUE: 成功
********************************************************************************************/
BOOL Airc_dev::ClosePort(void)
{
	if(m_hComm != INVALID_HANDLE_VALUE)
	{
		SetCommMask(m_hComm, 0);		
		PurgeComm(m_hComm, PURGE_TXCLEAR | PURGE_RXCLEAR);	/* 清除收/发缓冲 */
		CloseHandle(m_hComm);								/* 关闭串口操作句柄 */
		m_hComm = INVALID_HANDLE_VALUE;
		return TRUE;
	}

	return FALSE;
}

/*******************************************************************************************
函数名称: Airc_dev::OpenPort
描    述: 打开串口
输入参数: LPCTSTR Port: 串口名,如"COM0:","COM1:"
		  int BaudRate: 波特率
		  int DataBits: 数据位, 取值为7或8
		  int StopBits: 停止位
		  int Parity  : 奇偶校验位
输出参数: 无
返    回: FALSE: 失败;    TRUE: 成功
********************************************************************************************/
BOOL Airc_dev::OpenPort(LPCTSTR Port, int BaudRate, int DataBits, int StopBits, int Parity)
{
	COMMTIMEOUTS CommTimeOuts;

	// 打开串口
	m_hComm = CreateFile(Port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	if(m_hComm == INVALID_HANDLE_VALUE)
	{
		//MessageBox(_T("无法打开端口或端口已打开!请检查是否已被占用."));
		return FALSE;
	}

	GetCommState(m_hComm, &dcb);						/* 读取串口的DCB */
	dcb.BaudRate = BaudRate;			
	dcb.ByteSize = DataBits;
	dcb.Parity = Parity;
	dcb.StopBits = StopBits;	
	dcb.fParity = FALSE;								/* 禁止奇偶校验 */
	dcb.fBinary = TRUE;
	dcb.fDtrControl = 0;								/* 禁止流量控制 */
	dcb.fRtsControl = 0;
	dcb.fOutX = 0;
	dcb.fInX = 0;
	dcb.fTXContinueOnXoff = 0;
	
	//设置状态参数
	SetCommMask(m_hComm, EV_RXCHAR);					/* 串口事件:接收到一个字符 */	
	SetupComm(m_hComm, 16384, 16384);					/* 设置接收与发送的缓冲区大小,wince下为空调用 */
	if(!SetCommState(m_hComm, &dcb))					/* 设置串口的DCB */
	{
		//MessageBox(_T("无法按当前参数配置端口，请检查参数!"));
		ClosePort();
		return FALSE;
	}
		
	//设置超时参数
	GetCommTimeouts(m_hComm, &CommTimeOuts);		
	CommTimeOuts.ReadIntervalTimeout = 100;				/* 接收字符间最大时间间隔 */
	CommTimeOuts.ReadTotalTimeoutMultiplier = 1;		
	CommTimeOuts.ReadTotalTimeoutConstant = 100;		/* 读数据总超时常量 */
	CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
	CommTimeOuts.WriteTotalTimeoutConstant = 0;		
	if(!SetCommTimeouts(m_hComm, &CommTimeOuts))
	{
		//MessageBox(_T("无法设置超时参数!"));
		ClosePort();
		return FALSE;
	}
		
	PurgeComm(m_hComm, PURGE_TXCLEAR | PURGE_RXCLEAR);	 /* 清除收/发缓冲区 */		
	return TRUE;		
}

/*******************************************************************************************
函数名称: Airc_dev::CommRecv
描    述: 串口接收线程
输入参数: LPVOID lparam: 线程参数,创建线程时传入
输出参数: 无
返    回: 0: 线程退出, 返回值没特殊含义
********************************************************************************************/
BOOL Airc_dev::CommRecv(BYTE * recv_buf, int datalen)
{
	DWORD dwLength;
	
#if 0
	Airc_dev *pDlg = (Airc_dev*)lparam;

	while(TRUE)
	{																/* 等待线程退出事件 */
		if (WaitForSingleObject(pDlg->m_ExitThreadEvent, 0) == WAIT_OBJECT_0)
			break;	

		if (pDlg->m_hComm != INVALID_HANDLE_VALUE)
		{															/* 从串口读取数据 */
			BOOL fReadState = ReadFile(pDlg->m_hComm, recvBuf, 1024, &dwLength, NULL);
			if(!fReadState)
			{
				//MessageBox(_T("无法从串口读取数据!"));
			}
			else
			{
				if(dwLength != 0)
					OnCommRecv(NULL, recvBuf, dwLength);			/* 接收成功调用回调函数 */
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
		//MessageBox(_T("无法从串口读取数据!"));
		return FALSE;
	}
	else
	{
		return TRUE;
	}

}


/*******************************************************************************************
函数名称: Airc_dev::CommSend
描    述: 串口发送函数
输入参数: char* send_buf: 发送缓冲区首地址
          int datalen: 发送数据长度
输出参数: 无
返    回: 发送成功
          发送失败
********************************************************************************************/
BOOL Airc_dev::CommSend(BYTE* send_buf,int datalen)
{
	DWORD dwactlen;

	if((m_hComm == INVALID_HANDLE_VALUE)||(send_buf == NULL))
	{
		//MessageBox(_T("串口未打开!"));
		return FALSE;
	}

	WriteFile(m_hComm, send_buf, datalen, &dwactlen, NULL);	 /* 从串口发送数据 */
	return TRUE;
}

/*******************************************************************************************
函数名称: Airc_dev::OnOpenCom
描    述: "打开端口" 
输入参数: 无
输出参数: 无
返    回: 无
********************************************************************************************/
BOOL Airc_dev::OnOpenCom(LPCTSTR Port, int BaudRate, int DataBits, int StopBits, int Parity) 
{
	DWORD IDThread;
	//HANDLE hRecvThread;												/* 接收线程句柄 */

	BOOL ret = OpenPort(Port, BaudRate, DataBits, StopBits, Parity);	/* 打开串口 */
	if (ret == FALSE)
		return FALSE;

	//m_ExitThreadEvent = CreateEvent(NULL, TRUE, FALSE, NULL);		/* 创建串口接收线程退出事件*/

	#if 0
	// 创建串口接收线程
	hRecvThread = CreateThread(0, 0, CommRecvTread, this, 0, &IDThread);
	if (hRecvThread == NULL) 
	{
		return FALSE;
	}
	#endif
	return TRUE;
}


/*******************************************************************************************
函数名称: Airc_dev::OnCloseCom
描    述: "关闭端口" 
输入参数: 无
输出参数: 无
返    回: 无
********************************************************************************************/
void Airc_dev::OnCloseCom() 
{
	#if 0
	if (m_ExitThreadEvent != NULL)
	{
		SetEvent(m_ExitThreadEvent);					/* 通知线程退出 */
		Sleep(1000);
		CloseHandle(m_ExitThreadEvent);
		m_ExitThreadEvent = NULL;
	}
	#endif
	ClosePort();
}