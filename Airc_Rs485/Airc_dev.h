#pragma once

class Airc_dev
{
public:
	Airc_dev(void);
	virtual ~Airc_dev(void);
protected:
	DCB dcb;						/* 串口参数结构体 */
	HANDLE m_hComm;					/* 串口操作句柄 */
	//HANDLE m_ExitThreadEvent;		/* 串口接收线程退出事件 */
	
	BOOL ClosePort(void);			/* 关闭串口 */

	// 打开串口 
	BOOL OpenPort(LPCTSTR Port, int BaudRate, int DataBits, int StopBits, int Parity);

	// 串口接收线程
	//static DWORD CommRecvTread(LPVOID lparam);
public:
	// 串口发送
	BOOL CommSend(BYTE* send_buf,int datalen);
	BOOL CommRecv(BYTE* recv_buf,int datalen);
    BOOL OnOpenCom(LPCTSTR Port, int BaudRate, int DataBits, int StopBits, int Parity);
    void OnCloseCom();
	// 串口接收数据成功回调函数
	//typedef void (CALLBACK *ONCOMMRECV)(CWnd* pWnd, char *buf, int buflen);
	//static void CALLBACK OnCommRecv(CWnd* pWnd, char *buf, int buflen);

	HANDLE m_StartSendvent;		    /* 串口开始发送事件 */
	

};

#define AIRC_RECV_SIZE 200
#define AIRC_SEND_SIZE 50

extern	BYTE recvBuf[AIRC_RECV_SIZE];
extern	BYTE sendBuf[AIRC_SEND_SIZE];