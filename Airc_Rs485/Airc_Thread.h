#pragma once

#include "Air_Conditioner_Ctrl.h"


struct Airc_DataPack   //用于描述空调RS485协议格式
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

struct CMD_TYPE   //预留
{
	BYTE COMMAND_GROUP;
	BYTE COMMAND_ID;
	BYTE COMMAND_TYPE;  //10H 空调开机  1F空调关机
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
	bool FrameToBuffer();   //预留 帧转换成字符串
	bool BufferToFrame();
	/**********************************************************************************
	Function 		: Airc_ctrl
	Description		: 执行发送某一command
	Parameter		: 
		[IN]	 Cmd	 命令信息
		[IN]	 p		设置参数数据
	return			:  0 成功   -1失败
	*************************************************************************************/
	UINT Airc_ctrl(int Cmd, void *p=NULL); 
	/**********************************************************************************
	Function 		: SetParaData
	Description		: 将要设置命令类型
	Parameter		: 
		[IN]	 type	 命令信息
		[IN]	 para	 该命令值
	return			:  0 成功， -1失败
	*************************************************************************************/
	UINT SetParaData(UCHAR type, CHAR para);
	/**********************************************************************************
	Function 		: SetFrame
	Description		: 设置将要发送的命令帧，完成组成一完整帧功能
	Parameter		: 
		[IN]	 CID2	 命令信息
		[IN]	 addr	 设备地址
		[IN]	 type	 命令类型 取值 1开机，2关机 其它 设置参数
		[IN]	 para	 type=其它，需要设置此项值
		[IN]	 lenid	 长度标示码ASCII长度
	return			: 
	*************************************************************************************/
	void SetFrame(BYTE CID2, BYTE addr, BYTE type = 0, UINT para = 0, unsigned short lenid = 0);
	/**********************************************************************************
	Function 		: divdeFrame
	Description		: 接收到数后在这里进行判断验证码等信息是否正确，正解处理信息
	Parameter		: 
		[IN]	  * buf	 数据指针
		[IN]	 length	 数据长度
	return			: 0  成功   其它 失败
	*************************************************************************************/
	BYTE divdeFrame(BYTE * buf, UINT length);  
	/**********************************************************************************
	Function 		: VerifyCheck
	Description		: 生成校验码
	Parameter		: 
		[IN]	  * buf	 数据指针
		[IN]	 length	 数据长度
	return			: 此帧的校验
	*************************************************************************************/
	unsigned short VerifyCheck(BYTE *buf, UINT length);			   
	/**********************************************************************************
	Function 		: RTNstatus
	Description		: 返回响应帧错误
	Parameter		: 
		[IN]	  rtn	 响应数据
	return			: m_RTNstatus 的值
	*************************************************************************************/
	BYTE RTNstatus(BYTE rtn);				
	/**********************************************************************************
	Function 		: GetStatus
	Description		: 获取命令响应状态
	Parameter		: 
		[IN]	 
	return			: m_RTNstatus 
	*************************************************************************************/
	BYTE GetStatus();							
	/**********************************************************************************
	Function 		: GetLength
	Description		: 生成帧的LENGHT字段
	Parameter		: 
		[IN]	  lenid	 ASCII长度
	return			: LENGHT值
	*************************************************************************************/
	unsigned short GetLength(unsigned short lenid); //获取帧长度字段

	Airc_DataPack GetDataPack();   //获取包数据
	/**********************************************************************************
	Function 		: GetLow4
	Description		: 获取低4位的值
	Parameter		: 
		[IN]	  low	 值
	return			: 低4位值
	*************************************************************************************/
	BYTE GetLow4(BYTE low);        
	BYTE GetHigh4(BYTE high);
	BYTE SetHigh4(BYTE high);
	BYTE SetLow4(BYTE low);
	/**********************************************************************************
	Function 		: AscToHex
	Description		: Ascii转HEx
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
	unsigned char AscToHex(unsigned char aHex);  
	unsigned char HexToAsc(unsigned char aChar); //Hex转Ascii
	UINT convert(BYTE *sbuf, UINT length);
	BYTE* GetSendFrame();
	/**********************************************************************************
	Function 		: GetFrameLen
	Description		: 获取将要发送的帧的长度
	Parameter		: 
		[IN]	  aHex	 
	return			: 
	*************************************************************************************/
    UINT  GetFrameLen();

	BYTE *m_tempFrame;
	BYTE *m_sendFrame;
	BYTE *m_recvFrame;
	HANDLE m_comm;  //通信句柄
	BYTE m_RTNstatus;
//	BYTE m_comtype;
	BYTE m_flag;
	UINT m_length;
	UINT m_success;
	BYTE m_varsion;
	BYTE m_address;

};
	//空调数据帧结构
	Airc_DataPack m_cmdata;
	//空调遥测信息
	Airc_CONYC m_conyc;
	//空调参数信息
	Airc_Param m_airc_param;
	//空调运行状态信息
	Airc_Status	m_airc_status;
	//空调报警信息
	Airc_Warn m_airc_warn;
	//空调动作次数
	devrun_times m_dev_times;
	//空调运行时间
	devrun_times m_dev_time;
