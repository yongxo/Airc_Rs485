// Airc_Rs485.h : Airc_Rs485 DLL 的主头文件
//

#pragma once

#ifndef __AFXWIN_H__
	#error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif

#include "resource.h"

// CAirc_Rs485App
// 有关此类实现的信息，请参阅 Airc_Rs485.cpp
//

class CAirc_Rs485App : public CWinApp
{
public:
	CAirc_Rs485App();

// 重写
public:
	virtual BOOL InitInstance();

	DECLARE_MESSAGE_MAP()
};

