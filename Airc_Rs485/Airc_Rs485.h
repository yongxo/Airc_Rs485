// Airc_Rs485.h : Airc_Rs485 DLL ����ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"

// CAirc_Rs485App
// �йش���ʵ�ֵ���Ϣ������� Airc_Rs485.cpp
//

class CAirc_Rs485App : public CWinApp
{
public:
	CAirc_Rs485App();

// ��д
public:
	virtual BOOL InitInstance();

	DECLARE_MESSAGE_MAP()
};

