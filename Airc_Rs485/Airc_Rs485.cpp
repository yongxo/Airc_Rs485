// Airc_Rs485.cpp : ���� DLL �ĳ�ʼ�����̡�
//

#include "stdafx.h"
#include "Airc_Rs485.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//
//TODO: ����� DLL ����� MFC DLL �Ƕ�̬���ӵģ�
//		��Ӵ� DLL �������κε���
//		MFC �ĺ������뽫 AFX_MANAGE_STATE ����ӵ�
//		�ú�������ǰ�档
//
//		����:
//
//		extern "C" BOOL PASCAL EXPORT ExportedFunction()
//		{
//			AFX_MANAGE_STATE(AfxGetStaticModuleState());
//			// �˴�Ϊ��ͨ������
//		}
//
//		�˺������κ� MFC ����
//		������ÿ��������ʮ����Ҫ������ζ��
//		��������Ϊ�����еĵ�һ�����
//		���֣������������ж������������
//		������Ϊ���ǵĹ��캯���������� MFC
//		DLL ���á�
//
//		�й�������ϸ��Ϣ��
//		����� MFC ����˵�� 33 �� 58��
//


// CAirc_Rs485App

BEGIN_MESSAGE_MAP(CAirc_Rs485App, CWinApp)
END_MESSAGE_MAP()


// CAirc_Rs485App ����

CAirc_Rs485App::CAirc_Rs485App()
{
	// TODO: �ڴ˴���ӹ�����룬
	// ��������Ҫ�ĳ�ʼ�������� InitInstance ��
}


// Ψһ��һ�� CAirc_Rs485App ����

CAirc_Rs485App theApp;


// CAirc_Rs485App ��ʼ��

BOOL CAirc_Rs485App::InitInstance()
{
	CWinApp::InitInstance();

	return TRUE;
}
