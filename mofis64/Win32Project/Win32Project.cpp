// Win32Project.cpp : 定义应用程序的入口点。
//

#include "stdafx.h"
#include "Win32Project.h"
#include "..\FlowCam\IInterface.h"

IInterface* m_pShowUI = NULL;
#define MAX_LOADSTRING 100

void UpdateInstateCallback(int functionType, int para1, int para2, int para3)
{
	int i = 0;
}

void TestInit()
{
	::OutputDebugStringA("#22 TestInit.\n");

	IInterface* pShowUI = IInterface::CreateInstance();
	m_pShowUI = pShowUI;

	pShowUI->SetInitStateCallback(&UpdateInstateCallback);//设置事件响应回调

	//初始化
	pShowUI->Init();

}

// 全局变量: 


int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPTSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
	MSG msg;
	HACCEL hAccelTable;

	TestInit();
	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_WIN32PROJECT));

	// 主消息循环: 
	while (true)
	{
		if (GetMessage(&msg, NULL, 0, 0) && !TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}
}


