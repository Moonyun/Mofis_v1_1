#pragma once
#include <functional>


#ifdef FLOWCAM_EXPORTS
#define FLOWCAM_EXPORTS __declspec(dllexport)
#else
#define FLOWCAM_EXPORTS __declspec(dllimport)
#endif

class FLOWCAM_EXPORTS IInterface
{
protected:
	IInterface();

	virtual ~IInterface();

public:
	static IInterface* CreateInstance();

	static void DestoryInstance(IInterface* pIShowUI);

	//初始化
	virtual void Init() = 0;

	//设置状态等的回调函数
	//virtual void SetInitStateCallback(function<void(int, int, int, int)>* pCallback) = 0;
	virtual void SetInitStateCallback(void(*pCallback)(int, int, int, int)) = 0;

	//开始测量
	virtual bool CollectStart(long i_Volume, long i_FlowRate, long i_PlateType,
		long i_RowIndex, long i_ColumnIndex) = 0;

	/// <summary>
	/// 停止测量
	/// </summary>
	/// <returns></returns>
	virtual bool StopCollect() = 0;

	/// <summary>
	/// 维护
	/// </summary>
	/// <returns></returns>
	virtual bool Maintain(int functionType) = 0;

	/// <summary>
	/// 自检
	/// </summary>
	/// <returns></returns>
	virtual bool SelfCheck(int functionType) = 0;

	/// <summary>
	/// 关闭仪器
	/// </summary>
	/// <returns></returns>
	virtual bool ShutDown() = 0;

	/// <summary>
	/// 开关仓门
	/// </summary>
	/// <returns></returns>
	virtual bool OpenCloseDoor() = 0;
};

