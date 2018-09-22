// ================================
// EvTrackCore.h
// ================================

#ifndef EV_AMT_TRACK_CORE_H
#define EV_AMT_TRACK_CORE_H

#include "CommonDef.h"
#include "TrackerModule.h"
#include "DetecterModule.h"
#include "ExpertsModule.h"

static const char g_EvTrackCoreVer[] = "ver.13.08.21.17";

class EvTrackCore
{
public:
	EvTrackCore(char* pChannelID=NULL);
	~EvTrackCore();

	bool  SetConfig(int iImageW, int iImageH, AmtParam* Param, CvRect* ROI=NULL);
	bool  Execute(IplImage* pFrame, long long TimeStamp, int iMotionDirect);
	int   CreateObject(AmtBbox InitBox, int iObjID, long long TimeStamp);
	int   CleanObject(int iObjID);

	int   GetObjUsedNum() {return m_UsedObjNum;}
	int   GetObjCurrNum() {return m_CurrObjNum;}
	void  DrawResult(AmtSingleObject* pObj, IplImage* pFrame);
	int   GetObjTrajectory(int iObjID, CvPoint** pPoints, int* pNum);
	IplImage* GetAlgoDraw() {return m_DrawImage;}

	int   CreateLogDir(AmtLogRecord* pLog, const char* pParentDir, const char* pChildDir=NULL, const char* pChannelDir=NULL);
	int   PrintfLog(AmtLogRecord* pLog, long long timeStamp, const char* pstrLog);
	int   ConfigLog(AmtLogRecord* pLog, bool bReSet);

protected:
	bool  SingleObjInit(IplImage* pFrame, AmtBbox InitBox, int iID, long long TimeStamp, AmtSingleObject* pObj);
	bool  SingleObjExecute(AmtSingleObject* pObj, long long TimeStamp);
	
	bool  TrackerExecute(AmtSingleObject* pObj, AmtBbox* BB, IplImage* pImgPrev, IplImage* pImgCurr);
	int   DetecterExecute(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur);
	void  LearnerExecute(AmtSingleObject* pObj, AmtTraceResult* pFrmTraceResult);

protected:
	void  SetObjParam(AmtBbox InitBox, int iID, AmtSingleObject* pObj);
	void  BboxClusterConfidence(AmtDetectResult* pDetectCluster, AmtDetectResult* pDetectResult);
	void  BboxOverlap(const AmtBbox& srcBB, AmtGridBbox* baseBB, int iBboxNum, float* pGridOverlap, AmtObjectROI* pObjROI=NULL);
	float BboxOverlapOne(const AmtBbox& srcBB, const AmtBbox& baseBB);
	int   BboxPartition(int* labels, const AmtBbox* vec, int num);
	AmtBbox GeneratePositiveData(AmtSingleObject* pObj, float* pGridOverlap, IplImage* pImg, IplImage* pImgBlur, AmtPpar pPar);
	void GenerateNegativeData(AmtSingleObject* pObj, float* pGridOverlap, IplImage* pImg, IplImage* pImgBlur);

protected:
	// 目标结果异常漂移验证
	bool  CheckTrackDrift(AmtBbox &TrackResult, CvRect &BoundRect);
	// 目标结果景深变化限制  
	void  ConstrainTrackResult(AmtTraceResult* pTrackResult, AmtTraceResult* pLastTrace, AmtTraceResult* pInitResult, int iConstrainType);
	// 目标结果纹理分析
	int   CheckTrackTextureType(AmtBbox &TrackResult);
	// 目标结果阴影校正
	void  CorrectShadowResult(AmtBbox &TrackResult);
	// 目标远近方向判断
	int   GetObjDirection(AmtTraceResult* pTraceResult, int iTraceNum);
	
public:
	bool m_IsInit;          // 初始化标识
	int m_MaxiObjNum;       // 最大跟踪上限个数
	int m_UsedObjNum;       // 最大历史跟踪个数
	int m_CurrObjNum;       // 当前跟踪目标个数
	AmtSingleObject* m_ObjectSets;
	AmtExternMode m_ExternControl;
	AmtParam m_AlgoParam;
	char m_WinName[256];
	CvPoint m_ObjTraj[RECORD_POINTS_NUM];  //目标轨迹,供查询特定目标轨迹时填充
	char m_szChannelID[256];  // 通道号

protected:
	// ------------------
	AmtModel m_Model;
	AmtPpar m_PparInit;
	AmtPpar m_PparUpdata;
	AmtNpar m_Npar;

	// --- 模块 ---
	AmtTrackerModule  m_TrackerModule;  // 跟踪模块
	AmtDetecterModule m_DetecterModule; // 检测模块
//	AmtLearnerModule  m_LearnerModule;  // 学习模块
	AmtExpertsModule  m_ExpertsModule;  // 专家集模块

	// --- 成员资源 ---
	CvSize m_SrcImgSize;
	CvSize m_ImageSize;
	IplImage* m_DrawImage;
	IplImage* m_CurrImage;
	IplImage* m_PrevImage;
	IplImage* m_BlurImage;

//	AmtTrackResult   m_TrackResult;    // 跟踪模块输出
//	AmtDetectResult  m_DetectResult;   // 检测模块输出
//	AmtDetectResult  m_DetectCluster;  // 检测输出聚类
	AmtDetector     *m_DetectFilter;   // 检测模块中间结果
	int              m_DetectFiltNum;

	// 信息输出
	CvFont m_Font;
	char m_Text[256];

	// 时间戳
	long long m_CurrTimeStamp;
	long long m_LastTimeStamp;	
	int       m_iTimeInterval;

	// 计时
	clock_t m_AlgoBeginTime;
	clock_t m_AlgoEndinTime;
	long    m_AlgoUsedTime;
	long    m_AlgoShowTime;

	// 帧率统计
	int     m_iFrameRateCount;
	clock_t m_cFrameRateTime;

	// 日志记录
	AmtLogRecord mLogRecord;

private:
};

#endif



