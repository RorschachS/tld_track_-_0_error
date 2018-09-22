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
	// Ŀ�����쳣Ư����֤
	bool  CheckTrackDrift(AmtBbox &TrackResult, CvRect &BoundRect);
	// Ŀ��������仯����  
	void  ConstrainTrackResult(AmtTraceResult* pTrackResult, AmtTraceResult* pLastTrace, AmtTraceResult* pInitResult, int iConstrainType);
	// Ŀ�����������
	int   CheckTrackTextureType(AmtBbox &TrackResult);
	// Ŀ������ӰУ��
	void  CorrectShadowResult(AmtBbox &TrackResult);
	// Ŀ��Զ�������ж�
	int   GetObjDirection(AmtTraceResult* pTraceResult, int iTraceNum);
	
public:
	bool m_IsInit;          // ��ʼ����ʶ
	int m_MaxiObjNum;       // ���������޸���
	int m_UsedObjNum;       // �����ʷ���ٸ���
	int m_CurrObjNum;       // ��ǰ����Ŀ�����
	AmtSingleObject* m_ObjectSets;
	AmtExternMode m_ExternControl;
	AmtParam m_AlgoParam;
	char m_WinName[256];
	CvPoint m_ObjTraj[RECORD_POINTS_NUM];  //Ŀ��켣,����ѯ�ض�Ŀ��켣ʱ���
	char m_szChannelID[256];  // ͨ����

protected:
	// ------------------
	AmtModel m_Model;
	AmtPpar m_PparInit;
	AmtPpar m_PparUpdata;
	AmtNpar m_Npar;

	// --- ģ�� ---
	AmtTrackerModule  m_TrackerModule;  // ����ģ��
	AmtDetecterModule m_DetecterModule; // ���ģ��
//	AmtLearnerModule  m_LearnerModule;  // ѧϰģ��
	AmtExpertsModule  m_ExpertsModule;  // ר�Ҽ�ģ��

	// --- ��Ա��Դ ---
	CvSize m_SrcImgSize;
	CvSize m_ImageSize;
	IplImage* m_DrawImage;
	IplImage* m_CurrImage;
	IplImage* m_PrevImage;
	IplImage* m_BlurImage;

//	AmtTrackResult   m_TrackResult;    // ����ģ�����
//	AmtDetectResult  m_DetectResult;   // ���ģ�����
//	AmtDetectResult  m_DetectCluster;  // ����������
	AmtDetector     *m_DetectFilter;   // ���ģ���м���
	int              m_DetectFiltNum;

	// ��Ϣ���
	CvFont m_Font;
	char m_Text[256];

	// ʱ���
	long long m_CurrTimeStamp;
	long long m_LastTimeStamp;	
	int       m_iTimeInterval;

	// ��ʱ
	clock_t m_AlgoBeginTime;
	clock_t m_AlgoEndinTime;
	long    m_AlgoUsedTime;
	long    m_AlgoShowTime;

	// ֡��ͳ��
	int     m_iFrameRateCount;
	clock_t m_cFrameRateTime;

	// ��־��¼
	AmtLogRecord mLogRecord;

private:
};

#endif



