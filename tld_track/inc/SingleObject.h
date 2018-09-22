// ==============================
// SingleObject.h
// ==============================

#ifndef AMT_SINGLE_OBJECT_H
#define AMT_SINGLE_OBJECT_H

#include "CommonDef.h"

//**************************************************************
class AmtSingleObject
{
public:
	AmtSingleObject();
	~AmtSingleObject();

	void Release(void);
	void Clean(void);

	bool SetMotionROI(long long TimeStamp); // 输入当前帧时间戳

	bool m_IsInit;        // 是否有过初始化
	bool m_IsActive;      // 是否当前激活

	// --------------- Param ---------------------------
	int        m_ID;           // 目标ID
	int        m_iIndex;       // 目标数组索引
	CvScalar   m_ColorTag;     // 目标颜色标签
	AmtBbox    m_InitBox;
	AmtParam   m_ObjParam;	   // 算法参数
	AmtModel   m_Model;
	AmtPpar    m_PparInit;
	AmtPpar    m_PparUpdata;
	AmtNpar    m_Npar;

	// ***************  public  ********************
	AmtBbox         mOutPut;
	AmtObjectROI    mAnalyROI;
	int             mCurrCountMiss;  // 当前运动轨迹连续丢失计数
	int             mUnstableNum;    // 当前运动轨迹不稳定点计数
	bool            mUnstable;
	AmtTraceResult  mTraceResult[RECORD_POINTS_NUM];   // 目标运动轨迹
	int             mTraceNum;

	// ***************  protected  ********************
	AmtTrackResult   m_TrackResult;    // 跟踪模块输出
	AmtDetectResult  m_DetectResult;   // 检测模块输出
	AmtDetectResult  m_DetectCluster;  // 检测输出聚类
//	AmtDetector     *m_DetectFilter;   // 检测模块中间结果
	int              m_DetectFiltNum;

	// ---- for detector ----
	int      m_TreesNum;
	int      m_FeatsNum;
	int*     m_BboxOffSets;
	int      m_BboxOffSetsNum;
	int      m_MaxBboxOffSets;
	int*     m_FeatOffSets;
	int      m_FeatOffSetsNum;
 	int      m_MaxFeatOffSets;
	float**  m_ForestWeight;
	int**    m_CountP;
	int**    m_CountN;
	int      m_FeatsBitPerTree;


	// ***************  protected  ********************
	CvSize       m_ImgSize; 
	float        m_VarValue;
	AmtGridBbox* mGridBboxData;
	int          mGridBboxNum;
	int          mMaxGridBboxNum;
	CvSize*      mScaleSizeData;
	int          mScaleSizeNum;
	int          mMaxScaleSizeNum;

	// ---- Patch Examples For Experts----
	float*  mPatchExamplesP;
	int     mPatchNumP;
	int     mMaxPatchNumP;
	float*  mPatchExamplesN;
	int     mPatchNumN;
	int     mMaxPatchNumN;
};

#endif

