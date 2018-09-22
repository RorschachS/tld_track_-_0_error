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

	bool SetMotionROI(long long TimeStamp); // ���뵱ǰ֡ʱ���

	bool m_IsInit;        // �Ƿ��й���ʼ��
	bool m_IsActive;      // �Ƿ�ǰ����

	// --------------- Param ---------------------------
	int        m_ID;           // Ŀ��ID
	int        m_iIndex;       // Ŀ����������
	CvScalar   m_ColorTag;     // Ŀ����ɫ��ǩ
	AmtBbox    m_InitBox;
	AmtParam   m_ObjParam;	   // �㷨����
	AmtModel   m_Model;
	AmtPpar    m_PparInit;
	AmtPpar    m_PparUpdata;
	AmtNpar    m_Npar;

	// ***************  public  ********************
	AmtBbox         mOutPut;
	AmtObjectROI    mAnalyROI;
	int             mCurrCountMiss;  // ��ǰ�˶��켣������ʧ����
	int             mUnstableNum;    // ��ǰ�˶��켣���ȶ������
	bool            mUnstable;
	AmtTraceResult  mTraceResult[RECORD_POINTS_NUM];   // Ŀ���˶��켣
	int             mTraceNum;

	// ***************  protected  ********************
	AmtTrackResult   m_TrackResult;    // ����ģ�����
	AmtDetectResult  m_DetectResult;   // ���ģ�����
	AmtDetectResult  m_DetectCluster;  // ����������
//	AmtDetector     *m_DetectFilter;   // ���ģ���м���
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

