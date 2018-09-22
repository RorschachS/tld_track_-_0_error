// ================================
// TrackerModule.h
// ================================

#ifndef AMT_TRACKER_MODULE_H
#define AMT_TRACKER_MODULE_H

#include "CommonDef.h"

//*************************** OptFlowTracker STRUCT***********************************
class AmtTrackerModule 
{
public:
	AmtTrackerModule();
	~AmtTrackerModule();

	bool Execute(AmtTrackResult* pTrackResult, IplImage* pImgPrev, IplImage* pImgCurr, AmtBbox* BB, CvRect* pTrackROI=NULL);
	void Init(int iImageW, int iImageH);
	void Clean(void);

	int  ExecuteTempleMatch(AmtTrackResult* pTrackResult, AmtBbox* BB, CvRect* pTrackROI=NULL);

	bool m_IsInit;

	// ---- points grid ----
	int m_NumX;
	int m_NumY;
	int m_Margin;
	
	// --- find points ---
	int m_FindNum;
	float* m_FindErrorFB;
	float* m_FindNCC;
	CvPoint2D32f* m_FindPoints;
	int* m_FindPtsMark;

protected:
	void  iGenerateBboxPoints(CvPoint2D32f* pPts, int iNumM, int iNumN, int iMargin, AmtBbox* pBox);
	void  iCalcOpticalFlowFB(IplImage *pPrev, IplImage *pCurr);
	void  iCalcPointsDistance(CvPoint2D32f *pPt1, CvPoint2D32f *pPt2, int iPtsNum, char* pStatus, float *pMatch);
	void  iCalcPointsNCC(IplImage *pPrev, IplImage *pCurr, CvPoint2D32f *pPrevPts, CvPoint2D32f *pCurrPts, int iPtsNum, char *pStatus, int iMethod, float *pMatch);
	float iCalcMedianValue(float* pValue, char* pStatus, int iNum);
	void  iCalcBboxPredict(AmtBbox* pCurrBbox, AmtBbox* pPrevBbox, CvPoint2D32f* pCurrPts, CvPoint2D32f* pPrevPts, int* pReliableMask, int iReliableNum);

protected:
	CvSize m_ImageSizeLK;
	IplImage *m_CurrIMG;
	IplImage *m_PrevIMG;
	IplImage *m_CurrPYR;
	IplImage *m_PrevPYR;
	CvRect    m_TrackROI;
	IplImage  m_CurrImgROI;
	IplImage  m_PrevImgROI;
	IplImage  m_CurrPyrROI;
	IplImage  m_PrevPyrROI;
	int m_PointsNum;
	CvPoint2D32f* m_TemplatePoints;
	CvPoint2D32f* m_MacthingPoints;
	CvPoint2D32f* m_FBVerifyPoints;
	int m_WinSizeOF;  // OpticalFlow window size
	int m_LevelOptF;
	char *m_Status;

	int m_WinsizeCC;  // CrossCorrelation window size
	float *m_NccMatch;
	float *m_FbError;
	IplImage* m_BlockNcc[3]; // for normCrossCorrelation

protected:
	CvRect m_TrackRect;
	IplImage *m_RectIMG;
	IplImage *m_ProbIMG;
};

#endif

