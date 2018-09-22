// ================================
// TrackerModule.cpp
// ================================

#include "TrackerModule.h"
#include <limits>


AmtTrackerModule::AmtTrackerModule()
{
	m_NumX      = 10;
	m_NumY      = 10;
	m_Margin    = 5;
	m_PointsNum = m_NumX*m_NumY;

	m_FindNum   = 0;
	m_FindPoints  = NULL;
	m_FindErrorFB = NULL;
	m_FindNCC     = NULL;
	m_FindPtsMark = NULL;

	m_WinSizeOF = 4;
	m_WinsizeCC = 10; 
	m_LevelOptF = 5;
	
	m_ImageSizeLK.width  = 0;
	m_ImageSizeLK.height = 0;
	m_CurrIMG = NULL;
	m_PrevIMG = NULL;
	m_CurrPYR = NULL;
	m_PrevPYR = NULL;

	m_TemplatePoints = NULL; // template
	m_MacthingPoints = NULL; // target
	m_FBVerifyPoints = NULL; // forward-backward

	m_NccMatch = NULL;
	m_FbError  = NULL;
	m_Status   = NULL;

	m_BlockNcc[0] = NULL;
	m_BlockNcc[1] = NULL;
	m_BlockNcc[2] = NULL;

	// TEMPLEMATCH 
	m_RectIMG = NULL;
	m_ProbIMG = NULL;

	m_IsInit = false;
}

AmtTrackerModule::~AmtTrackerModule()
{
	if (true == m_IsInit)
	{
		cvReleaseImage(&m_CurrIMG);  m_CurrIMG = NULL;
		cvReleaseImage(&m_PrevIMG);  m_PrevIMG = NULL;
		cvReleaseImage(&m_CurrPYR);  m_CurrPYR = NULL;
		cvReleaseImage(&m_PrevPYR);  m_PrevPYR = NULL;

		delete [](m_FindPoints);     m_FindPoints  = NULL;
		delete [](m_FindErrorFB);    m_FindErrorFB = NULL;
		delete [](m_FindNCC);        m_FindNCC     = NULL;
		delete [](m_FindPtsMark);    m_FindPtsMark = NULL;

		delete [](m_TemplatePoints);  m_TemplatePoints = NULL;
		delete [](m_MacthingPoints);  m_MacthingPoints = NULL;
		delete [](m_FBVerifyPoints);  m_FBVerifyPoints = NULL;

		delete []m_Status;    m_Status   = NULL;
		delete []m_NccMatch;  m_NccMatch = NULL;
		delete []m_FbError;   m_FbError  = NULL;

		cvReleaseImage( &(m_BlockNcc[0]) );  m_BlockNcc[0] = NULL;
		cvReleaseImage( &(m_BlockNcc[1]) );  m_BlockNcc[1] = NULL;
		cvReleaseImage( &(m_BlockNcc[2]) );  m_BlockNcc[2] = NULL;

		m_NumX      = 10;
		m_NumY      = 10;
		m_Margin    = 5;
		m_WinSizeOF = 4;
		m_WinsizeCC = 10; 
		m_LevelOptF = 5;
		m_PointsNum = m_NumX*m_NumY;
		m_ImageSizeLK.width    = 0;
		m_ImageSizeLK.height   = 0;
		// TEMPLEMATCH 
		cvReleaseImage(&m_RectIMG);  m_RectIMG = NULL;
		cvReleaseImage(&m_ProbIMG);  m_ProbIMG = NULL;

		m_IsInit    = false;
	}
}


// for initialize
void AmtTrackerModule::Init(int iImageW, int iImageH)
{
	if (m_ImageSizeLK.width != iImageW || m_ImageSizeLK.height != iImageH)
	{
		if ( NULL != m_CurrIMG)
		{
			cvReleaseImage(&m_CurrIMG);   m_CurrIMG = NULL;
			cvReleaseImage(&m_PrevIMG);   m_PrevIMG = NULL;
			cvReleaseImage(&m_CurrPYR);   m_CurrPYR = NULL;
			cvReleaseImage(&m_PrevPYR);   m_PrevPYR = NULL;
		}
		m_ImageSizeLK.width = iImageW;
		m_ImageSizeLK.height= iImageH;
		m_CurrIMG = cvCreateImage( m_ImageSizeLK, 8, 1 );
		m_PrevIMG = cvCreateImage( m_ImageSizeLK, 8, 1 );
		m_CurrPYR = cvCreateImage( m_ImageSizeLK, 8, 1 );
		m_PrevPYR = cvCreateImage( m_ImageSizeLK, 8, 1 );
	}

	if ( false == m_IsInit )
	{
		assert( m_NumX*m_NumY == m_PointsNum);

		m_FindPoints  = new CvPoint2D32f[m_PointsNum];
		m_FindErrorFB = new float[m_PointsNum];
		m_FindNCC     = new float[m_PointsNum];
		m_FindPtsMark = new int[m_PointsNum];

		m_TemplatePoints = new CvPoint2D32f[m_PointsNum]; // template
		m_MacthingPoints = new CvPoint2D32f[m_PointsNum]; // target
		m_FBVerifyPoints = new CvPoint2D32f[m_PointsNum]; // forward-backward

		m_Status   = new char[m_PointsNum];
		memset(m_Status, 0, m_PointsNum*sizeof(char));
		m_NccMatch = new float[m_PointsNum];
		memset(m_NccMatch, 0, m_PointsNum*sizeof(float));
		m_FbError  = new float[m_PointsNum];
		memset(m_FbError, 0, m_PointsNum*sizeof(float));

		m_BlockNcc[0] = cvCreateImage( cvSize( 1, 1 ), IPL_DEPTH_32F, 1 );
		m_BlockNcc[1] = cvCreateImage( cvSize(m_WinsizeCC, m_WinsizeCC), 8, 1 );
		m_BlockNcc[2] = cvCreateImage( cvSize(m_WinsizeCC, m_WinsizeCC), 8, 1 );

		m_IsInit = true;
	}
}


// 清除数据,重置对象
void AmtTrackerModule::Clean(void)
{
	m_NumX      = 10;
	m_NumY      = 10;
	m_WinSizeOF = 4;
	m_WinsizeCC = 10; 
	m_LevelOptF = 5;
	m_PointsNum = m_NumX*m_NumY;

	cvZero(m_CurrIMG);
	cvZero(m_PrevIMG);
	cvZero(m_CurrPYR);
	cvZero(m_PrevPYR);

	memset(m_FindPoints,  0, (m_PointsNum)*sizeof(CvPoint2D32f));
	memset(m_FindErrorFB, 0, (m_PointsNum)*sizeof(float));
	memset(m_FindNCC,     0, (m_PointsNum)*sizeof(float));
	memset(m_FindPtsMark, 0, (m_PointsNum)*sizeof(int));
	m_FindNum = 0;

	memset(m_Status,   0, m_PointsNum*sizeof(char));
	memset(m_NccMatch, 0, m_PointsNum*sizeof(float));
	memset(m_FbError,  0, m_PointsNum*sizeof(float));
}

bool AmtTrackerModule::Execute(AmtTrackResult* pTrackResult, IplImage* pImgPrev, IplImage* pImgCurr, AmtBbox* BB, CvRect* pTrackROI)
{
	// initialize output variables
	pTrackResult->mBB = amtBbox(0,0,0,0);
	pTrackResult->mConf = 0.0;
	pTrackResult->mValid = false;
	pTrackResult->mFlag = false;

	if (!amtCheckBboxDef(BB))
		return false;
	if (abs(BB->mPointLT.x - BB->mPointRD.x) < MINIMUM_OBJECT_RECT || abs(BB->mPointLT.y - BB->mPointRD.y) < MINIMUM_OBJECT_RECT)
		return false;

	// check and set ROI for track
	m_TrackROI = (NULL == pTrackROI ? cvRect(0,0,m_ImageSizeLK.width,m_ImageSizeLK.height) : *pTrackROI);

	// **************************  estimate BB2  ***********************************
	// step 1 : generate 'numM*numN' grid of points within BB1 with margin 'margin' pixel
	iGenerateBboxPoints(m_TemplatePoints, m_NumX, m_NumY, m_Margin, BB);

	// step 2 : track all points by Lucas-Kanade tracker from frame prev to frame curr, 
	//          estimate Forward-Backward error, and NCC for each point
	iCalcOpticalFlowFB(pImgPrev, pImgCurr);

	// step 3 : get median of Forward-Backward error and median for NCC
	float medFB = iCalcMedianValue(m_FbError,  m_Status, m_PointsNum);
	float medNCC= iCalcMedianValue(m_NccMatch, m_Status, m_PointsNum);

	// step 4 : save selected points --> must less than a half of numPt(only for display purposes)
	m_FindNum = 0;

	for (int i=0; i<m_PointsNum; i++)
	{
		if (m_FbError[i] <= medFB && m_NccMatch[i] >= medNCC)
		{
			m_FindPoints[m_FindNum].x = m_MacthingPoints[i].x;
			m_FindPoints[m_FindNum].y = m_MacthingPoints[i].y;
			m_FindErrorFB[m_FindNum]  = m_FbError[i];
			m_FindNCC[m_FindNum]      = m_NccMatch[i];
			m_FindPtsMark[m_FindNum]  = i;
			m_FindNum ++;
		}
	}

	// Reliable Points too few
	if (m_FindNum <= 1)
		return false;

	// ----------------- filter by distance -----------------
	int iReliableMark[100];
	int iReliableNum = 0;
	for (int i=0; i<m_FindNum; i++)
	{
		int iPtIdx = m_FindPtsMark[i];

		float fOffsetX = m_TemplatePoints[iPtIdx].x - m_MacthingPoints[iPtIdx].x;
		float fOffsetY = m_TemplatePoints[iPtIdx].y - m_MacthingPoints[iPtIdx].y;
		if (fOffsetX*fOffsetX + fOffsetY*fOffsetY > 1)
		{
			iReliableMark[iReliableNum] = iPtIdx;
			iReliableNum ++;
		}
	}

	int iRate = (iReliableNum*10)/m_FindNum;
	if (iRate >= 5)
	{
		// save selected points after filter again by distance
		int iIdx;
		m_FindNum = 0;
		for (int i=0; i<iReliableNum; i++)
		{
			iIdx = iReliableMark[i];

			m_FindPoints[m_FindNum].x = m_MacthingPoints[iIdx].x;
			m_FindPoints[m_FindNum].y = m_MacthingPoints[iIdx].y;
			m_FindErrorFB[m_FindNum]  = m_FbError[iIdx];
			m_FindNCC[m_FindNum]      = m_NccMatch[iIdx];
			m_FindPtsMark[m_FindNum]  = iIdx;
			m_FindNum ++;
		}
	} 
	
	// step 5 : estimate BB2 using the reliable points only
	iCalcBboxPredict(&(pTrackResult->mBB), BB, m_MacthingPoints, m_TemplatePoints, m_FindPtsMark, m_FindNum);

	// **************************  detect failures  ***********************************
	if (!amtCheckBboxDef(&pTrackResult->mBB) || amtCheckBboxOut(&pTrackResult->mBB, m_ImageSizeLK))
	{
		// bounding box out of image
		pTrackResult->mBB = amtBbox(0,0,0,0);
		return false;
	}
	if (medFB > 10)
	{
		// too unstable predictions
		//	fprintf(stdout, "the track prediction is unstable !!!\n");
		//	pTrackResult->mBB = amtBbox(0,0,0,0);
		//	return false;
	}
	float medFB_Reliable = 0;
	for (int i=0; i<m_FindNum; i++)
	{
		medFB_Reliable += m_FindErrorFB[i];
	}
	medFB_Reliable = medFB_Reliable / m_FindNum;
	if (medFB_Reliable > 10)
	{
		//	fprintf(stdout, "跟踪误差较大，结果不确定!!! medFB = %0.3f\n", medFB_Reliable);
		pTrackResult->mBB = amtBbox(0,0,0,0);
		return false;
	}

	// 结果有效，规整跟踪预测结果
	pTrackResult->mBB.mPointLT.x = MIN(MAX(pTrackResult->mBB.mPointLT.x, 0), (m_ImageSizeLK.width -1));
	pTrackResult->mBB.mPointLT.y = MIN(MAX(pTrackResult->mBB.mPointLT.y, 0), (m_ImageSizeLK.height-1));
	pTrackResult->mBB.mPointRD.x = MIN(MAX(pTrackResult->mBB.mPointRD.x, 0), (m_ImageSizeLK.width -1));
	pTrackResult->mBB.mPointRD.y = MIN(MAX(pTrackResult->mBB.mPointRD.y, 0), (m_ImageSizeLK.height-1));

	return true;
}


// Generates numM * numN points on Bbox
void AmtTrackerModule::iGenerateBboxPoints(CvPoint2D32f* pPts, int iNumM, int iNumN, int iMargin, AmtBbox* pBox)
{
	AmtBbox bb;
	bb.mPointLT.x = pBox->mPointLT.x + iMargin;
	bb.mPointLT.y = pBox->mPointLT.y + iMargin;
	bb.mPointRD.x = pBox->mPointRD.x - iMargin;
	bb.mPointRD.y = pBox->mPointRD.y - iMargin;

	if (iNumM == 1 && iNumN == 1)
	{
		pPts[0].x = (bb.mPointLT.x + bb.mPointRD.x)*0.5;
		pPts[0].y = (bb.mPointLT.y + bb.mPointRD.y)*0.5;
		return;
	}

	if (iNumM == 1 && iNumN > 1)
	{
		float centerY = (bb.mPointLT.y + bb.mPointRD.y)*0.5;
		float stepW = (float)(bb.mPointRD.x - bb.mPointLT.x)/(float)(iNumN - 1);

		float fptX=(float)(bb.mPointLT.x);
		for (int i=0; i<iNumN; i++, fptX += stepW)
		{
			pPts[i].x = fptX;
			pPts[i].y = centerY;
		}
		return;
	}

	if (iNumM > 1 && iNumN == 1)
	{
		float centerX = (bb.mPointLT.x + bb.mPointRD.x)*0.5;
		float stepH = (float)(bb.mPointRD.y - bb.mPointLT.y)/(float)(iNumN - 1);

		float fptY=(float)(bb.mPointLT.y);
		for (int i=0; i<iNumM; i++, fptY += stepH)
		{
			pPts[i].x = centerX;
			pPts[i].y = fptY;
		}
		return;
	}

	float stepW = (float)(bb.mPointRD.x - bb.mPointLT.x)/(float)(iNumN - 1);
	float stepH = (float)(bb.mPointRD.y - bb.mPointLT.y)/(float)(iNumN - 1);

	float fptX=(float)(bb.mPointLT.x);
	for (int i=0; i<iNumN; i++, fptX += stepW)
	{
		float fptY=(float)(bb.mPointLT.y);
		for (int j=0; j<iNumM; j++, fptY += stepH)
		{
			pPts[i*iNumM+j].x = fptX;
			pPts[i*iNumM+j].y = fptY;
		}
	}
}


// track all points by Lucas-Kanade tracker from frame I to frame J, 
// estimate Forward-Backward error, and NCC for each point
void AmtTrackerModule::iCalcOpticalFlowFB(IplImage *pPrev, IplImage *pCurr)
{
	// Images
	assert(pPrev->nChannels == 1 && pPrev->depth == IPL_DEPTH_8U);
	assert(pCurr->nChannels == 1 && pCurr->depth == IPL_DEPTH_8U);

	if (false == m_IsInit)
	{
		CvSize imageSize = cvGetSize(pPrev);
		Init(imageSize.width, imageSize.height);
	}
	
	IplImage* pCurrIMG = NULL;
	IplImage* pPrevIMG = NULL;
	IplImage* pCurrPYR = NULL;
	IplImage* pPrevPYR = NULL;
	bool bIsROI = m_TrackROI.width != m_ImageSizeLK.width || m_TrackROI.height != m_ImageSizeLK.height;

	// ------------ Track ROI Prepare ------------
	if (true == bIsROI)
	{
		CvSize sSizeROI = cvSize(m_TrackROI.width, m_TrackROI.height);
		int iStride = (sSizeROI.width + 3) & -4;  // ROI图像widthStep -- 4字节对齐
		cvInitImageHeader(&m_CurrImgROI, sSizeROI, 8, 1);
		cvInitImageHeader(&m_PrevImgROI, sSizeROI, 8, 1);
		cvInitImageHeader(&m_CurrPyrROI, sSizeROI, 8, 1);
		cvInitImageHeader(&m_PrevPyrROI, sSizeROI, 8, 1);
		
		cvSetData(&m_CurrImgROI, m_CurrIMG->imageData, iStride);
		cvSetData(&m_PrevImgROI, m_PrevIMG->imageData, iStride);
		cvSetData(&m_CurrPyrROI, m_CurrPYR->imageData, iStride);
		cvSetData(&m_PrevPyrROI, m_PrevPYR->imageData, iStride);

		evCopy(pPrev, &m_PrevImgROI, &m_TrackROI, EV_COPY_ROI_TO_ALL);  
		evCopy(pCurr, &m_CurrImgROI, &m_TrackROI, EV_COPY_ROI_TO_ALL); 

		pCurrIMG = &m_CurrImgROI;
		pPrevIMG = &m_PrevImgROI;
		pCurrPYR = &m_CurrPyrROI;
		pPrevPYR = &m_PrevPyrROI;

		// 根据ROI偏移初始点集
		for (int i = 0; i < m_PointsNum; i++)
		{
			m_TemplatePoints[i].x = m_TemplatePoints[i].x - m_TrackROI.x; 
			m_TemplatePoints[i].y = m_TemplatePoints[i].y - m_TrackROI.y;
		}

		m_WinSizeOF = 4;
		m_LevelOptF = 3;
	}
	else
	{
		evCopy(pPrev, m_PrevIMG);
		evCopy(pCurr, m_CurrIMG);

		pCurrIMG = m_CurrIMG;
		pPrevIMG = m_PrevIMG;
		pCurrPYR = m_CurrPYR;
		pPrevPYR = m_PrevPYR;

		m_WinSizeOF = 4;
		m_LevelOptF = 5;
	}
	

 	// Points
  	for (int i = 0; i < m_PointsNum; i++)
  	{  
  		m_MacthingPoints[i].x = m_TemplatePoints[i].x; 
  		m_MacthingPoints[i].y = m_TemplatePoints[i].y;
  
  		m_FBVerifyPoints[i].x = m_TemplatePoints[i].x; 
  		m_FBVerifyPoints[i].y = m_TemplatePoints[i].y;
  	}

	evCalcOpticalFlowPyrLK( pPrevIMG, pCurrIMG, pPrevPYR, pCurrPYR, m_TemplatePoints, m_MacthingPoints, m_PointsNum, 
		                    cvSize(m_WinSizeOF,m_WinSizeOF), m_LevelOptF, m_Status, 0, 
							cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), CV_LKFLOW_INITIAL_GUESSES);
	evCalcOpticalFlowPyrLK( pCurrIMG, pPrevIMG, pCurrPYR, pPrevPYR, m_MacthingPoints, m_FBVerifyPoints, m_PointsNum, 
		                    cvSize(m_WinSizeOF,m_WinSizeOF), m_LevelOptF, 0      , 0, 
							cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), CV_LKFLOW_INITIAL_GUESSES | CV_LKFLOW_PYR_A_READY | CV_LKFLOW_PYR_B_READY );

	iCalcPointsNCC(pPrevIMG, pCurrIMG, m_TemplatePoints, m_MacthingPoints, m_PointsNum, m_Status, CV_TM_CCOEFF_NORMED, m_NccMatch);
	iCalcPointsDistance(m_TemplatePoints, m_FBVerifyPoints, m_PointsNum, m_Status, m_FbError);

	if (true == bIsROI)
	{
		// 根据ROI还原点集偏移
		for (int i = 0; i < m_PointsNum; i++)
		{
			m_TemplatePoints[i].x = m_TemplatePoints[i].x + m_TrackROI.x; 
			m_TemplatePoints[i].y = m_TemplatePoints[i].y + m_TrackROI.y;

			m_MacthingPoints[i].x = m_MacthingPoints[i].x + m_TrackROI.x; 
			m_MacthingPoints[i].y = m_MacthingPoints[i].y + m_TrackROI.y;
		}
	}
}


void AmtTrackerModule::iCalcPointsDistance(CvPoint2D32f *pPt1, CvPoint2D32f *pPt2, int iPtsNum, char* pStatus, float *pMatch) 
{
	for (int i = 0; i < iPtsNum; i++) 
	{
		if (1 == pStatus[i])
		{
			pMatch[i] = sqrt( (pPt1[i].x - pPt2[i].x)*(pPt1[i].x - pPt2[i].x) 
				            + (pPt1[i].y - pPt2[i].y)*(pPt1[i].y - pPt2[i].y) );
		}
		else
		{
			pMatch[i] = 1000;
		}
	}
}

void AmtTrackerModule::iCalcPointsNCC(IplImage *pPrev, IplImage *pCurr, CvPoint2D32f *pPrevPts, CvPoint2D32f *pCurrPts, 
											int iPtsNum, char *pStatus, int iMethod, float *pMatch) 
{
	assert(m_BlockNcc || m_BlockNcc[0] || m_BlockNcc[1] || m_BlockNcc[2]);

	CvSize win_size = cvSize(m_BlockNcc[1]->width, m_BlockNcc[1]->height);
	for (int i = 0; i < iPtsNum; i++) 
	{
		if (1 == pStatus[i])
		{
#if  0      // CV_Template
			cvGetRectSubPix( pPrev, m_BlockNcc[1], pPrevPts[i] );
			cvGetRectSubPix( pCurr, m_BlockNcc[2], pCurrPts[i] );
			cvMatchTemplate( m_BlockNcc[1], m_BlockNcc[2], m_BlockNcc[0], iMethod);
			pMatch[i] = ((float *)(m_BlockNcc[0]->imageData))[0]; 
#else
			// m_BlockNcc[1] & m_BlockNcc[1] -- m_WinsizeCC
			evGetRectPix( pPrev, m_BlockNcc[1], pPrevPts[i] );
			evGetRectPix( pCurr, m_BlockNcc[2], pCurrPts[i] );

			// CV_TMCCOEFF_NORMED -- 归一化相关系数
			float corre = 0;
			float norm1 = 0;
			float norm2 = 0;

			uchar* pData1 = (uchar*)(m_BlockNcc[1]->imageData);
			uchar* pData2 = (uchar*)(m_BlockNcc[2]->imageData);
			int iStep1 = m_BlockNcc[1]->widthStep;
			int iStep2 = m_BlockNcc[2]->widthStep;
			
			for (int y=0; y<win_size.height; y++, pData1 += iStep1, pData2 += iStep2) 
			{
				for (int x=0; x<win_size.width; x++)
				{
					corre += pData1[x] * pData2[x];
					norm1 += pData1[x] * pData1[x];
					norm2 += pData2[x] * pData2[x];
				}
			}

			// normalization to <0,1>
			pMatch[i] = (corre / sqrt(norm1*norm2) + 1) / 2.0;
#endif
		} 
		else 
		{
			pMatch[i] = 0.0;
		}
	}
}


float AmtTrackerModule::iCalcMedianValue(float* pValue, char* pStatus, int iNum)
{
	if (NULL == pValue || NULL == pStatus || 0 == iNum)
	{
		return 0;
	}

	float* pV = pValue;
	char * pS = pStatus; 
	float fMedianV = 0.;
	int iCount = 0;
	for (int i=0; i<iNum; i++)
	{
		if (1 == *pS)
		{
			fMedianV += *pV;
			iCount++;
		}
		pV ++;
		pS ++;
	}
	return 0 == iCount ? 0 : (fMedianV/iCount);
}


void AmtTrackerModule::iCalcBboxPredict(AmtBbox* pCurrBbox, AmtBbox* pPrevBbox, CvPoint2D32f* pCurrPts, CvPoint2D32f* pPrevPts, int* pReliableMask, int iReliableNum)
{
	float d0, d1;
	float dxSum = 0;
	float dySum = 0;
	float scaleSum = 0;
	int sNum = 0;
	for (int i=0; i<iReliableNum; i++)
	{
		// ------ just analyze reliable points ------
		// for drift
		int idxI = pReliableMask[i];
		dxSum += pCurrPts[idxI].x - pPrevPts[idxI].x;
		dySum += pCurrPts[idxI].y - pPrevPts[idxI].y;

		// for scale
		// for n points, (0 - 1),(0 - 2),...(0 - n-1),(1 - 2),(1 - 3),...(1 - n-1),.....(n-2 - n-1)
		for (int j=i+1; j<iReliableNum; j++)
		{
			int idxJ = pReliableMask[j];
			d0 = (pPrevPts[idxJ].x - pPrevPts[idxI].x)*(pPrevPts[idxJ].x - pPrevPts[idxI].x)
			   + (pPrevPts[idxJ].y - pPrevPts[idxI].y)*(pPrevPts[idxJ].y - pPrevPts[idxI].y);
			d1 = (pCurrPts[idxJ].x - pCurrPts[idxI].x)*(pCurrPts[idxJ].x - pCurrPts[idxI].x) 
			   + (pCurrPts[idxJ].y - pCurrPts[idxI].y)*(pCurrPts[idxJ].y - pCurrPts[idxI].y);
			if (d0 != 0 && d1 != 0)
			{
				scaleSum += sqrt(d1/d0);
				sNum ++;
			}
		}
	}

	float dx = dxSum / iReliableNum;
	float dy = dySum / iReliableNum;
	float scale = scaleSum / sNum;

	float scaleWidth  = (pPrevBbox->mPointRD.x - pPrevBbox->mPointLT.x + 1)*(scale - 1)*0.5;
	float scaleHeight = (pPrevBbox->mPointRD.y - pPrevBbox->mPointLT.y + 1)*(scale - 1)*0.5;

	// scale change
	pCurrBbox->mPointLT.x = pPrevBbox->mPointLT.x - scaleWidth;
	pCurrBbox->mPointLT.y = pPrevBbox->mPointLT.y - scaleHeight;
	pCurrBbox->mPointRD.x = pPrevBbox->mPointRD.x + scaleWidth;
	pCurrBbox->mPointRD.y = pPrevBbox->mPointRD.y + scaleHeight;

	// drift change
	pCurrBbox->mPointLT.x += dx;
	pCurrBbox->mPointLT.y += dy;
	pCurrBbox->mPointRD.x += dx;
	pCurrBbox->mPointRD.y += dy;
}
int AmtTrackerModule::ExecuteTempleMatch(AmtTrackResult* pTrackResult, AmtBbox* BB, CvRect* pTrackROI)
{
	// initialize output variables
	pTrackResult->mBB = amtBbox(0,0,0,0);
	pTrackResult->mConf = 0.0;
	pTrackResult->mValid = false;
	pTrackResult->mFlag = false;

	if (!amtCheckBboxDef(BB))
		return -1;
	if (abs(BB->mPointLT.x - BB->mPointRD.x) < MINIMUM_OBJECT_RECT || abs(BB->mPointLT.y - BB->mPointRD.y) < MINIMUM_OBJECT_RECT)
		return -2;

	// check and set ROI for track
	m_TrackROI = (NULL == pTrackROI ? cvRect(0,0,m_ImageSizeLK.width,m_ImageSizeLK.height) : *pTrackROI);
	m_TrackRect = cvRect(BB->mPointLT.x, BB->mPointLT.y, BB->mPointRD.x - BB->mPointLT.x, BB->mPointRD.y - BB->mPointLT.y);

	IplImage* pRectIMG = cvCreateImage(cvSize(m_TrackRect.width, m_TrackRect.height), IPL_DEPTH_8U, 1);
	IplImage* pProbIMG = cvCreateImage(cvSize(m_TrackROI.width-m_TrackRect.width+1, m_TrackROI.height-m_TrackRect.height+1), IPL_DEPTH_32F, 1);
	IplImage* pCurrIMG = NULL;
	IplImage* pPrevIMG = NULL;
	bool bIsROI = m_TrackROI.width != m_ImageSizeLK.width || m_TrackROI.height != m_ImageSizeLK.height;
	if (true == bIsROI)
	{
		// The Data had prepared when Execute()
		pCurrIMG = &m_CurrImgROI;
		pPrevIMG = &m_PrevImgROI;

		// ROI Shift
		m_TrackRect.x = m_TrackRect.x - m_TrackROI.x;
		m_TrackRect.y = m_TrackRect.y - m_TrackROI.y;
	}
	else
	{
		pCurrIMG = m_CurrIMG;
		pPrevIMG = m_PrevIMG;
	}

	evCopy(pPrevIMG, pRectIMG, &m_TrackRect, EV_COPY_ROI_TO_ALL);
	cvMatchTemplate(pCurrIMG, pRectIMG, pProbIMG, CV_TM_CCOEFF_NORMED);

	double dMaxVal = 0;
	CvPoint sMaxPoint;
	cvMinMaxLoc(pProbIMG, 0, &dMaxVal, 0, &sMaxPoint);

	CvRect sResultTrack = cvRect(sMaxPoint.x, sMaxPoint.y, m_TrackRect.width, m_TrackRect.height);
	pTrackResult->mBB = amtBbox(sResultTrack.x, sResultTrack.y, sResultTrack.x+sResultTrack.width-1, sResultTrack.y+sResultTrack.height-1);
	if (true == bIsROI)
	{
		// 根据ROI还原偏移
		pTrackResult->mBB.mPointLT.x = pTrackResult->mBB.mPointLT.x + m_TrackROI.x;
		pTrackResult->mBB.mPointLT.y = pTrackResult->mBB.mPointLT.y + m_TrackROI.y;
		pTrackResult->mBB.mPointRD.x = pTrackResult->mBB.mPointRD.x + m_TrackROI.x;
		pTrackResult->mBB.mPointRD.y = pTrackResult->mBB.mPointRD.y + m_TrackROI.y;
	}

	// 结果有效，规整跟踪预测结果
	pTrackResult->mBB.mPointLT.x = MIN(MAX(pTrackResult->mBB.mPointLT.x, 0), (m_ImageSizeLK.width -1));
	pTrackResult->mBB.mPointLT.y = MIN(MAX(pTrackResult->mBB.mPointLT.y, 0), (m_ImageSizeLK.height-1));
	pTrackResult->mBB.mPointRD.x = MIN(MAX(pTrackResult->mBB.mPointRD.x, 0), (m_ImageSizeLK.width -1));
	pTrackResult->mBB.mPointRD.y = MIN(MAX(pTrackResult->mBB.mPointRD.y, 0), (m_ImageSizeLK.height-1));

	cvReleaseImage(&pRectIMG);
	cvReleaseImage(&pProbIMG);

	return 0;
}


