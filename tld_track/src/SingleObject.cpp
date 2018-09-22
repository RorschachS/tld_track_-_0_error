// ==============================
// SingleObject.cpp
// ==============================

#include "SingleObject.h"

AmtSingleObject::AmtSingleObject()
{
	m_IsInit   = false;
	m_IsActive = false;

	m_ID       = -1;
	m_iIndex   = -1;
	m_ColorTag = cvScalar(0,0,0);
	mCurrCountMiss = 0;
	mUnstableNum   = 0;
	mUnstable      = false;
	mTraceNum      = 0;

	// ---- for detector ----
	m_TreesNum       = 0;
	m_FeatsNum       = 0;
	m_BboxOffSets    = NULL;
	m_BboxOffSetsNum = 0;
	m_MaxBboxOffSets = 0;
	m_FeatOffSets    = NULL;
	m_FeatOffSetsNum = 0;
	m_MaxFeatOffSets = 0;
	m_ForestWeight   = NULL;
	m_CountP         = NULL;
	m_CountN         = NULL;
	m_FeatsBitPerTree= 0;

	// ---- Grid and Scale -----
	m_ImgSize.width  = 0;
	m_ImgSize.height = 0;
	m_VarValue       = 0;
	mGridBboxData    = NULL;
	mGridBboxNum     = 0;
	mMaxGridBboxNum  = 0;
	mScaleSizeData   = NULL;
	mScaleSizeNum    = 0;
	mMaxScaleSizeNum = 0;
	mAnalyROI.mScanBboxIdx  = NULL;
	mAnalyROI.mScanBboxN    = 0;
	mAnalyROI.mScanBboxMaxN = 0;
	
	// ---- Patch Example ----
	mPatchExamplesP = NULL;
	mPatchExamplesN = NULL;
	mPatchNumP      = 0;
	mPatchNumN      = 0;
	mMaxPatchNumP   = 0;
	mMaxPatchNumN   = 0;
}

AmtSingleObject::~AmtSingleObject()
{
	Release();
}

void AmtSingleObject::Release(void)
{
	if (false == m_IsInit)
	{
		return;
	}

	// ----- Detect ------
	if (m_ForestWeight != NULL)
	{
		for (int i=0; i<m_TreesNum; i++) 
		{
			delete [](m_ForestWeight[i]); m_ForestWeight[i] = NULL;
			delete [](m_CountP[i]);       m_CountP[i]       = NULL;
			delete [](m_CountN[i]);       m_CountN[i]       = NULL;
		}
		delete [](m_ForestWeight);  m_ForestWeight = NULL;
		delete [](m_CountP);        m_CountP       = NULL;
		delete [](m_CountN);        m_CountN       = NULL;
	}

	delete [](m_BboxOffSets);  m_BboxOffSets = NULL;
	delete [](m_FeatOffSets);  m_FeatOffSets = NULL;
	m_BboxOffSetsNum = 0;
	m_MaxBboxOffSets = 0;
	m_FeatOffSetsNum = 0;
	m_MaxFeatOffSets = 0;
	m_TreesNum       = 0;
	m_FeatsNum       = 0;
	m_FeatsBitPerTree= 0;

	// ------ self resource -----
	delete[] mGridBboxData;        mGridBboxData    = NULL;
	delete[] mScaleSizeData;       mScaleSizeData   = NULL;
	mGridBboxNum     = 0;
	mMaxGridBboxNum  = 0;
	mScaleSizeNum    = 0;
	mMaxScaleSizeNum = 0;

	// ----- Scan ROI ------
	delete[] mAnalyROI.mScanBboxIdx;  mAnalyROI.mScanBboxIdx = NULL;
	mAnalyROI.mScanBboxN    = 0;
	mAnalyROI.mScanBboxMaxN = 0;

	// release examples
	delete []mPatchExamplesP;   mPatchExamplesP = NULL;
	delete []mPatchExamplesN;   mPatchExamplesN = NULL;
	mPatchNumP    = 0;
	mPatchNumN    = 0;
	mMaxPatchNumP = 0;
	mMaxPatchNumN = 0;

	m_ID       = -1;
	m_iIndex   = -1;
	m_ColorTag = cvScalar(0,0,0);
	mTraceNum  = 0;
	mCurrCountMiss = 0;
	mUnstableNum   = 0;
	mUnstable      = false;
	m_IsActive     = false;
	m_IsInit       = false;
}

void AmtSingleObject::Clean(void)
{
	m_ID = -1;
	m_iIndex = -1;
	m_ColorTag = cvScalar(0,0,0);
	m_IsActive = false;
	m_InitBox  = amtBbox(0,0,0,0);
	mCurrCountMiss = 0;
	mUnstableNum   = 0;
	mUnstable      = false;
	mTraceNum      = 0;
	for (int i=0; i<RECORD_POINTS_NUM; i++)
	{
		mTraceResult[i].Clean();
	}

	// ---- for detector ----
	if (m_MaxBboxOffSets > 0)
	{
		memset(m_BboxOffSets, 0, m_MaxBboxOffSets*sizeof(int));
	}
	m_BboxOffSetsNum = 0;
	if (m_MaxFeatOffSets > 0)
	{
		memset(m_FeatOffSets, 0, m_MaxFeatOffSets*sizeof(int));
	}
	m_FeatOffSetsNum = 0;
	if (m_TreesNum > 0 && NULL != m_ForestWeight)
	{
		for (int i = 0; i<m_TreesNum; i++) 
		{
			memset(m_ForestWeight[i], 0, m_FeatsBitPerTree*sizeof(float));
			memset(m_CountP[i],       0, m_FeatsBitPerTree*sizeof(int));
			memset(m_CountN[i],       0, m_FeatsBitPerTree*sizeof(int));
		}
	}
	
	// ---- Grid and Scale -----
	mGridBboxNum  = 0;
	mScaleSizeNum = 0;

	// ---- Scan ROI ----
	mAnalyROI.mScanBboxN = 0;
	if (mAnalyROI.mScanBboxMaxN > 0)
	{
		memset(mAnalyROI.mScanBboxIdx, 0, mAnalyROI.mScanBboxMaxN*sizeof(int));
	}
	
	// ---- Patch Example ----
	if (mMaxPatchNumP > 0)
	{
		memset(mPatchExamplesP, 0, m_Model.mPatchSize*m_Model.mPatchSize * mMaxPatchNumP*sizeof(float));
	}
	mPatchNumP = 0;
	if (mMaxPatchNumN > 0)
	{
		memset(mPatchExamplesN, 0, m_Model.mPatchSize*m_Model.mPatchSize * mMaxPatchNumN*sizeof(float));
	}
	mPatchNumN = 0;
}


bool AmtSingleObject::SetMotionROI(long long TimeStamp)
{
	AmtBbox sLastBbox = mTraceResult[mTraceNum-1].mBB;//���һ���켣��
	int bbWidth = sLastBbox.mPointRD.x - sLastBbox.mPointLT.x;//��Ŀ��
	int bbHeight= sLastBbox.mPointRD.y - sLastBbox.mPointLT.y;//��ĸ߶�
	mAnalyROI.mObjRect = cvRect(sLastBbox.mPointLT.x,  sLastBbox.mPointLT.y, 
		                        sLastBbox.mPointRD.x - sLastBbox.mPointLT.x + 1,
								sLastBbox.mPointRD.y - sLastBbox.mPointLT.y + 1);
	mAnalyROI.mScanScaleN = 0; // ���㵱ǰ֡Ŀ������߶Ȳ���
	mAnalyROI.mScanBboxN = 0;  // ���㵱ǰ֡Ŀ�����box����

	// 1. ���ݵ�ǰĿ��λ��ȷ������ROI
	CvRect _MotionROI;
	if (bbWidth == 0 || bbHeight == 0)
	{
		// mMotionROI = cvRect(0, 0, mImgSize.width, mImgSize.height);
		_MotionROI = cvRect(0, 0, 0, 0);
		mAnalyROI.mExpRate = 0;
	}
	else
	{		
		int iFrameIdx = 2; // ������ʷ֡�����Ԥ��
		if (mTraceNum < iFrameIdx) // Ŀ���ʼ��һ֡
		{
			//��ʼ��֡Ϊ1������ʼ�����һ֡(֡���4֡����Ϊ1��2֡��4֡Ϊ1.5��2֡����Ϊ2)	//ÿ��4֡����Ϊ1, ÿ��4֡��2֡Ϊ1.5 ÿ��2֡����Ϊ2  ����֡��������ROI����
			int iCurrTimeInterval = TimeStamp - mTraceResult[0].mTimeStamp;//��ǰ֡���һ֮֡���ʱ����
			float rate = (0 == m_IsActive) ? 1 :
				         (iCurrTimeInterval <= 250) ? 1 :
				         (iCurrTimeInterval > 250 && iCurrTimeInterval <= 500) ? 1.5 : 2;
			_MotionROI.x = sLastBbox.mPointLT.x - bbWidth *rate;//��߽�
			_MotionROI.y = sLastBbox.mPointLT.y - bbHeight*rate;//�ϱ߽�
			_MotionROI.width = bbWidth * (1 + 2*rate);//���
			_MotionROI.height= bbHeight* (1 + 2*rate);//�߶�
			mAnalyROI.mExpRate = rate;//֡��
			mAnalyROI.mShiftRX = 0;
			mAnalyROI.mShiftRY = 0;
		}
		else  // Ŀ���������֡  �ǳ�ʼ֡
		{
			AmtTraceResult *pLastTrace1 = &(mTraceResult[mTraceNum-1]);//������һ��
			AmtTraceResult *pLastTrace2 = &(mTraceResult[mTraceNum-iFrameIdx]); //�����ڶ���

			int iCurrTimeInterval = TimeStamp - pLastTrace1->mTimeStamp;
			int iLastTimeInterval = pLastTrace1->mTimeStamp - pLastTrace2->mTimeStamp;
			int iShiftX = pLastTrace1->mPos.x - pLastTrace2->mPos.x;//������һ��������ڵ����ڶ����λ��
			int iShiftY = pLastTrace1->mPos.y - pLastTrace2->mPos.y;
			
			float fTimeRate = iLastTimeInterval == 0 ? 0 : (float)iCurrTimeInterval / (float)iLastTimeInterval; // ����ʱ������Ϊ�����
			float fPredictX = (float)iShiftX * fTimeRate;//���������֡��Ԥ��λ��
			float fPredictY = (float)iShiftY * fTimeRate;
			mAnalyROI.mShiftRX = fabs(fPredictX/bbWidth);//����ڿ��ȵ�λ��
			mAnalyROI.mShiftRY = fabs(fPredictY/bbHeight);//����ڿ�߶ȵ�λ��

			float rate = MAX(MAX(mAnalyROI.mShiftRX, mAnalyROI.mShiftRY), 0.8);//ȥ����ƶ��ʵ����ֵ
			CvRect sLastROI = cvRect( sLastBbox.mPointLT.x - bbWidth *rate,
				                      sLastBbox.mPointLT.y - bbHeight*rate,
									  bbWidth * (1 + 2*rate),
									  bbHeight* (1 + 2*rate) ); 
			CvRect sCurrROI = cvRect( sLastBbox.mPointLT.x + fPredictX - bbWidth *rate,
									  sLastBbox.mPointLT.y + fPredictY - bbHeight*rate,
									  bbWidth * (1 + 2*rate),
									  bbHeight* (1 + 2*rate) );
			_MotionROI.x = MIN(sLastROI.x, sCurrROI.x);
			_MotionROI.y = MIN(sLastROI.y, sCurrROI.y);
			_MotionROI.width = MAX(sLastROI.x+sLastROI.width, sCurrROI.x+sCurrROI.width) - _MotionROI.x;
			_MotionROI.height= MAX(sLastROI.y+sLastROI.height, sCurrROI.y+sCurrROI.height) - _MotionROI.y;
			mAnalyROI.mExpRate = rate;
		}
		
		//check ROI
		_MotionROI.x = MAX( 0, MIN(_MotionROI.x, m_ImgSize.width -1) );
		_MotionROI.y = MAX( 0, MIN(_MotionROI.y, m_ImgSize.height-1) );
		_MotionROI.width = MAX( 0, MIN(_MotionROI.width,  m_ImgSize.width - _MotionROI.x) );
		_MotionROI.height= MAX( 0, MIN(_MotionROI.height, m_ImgSize.height- _MotionROI.y) );
	}
	mAnalyROI.mROI = _MotionROI; //������ ROI ����

	// 2. �߶�ѡ��-(��ǰ�߶Ȼ����������߶ȣ�������)
	float fAreaInit = fabs(m_InitBox.mPointRD.x - m_InitBox.mPointLT.x + 1) * fabs(m_InitBox.mPointRD.y - m_InitBox.mPointLT.y + 1);
	float fAreaLast = fabs(sLastBbox.mPointRD.x - sLastBbox.mPointLT.x + 1) * fabs(sLastBbox.mPointRD.y - sLastBbox.mPointLT.y + 1);
	float fScaleFactor = 1.2;     // �߶����Ų���
	float fRate = sqrt(fAreaLast/fAreaInit);//���һ���켣�����ڿ� �� ��ǰ�� �� ����
	float fScale = ( log10(fRate) )/( log10(fScaleFactor) ); // ��һ֡Ŀ���С���ڳ߶�
	fScale = MIN(MAX(fScale, -(m_ObjParam.mLowScaleLevel)), m_ObjParam.mHighScaleLevel); // �޶������߶Ȼ����ڲ�����Χ��  ����ĳ߶��Ǹ��������� fScaleFactorΪ����

	int iScaleIdx = EvRound(fScale);
	mAnalyROI.mScaleIdx = iScaleIdx; // Ŀ�굱ǰ���ڳ߶Ȳ�����

	int iScaleU = MIN(MAX((iScaleIdx+1), -(m_ObjParam.mLowScaleLevel)), m_ObjParam.mHighScaleLevel);//EvRound(fScale) + 1;
	int iScaleD = MIN(MAX((iScaleIdx-1), -(m_ObjParam.mLowScaleLevel)), m_ObjParam.mHighScaleLevel);//EvRound(fScale) - 1;

	int iScaleScanIdx = iScaleD;
	for (; iScaleScanIdx <= iScaleU; iScaleScanIdx ++)//�� iScaleD ���� ������ iScaleU
	{
		int iScanBboxIdx = -1;
		for (int i=0; i<mGridBboxNum; )
		{
			if (mGridBboxData[i].mScaleIndex == iScaleScanIdx)
			{
				iScanBboxIdx = i; // �˳߶Ȳ��һ��Bbox����
				break;
			}
			else
			{
				i += mGridBboxData[i].mNearNum;
			}
		}

		if (iScanBboxIdx >= 0)//���mGridBboxData����һ���иó߶�
		{
			// -------- ��GridBbox��ʼ��ʱ��Ӧ ---------
			int iBegin = 1;
			int bbW = mGridBboxData[iScanBboxIdx].mScaleSize.width;
			int bbH = mGridBboxData[iScanBboxIdx].mScaleSize.height;
			float bbShiftW = bbW * m_ObjParam.mShift;
			float bbShiftH = bbH * m_ObjParam.mShift;
			int numW = (((m_ImgSize.width - bbW - 2) - iBegin) / bbShiftW) + 1;
			int numH = (((m_ImgSize.height- bbH - 2) - iBegin) / bbShiftH) + 1;
			assert(numW*numH == mGridBboxData[iScanBboxIdx].mNearNum);

			// ����ROI��Bbox����������
			int &iScanNum = mAnalyROI.mScanScaleN;
			mAnalyROI.mScaleInfo[iScanNum].mScaleIdx = iScaleScanIdx;
			mAnalyROI.mScaleInfo[iScanNum].mFirstBboxIdx = iScanBboxIdx;
			mAnalyROI.mScaleInfo[iScanNum].mBboxNumW = numW;
			mAnalyROI.mScaleInfo[iScanNum].mBboxNumH = numH;
			int iIdxBeginX = EvCeil((_MotionROI.x - iBegin) / bbShiftW);
			int iIdxEndinX = EvFloor((_MotionROI.x+_MotionROI.width-1 - bbW - iBegin) / bbShiftW);
			int iIdxBeginY = EvCeil((_MotionROI.y - iBegin) / bbShiftH);
			int iIdxEndinY = EvFloor((_MotionROI.y+_MotionROI.height-1 - bbH - iBegin) / bbShiftH);

			// Bbox����������У��
			iIdxBeginX = MIN(MAX(iIdxBeginX, 0), numW-1);
			iIdxEndinX = MIN(MAX(iIdxEndinX, 0), numW-1);
			iIdxBeginY = MIN(MAX(iIdxBeginY, 0), numH-1); 
			iIdxEndinY = MIN(MAX(iIdxEndinY, 0), numH-1);
			if (iIdxEndinX < iIdxBeginX) 
			{
				iIdxEndinX = iIdxBeginX; 
			}
			if (iIdxEndinY < iIdxBeginY)
			{
				iIdxEndinY = iIdxBeginY;
			}

			mAnalyROI.mScaleInfo[iScanNum].mBboxIdxBeginX = iIdxBeginX;
			mAnalyROI.mScaleInfo[iScanNum].mBboxIdxEndinX = iIdxEndinX;
			mAnalyROI.mScaleInfo[iScanNum].mBboxIdxBeginY = iIdxBeginY;
			mAnalyROI.mScaleInfo[iScanNum].mBboxIdxEndinY = iIdxEndinY;

			mAnalyROI.mScanBboxN += (mAnalyROI.mScaleInfo[iScanNum].mBboxIdxEndinX - mAnalyROI.mScaleInfo[iScanNum].mBboxIdxBeginX + 1) *
				                    (mAnalyROI.mScaleInfo[iScanNum].mBboxIdxEndinY - mAnalyROI.mScaleInfo[iScanNum].mBboxIdxBeginY + 1);

			mAnalyROI.mScanScaleN ++;

			assert(mAnalyROI.mScanBboxN >= 0);
		}
	}

	// 3. ROI��Bbox����
	if (mAnalyROI.mScanBboxN > mAnalyROI.mScanBboxMaxN)
	{
		if (NULL != mAnalyROI.mScanBboxIdx)
		{
			delete[] mAnalyROI.mScanBboxIdx;
			mAnalyROI.mScanBboxIdx = NULL;
		}
		mAnalyROI.mScanBboxMaxN = mAnalyROI.mScanBboxN;
		mAnalyROI.mScanBboxIdx = new int[mAnalyROI.mScanBboxMaxN];
	}
	
	int iBboxFillIdx = 0;
	for (int i=0; i<mAnalyROI.mScanScaleN; i++)
	{
		int iBoxIdx = mAnalyROI.mScaleInfo[i].mFirstBboxIdx;
		int iBeginX = mAnalyROI.mScaleInfo[i].mBboxIdxBeginX;
		int iEndinX = mAnalyROI.mScaleInfo[i].mBboxIdxEndinX;
		int iBeginY = mAnalyROI.mScaleInfo[i].mBboxIdxBeginY;
		int iEndinY = mAnalyROI.mScaleInfo[i].mBboxIdxEndinY;
		int iBoxNumW = mAnalyROI.mScaleInfo[i].mBboxNumW;
		int iBoxNumH = mAnalyROI.mScaleInfo[i].mBboxNumH;

		int iIdx;
		for (int iY = iBeginY; iY <= iEndinY; iY++)
		{
			iIdx = iBoxIdx + iBoxNumW*iY + iBeginX;
			for (int iX = iBeginX; iX <= iEndinX; iX++, iIdx++)
			{
				mAnalyROI.mScanBboxIdx[iBboxFillIdx] = iIdx;
				iBboxFillIdx ++;
			}
		}
	}
	assert(iBboxFillIdx == mAnalyROI.mScanBboxN);

	return true;
}