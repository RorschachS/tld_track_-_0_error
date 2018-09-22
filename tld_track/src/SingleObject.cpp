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
	AmtBbox sLastBbox = mTraceResult[mTraceNum-1].mBB;//最后一个轨迹点
	int bbWidth = sLastBbox.mPointRD.x - sLastBbox.mPointLT.x;//框的宽度
	int bbHeight= sLastBbox.mPointRD.y - sLastBbox.mPointLT.y;//框的高度
	mAnalyROI.mObjRect = cvRect(sLastBbox.mPointLT.x,  sLastBbox.mPointLT.y, 
		                        sLastBbox.mPointRD.x - sLastBbox.mPointLT.x + 1,
								sLastBbox.mPointRD.y - sLastBbox.mPointLT.y + 1);
	mAnalyROI.mScanScaleN = 0; // 清零当前帧目标分析尺度层数
	mAnalyROI.mScanBboxN = 0;  // 清零当前帧目标分析box个数

	// 1. 根据当前目标位置确定分析ROI
	CvRect _MotionROI;
	if (bbWidth == 0 || bbHeight == 0)
	{
		// mMotionROI = cvRect(0, 0, mImgSize.width, mImgSize.height);
		_MotionROI = cvRect(0, 0, 0, 0);
		mAnalyROI.mExpRate = 0;
	}
	else
	{		
		int iFrameIdx = 2; // 根据历史帧结果做预测
		if (mTraceNum < iFrameIdx) // 目标初始第一帧
		{
			//初始化帧为1倍，初始化后第一帧(帧间隔4帧以上为1，2帧到4帧为1.5，2帧以下为2)	//每秒4帧以上为1, 每秒4帧到2帧为1.5 每秒2帧以下为2  根据帧率来估算ROI区域
			int iCurrTimeInterval = TimeStamp - mTraceResult[0].mTimeStamp;//当前帧与第一帧之间的时间间隔
			float rate = (0 == m_IsActive) ? 1 :
				         (iCurrTimeInterval <= 250) ? 1 :
				         (iCurrTimeInterval > 250 && iCurrTimeInterval <= 500) ? 1.5 : 2;
			_MotionROI.x = sLastBbox.mPointLT.x - bbWidth *rate;//左边界
			_MotionROI.y = sLastBbox.mPointLT.y - bbHeight*rate;//上边界
			_MotionROI.width = bbWidth * (1 + 2*rate);//宽度
			_MotionROI.height= bbHeight* (1 + 2*rate);//高度
			mAnalyROI.mExpRate = rate;//帧率
			mAnalyROI.mShiftRX = 0;
			mAnalyROI.mShiftRY = 0;
		}
		else  // 目标后续分析帧  非初始帧
		{
			AmtTraceResult *pLastTrace1 = &(mTraceResult[mTraceNum-1]);//倒数第一个
			AmtTraceResult *pLastTrace2 = &(mTraceResult[mTraceNum-iFrameIdx]); //倒数第二个

			int iCurrTimeInterval = TimeStamp - pLastTrace1->mTimeStamp;
			int iLastTimeInterval = pLastTrace1->mTimeStamp - pLastTrace2->mTimeStamp;
			int iShiftX = pLastTrace1->mPos.x - pLastTrace2->mPos.x;//倒数第一个点相对于倒数第二点的位移
			int iShiftY = pLastTrace1->mPos.y - pLastTrace2->mPos.y;
			
			float fTimeRate = iLastTimeInterval == 0 ? 0 : (float)iCurrTimeInterval / (float)iLastTimeInterval; // 兼容时间戳间隔为零情况
			float fPredictX = (float)iShiftX * fTimeRate;//根据最后两帧来预测位移
			float fPredictY = (float)iShiftY * fTimeRate;
			mAnalyROI.mShiftRX = fabs(fPredictX/bbWidth);//相对于框宽度的位移
			mAnalyROI.mShiftRY = fabs(fPredictY/bbHeight);//相对于框高度的位移

			float rate = MAX(MAX(mAnalyROI.mShiftRX, mAnalyROI.mShiftRY), 0.8);//去相对移动率的最大值
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
	mAnalyROI.mROI = _MotionROI; //划定的 ROI 区域

	// 2. 尺度选择-(当前尺度基础上下两尺度，共三层)
	float fAreaInit = fabs(m_InitBox.mPointRD.x - m_InitBox.mPointLT.x + 1) * fabs(m_InitBox.mPointRD.y - m_InitBox.mPointLT.y + 1);
	float fAreaLast = fabs(sLastBbox.mPointRD.x - sLastBbox.mPointLT.x + 1) * fabs(sLastBbox.mPointRD.y - sLastBbox.mPointLT.y + 1);
	float fScaleFactor = 1.2;     // 尺度缩放步伐
	float fRate = sqrt(fAreaLast/fAreaInit);//最后一个轨迹点所在框 与 当前框 的 比例
	float fScale = ( log10(fRate) )/( log10(fScaleFactor) ); // 上一帧目标大小所在尺度
	fScale = MIN(MAX(fScale, -(m_ObjParam.mLowScaleLevel)), m_ObjParam.mHighScaleLevel); // 限定搜索尺度基层在参数范围内  这里的尺度是个对数，以 fScaleFactor为底数

	int iScaleIdx = EvRound(fScale);
	mAnalyROI.mScaleIdx = iScaleIdx; // 目标当前所在尺度层索引

	int iScaleU = MIN(MAX((iScaleIdx+1), -(m_ObjParam.mLowScaleLevel)), m_ObjParam.mHighScaleLevel);//EvRound(fScale) + 1;
	int iScaleD = MIN(MAX((iScaleIdx-1), -(m_ObjParam.mLowScaleLevel)), m_ObjParam.mHighScaleLevel);//EvRound(fScale) - 1;

	int iScaleScanIdx = iScaleD;
	for (; iScaleScanIdx <= iScaleU; iScaleScanIdx ++)//从 iScaleD 往上 搜索到 iScaleU
	{
		int iScanBboxIdx = -1;
		for (int i=0; i<mGridBboxNum; )
		{
			if (mGridBboxData[i].mScaleIndex == iScaleScanIdx)
			{
				iScanBboxIdx = i; // 此尺度层第一个Bbox索引
				break;
			}
			else
			{
				i += mGridBboxData[i].mNearNum;
			}
		}

		if (iScanBboxIdx >= 0)//如果mGridBboxData在这一层有该尺度
		{
			// -------- 与GridBbox初始化时对应 ---------
			int iBegin = 1;
			int bbW = mGridBboxData[iScanBboxIdx].mScaleSize.width;
			int bbH = mGridBboxData[iScanBboxIdx].mScaleSize.height;
			float bbShiftW = bbW * m_ObjParam.mShift;
			float bbShiftH = bbH * m_ObjParam.mShift;
			int numW = (((m_ImgSize.width - bbW - 2) - iBegin) / bbShiftW) + 1;
			int numH = (((m_ImgSize.height- bbH - 2) - iBegin) / bbShiftH) + 1;
			assert(numW*numH == mGridBboxData[iScanBboxIdx].mNearNum);

			// 计算ROI的Bbox个数及索引
			int &iScanNum = mAnalyROI.mScanScaleN;
			mAnalyROI.mScaleInfo[iScanNum].mScaleIdx = iScaleScanIdx;
			mAnalyROI.mScaleInfo[iScanNum].mFirstBboxIdx = iScanBboxIdx;
			mAnalyROI.mScaleInfo[iScanNum].mBboxNumW = numW;
			mAnalyROI.mScaleInfo[iScanNum].mBboxNumH = numH;
			int iIdxBeginX = EvCeil((_MotionROI.x - iBegin) / bbShiftW);
			int iIdxEndinX = EvFloor((_MotionROI.x+_MotionROI.width-1 - bbW - iBegin) / bbShiftW);
			int iIdxBeginY = EvCeil((_MotionROI.y - iBegin) / bbShiftH);
			int iIdxEndinY = EvFloor((_MotionROI.y+_MotionROI.height-1 - bbH - iBegin) / bbShiftH);

			// Bbox个数及索引校验
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

	// 3. ROI中Bbox索引
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