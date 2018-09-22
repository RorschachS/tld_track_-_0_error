// ================================
// DetecterModule.cpp
// ================================

#include "DetecterModule.h"
#include "SingleObject.h"

//积分图数据指针下标索引
#define sub2idx(row,col,width)  (EvRound(row)*(width) + EvRound(col))
#define sub2coord(pIdx, width)  cvPoint((int)(pIdx%width), (int)(pIdx/width))


// ************************ fern function *******************************
AmtDetecterModule::AmtDetecterModule()
{
	m_ThresholdP = 0;
	m_ThresholdN = 0;
	m_TreesNum = 0;
	m_FeatsNum = 0;
	m_ImageH = 0;
	m_ImageW = 0;
	m_NoiseImage    = NULL;
	m_IntegralImage = NULL;
	m_IntegralImage2= NULL;
	m_BboxStep = 0;
	m_FeatBit = 0;
	m_FeatsBitPerTree = 0;

	// ----- Train Data -----
	m_FernTrainP.mData    = NULL;
	m_FernTrainP.mDataIdx = NULL;
	m_FernTrainN.mData    = NULL;
	m_FernTrainN.mDataIdx = NULL;
	m_FernTrainP.mNums    = 0;
	m_FernTrainP.mMaxNums = 0;
	m_FernTrainN.mNums    = 0;
	m_FernTrainN.mMaxNums = 0;

	// ---- public resource for object ------
	mFernFeatures    = NULL;
	mGridOverlapData = NULL;
	mGridFernConf    = NULL;
	mGridFernPatt    = NULL;
	mGridSelectIdx   = NULL;
	mMaxGridBboxNum  = 0;
}

AmtDetecterModule::~AmtDetecterModule()
{	
	if (NULL != m_NoiseImage)
	{
		cvReleaseImage(&m_NoiseImage); 
		m_NoiseImage = NULL;
	}

	if ( NULL != m_IntegralImage )
	{
		delete []m_IntegralImage;
		m_IntegralImage = NULL;
		delete []m_IntegralImage2;
		m_IntegralImage2 = NULL;
	}

	// release training data
	delete[] m_FernTrainP.mData;       m_FernTrainP.mData = NULL;
	delete[] m_FernTrainP.mDataIdx;    m_FernTrainP.mDataIdx = NULL;
	delete[] m_FernTrainN.mData;       m_FernTrainN.mData = NULL;
	delete[] m_FernTrainN.mDataIdx;    m_FernTrainN.mDataIdx = NULL;
	m_FernTrainP.mNums    = 0;
	m_FernTrainP.mMaxNums = 0;
	m_FernTrainN.mNums    = 0;
	m_FernTrainN.mMaxNums = 0;

	// ---- public resource for object ------
	delete[] mFernFeatures;        mFernFeatures    = NULL;
	delete[] mGridOverlapData;     mGridOverlapData = NULL;
	delete[] mGridFernConf;        mGridFernConf    = NULL;
	delete[] mGridFernPatt;        mGridFernPatt    = NULL;
	delete[] mGridSelectIdx;       mGridSelectIdx   = NULL;
	mMaxGridBboxNum  = 0;
}


void AmtDetecterModule::Init(int iImageW, int iImageH, int iTreesNum, int iFeatsNum)
{
	m_BboxStep = 7;
	m_FeatBit = 1;

	if (iTreesNum != m_TreesNum || iFeatsNum != m_FeatsNum)
	{
		if ( NULL != mFernFeatures)
		{
			delete []mFernFeatures;
			mFernFeatures = NULL;
		}
		mFernFeatures = new float[4*iFeatsNum*iTreesNum];
		m_TreesNum = iTreesNum;
		m_FeatsNum = iFeatsNum;
		m_FeatsBitPerTree = 1<<(m_FeatBit*m_FeatsNum);// pow(2, nBIT*nFEAT);
	}

	if (iImageW != m_ImageW || iImageH != m_ImageH)
	{
		if ( NULL != m_IntegralImage )
		{
			delete []m_IntegralImage;
			m_IntegralImage = NULL;
			delete []m_IntegralImage2;
			m_IntegralImage2 = NULL;
		}

		m_ImageW = iImageW;
		m_ImageH = iImageH;  
		m_NoiseImage = cvCreateImage(cvSize(m_ImageW, m_ImageH), IPL_DEPTH_8U, 1);
		m_IntegralImage  = new long long[m_ImageH*m_ImageW]; 
		m_IntegralImage2 = new long long[m_ImageH*m_ImageW];
	}
}

void AmtDetecterModule::Clean(void)
{
	if (m_FernTrainP.mMaxNums > 0)
	{
		m_FernTrainP.mNums = 0;
		memset(m_FernTrainP.mData, 0, m_FernTrainP.mMaxNums*m_TreesNum*sizeof(int));
		memset(m_FernTrainP.mDataIdx, 0, m_FernTrainP.mMaxNums*sizeof(int*));
	}
	if (m_FernTrainN.mMaxNums > 0)
	{
		m_FernTrainN.mNums = 0;
		memset(m_FernTrainN.mData, 0, m_FernTrainN.mMaxNums*m_TreesNum*sizeof(int));
		memset(m_FernTrainN.mDataIdx, 0, m_FernTrainN.mMaxNums*sizeof(int*));
	}
	if (m_FeatsNum > 0 && m_TreesNum > 0)
	{
		memset(mFernFeatures, 0, 4*m_FeatsNum*m_TreesNum*sizeof(float));
	}
}


bool AmtDetecterModule::BboxGridScan(AmtSingleObject* pObj)
{
	pObj->mGridBboxNum  = 0;
	pObj->mScaleSizeNum = 0;

	// Check if input Bound box is smaller than minimum检查box是否太小
	CvSize boxSize = cvSize(pObj->m_InitBox.mPointRD.x - pObj->m_InitBox.mPointLT.x + 1, pObj->m_InitBox.mPointRD.y - pObj->m_InitBox.mPointLT.y + 1); //输入的Box维度
	if (boxSize.width < pObj->m_ObjParam.mMinWin || boxSize.height < pObj->m_ObjParam.mMinWin)
	{
		return false;
	}

	// prepare scale factor
	int iMinScale = -(pObj->m_ObjParam.mLowScaleLevel);
	int iMaxScale = pObj->m_ObjParam.mHighScaleLevel;
	int iScaleNum = iMaxScale-iMinScale+1;  // 当前目标初始化尺度层数
	float fScaleFactor = 1.2;               // 尺度缩放步伐
	float pScaleCoefArray[30];              // 尺度缩放因子数组 - 最大允许尺度层数30
	int   pScaleLevelIndex[30];             // 尺度层索引
	int   pScaleBboxNum[30];                // 尺度层网格个数
	assert(iScaleNum <= 30);

	int iScaleIdx = iMinScale;
	for (int i=0; iScaleIdx<=iMaxScale; i++, iScaleIdx++)
	{
		pScaleCoefArray[i]  = pow(fScaleFactor, iScaleIdx);
		pScaleLevelIndex[i] = iScaleIdx;
		pScaleBboxNum[i] = 0;
	}

	// 统计目标网格个数
	iMinScale = 30; iMaxScale = -30; // 统计实际可分析尺度层
	int iBbegin = 1;
	for (int i=0; i<iScaleNum; i++) //遍历每一层
	{
		int bbW = EvRound(boxSize.width * pScaleCoefArray[i]);
		int bbH = EvRound(boxSize.height* pScaleCoefArray[i]);
		if (bbW < pObj->m_ObjParam.mMinWin || bbH < pObj->m_ObjParam.mMinWin) { continue; }

		float bbShiftW = bbW * pObj->m_ObjParam.mShift;
		float bbShiftH = bbH * pObj->m_ObjParam.mShift;
		int numW = (((pObj->m_ImgSize.width - bbW - 2) - iBbegin) / bbShiftW) + 1;
		int numH = (((pObj->m_ImgSize.height- bbH - 2) - iBbegin) / bbShiftH) + 1;
		if (numW < 1 || numH < 1){ continue; }

		pScaleBboxNum[i] = numW*numH;
		pObj->mGridBboxNum    += pScaleBboxNum[i];
		pObj->mScaleSizeNum   += 1;

		int iScaleLevelIdx = pScaleLevelIndex[i];
		if (iMinScale > iScaleLevelIdx)
		{
			iMinScale = iScaleLevelIdx;
		}
		if (iMaxScale < iScaleLevelIdx)
		{
			iMaxScale = iScaleLevelIdx;
		}
	}
	// 修正目标分析尺度层
	pObj->m_ObjParam.mLowScaleLevel  = -iMinScale;
	pObj->m_ObjParam.mHighScaleLevel = iMaxScale;

	// 根据目标网格分配
	if (pObj->mMaxGridBboxNum < pObj->mGridBboxNum)
	{
		if (NULL != pObj->mGridBboxData)
		{
			delete[] pObj->mGridBboxData;
			pObj->mGridBboxData = NULL;
		}
		pObj->mMaxGridBboxNum  = pObj->mGridBboxNum*MEMORY_EXPANSION_STEP;
		pObj->mGridBboxData    = new AmtGridBbox[pObj->mMaxGridBboxNum];

		if (mMaxGridBboxNum < pObj->mMaxGridBboxNum)
		{
			if (NULL != mGridOverlapData)
			{
				delete[] mGridOverlapData;
				mGridOverlapData = NULL;
			}
			if (NULL != mGridSelectIdx)
			{
				delete[] mGridSelectIdx;
				mGridSelectIdx = NULL;
			}
			if (NULL != mGridFernConf)
			{
				delete[] mGridFernConf;
				mGridFernConf = NULL;
			}
			if (NULL != mGridFernPatt)
			{
				delete[] mGridFernPatt;
				mGridFernPatt = NULL;
			}

			mMaxGridBboxNum  = pObj->mMaxGridBboxNum;
			mGridOverlapData = new float[mMaxGridBboxNum];
			mGridSelectIdx   = new int[mMaxGridBboxNum];
			mGridFernConf    = new float[mMaxGridBboxNum];
			mGridFernPatt    = new int[mMaxGridBboxNum * m_TreesNum];
		}
	}

	// 根据尺度层数分配
	if (pObj->mMaxScaleSizeNum < pObj->mScaleSizeNum)
	{
		if (NULL != pObj->mScaleSizeData)
		{
			delete[] pObj->mScaleSizeData;
			pObj->mScaleSizeData = NULL;
		}
		pObj->mMaxScaleSizeNum = pObj->mScaleSizeNum*MEMORY_EXPANSION_STEP;
		pObj->mScaleSizeData = new CvSize[pObj->mMaxScaleSizeNum];
	}

	// 填充目标网格框数据
	AmtBbox tBbox;
	AmtGridBbox* pGridBbox = pObj->mGridBboxData;
	CvSize* pScaleSize = pObj->mScaleSizeData;
	int iBboxCount = 0, iScaleCount = 0;
	for (int i=0; i<iScaleNum; i++)
	{
		if (pScaleBboxNum[i] > 0)
		{
			float fScaleCoef   = pScaleCoefArray[i];
			int iScaleLevelIdx = pScaleLevelIndex[i];
			int iScaleBboxNum  = pScaleBboxNum[i];

			int bbW = EvRound(boxSize.width * fScaleCoef);
			int bbH = EvRound(boxSize.height* fScaleCoef);
			int bbArea = bbW*bbH;  // Bbox面积
			float bbShiftW = bbW * pObj->m_ObjParam.mShift;
			float bbShiftH = bbH * pObj->m_ObjParam.mShift;
			int numW = (((pObj->m_ImgSize.width - bbW - 2) - iBbegin) / bbShiftW) + 1;
			int numH = (((pObj->m_ImgSize.height- bbH - 2) - iBbegin) / bbShiftH) + 1;

			// -----------------		
			for(int h=0; h<numH; h++)
			{
				tBbox.mPointLT.y = EvRound(iBbegin + h*bbShiftH);
				tBbox.mPointRD.y = tBbox.mPointLT.y + bbH - 1;

				for(int w=0; w<numW; w++)
				{
					tBbox.mPointLT.x = EvRound(iBbegin + w*bbShiftW);
					tBbox.mPointRD.x = tBbox.mPointLT.x + bbW - 1;

					pGridBbox->mBbox    = tBbox;
					pGridBbox->mArea    = bbArea;
					pGridBbox->mNearNum = iScaleBboxNum;
					pGridBbox->mIndex   = iScaleCount;           // 顺序尺度序列索引
					pGridBbox->mScaleIndex = iScaleLevelIdx;   // 实际尺度序列索引
					pGridBbox->mScaleCoef  = fScaleCoef;
					pGridBbox->mScaleSize  = cvSize(bbW, bbH);

					pGridBbox ++;
					iBboxCount ++;
				}
			}

			pScaleSize->width  = bbW;
			pScaleSize->height = bbH;
			pScaleSize ++;
			iScaleCount ++;
		}
	}
	assert(iScaleCount == pObj->mScaleSizeNum && iBboxCount == pObj->mGridBboxNum);

	return true;
}



int AmtDetecterModule::Execute(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur, int* pDetectIdx)
{
	int iDetectNum = 0;

	if (0 == pObj->mAnalyROI.mROI.width || 0 == pObj->mAnalyROI.mROI.height)
		return iDetectNum;

	// evaluates Ensemble Classifier: saves sum of posteriors to 'amt.tmp.conf', 
	// saves measured codes to 'amt.tmp.patt', does not considers patches with variance < tmd.var
	ObjectFernDetect(pObj, pImg, pImgBlur, pObj->m_VarValue, mGridFernConf, mGridFernPatt);

	// get indexes of bounding boxes that passed through the Ensemble Classifier. 
	// speedup: if there are more than "DETECT_NUM_PER_FRAME" detections, pick "DETECT_NUM_PER_FRAME" of the most confident only
	int iDim = pObj->mAnalyROI.mScanBboxN; //pObj->mGridBboxNum;
	int* pScanBboxs = pObj->mAnalyROI.mScanBboxIdx;
	for (int i=0; i<iDim; i++)
	{
		int iBboxIdx = pScanBboxs[i];
		if (mGridFernConf[iBboxIdx] > m_ThresholdP)
		{
			// 超过单帧最大检测个数，则寻找最小Conf结果，并替换
			if (iDetectNum == DETECT_NUM_PER_FRAME)
			{
				float fMinConf = mGridFernConf[pDetectIdx[0]];
				int iMinIdx = 0;
				int iConfIdx; 
				for (int k=1; k<iDetectNum; k++)
				{
					iConfIdx = pDetectIdx[k];
					if (mGridFernConf[iConfIdx] < fMinConf)
					{
						iMinIdx = k;
						fMinConf = mGridFernConf[iConfIdx];
					}
				}

				// 新增加的检测结果Conf是否大于之前最小Conf，是则替换
				if (mGridFernConf[iBboxIdx] > fMinConf)
				{
					pDetectIdx[iMinIdx] = iBboxIdx;
				}
			}
			else
			{
				pDetectIdx[iDetectNum] = iBboxIdx;
				iDetectNum ++;
			}
		}
	}
	assert(iDetectNum <= DETECT_NUM_PER_FRAME);

	return iDetectNum;
}


// INIT
// =============================================================================
void AmtDetecterModule::ObjectFernInit(AmtSingleObject* pObj, AmtGridBbox* grid, float* features, CvSize* scales)
{
	srand(0); // fix state of random generator

	m_ThresholdP = m_TreesNum * pObj->m_Model.mThrFern;
	m_ThresholdN = m_TreesNum * 0.5;
	// ---------------------

	// BBOX-OFFSET
	pObj->m_BboxOffSetsNum = m_BboxStep*pObj->mGridBboxNum;
	if (pObj->m_MaxBboxOffSets < pObj->m_BboxOffSetsNum)
	{
		if (NULL != pObj->m_BboxOffSets)
		{
			delete[] pObj->m_BboxOffSets;
			pObj->m_BboxOffSets = NULL;
		}
		pObj->m_MaxBboxOffSets = pObj->m_BboxOffSetsNum*MEMORY_EXPANSION_STEP;
		pObj->m_BboxOffSets	= new int[pObj->m_MaxBboxOffSets];
	}
	iCalcBboxOffsets(pObj->m_BboxOffSets, grid, pObj->mGridBboxNum);

	// FEAT-OFFSET
	pObj->m_FeatOffSetsNum = pObj->mScaleSizeNum*m_TreesNum*m_FeatsNum*2;
	if (pObj->m_MaxFeatOffSets < pObj->m_FeatOffSetsNum)
	{
		if (NULL != pObj->m_FeatOffSets)
		{
			delete[] pObj->m_FeatOffSets;
			pObj->m_FeatOffSets = NULL;
		}
		pObj->m_MaxFeatOffSets = pObj->m_FeatOffSetsNum*MEMORY_EXPANSION_STEP;
		pObj->m_FeatOffSets = new int[pObj->m_MaxFeatOffSets];
	}
	iCalcFeatOffsets(pObj->m_FeatOffSets, features, scales, pObj->mScaleSizeNum);

	// WEIGHT & COUNT-P/N  ---- Set with DetectorModule's init
	if (pObj->m_TreesNum != m_TreesNum || pObj->m_FeatsNum != m_FeatsNum)
	{
		if (0 != pObj->m_TreesNum)
		{
			for (int i = 0; i<pObj->m_TreesNum; i++) 
			{
				delete [](pObj->m_ForestWeight[i]); pObj->m_ForestWeight[i] = NULL;
				delete [](pObj->m_CountP[i]);       pObj->m_CountP[i]       = NULL;
				delete [](pObj->m_CountN[i]);       pObj->m_CountN[i]       = NULL;
			}
			delete [](pObj->m_ForestWeight);  pObj->m_ForestWeight = NULL;
			delete [](pObj->m_CountP);        pObj->m_CountP       = NULL;
			delete [](pObj->m_CountN);        pObj->m_CountN       = NULL;
		}

		pObj->m_TreesNum     = m_TreesNum;
		pObj->m_FeatsNum     = m_FeatsNum;
		pObj->m_ForestWeight = new float*[m_TreesNum];
		pObj->m_CountP       = new   int*[m_TreesNum];
		pObj->m_CountN       = new   int*[m_TreesNum];
		pObj->m_FeatsBitPerTree = m_FeatsBitPerTree;
		for (int i = 0; i<m_TreesNum; i++) 
		{
			pObj->m_ForestWeight[i] = new float[m_FeatsBitPerTree];
			pObj->m_CountP[i]       = new   int[m_FeatsBitPerTree];
			pObj->m_CountN[i]       = new   int[m_FeatsBitPerTree];

			memset(pObj->m_ForestWeight[i], 0, m_FeatsBitPerTree*sizeof(float));
			memset(pObj->m_CountP[i],       0, m_FeatsBitPerTree*sizeof(int));
			memset(pObj->m_CountN[i],       0, m_FeatsBitPerTree*sizeof(int));
		}
	}
	assert(pObj->m_TreesNum == m_TreesNum && pObj->m_FeatsNum == m_FeatsNum);

	// TRAIN DATA
	if (m_FernTrainP.mMaxNums < pObj->m_PparInit.mNumWarps * pObj->m_PparInit.mNumClosest)  // 固定值
	{
		if (NULL != m_FernTrainP.mData)
		{
			delete[] m_FernTrainP.mData;
			m_FernTrainP.mData = NULL;
		}
		if (NULL != m_FernTrainP.mDataIdx)
		{
			delete[] m_FernTrainP.mDataIdx;
			m_FernTrainP.mDataIdx = NULL;
		}
		m_FernTrainP.mMaxNums = pObj->m_PparInit.mNumWarps * pObj->m_PparInit.mNumClosest;
		m_FernTrainP.mData    = new int[m_FernTrainP.mMaxNums*m_TreesNum]; // 20*10(row)*10(col)
		m_FernTrainP.mDataIdx = new int*[m_FernTrainP.mMaxNums];
	}

	if (m_FernTrainN.mMaxNums < pObj->mGridBboxNum)  // 非固定,根据目标大小变化
	{
		if (NULL != m_FernTrainN.mData)
		{
			delete[] m_FernTrainN.mData;
			m_FernTrainN.mData = NULL;
		}
		if (NULL != m_FernTrainN.mDataIdx)
		{
			delete[] m_FernTrainN.mDataIdx;
			m_FernTrainN.mDataIdx = NULL;
		}
		m_FernTrainN.mMaxNums = pObj->mGridBboxNum*MEMORY_EXPANSION_STEP;
		m_FernTrainN.mData    = new int[m_FernTrainN.mMaxNums*m_TreesNum];
		m_FernTrainN.mDataIdx = new int*[m_FernTrainN.mMaxNums];
	}

	return;
}

// UPDATE
// =============================================================================
void AmtDetecterModule::ObjectFernUpDate(AmtSingleObject* pObj, AmtFernTrainData* pTrainDataP, AmtFernTrainData* pTrainDataN, int iBootstrp, bool respon)
{
	int iTrainNumP = pTrainDataP->mNums;
	int iTrainNumN = pTrainDataN->mNums;
	int iTotalTrainNum = iTrainNumP + iTrainNumN;
	int iStep = iTotalTrainNum / 10;

	int iIdxP, iIdxN, iIdxI;
	for (int b = 0; b < iBootstrp; b++)
	{
		for (int i = 0; i < iStep; i++)
		{
			for (int k = 0; k < 10; k++)
			{
				iIdxI = k*iStep + i;
				if (iIdxI < iTrainNumP)  // 正样本数据集中
				{
					iIdxP = iIdxI;
					if (iMeasureForest(pTrainDataP->mDataIdx[iIdxP], m_TreesNum, pObj->m_ForestWeight) <= m_ThresholdP)
						iUpdateWeight(pTrainDataP->mDataIdx[iIdxP], 1, 1, m_TreesNum, pObj->m_ForestWeight, pObj->m_CountP, pObj->m_CountN);
				}
				else
				{
					iIdxN = iIdxI - iTrainNumP;
					if (iMeasureForest(pTrainDataN->mDataIdx[iIdxN], m_TreesNum, pObj->m_ForestWeight) >= m_ThresholdN)
						iUpdateWeight(pTrainDataN->mDataIdx[iIdxN], 0, 1, m_TreesNum, pObj->m_ForestWeight, pObj->m_CountP, pObj->m_CountN);
				}
			}
		}
	}
}


// EVALUATE PATTERNS
// =============================================================================
float AmtDetecterModule::ObjectFernEvaluate(AmtFernTrainData &xSample, float** pWeight)
{
	// Get the max-conf for xSample
	int numX = xSample.mNums;
	float fRespConf, fMaxConf = 0;

	for (int i = 0; i < numX; i++) 
	{
		int *X = xSample.mDataIdx[i];
		fRespConf = iMeasureForest(X/*+nTREES*i*/, m_TreesNum, pWeight);

		if (fRespConf > fMaxConf)
		{
			fMaxConf = fRespConf;
		}
	}

	return fMaxConf;
}

// DETECT: TOTAL RECALL
// =============================================================================
void AmtDetecterModule::ObjectFernDetect(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur, float minVar, float* pConf, int* pPatt)
{
	// Pointer to preallocated output matrices 
	if ( NULL == pConf || NULL == pPatt) 
	{ 
		fprintf(stdout, "Wrong input!\n"); 
		return;
	}

	// Input images
	unsigned char *input = (unsigned char*)(pImg->imageData);
	unsigned char *blur  = (unsigned char*)(pImgBlur->imageData);

	// ---------------------------
	int iIdx = 0;
	int* pBboxIdxList = pObj->mAnalyROI.mScanBboxIdx;
	int iBboxIdxNum = pObj->mAnalyROI.mScanBboxN;
	for (int i=0; i<iBboxIdxNum; i++)
	{
		iIdx = pBboxIdxList[i];
		pConf[iIdx] = iMeasureBboxWithOffset(pObj, blur, iIdx, minVar, pPatt+m_TreesNum*iIdx);
	}

	return;
}

// GET PATTERNS
// =============================================================================
void AmtDetecterModule::GetFernPatterns(AmtSingleObject* pObj, IplImage* pImgBlur, int* pSelectIdx, int iSelectNum, int* patterns)
{
	uchar* blur  = (uchar*)(pImgBlur->imageData);

	// bbox indexes
	for (int j = 0; j < iSelectNum; j++)
	{
		int *tPatt = patterns + j*m_TreesNum;
		for (int i = 0; i < m_TreesNum; i++) 
		{
			tPatt[i] = iMeasureTreeWithOffset(pObj, blur, pSelectIdx[j], i);
		}
	}
}



// -----------------------------------------------------------------------
//在四个象限内生成随机数, 储存在mFernFeatures中，
void AmtDetecterModule::GenerateFeaturesTemplate(bool show)
{
	//将 mFernFeatures 开始的 4* m_FeatsNum*m_TreesNum 空间大小的内存设定为0
	float* pFeatureData = mFernFeatures;
	int iFeatPointNum = 4*m_FeatsNum*m_TreesNum;
	memset(pFeatureData, 0, iFeatPointNum*sizeof(int));// 此处貌似为bug, 应该是sizeof(float),效果一样

#if 0
	// Set Random-Forest with Object As a Whole
	for (int i=0; i<iFeatPointNum; i++)
	{
		*pFeatureData ++ = amtRandFloat();
	}

#else
	// Set Random-Forest with Object As Sample Quartering
	int iRegionN = 4; //总共有4个区域
	int iTreesRegion = m_TreesNum / iRegionN;

	// region 0
	for (int i=0; i<iTreesRegion; i++)
	{
		for (int j=0; j<m_FeatsNum; j++)
		{
			*pFeatureData ++ = amtRandFloat()*0.5; // x0
			*pFeatureData ++ = amtRandFloat()*0.5; // y0
			*pFeatureData ++ = amtRandFloat()*0.5; // x1
			*pFeatureData ++ = amtRandFloat()*0.5; // y1
		}
	}
	// region 1
	for (int i=0; i<iTreesRegion; i++)
	{
		for (int j=0; j<m_FeatsNum; j++)
		{
			*pFeatureData ++ = amtRandFloat()*0.5+0.5; // x0
			*pFeatureData ++ = amtRandFloat()*0.5; // y0
			*pFeatureData ++ = amtRandFloat()*0.5+0.5; // x1
			*pFeatureData ++ = amtRandFloat()*0.5; // y1
		}
	}
	// region 2
	for (int i=0; i<iTreesRegion; i++)
	{
		for (int j=0; j<m_FeatsNum; j++)
		{
			*pFeatureData ++ = amtRandFloat()*0.5; // x0
			*pFeatureData ++ = amtRandFloat()*0.5+0.5; // y0
			*pFeatureData ++ = amtRandFloat()*0.5; // x1
			*pFeatureData ++ = amtRandFloat()*0.5+0.5; // y1
		}
	}
	// region 3
	for (int i=0; i<iTreesRegion; i++)
	{
		for (int j=0; j<m_FeatsNum; j++)
		{
			*pFeatureData ++ = amtRandFloat()*0.5+0.5; // x0
			*pFeatureData ++ = amtRandFloat()*0.5+0.5; // y0
			*pFeatureData ++ = amtRandFloat()*0.5+0.5; // x1
			*pFeatureData ++ = amtRandFloat()*0.5+0.5; // y1
		}
	}

#endif

#if 0
	// ==== Test Feature Show=====
	IplImage* FeatureImg = cvCreateImage(cvSize(100, 100), 8, 1);
	char* WinName = "Feature";
	cvNamedWindow(WinName, 1);

	pFeatureData = pObj->mFernFeatures;
	CvPoint pt0, pt1;
	for (int i=0; i<iTrees; i++)
	{
		cvZero(FeatureImg);
		for (int j=0; j<iFeatures; j++)
		{
			pt0 = cvPoint(pFeatureData[0]*99, pFeatureData[1]*99);
			pt1 = cvPoint(pFeatureData[2]*99, pFeatureData[3]*99);
			cvLine(FeatureImg, pt0, pt1, cvScalar(255), 1, 8, 0);

			pFeatureData += 4;
		}
		cvShowImage(WinName, FeatureImg);
		cvWaitKey(0);
	}
	cvDestroyWindow(WinName);
#endif

}


void AmtDetecterModule::CalcPositivePattern(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur, int* pSelectIdx, int iSelectNum, AmtPpar pPar)
{
	AmtFernTrainData* pTrainData = &(m_FernTrainP);
	AmtGridBbox* pGridBboxData = pObj->mGridBboxData;

	// 数据清零
	pTrainData->mNums = 0; 

	if (iSelectNum <= 0)
	{
		return;
	}

	if (iSelectNum > pPar.mNumClosest)
	{
		for (int i=0; i<iSelectNum; i++)
		{
			for (int j=i+1; j<iSelectNum; j++)
			{
				if (mGridOverlapData[pSelectIdx[i]] < mGridOverlapData[pSelectIdx[j]])
				{
					int tmp = pSelectIdx[i];
					pSelectIdx[i] = pSelectIdx[j];
					pSelectIdx[j] = tmp;
				}
			}
		}
		iSelectNum = pPar.mNumClosest;
	}

	// Get hull
	AmtGridBbox* pGridBbox = NULL;
	AmtBbox bbH = pGridBboxData[ pSelectIdx[0] ].mBbox; 
	for (int i=1; i<iSelectNum; i++)
	{
		pGridBbox = &(pGridBboxData[ pSelectIdx[i] ]);
		// min(grid(1,:))
		bbH.mPointLT.x = MIN(bbH.mPointLT.x,  pGridBbox->mBbox.mPointLT.x);

		// min(grid(2,:))
		bbH.mPointLT.y = MIN(bbH.mPointLT.y,  pGridBbox->mBbox.mPointLT.y);

		// max(grid(3,:))
		bbH.mPointRD.x = MAX(bbH.mPointRD.x,  pGridBbox->mBbox.mPointRD.x);

		// max(grid(4,:))
		bbH.mPointRD.y = MAX(bbH.mPointRD.y,  pGridBbox->mBbox.mPointRD.y);
	}

	int* xData = pTrainData->mData;
	IplImage* pBlurImage = NULL;  int icount = 0;
	for (int i=0; i<pPar.mNumWarps; i++)
	{
		if (i > 0)
		{
			iGetNoiseImage(pImgBlur, m_NoiseImage, &bbH, pPar);
			pBlurImage = m_NoiseImage;
		}
		else  // i==0
		{
			pBlurImage = pImgBlur;
		}

		GetFernPatterns(pObj, pBlurImage, pSelectIdx, iSelectNum, xData);

		for (int j=0; j<iSelectNum; j++)
		{
			pTrainData->mDataIdx[pTrainData->mNums++] = xData;
			xData += m_TreesNum;
		}
	}
}

void AmtDetecterModule::CalcNegativePattern(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur, int* pSelectIdx, int& iSelectNum, AmtNpar nPar)
{
	// 数据清零
	m_FernTrainN.mNums = 0;

	if (iSelectNum <= 0)
	{
		return;
	}

	// if Object's Variance is effective, then Select again by Bound_box Variance
	float fMiniVar = pObj->m_VarValue/2;
	if (fMiniVar > 0) 
	{
		{
			CalcIntegralImage(pImg, &(pObj->mAnalyROI.mROI));
		}

		int iSelectNumByVar = 0;
		float fBboxVar = 0;
		for (int i=0; i<iSelectNum; i++)
		{
			fBboxVar = iCalcBboxVariance(m_IntegralImage, m_IntegralImage2, pObj->m_BboxOffSets+pSelectIdx[i]*m_BboxStep); 
			if (fBboxVar < fMiniVar)
			{	
				continue; 
			}
			pSelectIdx[iSelectNumByVar ++] = pSelectIdx[i];
		}

		iSelectNum = iSelectNumByVar;
	}


	// Get Fern Pattern
	GetFernPatterns(pObj, pImgBlur, pSelectIdx, iSelectNum, m_FernTrainN.mData);

	int* pDataNX = m_FernTrainN.mData;
	for (int i=0; i<iSelectNum; i++)
	{
		m_FernTrainN.mDataIdx[m_FernTrainN.mNums++] = pDataNX;
		pDataNX += m_TreesNum;
	}
}

void AmtDetecterModule::SplitNegativeData(AmtFernTrainData& nX, AmtFernTrainData &nX1, AmtFernTrainData &nX2)
{
	//  Splits negative data to training and validation set
	int N = nX.mNums;
	int bound = N>>1; // N/2

	int j; 
	int*t;
	// 重新随机打乱样本顺序
	for(int i=0; i<N; i++)
	{
		j = rand()%(N-i)+i;
		t = nX.mDataIdx[j];
		nX.mDataIdx[j] = nX.mDataIdx[i];
		nX.mDataIdx[i] = t;
	}

	nX1.mDataIdx = nX.mDataIdx;
	nX1.mNums = bound;

	//N如果为奇数，则从bound+1开始，舍弃中间数据
	//example: 1,2,3,4,5 -- x1=[1,2],x2=[4,5]
	//         1,2,3,4   -- x1=[1,2],x2=[3,4]
	nX2.mDataIdx = nX.mDataIdx + (N&0x1 ? bound+1 : bound);
	nX2.mNums = bound;
}


void AmtDetecterModule::CalcIntegralImage(IplImage* pImg, CvRect* pROI) 
{
	assert(1 == pImg->nChannels);
	assert(pImg->width == m_ImageW && pImg->height == m_ImageH);

	if (NULL == pROI)
	{
		unsigned char* pSrcData = (unsigned char*)(pImg->imageData);
		long long *ii  = m_IntegralImage;
		long long *ii2 = m_IntegralImage2;

		long long *prev_line  = ii;
		long long *prev_line2 = ii2;
		long lSum, lSum2;
		long lData;
		int iByteDiff = pImg->widthStep - pImg->width;  // 四字节未对齐差异

		lData = *pSrcData;
		*ii  = lData;
		*ii2 = lData*lData; 

		for (int x=1; x<m_ImageW; x++) 
		{
			ii++; 
			ii2++; 
			pSrcData++; // 数据有效，则指针累加

			lData = *pSrcData;
			*ii  = lData + *(ii-1);// 左边点的值加上该点像素值
			*ii2 = lData*lData + *(ii2-1); //左边点的值加上该点像素值平方
		}
		pSrcData += iByteDiff; // 跳过四字节补齐差异

		for (int y=1; y<m_ImageH; y++) 
		{
			lSum = 0; lSum2 = 0;
			for (int x=0; x<m_ImageW; x++) 
			{
				ii++; 
				ii2++; 
				pSrcData++; 

				lData = *pSrcData;
				lSum  += lData;
				lSum2 += lData * lData;
				*ii  = lSum + *prev_line;
				*ii2 = lSum2 + *prev_line2;
				prev_line++; 
				prev_line2++;
			}
			pSrcData += iByteDiff; // 跳过四字节补齐差异
		}
	}
	else
	{
		int iBeginX = pROI->x;
		int iEndinX = pROI->x+pROI->width-1;
		int iBeginY = pROI->y;
		int iEndinY = pROI->y+pROI->height-1;
		assert(iBeginX >=0 && iEndinX < m_ImageW && iBeginY >=0 && iEndinY < m_ImageH);

		unsigned char* pSrcData = (unsigned char*)(pImg->imageData);
		long long *ii  = m_IntegralImage;
		long long *ii2 = m_IntegralImage2;

		long long *prev_line  = ii;
		long long *prev_line2 = ii2;
		long lSum, lSum2;
		long lData;
		int iByteDiff = pImg->widthStep - pImg->width;  // 四字节未对齐差异

		// ------------
		int iOffset = m_ImageW*iBeginY + iBeginX;  // 初始偏移
		pSrcData += iOffset;
		ii  += iOffset;
		ii2 += iOffset;
		prev_line  += iOffset;
		prev_line2 += iOffset;

		lData = *pSrcData;
		*ii  = lData;
		*ii2 = lData*lData; 

		for (int x=iBeginX+1; x<=iEndinX; x++) 
		{
			ii++; 
			ii2++; 
			pSrcData++; // 数据有效，则指针累加

			lData = *pSrcData;
			*ii  = lData + *(ii-1);
			*ii2 = lData*lData + *(ii2-1);
		}
		iOffset = (m_ImageW-1) - iEndinX + iBeginX; // 换行偏移
		pSrcData += iOffset + iByteDiff;  // 跳过四字节补齐差异
		ii  += iOffset;
		ii2 += iOffset;

		for (int y=iBeginY+1; y<=iEndinY; y++) 
		{
			lSum = 0; lSum2 = 0;
			for (int x=iBeginX; x<=iEndinX; x++) 
			{
				ii++; 
				ii2++; 
				pSrcData++; 

				lData = *pSrcData;
				lSum  += lData;
				lSum2 += lData * lData;
				*ii  = lSum + *prev_line;
				*ii2 = lSum2 + *prev_line2;
				prev_line++; 
				prev_line2++;
			}
			pSrcData += iOffset + iByteDiff;  // 跳过四字节补齐差异
			ii  += iOffset;
			ii2 += iOffset;
			prev_line  += iOffset;
			prev_line2 += iOffset;
		}
	}

/*	//-----debug __ show IntegralImage Roi-----

	IplImage* pIntImg  = cvCreateImage(cvSize(m_ImageW,m_ImageH), 8, 1);
	IplImage* pIntImg2 = cvCreateImage(cvSize(m_ImageW,m_ImageH), 8, 1);
	cvZero(pIntImg);
	cvZero(pIntImg2);

	long long* ii = m_IntegralImage;
	long long* ii2 = m_IntegralImage2;

	for (int y=0; y<m_ImageH; y++)
	{
		for (int x=0; x<m_ImageW; x++)
		{
			if (*ii > 0)
			{
				((uchar*)(pIntImg->imageData + pIntImg->widthStep*y))[x] = 255;
			}
			if (*ii2 > 0)
			{
				((uchar*)(pIntImg2->imageData + pIntImg2->widthStep*y))[x] = 255;
			}

			ii ++;
			ii2++;
		}
	}

	cvNamedWindow("ii", 1);
	cvNamedWindow("ii2", 1);
	cvShowImage("ii", pIntImg);
	cvShowImage("ii2", pIntImg2);
	cvWaitKey(0);

	cvReleaseImage(&pIntImg);
	cvReleaseImage(&pIntImg2);
*/
}


// ========================================================================
void AmtDetecterModule::iUpdateWeight(int *x, int C, int N, int iTrees, float** pWeight, int** pnP, int** pnN) 
{
	for (int i = 0; i < iTrees; i++)
	{
		int idx = (int) x[i];

		(C==1) ? pnP[i][idx] += N : pnN[i][idx] += N;

		if (pnP[i][idx]==0) 
		{
			pWeight[i][idx] = 0;
		} 
		else 
		{
			pWeight[i][idx] = ((float) (pnP[i][idx])) / (pnP[i][idx] + pnN[i][idx]);
		}
	}
}


float AmtDetecterModule::iMeasureForest(int *pIdx, int iTrees, float** pWeight)
{
	float votes = 0;
	for (int i = 0; i < iTrees; i++)
	{ 
		votes += pWeight[i][pIdx[i]];
	}
	return votes;
}


int AmtDetecterModule::iMeasureTreeWithOffset(AmtSingleObject* pObj, unsigned char *pImg, int iBboxIdx, int iTreeIdx)
{
	int index = 0;
	int *bbox = pObj->m_BboxOffSets + iBboxIdx*m_BboxStep;
	int *off = pObj->m_FeatOffSets + bbox[5] + iTreeIdx*2*m_FeatsNum;
	for (int i=0; i<m_FeatsNum; i++) 
	{
		index<<=1; 
		int fp0 = pImg[off[0]+bbox[0]];
		int fp1 = pImg[off[1]+bbox[0]];
		if (fp0>fp1) { index |= 1;}
		off += 2;
	}
	return index;	
}


float AmtDetecterModule::iMeasureBboxWithOffset(AmtSingleObject* pObj, unsigned char *pBlur, int iBboxIdx, float fMinVar, int *pPattern) 
{
	float conf = 0.0;
	float bboxvar = iCalcBboxVariance(m_IntegralImage, m_IntegralImage2, pObj->m_BboxOffSets+iBboxIdx*m_BboxStep);
	if (bboxvar < fMinVar) 
	{	
		return conf; 
	}

	for (int i = 0; i < m_TreesNum; i++)
	{ 
		int idx = iMeasureTreeWithOffset(pObj, pBlur,iBboxIdx,i);
		pPattern[i] = idx;
		conf += pObj->m_ForestWeight[i][idx];
	}
	return conf;
}

void AmtDetecterModule::iCalcFeatOffsets(int *pFeatOffSets, float *pFeatures, CvSize *pScales, int iScale)
{
	int *off = pFeatOffSets;

	CvSize *scale = NULL;
	for (int k = 0; k < iScale; k++)
	{
		scale = &(pScales[k]);
		for (int i = 0; i < m_TreesNum; i++) 
		{
			for (int j = 0; j < m_FeatsNum; j++) 
			{
				float *x  = pFeatures +4*j + (4*m_FeatsNum)*i;
				*off++ = sub2idx((scale->height-1)*x[1], (scale->width-1)*x[0], m_ImageW);
				*off++ = sub2idx((scale->height-1)*x[3], (scale->width-1)*x[2], m_ImageW);
			}
		}
	}
}



//根据初始化bb_grid计算每个bb在图像上的一维指针索引位置
void AmtDetecterModule::iCalcBboxOffsets(int* pBboxOffSets, AmtGridBbox *pBboxs, int iBBoxNum)
{
	int *off = pBboxOffSets;

	AmtGridBbox* pGridBbox = NULL;
	for (int i = 0; i < iBBoxNum; i++) 
	{
		pGridBbox = &(pBboxs[i]);
		//   off[0] --- off[2]
		//   off[1] --- off[3]
		*off++ = sub2idx(pGridBbox->mBbox.mPointLT.y, pGridBbox->mBbox.mPointLT.x, m_ImageW);
		*off++ = sub2idx(pGridBbox->mBbox.mPointRD.y, pGridBbox->mBbox.mPointLT.x, m_ImageW);
		*off++ = sub2idx(pGridBbox->mBbox.mPointLT.y, pGridBbox->mBbox.mPointRD.x, m_ImageW);
		*off++ = sub2idx(pGridBbox->mBbox.mPointRD.y, pGridBbox->mBbox.mPointRD.x, m_ImageW);
		*off++ = pGridBbox->mArea;
		*off++ = (int) (pGridBbox->mIndex)*2*m_FeatsNum*m_TreesNum; // pointer to features for this scale
		*off++ = pGridBbox->mNearNum; // number of left-right bound_boxes, will be used for searching neighbors
	}
}

// =================================== protected function ===================================
float AmtDetecterModule::iCalcBboxVariance(long long *ii, long long *ii2, int *off)
{
	// off[0-3] corners of bbox, off[4] area
	float mX  = (ii[off[3]] - ii[off[2]] - ii[off[1]] + ii[off[0]]) / (float) off[4];
	float mX2 = (ii2[off[3]] - ii2[off[2]] - ii2[off[1]] + ii2[off[0]]) / (float) off[4];
	return mX2 - mX*mX;
}


void AmtDetecterModule::iGetNoiseImage(IplImage* pSrcImg, IplImage* pNoiseImg, AmtBbox* bb, AmtPpar pPar)
{
	CvRect sROI;
	sROI.x = bb->mPointLT.x;
	sROI.y = bb->mPointLT.y;
	sROI.width = (int)fabs(bb->mPointRD.x - bb->mPointLT.x + 1);
	sROI.height= (int)fabs(bb->mPointRD.y - bb->mPointLT.y + 1);
	if (sROI.width >= 1 && sROI.height >= 1)
	{
		cvSetImageROI(pNoiseImg, sROI);
		cvZero(pNoiseImg);

		double randomize = (double)time(NULL);
		CvRNG rng_state = cvRNG(int(randomize));
		cvRandArr(&rng_state, pNoiseImg, CV_RAND_NORMAL, cvRealScalar(0), cvRealScalar(pPar.mNoise));

		cvResetImageROI(pNoiseImg);
	}

	//	Replace [ cvAdd(pSrcImg, pNoiseImg, pNoiseImg) ]
	int iBeginX = bb->mPointLT.x;
	int iBeginY = bb->mPointLT.y;
	int iEndinX = bb->mPointRD.x;
	int iEndinY = bb->mPointRD.y;
	for (int y=iBeginY; y<=iEndinY; y++)
	{
		uchar* pRowDataDst = (uchar*)(pNoiseImg->imageData + pNoiseImg->widthStep*y);
		uchar* pRowDataSrc = (uchar*)(pSrcImg->imageData + pSrcImg->widthStep*y);
		for (int x=iBeginX; x<=iEndinX; x++)
		{
			pRowDataDst[x] += pRowDataSrc[x];
		}
	}
}

