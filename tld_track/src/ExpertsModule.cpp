// ================================
// ExpertsModule.cpp
// ================================

#include "ExpertsModule.h"

AmtExpertsModule::AmtExpertsModule()
{
	m_PatchSize  = 0;
	m_PatternDim = 0;
	m_PattAndNormDim = 0;
	m_PatchPattern   = NULL;
	m_PatchImage     = NULL;

	m_PatchTrainP.mData    = NULL;
	m_PatchTrainP.mDataIdx = NULL;
	m_PatchTrainN.mData    = NULL;
	m_PatchTrainN.mDataIdx = NULL;
	m_PatchTrainP.mNums    = 0;
	m_PatchTrainP.mMaxNums = 0;
	m_PatchTrainN.mNums    = 0;
	m_PatchTrainN.mMaxNums = 0;
}

AmtExpertsModule::~AmtExpertsModule()
{
	if (NULL != m_PatchPattern)
	{
		delete []m_PatchPattern;
	}
	if (NULL != m_PatchImage)
	{
		cvReleaseImage(&m_PatchImage);
	}

	// release training data
	delete[] m_PatchTrainP.mData;      m_PatchTrainP.mData    = NULL;
	delete[] m_PatchTrainP.mDataIdx;   m_PatchTrainP.mDataIdx = NULL;
	delete[] m_PatchTrainN.mData;      m_PatchTrainN.mData    = NULL;
	delete[] m_PatchTrainN.mDataIdx;   m_PatchTrainN.mDataIdx = NULL;
	m_PatchTrainP.mNums    = 0;
	m_PatchTrainP.mMaxNums = 0;
	m_PatchTrainN.mNums    = 0;
	m_PatchTrainN.mMaxNums = 0;
}

void AmtExpertsModule::Init(int iPatchSize)
{
	if (iPatchSize != m_PatchSize)
	{
		if (NULL != m_PatchPattern)
		{
			delete []m_PatchPattern;
			m_PatchPattern = NULL;
		}
		if (NULL != m_PatchImage)
		{
			cvReleaseImage(&m_PatchImage);
			m_PatchImage = NULL;
		}

		m_PatchSize = iPatchSize;
		m_PatternDim = m_PatchSize*m_PatchSize;
		m_PattAndNormDim = m_PatternDim + 1;
		m_PatchPattern = new float[m_PattAndNormDim];
		m_PatchImage = cvCreateImage(cvSize(m_PatchSize, m_PatchSize), IPL_DEPTH_8U, 1);
	}
}

void AmtExpertsModule::Clean(void)
{
	memset(m_PatchPattern, 0, m_PattAndNormDim*sizeof(float));
	cvZero(m_PatchImage);

	if (m_PatchTrainP.mMaxNums > 0)
	{
		m_PatchTrainP.mNums = 0;
		memset(m_PatchTrainP.mData, 0, m_PatchTrainP.mMaxNums*m_PattAndNormDim*sizeof(float));
		memset(m_PatchTrainP.mDataIdx, 0, m_PatchTrainP.mMaxNums*sizeof(float*));
	}
	if (m_PatchTrainN.mMaxNums > 0)
	{
		m_PatchTrainN.mNums = 0;
		memset(m_PatchTrainN.mData, 0, m_PatchTrainN.mMaxNums*m_PattAndNormDim*sizeof(float));
		memset(m_PatchTrainN.mDataIdx, 0, m_PatchTrainN.mMaxNums*sizeof(float*));
	}
}


void AmtExpertsModule::ObjectInitNN(AmtSingleObject* pObj)
{
	int iObjPatchNumP = 1;
	if (m_PatchTrainP.mMaxNums < iObjPatchNumP)   //  固定值
	{
		if (NULL != m_PatchTrainP.mData)
		{
			delete[] m_PatchTrainP.mData;
			m_PatchTrainP.mData = NULL;
		}
		if (NULL != m_PatchTrainP.mDataIdx)
		{
			delete[] m_PatchTrainP.mDataIdx;
			m_PatchTrainP.mDataIdx = NULL;
		}
		m_PatchTrainP.mMaxNums = iObjPatchNumP;
		m_PatchTrainP.mData    = new float[m_PatchTrainP.mMaxNums * m_PattAndNormDim];  // 15*15 = 225
		m_PatchTrainP.mDataIdx = new float*[m_PatchTrainP.mMaxNums];
	}

	int iObjPatchNumN = pObj->m_Npar.mNumPatches;
	if (m_PatchTrainN.mMaxNums < iObjPatchNumN) // 固定值
	{
		if (NULL != m_PatchTrainN.mData)
		{
			delete[] m_PatchTrainN.mData;
			m_PatchTrainN.mData = NULL;
		}
		if (NULL != m_PatchTrainN.mDataIdx)
		{
			delete[] m_PatchTrainN.mDataIdx;
			m_PatchTrainN.mDataIdx = NULL;
		}
		m_PatchTrainN.mMaxNums = iObjPatchNumN;
		m_PatchTrainN.mData    = new float[m_PatchTrainN.mMaxNums*m_PattAndNormDim];
		m_PatchTrainN.mDataIdx = new float*[m_PatchTrainN.mMaxNums];
	}
}


// evaluate nearest neighbor classifier
//
void AmtExpertsModule::ObjectEvaluateNN(AmtSingleObject* pObj, float nccThreshold, float validThreshold, float* x, float& conf1, float& conf2, int* isin)
{
	// 'conf1' ... full model (Relative Similarity)
	// 'conf2' ... validated part of model (Conservative Similarity)
	// 'isin'  ... inside positive ball, id positive ball, inside negative ball

	if (NULL != isin) 
	{
		isin[0] = 0; isin[1] = 0; isin[2] = 0;
	}

	// IF positive examples in the model are not defined THEN everything is negative
	if (0 == pObj->mPatchNumP)
	{
		conf1 = 0;
		conf2 = 0;
		return;
	}

	// IF negative examples in the model are not defined THEN everything is positive
	if (0 == pObj->mPatchNumN)
	{
		conf1 = 1;
		conf2 = 1;
		return;
	}

	//---- positive example ------------	
	int   NumP = pObj->mPatchNumP;
	int   csNum = cvCeil(validThreshold * NumP);
	float *pPExample = pObj->mPatchExamplesP;
	float CmpValueP = 0, MaxnccP = 0;
	float MaxP = 0;
	int   IdxMaxnccP = 0;
	for (int i=0; i<NumP; i++)
	{
		// measure NCC to positive examples  将X与 pPExample开始的patch做距离判断 0.5(  NCC(Pi, Pj) + 1)
		CmpValueP = PatchCompare(x, pPExample, m_PattAndNormDim, 1);
	
		// max positive NCC 找最大距离的那个
		if (MaxnccP < CmpValueP)
		{
			MaxnccP = CmpValueP;
			IdxMaxnccP = i;
		}
		
		// Conservative Positive NCC
		if (i < csNum)
		{
			if (MaxP < CmpValueP)
			{
				MaxP = CmpValueP;
			}
		}
		pPExample += m_PattAndNormDim;
	}

	//isin[0]储存是否找到相似块 isin[1]存储该块的序号
	if (NULL != isin) 
	{
		// IF the query patch is highly correlated with any positive patch in the model THEN it is considered to be one of them
		if (MaxnccP > nccThreshold)
		{
			isin[0] = 1;
		}
		// get the index of the maximal correlated positive patch
		isin[1] = IdxMaxnccP;
	}



	//---- negative example 处理N值------------
	int NumN = pObj->mPatchNumN;
	float *pNExample = pObj->mPatchExamplesN;
	float CmpValueN = 0, MaxnccN = 0;
	int IdxMaxnccN = 0;
	for (int i=0; i<NumN; i++)
	{
		// measure NCC to negative examples
		CmpValueN = PatchCompare(x, pNExample, m_PattAndNormDim, 1);

		// max negative NCC
		if (MaxnccN < CmpValueN)
		{
			MaxnccN = CmpValueN;
			IdxMaxnccN = i;
		}
		pNExample += m_PattAndNormDim;
	}
	// IF the query patch is highly correlated with any negative patch in the model THEN it is considered to be one of them
	if (NULL != isin && MaxnccN > nccThreshold)
	{
		isin[2] = 1;
	}

	// measure Relative Similarity
	float disP = 1 - MaxnccP;
	float disN = 1 - MaxnccN;
	conf1 = disN / (disN + disP);

	// measure Conservative Similarity
	disP = 1 - MaxP;
	conf2 = disN / (disN + disP);

	return;
}

void AmtExpertsModule::ObjectUpDateNN(AmtSingleObject* pObj, AmtPatchTrainData* pEx, AmtPatchTrainData* nEx)
{
	int iNumP = pEx->mNums;
	int iNumN = nEx->mNums;
	int iNumEx = iNumP+iNumN;
	assert(iNumEx < 200);
	
	float* pX[200] = {NULL};
	int    pY[200] = {-1};
	float** pTempX = pX;
	int* pTempY = pY;
	if (iNumP > 0)
	{
		// always add the first positive patch as the first (important in initialization)
		*pTempX++ = pEx->mDataIdx[0];
		*pTempY++ = 1;
		iNumEx = iNumEx+1;
	}
	for (int i=0; i<iNumP; i++)
	{
		*pTempX++ = pEx->mDataIdx[i]; 
		*pTempY++ = 1;
	}
	for (int i=0; i<iNumN; i++)
	{
		*pTempX++ = nEx->mDataIdx[i];
		*pTempY++ = 0;
	}
	if (iNumP > 0)
	{
		int i, j;
		float* tX;
		int tY;
		for(i=1; i<iNumEx; i++) // 随机排列不包括第一个
		{
			j = rand()%(iNumEx-i)+i;
			tX = pX[j];
			pX[j] = pX[i];
			pX[i] = tX;

			tY = pY[j];
			pY[j] = pY[i];
			pY[i] = tY;
		}
	}

	// Bootstrap
	for (int bootstrp=0; bootstrp<1; bootstrp++)
	{
		for (int i=0; i<iNumEx; i++)
		{
			// measure Relative similarity
			float conf1, conf2;
			ObjectEvaluateNN(pObj, pObj->m_Model.mNccThesame, pObj->m_Model.mValid, pX[i], conf1, conf2);

			// Positive
			if (pY[i] == 1 && conf1 <= pObj->m_Model.mThrNN) //0.65
			{
				if (pObj->mPatchNumP == pObj->mMaxPatchNumP)
				{
					pObj->mMaxPatchNumP += 10;
					float* pPatchExample = new float[pObj->mMaxPatchNumP*m_PattAndNormDim];

					if (0 != pObj->mPatchNumP)
					{
						memcpy(pPatchExample, pObj->mPatchExamplesP, pObj->mPatchNumP*m_PattAndNormDim*sizeof(float));
						delete []pObj->mPatchExamplesP;
					}
					pObj->mPatchExamplesP = pPatchExample;
				}
				// 新增正样本
				float* positive = pObj->mPatchExamplesP + pObj->mPatchNumP*m_PattAndNormDim;
				memcpy(positive, pX[i], m_PattAndNormDim*sizeof(float));
				pObj->mPatchNumP ++;
			}

			// Negative
			if (pY[i] == 0 && conf1 > 0.5)
			{
				if (pObj->mPatchNumN == pObj->mMaxPatchNumN)
				{
					pObj->mMaxPatchNumN += 10;
					float* pPatchExample = new float[pObj->mMaxPatchNumN*m_PattAndNormDim];

					if (0 != pObj->mPatchNumN)
					{
						memcpy(pPatchExample, pObj->mPatchExamplesN, pObj->mPatchNumN*m_PattAndNormDim*sizeof(float));
						delete []pObj->mPatchExamplesN;
					}
					pObj->mPatchExamplesN = pPatchExample;
				}
				// 新增负样本
				float* negative = pObj->mPatchExamplesN + pObj->mPatchNumN*m_PattAndNormDim;
				memcpy(negative, pX[i], m_PattAndNormDim*sizeof(float));
				pObj->mPatchNumN ++;
			}
		}
	}

	return;
}

// get patch under bounding box (bb), normalize it size, reshape to a column
// vector and normalize to zero mean and unit variance (ZMUV)
// 在图像pImg上, 对于bb划定的框, 对其做零均值处理，将方差存在最后一个位置
void AmtExpertsModule::GetPatchPattern(float* outPattern, IplImage* pImg, AmtBbox* bb)
{
	assert( &(outPattern[0]) );
	memset(outPattern, 0, m_PattAndNormDim*sizeof(float));

	CvRect tempROI;
	tempROI.x = bb->mPointLT.x;
	tempROI.y = bb->mPointLT.y;
	tempROI.width = (int)fabs(bb->mPointRD.x - bb->mPointLT.x + 1);
	tempROI.height= (int)fabs(bb->mPointRD.y - bb->mPointLT.y + 1);
	if (tempROI.width >= 1 && tempROI.height >= 1)
	{
		// normalize size to 'patchsize' and normalize intensities to zero mean and unit variance(ZMUV)
		cvSetImageROI(pImg, tempROI);
		cvResize(pImg, m_PatchImage);//从图像中挖到图像块
		cvResetImageROI(pImg);

		CvScalar m = cvAvg(m_PatchImage);
		float* dstData = outPattern;//输出
		uchar* srcData = (uchar*)(m_PatchImage->imageData);
		float  fNorm = 0;
		for (int i=0; i<m_PatternDim; i++)//所有有效像素个数
		{
			*dstData = (float)(*srcData - m.val[0]);//每个像素点减去全局平均值
			fNorm += (*dstData) * (*dstData);

			dstData ++;
			srcData ++;
		}
		*dstData = fNorm; // == outPattern[m_PatterDim]//将 均方和值存在最后一个位置
	}
}


//将pImg中 有pBbox对应的框做 零均值 处理, 存入到m_PatchTrainP.mDataIdx[0]中
void AmtExpertsModule::CalcPositivePattern(IplImage* pImg, AmtBbox* pBbox)
{
	m_PatchTrainP.mNums = 0;

	// --- Check Bound_box ---
	if ( fabs(pBbox->mPointLT.x - pBbox->mPointRD.x) < 1 || 
		 fabs(pBbox->mPointLT.y - pBbox->mPointRD.y) < 1 )
	{
		return;
	}

	m_PatchTrainP.mDataIdx[0] = m_PatchTrainP.mData;
	m_PatchTrainP.mNums = 1;
	GetPatchPattern(m_PatchTrainP.mDataIdx[0], pImg, pBbox); //将图像pImg上由 pBbox圈定的框 做零均值处理， 存在m_PatchTrainP.mDataIdx[0]
}

void AmtExpertsModule::CalcNegativePattern(AmtSingleObject* pObj, IplImage* pImg, int* pSelectIdx, int iSelectNum, AmtNpar nPar)
{
	m_PatchTrainN.mNums = 0;

	if (iSelectNum <= 0)
	{
		return;
	}

	//2. Randomly select 'num_patches' bound-boxes and measure patches
	amtRandPermutation(iSelectNum, pSelectIdx, false);
	iSelectNum = MIN(iSelectNum, pObj->m_Npar.mNumPatches);

	// ------------------- measure patches ------------------
	AmtGridBbox* pGridBbox = pObj->mGridBboxData;
	float* pDataNEx = m_PatchTrainN.mData;
	for (int i=0; i<iSelectNum; i++)
	{
		int idxGrid = pSelectIdx[i];
		GetPatchPattern(pDataNEx, pImg, &(pGridBbox[idxGrid].mBbox));
		m_PatchTrainN.mDataIdx[m_PatchTrainN.mNums++] = pDataNEx;
		pDataNEx += m_PattAndNormDim;
	}
}

void AmtExpertsModule::SplitNegativeData(AmtPatchTrainData& nEx, AmtPatchTrainData &nEx1, AmtPatchTrainData &nEx2)
{
	//  Splits negative data to training and validation set
	int N = nEx.mNums;
	int bound = N>>1; // N/2

	int j; 
	float* t;
	// 重新随机打乱样本顺序
	for(int i=0; i<N; i++)
	{
		j = rand()%(N-i)+i;
		t = nEx.mDataIdx[j];
		nEx.mDataIdx[j] = nEx.mDataIdx[i];
		nEx.mDataIdx[i] = t;
	}

	nEx1.mDataIdx = nEx.mDataIdx;
	nEx1.mNums = bound;

	//N如果为奇数，则从bound+1开始，舍弃中间数据
	//example: 1,2,3,4,5 -- x1=[1,2],x2=[4,5]
	//         1,2,3,4   -- x1=[1,2],x2=[3,4]
	nEx2.mDataIdx = nEx.mDataIdx + (N&0x1 ? bound+1 : bound);
	nEx2.mNums = bound;
}




//计算图像块之间相似性  0.5（NCC(Pi, Pj) + 1)
float AmtExpertsModule::PatchCompare(float* f1, float* f2, int dim, int flag)
{
	float resp;
	switch (flag)
	{
	case 1 :   // normalized correlation
		{
			float corre = 0;
			float norm1 = 0;
			float norm2 = 0;

			assert(m_PattAndNormDim == dim);

			for (int i = 0; i<m_PatternDim; i++) 
			{
				corre += f1[i] * f2[i];
	//			norm1 += f1[i] * f1[i];
	//			norm2 += f2[i] * f2[i];
			}
			norm1 = f1[m_PatternDim];
			norm2 = f2[m_PatternDim];

			// normalization to <0,1>
			resp = (corre / sqrt(norm1*norm2) + 1) / 2.0;
		}
		break;
	case 2 :   // euclidean distance
		{
			float sum = 0;
			for (int i = 0; i<dim; i++) 
			{
				sum += (f1[i]-f2[i])*(f1[i]-f2[i]);
			}

			resp = sqrt(sum);
		}
		break;
	}

	return resp;
}


//计算方差
float AmtExpertsModule::PatchVariance(float* x, int dim)
{
	float m = 0;
	for (int i=0; i<dim; i++)
	{
		m += x[i];
	}
	m = m / dim;//均值

	float v = 0;
	for (int i=0; i<dim; i++)
	{
		v += (x[i] - m) * (x[i] - m);
	}
	v = v / dim;//求得方差

	return v;
}
