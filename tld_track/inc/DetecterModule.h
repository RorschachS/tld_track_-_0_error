// ================================
// DetecterModule.h
// ================================

#ifndef AMT_DETECTER_MODULE_H
#define AMT_DETECTER_MODULE_H

#include "CommonDef.h"
#include "SingleObject.h"

//*************************** FernClassifier STRUCT***********************************
class AmtDetecterModule 
{
public:
	AmtDetecterModule();
	~AmtDetecterModule();

	int   Execute(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur, int* pDetectIdx);
	void  Init(int iImageW, int iImageH, int iTreesNum, int iFeatsNum);
	void  Clean(void);
	bool  BboxGridScan(AmtSingleObject* pObj);

	void  ObjectFernInit(AmtSingleObject* pObj, AmtGridBbox* grid, float* features, CvSize* scales);
	void  ObjectFernUpDate(AmtSingleObject* pObj, AmtFernTrainData* pTrainDataP, AmtFernTrainData* pTrainDataN, int iBootstrp, bool respon);
	float ObjectFernEvaluate(AmtFernTrainData &xSample, float** pWeight);
	void  ObjectFernDetect(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur, float minVar, float* conf, int* patt);
	void  GetFernPatterns(AmtSingleObject* pObj, IplImage* pImgBlur, int* pSelectIdx, int iSelectNum, int* patterns);
	void  GenerateFeaturesTemplate(bool show=false);

	void  CalcPositivePattern(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur, int* pSelectIdx, int  iSelectNum, AmtPpar pPar);
	void  CalcNegativePattern(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur, int* pSelectIdx, int& iSelectNum, AmtNpar nPar);
	void  SplitNegativeData(AmtFernTrainData& nX, AmtFernTrainData &nX1, AmtFernTrainData &nX2);
	void  CalcIntegralImage(IplImage* pImg, CvRect* pROI=NULL);

protected:
	void  iUpdateWeight(int *x, int C, int N, int iTrees, float** pWeight, int** pnP, int** pnN);
	float iMeasureForest(int *pIdx, int iTrees, float** pWeight);
	int   iMeasureTreeWithOffset(AmtSingleObject* pObj, unsigned char *pImg, int iBboxIdx, int iTreeIdx);
	float iMeasureBboxWithOffset(AmtSingleObject* pObj, unsigned char *pBlur, int iBboxIdx, float fMinVar, int *pPattern);
	void  iCalcFeatOffsets(int *pFeatOffSets, float *pFeatures, CvSize *pScales, int iScale);
	void  iCalcBboxOffsets(int* pBboxOffSets, AmtGridBbox *pBboxs, int iBBoxNum);

protected:
	float iCalcBboxVariance(long long *ii, long long *ii2, int *off);
	void  iGetNoiseImage(IplImage* pSrcImg, IplImage* pNoiseImg, AmtBbox* bb, AmtPpar pPar);

public: // ---------------------------------

	AmtFernTrainData  m_FernTrainP;
	AmtFernTrainData  m_FernTrainN;

	// ----- public resource for object -----
	float*  mFernFeatures;     // 4*FeatsNum*TreesNum
	float*  mGridOverlapData;  // 1*var
	float*  mGridFernConf;     // 1*var
	int*    mGridFernPatt;     // mTreesNum*var
	int*    mGridSelectIdx;    // 1*var
	int     mMaxGridBboxNum;

//protected:
public:
	int        m_ImageH;
	int        m_ImageW;
	IplImage*  m_NoiseImage;
	float      m_ThresholdP;
	float      m_ThresholdN;
	int        m_TreesNum;
	int        m_FeatsNum;
	bool       m_IsCalcIntegral;
	long long* m_IntegralImage;
	long long* m_IntegralImage2;
	int        m_BboxStep;
	int        m_FeatBit;
	int        m_FeatsBitPerTree;
};

#endif

