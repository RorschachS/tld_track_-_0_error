// ================================
// ExpertsModule.h
// ================================

#ifndef AMT_EXPERTS_MODULE_H
#define AMT_EXPERTS_MODULE_H

#include "CommonDef.h"
#include "SingleObject.h"

class AmtExpertsModule 
{
public:
	AmtExpertsModule();
	~AmtExpertsModule();

	void Init(int iPatchSize);
	void Clean(void);

	void ObjectInitNN(AmtSingleObject* pObj);
	void ObjectEvaluateNN(AmtSingleObject* pObj, float nccThreshold, float validThreshold, float* x, float& conf1, float& conf2, int* isin=NULL);
	void ObjectUpDateNN(AmtSingleObject* pObj, AmtPatchTrainData* pEx, AmtPatchTrainData* nEx);
	void GetPatchPattern(float* outPattern, IplImage* pImg, AmtBbox* bb);
	void CalcPositivePattern(IplImage* pImg, AmtBbox* pBbox);
	void CalcNegativePattern(AmtSingleObject* pObj, IplImage* pImg, int* pSelectIdx, int iSelectNum, AmtNpar nPar);
	void SplitNegativeData(AmtPatchTrainData& nEx, AmtPatchTrainData &nEx1, AmtPatchTrainData &nEx2);

public:
	float PatchCompare(float* sample, float* base, int dim, int flag);
	float PatchVariance(float* x, int dim);

public:
	int       m_PatchSize;
	int       m_PatternDim;
	int       m_PattAndNormDim;  // size: m_PatternDim + 1(norm)
	float*    m_PatchPattern;     
	IplImage* m_PatchImage;

	AmtPatchTrainData  m_PatchTrainP;
	AmtPatchTrainData  m_PatchTrainN;
};

#endif




