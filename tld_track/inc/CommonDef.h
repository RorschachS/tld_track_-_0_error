// ================================
// Common Define
// ================================

#ifndef AMT_COMMON_DEF_H
#define AMT_COMMON_DEF_H

#ifdef  WIN32 || WIN64
#include <afxmt.h>
#include <tchar.h>
#else   //for linux
#include <unistd.h>
#include <sys/stat.h>
#include <pthread.h>
#endif

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "LK.h"

#define  MEMORY_EXPANSION_STEP  1.5     // �ڴ���չ����
#define  MAX_ANALY_OBJECT_NUM   10      // ������Ŀ����� shj

const int DETECT_NUM_PER_FRAME = 30;    // ÿ֡Ŀ��������
const int RECORD_POINTS_NUM    = 250;   // �켣�㳤������
const int SAMPLE_REGION_NUM    = 5;     // �켣����������
const int MINIMUM_OBJECT_RECT  = 10;    // ��СĿ���С

// ------------------- Define For Platform ------------------------------
#ifndef  MAX_PATH
#define  MAX_PATH  260
#endif

#ifndef  INT_MAX
#define  INT_MAX  2147483647
#endif
// ----------------------------------------------------------------------

// --------- for math ---------
#define  EvFloor(x)      ( x>=0 ? int(x) : int(x)-1 )
#define  EvRound(x)      ( x>=0 ? int(x+0.5) : int(x-0.5) )
#define  EvCeil(x)       ( x>0  ? int(x)+1 : int(x) )

// ��alignΪ��׼����
inline int evAlignFill( int size, int align )
{
	assert( (align & (align-1)) == 0 && size < INT_MAX );
	return (size + align - 1) & -align;
}

inline int evAlignCut( int size, int align )
{
	assert( (align & (align-1)) == 0 && size < INT_MAX );
	return (size) & -align;
}

// ����0 - 1�����
inline float amtRandFloat()  { return rand()/(float(RAND_MAX)+1); }

// -------------------------------------
typedef struct AmtModel
{
	int    mNumInit;
	int    mMinWin;
	int    mTreesNum;
	int    mFeatsNum;
	float  mNccThesame;
	float  mValid;
	float  mThrFern;
	float  mThrNN;
	float  mThrNNvalid;
	int    mPatchSize;
}AmtModel;

inline AmtModel amtModel(int iMinWin, int iPatchSize, float fNccThesame, float fValid,
				         int iTreesNum, int iFeatsNum, float fThrFern, float fThrNN, float fThrNNvalid)
{
	AmtModel model;
	model.mMinWin     = iMinWin;
	model.mPatchSize  = iPatchSize;
	model.mNccThesame = fNccThesame;
	model.mValid      = fValid;
	model.mTreesNum   = iTreesNum;
	model.mFeatsNum   = iFeatsNum;
	model.mThrFern    = fThrFern;
	model.mThrNN      = fThrNN;
	model.mThrNNvalid = fThrNNvalid;
	return model;
}

// -------------------------------------
// synthesis of positive examples during initialization
typedef struct AmtPpar
{
	int mNumClosest;
	int mNumWarps;
	int mNoise;
	int mAngle;
	float mShift;
	float mScale;
}AmtPparInit,AmtPparUpdata;

inline AmtPpar amtPpar(int iNumClosest, int iNumWarps, int iNoise, int iAngle, float iShift, float iScale)
{
	AmtPpar ppar;
	ppar.mNumClosest = iNumClosest;
	ppar.mNumWarps = iNumWarps;
	ppar.mNoise = iNoise;
	ppar.mAngle = iAngle;
	ppar.mShift = iShift;
	ppar.mScale = iScale;
	return ppar;
}
// -------------------------------------
// negative examples initialization/update
typedef struct AmtNpar
{
	float mOverlap;
	int   mNumPatches;
}AmtNpar;

inline AmtNpar amtNpar(float fOverlap, int iNumPatches)
{
	AmtNpar Npar;
	Npar.mOverlap = fOverlap;
	Npar.mNumPatches = iNumPatches;
	return Npar;
}

// ----------------------------------------
struct AmtBbox
{
	CvPoint2D32f mPointLT;
	CvPoint2D32f mPointRD;
	const AmtBbox& operator=(const AmtBbox& src)
	{
		if(this != &src)
		{
			mPointLT.x = src.mPointLT.x;
			mPointLT.y = src.mPointLT.y;
			mPointRD.x = src.mPointRD.x;
			mPointRD.y = src.mPointRD.y;
		}
		return *this;
	}
};

inline AmtBbox amtBbox(float ix1, float iy1, float ix2, float iy2)
{
	AmtBbox Bbox;
	Bbox.mPointLT.x = ix1;
	Bbox.mPointLT.y = iy1;
	Bbox.mPointRD.x = ix2;
	Bbox.mPointRD.y = iy2;
	return Bbox;
}

// -------------------------------------
typedef struct AmtDetectResult
{
	AmtBbox mBB[DETECT_NUM_PER_FRAME];
	float   mConf[DETECT_NUM_PER_FRAME];
	int     mSize[DETECT_NUM_PER_FRAME];
	int     mMatchNum;
}AmtDetectResult;


typedef struct AmtDetector
{
	int     mIdx;
	AmtBbox mBB;
	float   mConf1;        // 1
	float   mConf2;        // 1
	int     mIsIn[3];      // 3
	float   mFernConf;     // 1
	int*    mPattern;      // mTreesNum
	float*  mPatch;        // mPatchSize*mPatchSize
}AmtDetector;


typedef struct AmtTrackResult
{
	AmtBbox mBB;
	CvPoint mPos;
	float   mConf;
	bool    mValid;
	bool    mFlag;
	void clean()
	{
		mBB    = amtBbox(0, 0, 0, 0);
		mPos   = cvPoint(0, 0);
		mConf  = 0;
		mValid = false;
		mFlag  = false;
	}
}AmtTrackResult;

typedef struct AmtGridBbox
{
	int		 mIndex;
	AmtBbox  mBbox;
	int      mArea;
	int      mNearNum;
	int		 mScaleIndex;
	float    mScaleCoef;
	CvSize   mScaleSize;
}AmtGridBbox;

typedef struct AmtObjectROI
{
	CvRect mObjRect;
	CvRect mROI;        // Ԥ��ROI����
	float  mExpRate;    // ROI��չϵ��
	float  mShiftRX;
	float  mShiftRY;
	int    mScaleIdx;   // ��ǰ���ڳ߶Ȳ�����
	int*   mScanBboxIdx;
	int    mScanBboxN;
	int    mScanBboxMaxN;

	int    mScanScaleN;
	struct _ROIScaleInfo
	{
		int mScaleIdx;
		int mFirstBboxIdx;
		int mBboxNumW;
		int mBboxNumH;
		int mBboxIdxBeginX;
		int mBboxIdxEndinX;
		int mBboxIdxBeginY;
		int mBboxIdxEndinY;
	}mScaleInfo[20];

}AmtObjectROI;

typedef struct AmtFernTrainData
{
	int   mNums;
	int   mMaxNums;
	int*  mData;
	int** mDataIdx;
}AmtFernTrainData;

typedef struct AmtPatchTrainData
{
	int     mNums;
	int     mMaxNums;
	float*  mData;
	float** mDataIdx;
}AmtPatchTrainData;


// ----------------------------------------
struct AmtTraceResult
{
	AmtBbox   mBB;
	CvPoint   mPos;
	double    mConf;
	int       mSize;
	bool      mValid;
	long long mTimeStamp;
	int       mMoveTime;//��ǰ����ǰһ����˶�ʱ����(������PauseTime)(ms)
	int       mPauseTime;//�ڵ�ǰ��ֹͣ������ʱ����(ms)
	int		  mMissTime;//�ڵ�ǰ��켣��ʧ��ʱ����(ms)
	int       mPosType;//0(Ĭ��ֵ,�޽��),1(���ٳɹ���ȷ��),2(���ٳɹ��Ҿ�1��ƫ��),3(���ٳɹ��Ҿ۶���ƫ���û�о���ƫ��),4(���ٳɹ�����ȷ�����޾����м����),5(���ٳɹ�����ȷ��)

	void Clean()    //6(����ʧ��),7(����ʧ���Ҿ�1��)��8(����ʧ���Ҿ۶���),9(����ʧ�����޾���),11(��ʧ��),12(��������)
	{
		mBB.mPointLT.x = 0;
		mBB.mPointLT.y = 0;
		mBB.mPointRD.x = 0;
		mBB.mPointRD.y = 0;
		mPos.x = 0;
		mPos.y = 0;
		mConf  = 0;
		mSize  = 0;
		mValid = false;
		mTimeStamp = 0;
		mMoveTime  = 0;
		mPauseTime = 0;
		mMissTime  = 0;
		mPosType   = 0;
	}

	const AmtTraceResult& operator=(const AmtTraceResult& src)
	{
		if(this != &src)
		{
			mBB = src.mBB;
			mPos.x = src.mPos.x;
			mPos.y = src.mPos.y;
			mConf  = src.mConf;
			mSize  = src.mSize;
			mValid = src.mValid;
			mTimeStamp = src.mTimeStamp;
			mMoveTime  = src.mMoveTime;
			mPauseTime = src.mPauseTime;
			mMissTime  = src.mMissTime;
			mPosType   = src.mPosType;
		}
		return *this;
	}
};


// ----------------------------------------�����㷨����---------------------------
struct AmtParam
{
public:
	AmtParam() : mMinWin(24), mLowScaleLevel(10), mHighScaleLevel(10),
		mShift(0.1), mPatchsize(15), mDrawResults(0)
	{}

	AmtParam(int _minWin, int _lowScaleLevel, int _highScaleLevel, float _shift=0.1, int _patchsize=15, int _drawResults=0)
		: mMinWin(_minWin), mLowScaleLevel(_lowScaleLevel), mHighScaleLevel(_highScaleLevel),
		mShift(_shift), mPatchsize(_patchsize), mDrawResults(_drawResults)
	{}

	~AmtParam() {}

	int mMinWin;
	int mLowScaleLevel;
	int mHighScaleLevel;
	float mShift;
	int mPatchsize;
	int mDrawResults;
};

struct AmtExternMode
{
	bool mShadowCorrect;

	// ��������չ
};

struct AmtLogRecord
{
	char mLogPath[MAX_PATH];
	FILE* mLogFile;
	int mLogFileDay;
	char mLogInfor[256];
};

// ========================= utile function ==============================
// @output: (bool) % ���ο��Ƿ���Ч
// @input : bb     % Ŀ����ο�
inline bool amtCheckBboxDef(AmtBbox* bb)
{return (bb->mPointRD.x - bb->mPointLT.x) > 0 && (bb->mPointRD.y - bb->mPointLT.y) > 0;}

// @output: (bool)  % ���ο��Ƿ񳬳�ͼ��߽�
// @input : BB      % Ŀ����ο�
//          imgSize % ͼ��ߴ�
inline bool amtCheckBboxOut(AmtBbox* BB, CvSize imgSize)
{return BB->mPointLT.x >= imgSize.width || BB->mPointLT.y >= imgSize.height || BB->mPointRD.x < 1 || BB->mPointRD.y < 1;}

// �������п��ܵ�Ԫ��
CvMat* amtNtuples(CvMat* col, CvMat* row, bool colUp);

//�����������,Ԫ��Ϊ0 ~ n-1
void amtRandPermutation(int n, int* perm, bool IsEmpty=true);

// =========================== Cv function ======================================
#define EV_COPY_ROI_TO_ALL 0
#define EV_COPY_ROI_TO_ROI 1
#define EV_COPY_ALL_TO_ROI 2
/*void evCopy(uchar* pSrc, uchar* pDst, int imgW, int imgH, CvRect* pROI=NULL, int algoType=EV_COPY_ROI_TO_ALL);*/
void evCopy(IplImage* pSrc, IplImage* pDst, CvRect* pROI=NULL, int algoType=EV_COPY_ROI_TO_ALL);


// ------------- AmtMutex ---------
class AmtMutex
{
public:
	AmtMutex();
	~AmtMutex();
	int Lock();
	int Unlock();

protected:

#ifdef WIN32 || WIN64
	CMutex m_Mutex;
#else // for Linux
	pthread_mutex_t m_Mutex;
#endif

private:
};

#endif


