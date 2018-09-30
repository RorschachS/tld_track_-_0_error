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

#define  MEMORY_EXPANSION_STEP  1.5     // 内存扩展步长
#define  MAX_ANALY_OBJECT_NUM   10      // 最大分析目标个数 shj

const int DETECT_NUM_PER_FRAME = 30;    // 每帧目标检测上限
const int RECORD_POINTS_NUM    = 250;   // 轨迹点长度上限
const int SAMPLE_REGION_NUM    = 5;     // 轨迹采样区间数
const int MINIMUM_OBJECT_RECT  = 10;    // 最小目标大小

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

// 以align为基准补齐
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

// 产生0 - 1随机数
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
	CvRect mROI;        // 预测ROI区域
	float  mExpRate;    // ROI扩展系数
	float  mShiftRX;
	float  mShiftRY;
	int    mScaleIdx;   // 当前所在尺度层索引
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
	int       mMoveTime;//当前点与前一点的运动时间间隔(不包括PauseTime)(ms)
	int       mPauseTime;//在当前点停止不动的时间间隔(ms)
	int		  mMissTime;//在当前点轨迹丢失的时间间隔(ms)
	int       mPosType;//0(默认值,无结果),1(跟踪成功且确定),2(跟踪成功且聚1类偏离),3(跟踪成功且聚多类偏离或没有聚类偏离),4(跟踪成功但不确定且无聚类有检测结果),5(跟踪成功但不确定)

	void Clean()    //6(跟踪失败),7(跟踪失败且聚1类)，8(跟踪失败且聚多类),9(跟踪失败且无聚类),11(丢失点),12(降采样点)
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


// ----------------------------------------设置算法参数---------------------------
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

	// 后续待扩展
};

struct AmtLogRecord
{
	char mLogPath[MAX_PATH];
	FILE* mLogFile;
	int mLogFileDay;
	char mLogInfor[256];
};

// ========================= utile function ==============================
// @output: (bool) % 矩形框是否有效
// @input : bb     % 目标矩形框
inline bool amtCheckBboxDef(AmtBbox* bb)
{return (bb->mPointRD.x - bb->mPointLT.x) > 0 && (bb->mPointRD.y - bb->mPointLT.y) > 0;}

// @output: (bool)  % 矩形框是否超出图像边界
// @input : BB      % 目标矩形框
//          imgSize % 图像尺寸
inline bool amtCheckBboxOut(AmtBbox* BB, CvSize imgSize)
{return BB->mPointLT.x >= imgSize.width || BB->mPointLT.y >= imgSize.height || BB->mPointRD.x < 1 || BB->mPointRD.y < 1;}

// 计算所有可能的元组
CvMat* amtNtuples(CvMat* col, CvMat* row, bool colUp);

//随机排列数组,元素为0 ~ n-1
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


