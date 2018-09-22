// ================================================================
// AMT[Adaptive Matching Tracking] -- Tracker and Classifier Fusion
// ================================================================

#ifndef EV_AMT_H
#define EV_AMT_H

#ifdef _cplusplus
extern "C" 
{
#endif

#ifdef EVAMT_DLL
	#ifdef WIN32 || DSP_PLATFORM
		#ifdef EVAMT_EXPORTS
			#define EVAMT_API __declspec(dllexport)
		#else
			#define EVAMT_API __declspec(dllimport)
		#endif
	#else
		#define EVAMT_API __attribute__ ((__visibility__("default")))
	#endif
#else
	#define     EVAMT_API  
#endif

#ifdef DSP_PLATFORM
	#include "Base_OpenCV_Types.h"  //DSP_PLATFORM
#else
	#include <cv.h>   //WIN32 or Linux 
#endif

#define  MAX_OBJECT_NUM   10      // 最大分析目标个数 shj

// Motion Direct
enum{AMT_NEAR_TO_FAR = 0, AMT_FAR_TO_NEAR = 1, AMT_ALL_AROUND = 2};

/************************************************************************/
/* Struct of Object Status                                              */
/************************************************************************/
typedef struct _AmtObjStatus
{
	int      mStatus;      // 目标状态
	int      mID;          // 目标ID编号
	CvRect   mBbox;        // 目标边界框
	CvPoint  mPos;         // 目标中心点
	int      mCountMiss;   // 目标丢失计数
	int      mUnstableNum; // 目标不稳定计数
}AmtObjStatus;

/************************************************************************/
/* Struct of Algorithm Parameter                                        */
/************************************************************************/
typedef struct _AmtAlgoParam
{
	int   mMinWin;
	int   mLowScaleLevel;
	int   mHighScaleLevel;
	float mShift;
	int   mPatchsize;
	int   mDrawResults;
}AmtAlgoParam;

EVAMT_API AmtAlgoParam AmtParamInit(int _minWin, int _lowScaleLevel, int _highScaleLevel, float _shift, int _patchsize, int _drawResults);

/************************************************************************/
/* Struct of Algorithm Control Parameter of Extern Mode                 */
/************************************************************************/
typedef struct _AmtExternParam
{
	int mShadowCorrect;

	// 后续待扩展
}AmtExternParam;

/************************************************************************/
/* API : Class of EvAMT(Adaptive Matching Tracking)                     */
/************************************************************************/
typedef struct  _EvAMT
{
	AmtObjStatus m_ObjStatus[MAX_OBJECT_NUM];
	AmtExternParam m_ExternParam;

	char m_szChannelID[256];  // 通道号
	int  m_iChannelIdx;       // 通道索引
}EvAMT;

EVAMT_API int AmtStructInit(EvAMT* pAMT);
EVAMT_API int AmtStructUnInit(EvAMT* pAMT);

// brief  : Init/Reset Algorithm 
// retval : Channel Index [0 - 9: succeed, other: fail]
EVAMT_API int AmtSetConfig(EvAMT* pAMT, const char* const szChannelID, int iImageWidth, int iImageHeight, AmtAlgoParam sParam, CvRect* ROI);

// brief  : Run Algorithm 
// retval : The Number of Object That Still Active
EVAMT_API int AmtExecute(EvAMT* pAMT, IplImage* pFrame, long long TimeStamp, int iMotionDirect);

// brief  : Create a New Object
// retval : Object Index [0 - MAX_OBJECT_NUM: succeed, other: fail]
EVAMT_API int AmtCreateObject(EvAMT* pAMT, CvRect InitBox, int iObjID, long long TimeStamp);

// brief  : Clean a Object
// retval : [0: succeed, other: fail]
EVAMT_API int AmtCleanObject(EvAMT* pAMT, int iObjID);

// brief  : Get Number of Object has Been Used
// retval : The Number
EVAMT_API int AmtGetObjUsedNum(EvAMT* pAMT);

// brief  : Get Number of Object used Real-time
// retval : The Number
EVAMT_API int AmtGetObjCurrNum(EvAMT* pAMT);

// brief  : Get Algorithm-Version
// retval : The String of Version Information
EVAMT_API const char* AmtGetAlgoVersion();

// brief  : Get The Object Trajectory[input:iObjID, output:pPoints,pNum]
// retval : The Number of Trajectory
EVAMT_API int AmtGetObjTrajectory(EvAMT* pAMT, int iObjID, CvPoint** pPoints, int* pNum);

EVAMT_API const IplImage* AmtGetAlgoDraw(EvAMT* pAMT);

#ifdef _cplusplus
}
#endif  //_cplusplus

#endif //EV_AMT_H





