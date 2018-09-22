// ================================================================
// AMT[Adaptive Matching Tracking] -- Tracker and Classifier Fusion
// ================================================================

#include "AMT.h"
#include "Track.h"

EvTrackCore* g_TrackCore[10] = {NULL};
AmtMutex MutexAMT;


AmtAlgoParam AmtParamInit(int _minWin, int _lowScaleLevel, int _highScaleLevel, float _shift, int _patchsize, int _drawResults)
{
	AmtAlgoParam temp;

	temp.mMinWin         = _minWin;
	temp.mLowScaleLevel  = _lowScaleLevel;
	temp.mHighScaleLevel = _highScaleLevel;
	temp.mShift          = _shift;
	temp.mPatchsize      = _patchsize;
	temp.mDrawResults    = _drawResults;

	return temp;
}

//对于 pAMT 的初始化， 主要是 m_ObjStatus列表，以及 m_szChannelID
int AmtStructInit(EvAMT* pAMT)
{
	AmtObjStatus* m_ObjStatus;
	int i;

	if (NULL == pAMT)
	{
		return 1;
	}

	m_ObjStatus = pAMT->m_ObjStatus;
	for (i=0; i<MAX_OBJECT_NUM; i++)
	{
		m_ObjStatus[i].mStatus      = 0;
		m_ObjStatus[i].mID          = -1;
		m_ObjStatus[i].mBbox        = cvRect(0, 0, 0, 0);
		m_ObjStatus[i].mPos         = cvPoint(0, 0);
		m_ObjStatus[i].mCountMiss   = 0;
		m_ObjStatus[i].mUnstableNum = 0;
	}

	// 扩展模式参数初始化
	pAMT->m_ExternParam.mShadowCorrect = 0;

	memset(pAMT->m_szChannelID, 0, 256*sizeof(char));
	pAMT->m_iChannelIdx = -1;

	return 0;
}

int AmtStructUnInit(EvAMT* pAMT)
{
	int m_iChannelIdx;
	if (NULL == pAMT)
	{
		return 1;
	}

	m_iChannelIdx = pAMT->m_iChannelIdx;
	if (m_iChannelIdx >= 0)
	{
		delete g_TrackCore[m_iChannelIdx];
		g_TrackCore[m_iChannelIdx] = NULL;
	}

	return 0;
}

//在g_TrackCore[10]中寻找空余的空间，如果找到了，将序号传给pAMT->m_iChannelIdx, 分配空间

// Init/Reset Algorithm 
int AmtSetConfig(EvAMT* pAMT, const char* const szChannelID, int iImageWidth, int iImageHeight, AmtAlgoParam sParam, CvRect* ROI)
{
	if(NULL == szChannelID)
	{
		return 101;
	}

	if(0 == strlen(szChannelID))
	{
		return 102;
	}

	if (iImageWidth <= 0 || iImageHeight <= 0)
	{
		return 103;
	}

	if (pAMT->m_iChannelIdx < 0)
	{
		MutexAMT.Lock();

		// 此对象跟踪核未创建
		int i;
		for (i=0; i<10; i++)
		{
			if (NULL == g_TrackCore[i])
			{
				pAMT->m_iChannelIdx = i;
				break;
			}
		}

		if (pAMT->m_iChannelIdx < 0)
		{
			return 104;
		}

		// 保存通道PUID
		strcpy(pAMT->m_szChannelID, szChannelID);

		g_TrackCore[pAMT->m_iChannelIdx] = new EvTrackCore(pAMT->m_szChannelID);
		sprintf(g_TrackCore[pAMT->m_iChannelIdx]->m_WinName, "TrackCore_%s_%s", g_EvTrackCoreVer, pAMT->m_szChannelID);

		MutexAMT.Unlock();
	}
	g_TrackCore[pAMT->m_iChannelIdx]->SetConfig(iImageWidth, iImageHeight, (AmtParam*)(&sParam), ROI);

	return pAMT->m_iChannelIdx;  // 返回通道索引
}

// Run Algorithm
int AmtExecute(EvAMT* pAMT, IplImage* pFrame, long long TimeStamp, int iMotionDirect)
{
	EvTrackCore* sTrackCore;
	AmtSingleObject* pObj;
	AmtObjStatus* m_ObjStatus;
	int iObjActiveNum;
	int i;

	if (NULL == pAMT)
	{
		return 106;
	}

	if (NULL == pFrame)
	{
		return 105;
	}

	if (pAMT->m_iChannelIdx < 0)
	{
		return 104;
	}

	sTrackCore = g_TrackCore[pAMT->m_iChannelIdx];
	// 更新传递扩展参数
	sTrackCore->m_ExternControl.mShadowCorrect = pAMT->m_ExternParam.mShadowCorrect;
	// 执行当前帧分析
	sTrackCore->Execute(pFrame, TimeStamp, iMotionDirect);

	// 填充目标跟踪输出结果
	pObj = sTrackCore->m_ObjectSets;
	m_ObjStatus = pAMT->m_ObjStatus;
	iObjActiveNum = 0;
	for (i=0; i<MAX_OBJECT_NUM; i++)
	{
		if (true == pObj->m_IsActive)
		{
			AmtTraceResult* pTraceResult = pObj->mTraceResult;
			int iTraceNum = pObj->mTraceNum;
			AmtBbox BB = pTraceResult[iTraceNum-1].mBB;
			m_ObjStatus[i].mBbox = cvRect(BB.mPointLT.x, BB.mPointLT.y, BB.mPointRD.x-BB.mPointLT.x+1, BB.mPointRD.y-BB.mPointLT.y+1);

			m_ObjStatus[i].mPos = cvPoint(pTraceResult[iTraceNum-1].mPos.x, pTraceResult[iTraceNum-1].mPos.y);
			m_ObjStatus[i].mID = pObj->m_ID;
			m_ObjStatus[i].mCountMiss   = pObj->mCurrCountMiss;
			m_ObjStatus[i].mUnstableNum = pObj->mUnstableNum;
			m_ObjStatus[i].mStatus = 1; // 0-目标不存在, 1-目标激活状态

			iObjActiveNum ++;
		}
		else
		{
			m_ObjStatus[i].mStatus = 0;
		}
		pObj ++;
	}
	
	// 返回目标跟踪结果更新个数
	return iObjActiveNum;
}

// Create a New Object
int AmtCreateObject(EvAMT* pAMT, CvRect InitBox, int iObjID, long long TimeStamp) // 此处InitBox是车身块 
{
	int iObjIndex = -1;
	AmtBbox tInitBB;

	if (NULL == pAMT)
	{
		return 105;
	}

	if (pAMT->m_iChannelIdx < 0)
	{
		return 104;
	}

	// 新目标索引
	tInitBB = amtBbox(InitBox.x, InitBox.y, InitBox.x+InitBox.width-1, InitBox.y+InitBox.height-1);  //车身块参数传入
	iObjIndex = g_TrackCore[pAMT->m_iChannelIdx]->CreateObject(tInitBB, iObjID, TimeStamp);//根据通道号选择g_TrackCore, 然后用 传入的 车身块, ID, 时间戳 三个信息来创建

	// 更新对应目标状态
	if (iObjIndex >= 0)
	{
		AmtSingleObject* pObj = &(g_TrackCore[pAMT->m_iChannelIdx]->m_ObjectSets[iObjIndex]);
		AmtObjStatus* m_ObjStatus = pAMT->m_ObjStatus;

		// 如果目标轨迹点为1，则为新创建目标，更新初始数据
		// 否则为已创建的同ID目标
		if ( 1 == pObj->mTraceNum )
		{
			AmtTraceResult* pTraceResult = pObj->mTraceResult;
			AmtBbox BB = pTraceResult[0].mBB;
			m_ObjStatus[iObjIndex].mBbox = cvRect(BB.mPointLT.x, BB.mPointLT.y, BB.mPointRD.x-BB.mPointLT.x+1, BB.mPointRD.y-BB.mPointLT.y+1);
			m_ObjStatus[iObjIndex].mPos = cvPoint(pTraceResult[0].mPos.x, pTraceResult[0].mPos.y);
			m_ObjStatus[iObjIndex].mID = pObj->m_ID;
			m_ObjStatus[iObjIndex].mCountMiss   = 0;
			m_ObjStatus[iObjIndex].mUnstableNum = 0;
			m_ObjStatus[iObjIndex].mStatus = 1;
		}
	}

	return iObjIndex;  // 返回目标索引
}

// Clean a Object
int AmtCleanObject(EvAMT* pAMT, int iObjID)
{
	int iObjIndex = -1;
	AmtObjStatus* m_ObjStatus;

	if (NULL == pAMT)
	{
		return 105;
	}

	if (pAMT->m_iChannelIdx < 0)
	{
		return 104;
	}
	
	iObjIndex = g_TrackCore[pAMT->m_iChannelIdx]->CleanObject(iObjID);

	// 目标找到并成功销毁，更新对应状态
	m_ObjStatus = pAMT->m_ObjStatus;
	if (iObjIndex >= 0)
	{
		assert(m_ObjStatus[iObjIndex].mID == iObjID);
		m_ObjStatus[iObjIndex].mStatus      = 0;
		m_ObjStatus[iObjIndex].mID          = -1;
		m_ObjStatus[iObjIndex].mBbox        = cvRect(0, 0, 0, 0);
		m_ObjStatus[iObjIndex].mPos         = cvPoint(0, 0);
		m_ObjStatus[iObjIndex].mCountMiss   = 0;
		m_ObjStatus[iObjIndex].mUnstableNum = 0;
	}

	return 0;
}

int AmtGetObjUsedNum(EvAMT* pAMT)
{
	int iObjUsedNum = 0;
	if (pAMT && pAMT->m_iChannelIdx >= 0)
	{
		iObjUsedNum = g_TrackCore[pAMT->m_iChannelIdx]->GetObjUsedNum();
	}
	
	return iObjUsedNum;
}

int AmtGetObjCurrNum(EvAMT* pAMT)
{
	int iObjCurrNum = 0;
	if (pAMT && pAMT->m_iChannelIdx >= 0)
	{
		iObjCurrNum = g_TrackCore[pAMT->m_iChannelIdx]->GetObjCurrNum();
	}

	return iObjCurrNum;
}

const char* AmtGetAlgoVersion()
{
	return g_EvTrackCoreVer;
}

int AmtGetObjTrajectory(EvAMT* pAMT, int iObjID, CvPoint** pPoints, int* pNum)
{
	int iObjPointsNum = 0;
	if (pAMT->m_iChannelIdx >= 0)
	{
		iObjPointsNum = g_TrackCore[pAMT->m_iChannelIdx]->GetObjTrajectory(iObjID, pPoints, pNum);
	}

	return iObjPointsNum;
}

const IplImage* AmtGetAlgoDraw(EvAMT* pAMT)
{
	IplImage* pAlgoDraw = NULL;
	if (pAMT->m_iChannelIdx >= 0)
	{
		pAlgoDraw = g_TrackCore[pAMT->m_iChannelIdx]->GetAlgoDraw();
	}

	return pAlgoDraw;
}