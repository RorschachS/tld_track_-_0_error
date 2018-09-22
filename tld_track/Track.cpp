// ================================
// EvTrackCore.cpp
// ================================

#include "Track.h"


EvTrackCore::EvTrackCore(char* pChannelID)
{
	m_MaxiObjNum = MAX_ANALY_OBJECT_NUM;  // 最大跟踪上限个数
	m_ObjectSets = new AmtSingleObject[m_MaxiObjNum];
	m_UsedObjNum = 0;
	m_CurrObjNum = 0;

	// ------------------ Extern Mode ----------------
	m_ExternControl.mShadowCorrect = false;

	// -------------------- Parameter --------------------
	m_AlgoParam.mMinWin = 24;
	m_AlgoParam.mLowScaleLevel = 10;
	m_AlgoParam.mHighScaleLevel= 10;
	m_AlgoParam.mShift = 0.1;
	m_AlgoParam.mPatchsize = 15;
	m_AlgoParam.mDrawResults = 0;

	// ------------------- Model Param ----------------------
	// ------------------ Do-Not-Change -----------------------------
	//opt->model = struct('min_win',min_win, 'patchsize',patchsize, 'ncc_thesame',0.95, 'valid',0.5,
	//                    'num_trees',10, 'num_features',13, 'thr_fern',0.5, 'thr_nn',0.65, 'thr_nn_valid',0.7);
	m_Model = amtModel(m_AlgoParam.mMinWin, m_AlgoParam.mPatchsize, 0.95, 0.5, 10, 13, 0.5, 0.65, 0.7);

	// opt.p_par_init = struct('num_closest',10, 'num_warps',20, 'noise',5, 'angle',20, 'shift',0.02, 'scale',0.02);
	m_PparInit = amtPpar(10, 20, 5, 20, 0.02, 0.02);

	//opt.p_par_update = struct('num_closest',10, 'num_warps',10, 'noise',5, 'angle',10, 'shift',0.02, 'scale',0.02);
	m_PparUpdata = amtPpar(10, 10, 5, 10, 0.02, 0.02);

	//opt.n_par = struct('overlap',0.2,'num_patches',100);
	m_Npar = amtNpar(0.2, 100);

	// ================ Clear Object's Pointer ====================
	// -------------------- Null for Pointer -----------------------
	m_SrcImgSize.width = 0;
	m_SrcImgSize.height= 0;
	m_ImageSize.width  = 0;
	m_ImageSize.height = 0;
	m_DrawImage     = NULL;
	m_CurrImage     = NULL;
	m_BlurImage     = NULL;
	m_PrevImage     = NULL;

	// ---------------------- Public Resource ----------------
	// 检测模块中间结果
	m_DetectFiltNum = 0;
	m_DetectFilter = new AmtDetector[DETECT_NUM_PER_FRAME];
	for (int i=0; i<DETECT_NUM_PER_FRAME; i++)
	{
		m_DetectFilter[i].mPattern = new int[m_Model.mTreesNum];
		m_DetectFilter[i].mPatch   = new float[m_Model.mPatchSize * m_Model.mPatchSize + 1]; // 1 for norm
	}

	// ---------------------- RESULT OUTPUT ----------------------
	m_Font = cvFont(1, 1);
	m_CurrTimeStamp = 0;
	m_LastTimeStamp = 0;
	m_iTimeInterval = 0;
	m_AlgoUsedTime  = 0;
	m_AlgoShowTime  = 0;
	m_iFrameRateCount = 0;
	m_cFrameRateTime  = 0;

	//通道ID
	m_szChannelID[0] = 0;
	if (pChannelID)
	{
		strcpy(m_szChannelID, pChannelID);
	}

	// 开启日志记录
	mLogRecord.mLogFile = NULL;
	mLogRecord.mLogFileDay = -1;
	CreateLogDir(&mLogRecord, "AlgoLog", "EvAMT", pChannelID);

	// 算法创建日志记录
	//ConfigLog(&mLogRecord, false);

	m_IsInit = false;
}

EvTrackCore::~EvTrackCore()
{
	// 算法释放日志
	PrintfLog(&mLogRecord, m_CurrTimeStamp ? m_CurrTimeStamp : time(NULL)*1000, "Algo Release! \r\n\r\n");

	// --- 目标集资源释放 ---
	for (int i=0; i<m_MaxiObjNum; i++)
	{
		m_ObjectSets[i].Release();
	}
	delete []m_ObjectSets;
	m_ObjectSets = NULL;

	// --- 公用资源释放 ---
	for (int i=0; i<DETECT_NUM_PER_FRAME; i++)
	{
		delete [](m_DetectFilter[i].mPattern);
		m_DetectFilter[i].mPattern = NULL;
		delete [](m_DetectFilter[i].mPatch);
		m_DetectFilter[i].mPatch = NULL;
	}
	delete []m_DetectFilter;
	m_DetectFilter = NULL;

	// 关闭日志记录
	if(NULL != mLogRecord.mLogFile)
	{
		fclose(mLogRecord.mLogFile);
		mLogRecord.mLogFile = NULL;
	}

	if (true == m_IsInit)
	{
		cvReleaseImage(&m_DrawImage);       m_DrawImage = NULL;
		cvReleaseImage(&m_CurrImage);       m_CurrImage = NULL;
		cvReleaseImage(&m_BlurImage);       m_BlurImage = NULL;
		cvReleaseImage(&m_PrevImage);       m_PrevImage = NULL;

		m_IsInit = false;
	}
}

bool EvTrackCore::SetConfig(int iImageW, int iImageH, AmtParam* Param, CvRect* ROI)
{
	// ---------------------- Image ----------------------
	if (iImageW != m_SrcImgSize.width || iImageH != m_SrcImgSize.height)
	{
		if (true == m_IsInit)
		{
			cvReleaseImage(&m_DrawImage);
			cvReleaseImage(&m_CurrImage);
			cvReleaseImage(&m_BlurImage);
			cvReleaseImage(&m_PrevImage);
		}

		m_SrcImgSize = cvSize(iImageW, iImageH);

		m_ImageSize  = cvSize( evAlignCut(iImageW, 4), iImageH );
		m_DrawImage  = cvCreateImage(m_ImageSize, IPL_DEPTH_8U, 1);
		m_CurrImage  = cvCreateImage(m_ImageSize, IPL_DEPTH_8U, 1);
		m_BlurImage  = cvCreateImage(m_ImageSize, IPL_DEPTH_8U, 1);
		m_PrevImage  = cvCreateImage(m_ImageSize, IPL_DEPTH_8U, 1);
	}

	// 检测模块中间结果
	m_DetectFiltNum = 0;
	if (m_Model.mPatchSize != Param->mPatchsize)
	{
		for (int i=0; i<DETECT_NUM_PER_FRAME; i++)
		{
			delete [](m_DetectFilter[i].mPatch);
			m_DetectFilter[i].mPatch = new float[Param->mPatchsize * Param->mPatchsize + 1]; // 1 for norm
		}
	}

	// ---------------------- Param ---------------------- 读入外部传入的参数
	m_AlgoParam.mMinWin = Param->mMinWin;
	m_AlgoParam.mLowScaleLevel = Param->mLowScaleLevel;
	m_AlgoParam.mHighScaleLevel= Param->mHighScaleLevel;
	m_AlgoParam.mShift = Param->mShift;
	m_AlgoParam.mPatchsize = Param->mPatchsize;
	m_AlgoParam.mDrawResults = Param->mDrawResults;

	// ------------------ Do-Not-Change -----------------------------
	m_Model      = amtModel(m_AlgoParam.mMinWin, m_AlgoParam.mPatchsize, 0.95, 0.5, 12, 13, 0.5, 0.65, 0.7);
	m_PparInit   = amtPpar(10, 20, 5, 20, 0.02, 0.02);
	m_PparUpdata = amtPpar(10, 10, 5, 10, 0.02, 0.02);
	m_Npar       = amtNpar(0.2, 100); // (0.2, 100)

	// ----- Init Module -----
	m_DetecterModule.Init(m_ImageSize.width, m_ImageSize.height, m_Model.mTreesNum, m_Model.mFeatsNum);
	m_TrackerModule.Init(m_ImageSize.width, m_ImageSize.height);
	m_ExpertsModule.Init(m_Model.mPatchSize);

	// ---------------------- Object sets ----------------------
	for (int i=0; i<m_MaxiObjNum; i++)
	{
		m_ObjectSets[i].Release();//将跟踪目标列表清空
	}
	m_UsedObjNum = 0;
	m_CurrObjNum = 0;

	m_CurrTimeStamp = 0;
	m_LastTimeStamp = 0;
	m_iTimeInterval = 0;
	m_iFrameRateCount = 0;
	m_cFrameRateTime  = 0;

	ConfigLog(&mLogRecord, true);

	m_IsInit = true;
	return m_IsInit;
}

void EvTrackCore::SetObjParam(AmtBbox InitBox, int iID, AmtSingleObject* pObj)
{
	pObj->m_ID = iID;
	pObj->m_InitBox.mPointLT = cvPoint2D32f(InitBox.mPointLT.x, InitBox.mPointLT.y);
	pObj->m_InitBox.mPointRD = cvPoint2D32f(InitBox.mPointRD.x, InitBox.mPointRD.y);

	AmtParam* pInitParam = &(pObj->m_ObjParam);
	pInitParam->mMinWin         = m_AlgoParam.mMinWin;
	pInitParam->mLowScaleLevel  = m_AlgoParam.mLowScaleLevel;
	pInitParam->mHighScaleLevel = m_AlgoParam.mHighScaleLevel;
	pInitParam->mShift          = m_AlgoParam.mShift;
	pInitParam->mPatchsize      = m_AlgoParam.mPatchsize;
	pInitParam->mDrawResults    = m_AlgoParam.mDrawResults;

	// ------------------ Do-Not-Change -----------------------------
	pObj->m_Model      = amtModel(m_Model.mMinWin, m_Model.mPatchSize, m_Model.mNccThesame, m_Model.mValid,
		                          m_Model.mTreesNum, m_Model.mFeatsNum, m_Model.mThrFern, m_Model.mThrNN, m_Model.mThrNNvalid);
	pObj->m_PparInit   = amtPpar(m_PparInit.mNumClosest, m_PparInit.mNumWarps, m_PparInit.mNoise, m_PparInit.mAngle, m_PparInit.mShift, m_PparInit.mScale);
	pObj->m_PparUpdata = amtPpar(m_PparUpdata.mNumClosest, m_PparUpdata.mNumWarps, m_PparUpdata.mNoise, m_PparUpdata.mAngle, m_PparUpdata.mShift, m_PparUpdata.mScale);
	pObj->m_Npar       = amtNpar(m_Npar.mOverlap, m_Npar.mNumPatches);
}



//根据传入的块, ID, 时间戳创建目标
//如果该ID已有，且对应的目标激活，则返回该序号
//如果该ID没有加入，则找到空闲空间，
int EvTrackCore::CreateObject(AmtBbox InitBox, int iObjID, long long TimeStamp)
{
	AmtSingleObject *pObj = m_ObjectSets;
	int iObjIndex = -1;
	// ----- 查找同ID目标是否存在 -----
	for(int i=0; i<m_MaxiObjNum; i++)
	{
		// 目标处于激活状态，且目标ID对应
		if(true == pObj->m_IsActive && iObjID == pObj->m_ID)
		{
			iObjIndex = i;
			break;
		}
		pObj ++;
	}
	if (iObjIndex >= 0)  // 如果找到同ID目标，则返回此目标索引
	{
		return iObjIndex;
	}

	// ----- 查找空闲目标堆栈 -----
	pObj = m_ObjectSets;
	iObjIndex = -1;
	for(int i=0; i<m_MaxiObjNum; i++)
	{
		if(false == pObj->m_IsActive)
		{
			iObjIndex = i;
			break;
		}
		pObj ++;
	}

	if(iObjIndex < 0)
	{
		return -1;//都在激活状态, 没有空间
	}

	// 如果目标缓存没有被初始化，则使用记录计数
	if(false == pObj->m_IsInit)
	{
		m_UsedObjNum ++;
	}

	// 初始目标
	bool Re = SingleObjInit(m_CurrImage, InitBox, iObjID, TimeStamp, pObj);
	if ( false == Re)
	{
		return -1;
	}

	pObj->m_iIndex = iObjIndex;
	pObj->m_ColorTag = cvScalar(255*amtRandFloat(), 255*amtRandFloat(), 255*amtRandFloat());
	return iObjIndex;
}

int EvTrackCore::CleanObject(int iObjID)
{
	AmtSingleObject *pObj = m_ObjectSets;
	int iObjIndex = -1;
	for(int i=0; i<m_MaxiObjNum; i++)
	{
		// 目标处于激活状态，且目标ID对应
		if(true == pObj->m_IsActive && iObjID == pObj->m_ID)
		{
			iObjIndex = i;
			break;
		}
		pObj ++;
	}
	if (iObjIndex < 0)
		return -1;

	// Clean Object
	pObj->Clean();

	return iObjIndex;
}

// 创建目标
bool EvTrackCore::SingleObjInit(IplImage* PFrame, AmtBbox InitBox, int iID, long long TimeStamp, AmtSingleObject* pObj)
{
	// ================ Clear Object's Pointer ====================
	// check box
	pObj->m_ImgSize = cvGetSize(PFrame);//得到图像维度
	if (amtCheckBboxOut(&InitBox, pObj->m_ImgSize))//检查 InitBox 是否超出图像
	{
		//	fprintf(stdout, "[TLD init] Error: The init-box is out image !!!! \n");
		return false;
	}

	// ----------- 修正初始框满足左上右下 ------
	if (InitBox.mPointLT.x > InitBox.mPointRD.x)
	{
		float temp = InitBox.mPointLT.x;
		InitBox.mPointLT.x = InitBox.mPointRD.x;
		InitBox.mPointRD.x = temp;
		//	fprintf(stdout, "[TLD init] Error: 目标初始化点 [x] 相反 !!! \n");
	}
	if (InitBox.mPointLT.y > InitBox.mPointRD.y)
	{
		float temp = InitBox.mPointLT.y;
		InitBox.mPointLT.y = InitBox.mPointRD.y;
		InitBox.mPointRD.y = temp;
		//	fprintf(stdout, "[TLD init] Error: 目标初始化点 [y] 相反 !!! \n");
	}
	// 规则目标初始框
	if (InitBox.mPointLT.x < 0 || InitBox.mPointLT.x >= pObj->m_ImgSize.width || InitBox.mPointRD.x < 0 || InitBox.mPointRD.x >= pObj->m_ImgSize.width ||
		InitBox.mPointLT.y < 0 || InitBox.mPointLT.y >= pObj->m_ImgSize.height|| InitBox.mPointRD.y < 0 || InitBox.mPointRD.y >= pObj->m_ImgSize.height)
	{
		//	fprintf(stdout, "[TLD init] Warning : 目标初始越界!! \n");
		InitBox.mPointLT.x = MIN(MAX(InitBox.mPointLT.x, 0), (pObj->m_ImgSize.width -1));
		InitBox.mPointLT.y = MIN(MAX(InitBox.mPointLT.y, 0), (pObj->m_ImgSize.height-1));
		InitBox.mPointRD.x = MIN(MAX(InitBox.mPointRD.x, 0), (pObj->m_ImgSize.width -1));
		InitBox.mPointRD.y = MIN(MAX(InitBox.mPointRD.y, 0), (pObj->m_ImgSize.height-1));
	}

	CvSize boxSize = cvSize(InitBox.mPointRD.x - InitBox.mPointLT.x + 1, InitBox.mPointRD.y - InitBox.mPointLT.y + 1);
	if (boxSize.height < MINIMUM_OBJECT_RECT || boxSize.width < MINIMUM_OBJECT_RECT) //如果目标小于10x10,则过小
	{
		//	fprintf(stderr, "初始目标过小,无法跟踪!!! \n");
		return false;
	}

	if (boxSize.height < m_AlgoParam.mMinWin || boxSize.width < m_AlgoParam.mMinWin)
	{
		//	fprintf(stderr, "The init-box is litter small and re-init minWin with it ! \n");
		m_AlgoParam.mMinWin = boxSize.width < boxSize.height ? boxSize.width : boxSize.height;  // re-init minWin with box
	}

	// 设置部分初始值
	SetObjParam(InitBox, iID, pObj);//将 ID, InitBox 设置给 pObj中对应参数

	//========================= INITIALIZE DETECTOR ================================
	// Scanning grid
	if ( !(m_DetecterModule.BboxGridScan(pObj)) )
		return false;

	// Genera Features Template  放在mFernFeatures中
	m_DetecterModule.GenerateFeaturesTemplate(0);

	// Initialize Object's Detector
	m_DetecterModule.ObjectFernInit(pObj, pObj->mGridBboxData, m_DetecterModule.mFernFeatures, pObj->mScaleSizeData);

	// Initialize Object's NN_Experts
	m_ExpertsModule.ObjectInitNN(pObj);

	// ---------------------------------------------------------------------------
	// Initialize Trajectory
	pObj->mTraceResult[0].mBB    = pObj->m_InitBox;
	pObj->mTraceResult[0].mConf  = 1;
	pObj->mTraceResult[0].mSize  = 1;
	pObj->mTraceResult[0].mValid = true;
	pObj->mTraceResult[0].mPos   = cvPoint( (int)((pObj->m_InitBox.mPointLT.x + pObj->m_InitBox.mPointRD.x)*0.5),
		                                    (int)((pObj->m_InitBox.mPointLT.y + pObj->m_InitBox.mPointRD.y)*0.5) );
	pObj->mTraceResult[0].mTimeStamp = TimeStamp;
	pObj->mTraceNum = 1;
	pObj->SetMotionROI(TimeStamp);

	// 图像模糊处理
	cvSetImageROI(m_CurrImage, pObj->mAnalyROI.mROI);
	cvSetImageROI(m_BlurImage, pObj->mAnalyROI.mROI);
	cvSmooth(m_CurrImage, m_BlurImage, CV_GAUSSIAN, 11, 0, 2, 0);
	cvResetImageROI(m_CurrImage);
	cvResetImageROI(m_BlurImage);

	//============================= TRAIN DETECTOR =============================
	// Initialize structures
	BboxOverlap(pObj->m_InitBox, pObj->mGridBboxData, pObj->mGridBboxNum, m_DetecterModule.mGridOverlapData, &(pObj->mAnalyROI));

	// Generate Positive Examples
	AmtBbox bbP = GeneratePositiveData(pObj, m_DetecterModule.mGridOverlapData, m_CurrImage, m_BlurImage, pObj->m_PparInit);

	// Correct initial bound-box
	pObj->mTraceResult[0].mBB = bbP;

	// Variance threshold
	pObj->m_VarValue = m_ExpertsModule.PatchVariance(m_ExpertsModule.m_PatchTrainP.mDataIdx[0], m_ExpertsModule.m_PatternDim) / 2;

	// Generate Negative Examples
	GenerateNegativeData(pObj, m_DetecterModule.mGridOverlapData, m_CurrImage, m_BlurImage);

	// Split Negative Data to Training set and Validation set
	AmtFernTrainData  nX1, nX2;
	m_DetecterModule.SplitNegativeData(m_DetecterModule.m_FernTrainN, nX1, nX2);
	AmtPatchTrainData nEx1, nEx2;
	m_ExpertsModule.SplitNegativeData(m_ExpertsModule.m_PatchTrainN, nEx1, nEx2);

	//======================= Train using training set ========================
	// Fern
	m_DetecterModule.ObjectFernUpDate(pObj, &(m_DetecterModule.m_FernTrainP), &(nX1), 2, false);

	// Nearest Neighbor
	m_ExpertsModule.ObjectUpDateNN(pObj, &(m_ExpertsModule.m_PatchTrainP), &(nEx1));
	pObj->m_Model.mNumInit = pObj->mPatchNumP;

	//================== Estimate thresholds on validation set ===================
	// Fern
	float max_ConfFern = m_DetecterModule.ObjectFernEvaluate(nX2, pObj->m_ForestWeight);
	pObj->m_Model.mThrFern = MAX(max_ConfFern/pObj->m_Model.mTreesNum, pObj->m_Model.mThrFern);

	// Nearest neighbor
	int numNN = nEx2.mNums;
	float conf1, conf2;
	float max_ConfNN = 0;
	for (int i=0; i<numNN; i++)
	{
		m_ExpertsModule.ObjectEvaluateNN(pObj, pObj->m_Model.mNccThesame, pObj->m_Model.mValid, nEx2.mDataIdx[i], conf1, conf2);

		if (max_ConfNN < conf1)
		{
			max_ConfNN = conf1;
		}
	}
	pObj->m_Model.mThrNN = MAX(pObj->m_Model.mThrNN, max_ConfNN);
	pObj->m_Model.mThrNNvalid = MAX(pObj->m_Model.mThrNNvalid, pObj->m_Model.mThrNN);

	// ---- 目标初始状态标记 ----
	pObj->m_IsInit   = true;
	pObj->m_IsActive = true;
	return true;

}


bool EvTrackCore::Execute(IplImage* pFrame, long long TimeStamp, int iMotionDirect)
{
	// ========================== 算法处理 =============================
	m_AlgoBeginTime = clock();

	// 时间戳更新
	if(0 == m_LastTimeStamp && 0 == m_CurrTimeStamp)
	{
		m_iTimeInterval = 0;
		m_CurrTimeStamp = TimeStamp;
		m_LastTimeStamp = TimeStamp;
	}
	else
	{
		m_LastTimeStamp = m_CurrTimeStamp;
		m_CurrTimeStamp = TimeStamp;
		m_iTimeInterval = m_CurrTimeStamp - m_LastTimeStamp;
	}

	assert(pFrame->width == m_SrcImgSize.width && pFrame->height == m_SrcImgSize.height);
	assert(pFrame->nChannels == 1);

	// 图像格式转换
	{
		if (m_SrcImgSize.width == m_ImageSize.width)
		{
			evCopy(pFrame, m_CurrImage);
		}
		else
		{
			CvRect sAlignRect = cvRect(0,0,m_ImageSize.width, m_ImageSize.height);
			evCopy(pFrame, m_CurrImage, &sAlignRect, EV_COPY_ROI_TO_ALL);
		}

		if (m_DrawImage->nChannels == 3)
		{
			cvReleaseImage(&m_DrawImage);
			m_DrawImage = cvCreateImage(m_ImageSize, IPL_DEPTH_8U, 1);
		}
		if (true == m_AlgoParam.mDrawResults)
		{
			evCopy(m_CurrImage, m_DrawImage);
		}
	}

	AmtSingleObject* pObj = NULL;
	// ----- 统筹分析目标ROI及整体积分图 -----
	pObj = m_ObjectSets;
	m_DetecterModule.m_IsCalcIntegral = false;  // 保证每帧积分图计算一次
	int iHullX_LT = 10000, iHullX_RD = 0, iHullY_LT = 10000, iHullY_RD = 0;
	for (int i=0; i<m_MaxiObjNum; i++)
	{
		if (true == pObj->m_IsActive)
		{
			pObj->SetMotionROI(TimeStamp);

			CvRect &sObjROI = pObj->mAnalyROI.mROI;
			if (0 == sObjROI.width || 0 == sObjROI.height)
			{
				continue;
			}

			// 统计全局ROI
			iHullX_LT = MIN(iHullX_LT, sObjROI.x);
			iHullY_LT = MIN(iHullY_LT, sObjROI.y);
			iHullX_RD = MAX(iHullX_RD, sObjROI.x+sObjROI.width-1);
			iHullY_RD = MAX(iHullY_RD, sObjROI.y+sObjROI.height-1);
		}
		pObj ++;
	}

	CvRect iHullRect = cvRect(iHullX_LT, iHullY_LT, iHullX_RD-iHullX_LT+1, iHullY_RD-iHullY_LT+1);
	if (iHullRect.width >0 && iHullRect.width < m_ImageSize.width &&
		iHullRect.height>0 && iHullRect.height < m_ImageSize.height)
	{
		m_DetecterModule.CalcIntegralImage(m_CurrImage, &iHullRect);
		m_DetecterModule.m_IsCalcIntegral = true;

		// 图像模糊处理
		cvSetImageROI(m_CurrImage, iHullRect);
		cvSetImageROI(m_BlurImage, iHullRect);
		cvSmooth(m_CurrImage, m_BlurImage, CV_GAUSSIAN, 11, 0, 2, 0);
		cvResetImageROI(m_CurrImage);
		cvResetImageROI(m_BlurImage);
	}

	// 目标处理
	m_CurrObjNum = 0;
	pObj = m_ObjectSets;
	for (int i=0; i<m_MaxiObjNum; i++)
	{
		if (true == pObj->m_IsActive)
		{
			SingleObjExecute(pObj, m_CurrTimeStamp);

			// ----------- Check Time Stamp ---------------
			if (m_iTimeInterval <= 0 || m_iTimeInterval > 300)
			{
				sprintf(mLogRecord.mLogInfor, "[Warning] : ID_%d - TimeInterval_%d ms 帧间隔异常!\n", pObj->m_ID, m_iTimeInterval);
				PrintfLog(&mLogRecord, m_CurrTimeStamp, mLogRecord.mLogInfor);
			}

			// ----------- Check Track Drift ----------------
			if ( CheckTrackDrift(pObj->mTraceResult[pObj->mTraceNum-1].mBB, pObj->mAnalyROI.mROI) )
			{
				sprintf(mLogRecord.mLogInfor, "[Clean] : ID_%d - BB_[%d,%d,%d,%d] - ROI_[%d,%d,%d,%d]-跟踪结果漂移\n", pObj->m_ID, 
					                           (int)pObj->mTraceResult[pObj->mTraceNum-1].mBB.mPointLT.x, (int)pObj->mTraceResult[pObj->mTraceNum-1].mBB.mPointLT.y,
											   (int)pObj->mTraceResult[pObj->mTraceNum-1].mBB.mPointRD.x, (int)pObj->mTraceResult[pObj->mTraceNum-1].mBB.mPointRD.y, 
											   pObj->mAnalyROI.mROI.x, pObj->mAnalyROI.mROI.y, 
											   pObj->mAnalyROI.mROI.x+pObj->mAnalyROI.mROI.width, 
											   pObj->mAnalyROI.mROI.y+pObj->mAnalyROI.mROI.height);
				PrintfLog(&mLogRecord, m_CurrTimeStamp, mLogRecord.mLogInfor);

				pObj->Clean();
			}
			else
			{
				iMotionDirect = GetObjDirection(pObj->mTraceResult, pObj->mTraceNum);
				ConstrainTrackResult(&(pObj->mTraceResult[pObj->mTraceNum-1]), &(pObj->mTraceResult[pObj->mTraceNum-2]), &(pObj->mTraceResult[0]), iMotionDirect);
				m_CurrObjNum ++;
			}

			// ----------- 确认目标输出有效性 ----------
			int iTextureType = 0;
		//	iTextureType = CheckTrackTextureType(pObj->mTraceResult[pObj->mTraceNum-1].mBB);
			if ( iTextureType > 0 )
			{
				sprintf(mLogRecord.mLogInfor, "[Clean] : ID_%d - 目标纹理弱\n", pObj->m_ID);
				PrintfLog(&mLogRecord, m_CurrTimeStamp, mLogRecord.mLogInfor);

				pObj->Clean();
			}

			// ---------- 阴影切割校正 ---------
			if (true == m_ExternControl.mShadowCorrect)
			{
				CorrectShadowResult(pObj->mTraceResult[pObj->mTraceNum-1].mBB);
			}
		}
		pObj ++;
	}
	evCopy(m_CurrImage, m_PrevImage);
	m_AlgoEndinTime = clock();
	m_AlgoUsedTime = m_AlgoEndinTime - m_AlgoBeginTime;

	// ========================== 算法状态显示 ==========================
	m_AlgoBeginTime = clock();
	if (true == m_AlgoParam.mDrawResults)
	{
		// ------ DRAW RESULT ------
		for (int i=0; i<m_MaxiObjNum; i++)
		{
			if (true == m_ObjectSets[i].m_IsActive)
			{
				DrawResult(&(m_ObjectSets[i]), m_DrawImage);
			}
		}
		sprintf(m_Text, "#ImgSize: %d*%d #Time = %dms[%dms]", m_ImageSize.width, m_ImageSize.height, m_AlgoUsedTime, m_AlgoShowTime);
		cvPutText(m_DrawImage, m_Text, cvPoint(5,15), &m_Font, CV_RGB(0, 255, 0));
		sprintf(m_Text, "#UseObjNum : [%d] #CurrObjNum : [%d]", m_UsedObjNum, m_CurrObjNum);
		cvPutText(m_DrawImage, m_Text, cvPoint(5,30), &m_Font, CV_RGB(0, 255, 0));

		cvNamedWindow(m_WinName, 1);
		cvShowImage(m_WinName, m_DrawImage);
		cvWaitKey(1);
	}
	else
	{
		if (cvGetWindowHandle(m_WinName))
		{
			cvDestroyWindow(m_WinName);
		}
	}
	m_AlgoEndinTime = clock();
	m_AlgoShowTime = m_AlgoEndinTime - m_AlgoBeginTime;

	// ===================== 帧率统计 =====================
	if ( 0 == m_cFrameRateTime )
	{
		m_cFrameRateTime = clock();
		m_iFrameRateCount = 0;
	}
	else
	{
		m_iFrameRateCount ++;

		clock_t tempTime = clock();
		if ( tempTime - m_cFrameRateTime > 10000 )  // 间隔十秒
		{
			float fFPS = m_iFrameRateCount / 10.;
			if (fFPS < 5) // 过低帧率日志输出
			{
				sprintf(mLogRecord.mLogInfor, "[Warning] : 帧率过低! AveFPS is %.1f in 10 Sec.\n", fFPS);
				PrintfLog(&mLogRecord, m_CurrTimeStamp, mLogRecord.mLogInfor);
			}
			m_iFrameRateCount = 0;
			m_cFrameRateTime = tempTime;
		}
	}

	return true;
}

bool EvTrackCore::SingleObjExecute(AmtSingleObject* pObj, long long TimeStamp)
{
	AmtTraceResult tFrameResult;
	AmtBbox tLastBbox = pObj->mTraceResult[pObj->mTraceNum-1].mBB;

	// ========================= TRACKER ==============================
	// frame-to-frame tracking (MedianFlow)
	bool trackflag = TrackerExecute(pObj, &(tLastBbox), m_PrevImage, m_CurrImage);

	// ========================= DETECTOR ============================
	// detect appearances by cascaded detector (variance filter -> ensemble classifier -> nearest neighbored)
	int matchNum = DetecterExecute(pObj, m_CurrImage, m_BlurImage);

	// ======================== INTEGRATOR ==========================
	// if tracker is defined
	if (trackflag == true)
	{
		tFrameResult.mBB    = pObj->m_TrackResult.mBB;
		tFrameResult.mConf  = pObj->m_TrackResult.mConf;
		tFrameResult.mSize  = 1;
		tFrameResult.mValid = pObj->m_TrackResult.mValid;

		// if detection are also defined
		if (matchNum != 0)
		{
			// cluster detections
			BboxClusterConfidence(&pObj->m_DetectCluster, &pObj->m_DetectResult);

			// get indexes of all clusters that are far from tracker and are more confident than the tracker
			int CorrectIdx = 0;
			int CorrectNum = 0;
			for (int i=0; i<pObj->m_DetectCluster.mMatchNum; i++)
			{
				if (BboxOverlapOne(pObj->m_DetectCluster.mBB[i], pObj->m_TrackResult.mBB) < 0.5 && pObj->m_DetectCluster.mConf[i] > pObj->m_TrackResult.mConf)
				{
					CorrectIdx = i;
					CorrectNum ++;
					if (CorrectNum > 1)
					{
						break; // 如果找到两个满足条件的检测聚类，则无需用聚类结果矫正
					}
				}
			}

			// if there is ONE such a cluster, re-initialize the tracker
			if ( 1 == CorrectNum )
			{
				tFrameResult.mBB    = pObj->m_DetectCluster.mBB[ CorrectIdx ];
				tFrameResult.mConf  = pObj->m_DetectCluster.mConf[ CorrectIdx ];
				tFrameResult.mSize  = pObj->m_DetectCluster.mSize[ CorrectIdx ];
				tFrameResult.mValid = false;
			}
			else  // otherwise adjust the tracker's trajectory
			{
				AmtBbox meanBB   = amtBbox(0, 0, 0, 0);
				float   meanConf = 0;
				int iCloseNum = 0;
				for (int i=0; i<pObj->m_DetectResult.mMatchNum; i++)
				{
					// get sum of close detections with last track-result
					if (BboxOverlapOne(pObj->m_TrackResult.mBB, pObj->m_DetectResult.mBB[i]) > 0.5) // 0.7
					{
						meanBB.mPointLT.x += pObj->m_DetectResult.mBB[i].mPointLT.x;
						meanBB.mPointLT.y += pObj->m_DetectResult.mBB[i].mPointLT.y;
						meanBB.mPointRD.x += pObj->m_DetectResult.mBB[i].mPointRD.x;
						meanBB.mPointRD.y += pObj->m_DetectResult.mBB[i].mPointRD.y;
						meanConf          += pObj->m_DetectResult.mConf[i];
						iCloseNum         += 1;
					}
				}
				if (iCloseNum > 0)
				{
					// weighted average trackers trajectory with the close detections
					int weight;
					if (pObj->m_TrackResult.mValid == false) // 跟踪结果不可靠，则均值权重较低
						weight = 0.5 * iCloseNum;
					else
						weight = 2 * iCloseNum; // 权重较高
					meanBB.mPointLT.x = (meanBB.mPointLT.x + pObj->m_TrackResult.mBB.mPointLT.x*weight) / (weight+iCloseNum);
					meanBB.mPointLT.y = (meanBB.mPointLT.y + pObj->m_TrackResult.mBB.mPointLT.y*weight) / (weight+iCloseNum);
					meanBB.mPointRD.x = (meanBB.mPointRD.x + pObj->m_TrackResult.mBB.mPointRD.x*weight) / (weight+iCloseNum);
					meanBB.mPointRD.y = (meanBB.mPointRD.y + pObj->m_TrackResult.mBB.mPointRD.y*weight) / (weight+iCloseNum);
					meanConf          = (meanConf + pObj->m_TrackResult.mConf*weight) / (weight+iCloseNum);

					tFrameResult.mBB   = meanBB;
					tFrameResult.mConf = meanConf;

					if ( 0 == CorrectNum )
					{
						// 如果检测结果没有距离跟踪结果远的聚类，则均值结果可认为可靠
						tFrameResult.mValid = true;
					}
				}
			}
		}
		else if (pObj->m_TrackResult.mValid == false && m_DetectFiltNum != 0)
		{
			AmtBbox meanBB = amtBbox(0, 0, 0, 0);
			float   meanConf = 0;
			int iCloseNum = 0;
			for (int i=0; i<m_DetectFiltNum; i++)
			{
				// get sum of close detections with last track-result
				if (BboxOverlapOne(pObj->m_TrackResult.mBB, m_DetectFilter[i].mBB) > 0.5)
				{
					meanBB.mPointLT.x += m_DetectFilter[i].mBB.mPointLT.x;
					meanBB.mPointLT.y += m_DetectFilter[i].mBB.mPointLT.y;
					meanBB.mPointRD.x += m_DetectFilter[i].mBB.mPointRD.x;
					meanBB.mPointRD.y += m_DetectFilter[i].mBB.mPointRD.y;
					meanConf          += m_DetectFilter[i].mConf2;
					iCloseNum += 1;
				}
			}
			if (iCloseNum > 0)
			{
				// weighted average trackers trajectory with the close detections
				int weight = 0.5*iCloseNum; //检测结果权重较高;
				meanBB.mPointLT.x = (meanBB.mPointLT.x + pObj->m_TrackResult.mBB.mPointLT.x*weight) / (weight+iCloseNum);
				meanBB.mPointLT.y = (meanBB.mPointLT.y + pObj->m_TrackResult.mBB.mPointLT.y*weight) / (weight+iCloseNum);
				meanBB.mPointRD.x = (meanBB.mPointRD.x + pObj->m_TrackResult.mBB.mPointRD.x*weight) / (weight+iCloseNum);
				meanBB.mPointRD.y = (meanBB.mPointRD.y + pObj->m_TrackResult.mBB.mPointRD.y*weight) / (weight+iCloseNum);
				meanConf          = (meanConf + pObj->m_TrackResult.mConf*weight) / (weight+iCloseNum);

				tFrameResult.mBB   = meanBB;
				tFrameResult.mConf = meanConf;
			}
		}
	}
	else // if tracker is not defined
	{
		tFrameResult.mBB    = amtBbox(0,0,0,0);
		tFrameResult.mConf  = 0;
		tFrameResult.mSize  = 0;
		tFrameResult.mValid = false;

		//if detector is defined
		if (matchNum != 0)
		{
			// cluster detections
			BboxClusterConfidence(&pObj->m_DetectCluster, &pObj->m_DetectResult);

			// and if there is just a single cluster, re-initialize the tracker
			if ( 1 == pObj->m_DetectCluster.mMatchNum )
			{
				tFrameResult.mBB    = pObj->m_DetectCluster.mBB[0];
				tFrameResult.mConf  = pObj->m_DetectCluster.mConf[0];
				tFrameResult.mSize  = pObj->m_DetectCluster.mSize[0];
				tFrameResult.mValid = false;
			}
			else
			{
				// maximum confidence
				int maxIdx = 0;
				float maxConf = pObj->m_DetectCluster.mConf[0];
				for (int i=0; i<pObj->m_DetectCluster.mMatchNum; i++)
				{
					if (maxConf < pObj->m_DetectCluster.mConf[i])
					{
						maxConf = pObj->m_DetectCluster.mConf[i];
						maxIdx = i;
					}
				}
				tFrameResult.mBB    = pObj->m_DetectCluster.mBB[maxIdx];
				tFrameResult.mConf  = pObj->m_DetectCluster.mConf[maxIdx];
				tFrameResult.mSize  = 1;
				tFrameResult.mValid = false;
			}
		}
		else
		{
			// ------------------------------------------------
			AmtBbox meanBB = amtBbox(0, 0, 0, 0);
			float   meanConf = 0;
			for (int i=0; i<m_DetectFiltNum; i++)
			{
				meanBB.mPointLT.x += m_DetectFilter[i].mBB.mPointLT.x;
				meanBB.mPointLT.y += m_DetectFilter[i].mBB.mPointLT.y;
				meanBB.mPointRD.x += m_DetectFilter[i].mBB.mPointRD.x;
				meanBB.mPointRD.y += m_DetectFilter[i].mBB.mPointRD.y;
				meanConf          += m_DetectFilter[i].mConf2;
			}
			if (m_DetectFiltNum > 0)
			{
				meanBB.mPointLT.x = (meanBB.mPointLT.x) / (m_DetectFiltNum);
				meanBB.mPointLT.y = (meanBB.mPointLT.y) / (m_DetectFiltNum);
				meanBB.mPointRD.x = (meanBB.mPointRD.x) / (m_DetectFiltNum);
				meanBB.mPointRD.y = (meanBB.mPointRD.y) / (m_DetectFiltNum);
				meanConf          = (meanConf) / (m_DetectFiltNum);

				tFrameResult.mBB    = meanBB;
				tFrameResult.mConf  = meanConf;
				tFrameResult.mSize  = m_DetectFiltNum;
				tFrameResult.mValid = false;
			}
		}
	}

	// ======================== LEARNING ==========================
	if (true == tFrameResult.mValid)
	{
		LearnerExecute(pObj, &tFrameResult);
	}

	// ======================== OUTPUT RESULT =========================
	// fill trajectory
	if ( amtCheckBboxDef(&(tFrameResult.mBB)) )
	{
		tFrameResult.mPos = cvPoint( (int)((tFrameResult.mBB.mPointLT.x + tFrameResult.mBB.mPointRD.x)*0.5),
			                         (int)((tFrameResult.mBB.mPointLT.y + tFrameResult.mBB.mPointRD.y)*0.5) );
		tFrameResult.mTimeStamp = TimeStamp;
		tFrameResult.mMoveTime  = m_iTimeInterval;
		tFrameResult.mPauseTime = 0;
		tFrameResult.mMissTime  = 0;

		// copy result to OutPut
		pObj->mOutPut = tFrameResult.mBB;

		// 如果跟踪结果有效，则保存当前帧新的跟踪结果
		pObj->mTraceResult[pObj->mTraceNum] = tFrameResult;
		pObj->mTraceNum ++;

		// 如果跟踪结果有效，则丢失计数清零
		pObj->mCurrCountMiss = 0;
	}
	else
	{
		// 保存上一帧目标
		pObj->mTraceResult[pObj->mTraceNum].mBB    = pObj->mTraceResult[pObj->mTraceNum-1].mBB;
		pObj->mTraceResult[pObj->mTraceNum].mPos.x = pObj->mTraceResult[pObj->mTraceNum-1].mPos.x;
		pObj->mTraceResult[pObj->mTraceNum].mPos.y = pObj->mTraceResult[pObj->mTraceNum-1].mPos.y;
		pObj->mTraceResult[pObj->mTraceNum].mConf  = 0;
		pObj->mTraceResult[pObj->mTraceNum].mSize  = 0;
		pObj->mTraceResult[pObj->mTraceNum].mValid = false;
		pObj->mTraceResult[pObj->mTraceNum].mTimeStamp = TimeStamp;
		pObj->mTraceResult[pObj->mTraceNum].mMissTime  = pObj->mTraceResult[pObj->mTraceNum-1].mMissTime + m_iTimeInterval;
		pObj->mTraceResult[pObj->mTraceNum].mMoveTime  = 0;
		pObj->mTraceResult[pObj->mTraceNum].mPauseTime = 0;
		pObj->mOutPut = pObj->mTraceResult[pObj->mTraceNum].mBB;

		pObj->mTraceNum ++;

		// 如果跟踪结果无效，则进行持续性计数 - for destroy
		pObj->mCurrCountMiss ++;
	}

	// 如果轨迹点过多，则进行采样保留
	if (RECORD_POINTS_NUM == pObj->mTraceNum)
	{
		pObj->mTraceNum = 0;

		int NumPerRegion = RECORD_POINTS_NUM / SAMPLE_REGION_NUM;
		for (int i=0; i<SAMPLE_REGION_NUM; i++)
		{
			int RBegin = i*NumPerRegion;
			int REnd   = (i+1)*NumPerRegion;
			int SpanN  = SAMPLE_REGION_NUM - i;
			for (int j=RBegin; j<REnd; j++)
			{
				if (j%SpanN == 0)
				{
					pObj->mTraceResult[pObj->mTraceNum] = pObj->mTraceResult[j];
					pObj->mTraceNum ++;
				}
			}
		}
	}

	// ------------------- 不稳定跟踪状态计数处理 --------------------------
	if (false == tFrameResult.mValid && tFrameResult.mConf < 0.5 && 0 == m_DetectFiltNum)
	{
		// 不稳定跟踪(无检测结果，跟踪结果不确定)
		pObj->mUnstableNum ++;
	}
	else
	{
		pObj->mUnstableNum = 0;
	}
	if (pObj->mUnstableNum > 10)  // 10帧 -- 一秒
	{
		pObj->mUnstable = true; // 不稳定状态标识
	}

	// ----------------------- DetectFilter Result -----------------------
	if (true == m_AlgoParam.mDrawResults)
	{
		for (int i=0; i<m_DetectFiltNum; i++)
		{
			cvRectangle(m_DrawImage, cvPointFrom32f(m_DetectFilter[i].mBB.mPointLT), cvPointFrom32f(m_DetectFilter[i].mBB.mPointRD), cvScalar(125,125,125), 1, 8, 0);
		}

		for (int i=0; i<m_TrackerModule.m_FindNum; i++)
		{
			cvCircle(m_DrawImage, cvPointFrom32f(m_TrackerModule.m_FindPoints[i]), 2, cvScalar(255), 1, 8, 0);
		}
	}

	return true;
}

bool EvTrackCore::TrackerExecute(AmtSingleObject* pObj, AmtBbox* BB, IplImage* pImgPrev, IplImage* pImgCurr)
{
	pObj->m_TrackResult.clean();

	// --------------  get track result using FB optical flow   --------------
	if ( !m_TrackerModule.Execute( &pObj->m_TrackResult, pImgPrev, pImgCurr, BB, &(pObj->mAnalyROI.mROI) ) )
	{
		if ( 0 > m_TrackerModule.ExecuteTempleMatch(&pObj->m_TrackResult, BB, &(pObj->mAnalyROI.mROI)) )
			return false;
	}
	// -----------------  estimate confidence and validity  -----------------
	m_ExpertsModule.GetPatchPattern(m_ExpertsModule.m_PatchPattern, m_CurrImage, &pObj->m_TrackResult.mBB);
	float Conf1, Conf2;
	m_ExpertsModule.ObjectEvaluateNN(pObj, pObj->m_Model.mNccThesame, pObj->m_Model.mValid, m_ExpertsModule.m_PatchPattern, Conf1, Conf2);
	pObj->m_TrackResult.mConf = Conf2;

	// ------------------------  Validity  ------------------------
	pObj->m_TrackResult.mValid = pObj->mTraceResult[pObj->mTraceNum-1].mValid;  // from previous frame
	pObj->m_TrackResult.mFlag  = true;
	if (pObj->m_TrackResult.mConf > pObj->m_Model.mThrNNvalid)
	{
		//	if Conf > amt.model.thr_nn_valid, Valid = 1; end % tracker is inside the 'core'
		pObj->m_TrackResult.mValid = true;
	}
	if (pObj->m_TrackResult.mConf < 0.5)
	{
		// tracker is true, but not valid and will not be learning
		pObj->m_TrackResult.mValid = false;
	}

	return true;
}


// scans the image(I) with a sliding window, returns a list of bounding
// boxes and their confidences that match the object description
int EvTrackCore::DetecterExecute(AmtSingleObject* pObj, IplImage* pImg, IplImage* pImgBlur)
{
	pObj->m_DetectResult.mMatchNum = 0;
	pObj->m_DetectFiltNum = 0;
	m_DetectFiltNum = 0;

	// get indexes of bounding boxes that passed through the Ensemble Classifier.
	int pDetectIdx[DETECT_NUM_PER_FRAME];
	m_DetectFiltNum = m_DetecterModule.Execute(pObj, pImg, pImgBlur, pDetectIdx);

	// if nothing detected, return
	if (m_DetectFiltNum == 0)
	{
		return pObj->m_DetectResult.mMatchNum;
	}

	// initialize detection structure
	for (int i=0; i<m_DetectFiltNum; i++)
	{
		int idx = pDetectIdx[i];
		m_DetectFilter[i].mIdx = idx;
		m_DetectFilter[i].mBB = pObj->mGridBboxData[idx].mBbox;

		// save fern pattern
		int iPattDim = m_Model.mTreesNum;
		int* dstData = m_DetectFilter[i].mPattern;
		int* srcData = m_DetecterModule.mGridFernPatt + iPattDim*idx;
		memcpy(dstData, srcData, iPattDim*sizeof(int));

		// measure patch
		float* fPattern = m_DetectFilter[i].mPatch;
		m_ExpertsModule.GetPatchPattern(fPattern, pImg, &(m_DetectFilter[i].mBB));

		// evaluate nearest neighbor classifier
		float &conf1 = m_DetectFilter[i].mConf1;
		float &conf2 = m_DetectFilter[i].mConf2;
		int   *isin  = m_DetectFilter[i].mIsIn;
		m_ExpertsModule.ObjectEvaluateNN(pObj, pObj->m_Model.mNccThesame, pObj->m_Model.mValid, fPattern, conf1, conf2, isin);

		// get all indexes that made it through the nearest neighbor
		if (conf1 > pObj->m_Model.mThrNN)
		{
			pObj->m_DetectResult.mBB[pObj->m_DetectResult.mMatchNum]   = m_DetectFilter[i].mBB;
			pObj->m_DetectResult.mConf[pObj->m_DetectResult.mMatchNum] = conf2;
			pObj->m_DetectResult.mMatchNum ++;
		}
	}
	pObj->m_DetectFiltNum = m_DetectFiltNum;

	return pObj->m_DetectResult.mMatchNum;
}

void EvTrackCore::LearnerExecute(AmtSingleObject* pObj, AmtTraceResult* pFrmTraceResult)
{
	AmtBbox bb = pFrmTraceResult->mBB;

	// ========================= Check Consistency ===========================
	// 评估输出结果
	m_ExpertsModule.GetPatchPattern(m_ExpertsModule.m_PatchPattern, m_CurrImage, &bb);
	float fPatchVar = m_ExpertsModule.PatchVariance(m_ExpertsModule.m_PatchPattern, m_ExpertsModule.m_PatternDim);
	if ( fPatchVar < pObj->m_VarValue )
	{
		//	fprintf(stdout, "Low variance!\n");
		pFrmTraceResult->mValid = false;
		return;
	}

	float fConf1, fConf2;
	int pIsin[3];
	m_ExpertsModule.ObjectEvaluateNN(pObj, pObj->m_Model.mNccThesame, pObj->m_Model.mValid, m_ExpertsModule.m_PatchPattern, fConf1, fConf2, pIsin);
	if (fConf1 < 0.5)
	{
		//	fprintf(stdout, "Fast change!\n");
		pFrmTraceResult->mValid = false;
		return;
	}

	if (pIsin[2] == 1)
	{
		//	fprintf(stdout, "In negative data!\n");
		pFrmTraceResult->mValid = false;
		return;
	}

	// ============================ Update ==============================
	BboxOverlap(bb, pObj->mGridBboxData, pObj->mGridBboxNum, m_DetecterModule.mGridOverlapData, &(pObj->mAnalyROI));

	// -------------------- generate positive data -----------------------
	// 生成overlap最大的目标区域pattern(pEx)，以及较大overlap目标区域集的外围区域fern-pattern(pX)
	// generate positive examples from all bounding boxes that are highly overlapping with current bounding box
	GeneratePositiveData(pObj, m_DetecterModule.mGridOverlapData, m_CurrImage, m_BlurImage, pObj->m_PparUpdata);

	// -------------------- generate negative patch data -------------------------
	m_ExpertsModule.m_PatchTrainN.mNums = 0;
	int stepDT = m_ExpertsModule.m_PattAndNormDim;
	float* dataNX = m_ExpertsModule.m_PatchTrainN.mData;
	// 1. measure overlap of the current bounding box with detections
	// 2. get negative patches that are far from current bounding box
	for (int i=0; i<m_DetectFiltNum; i++)
	{
		float fOverlap = m_DetecterModule.mGridOverlapData[m_DetectFilter[i].mIdx];
		if (fOverlap < pObj->m_Npar.mOverlap)
		{
			memcpy(dataNX, m_DetectFilter[i].mPatch, stepDT*sizeof(float));
			m_ExpertsModule.m_PatchTrainN.mDataIdx[m_ExpertsModule.m_PatchTrainN.mNums++] = dataNX;
			dataNX += stepDT;
		}
	}

	// update nearest neighbor
	m_ExpertsModule.ObjectUpDateNN(pObj, &(m_ExpertsModule.m_PatchTrainP), &(m_ExpertsModule.m_PatchTrainN));

	// -------------------- generate negative Fern data -------------------------
	// get indexes of negative bounding boxes on the grid
	// (bounding boxes on the grid that are far from current bounding box and which confidence was larger than 0)
	m_DetecterModule.m_FernTrainN.mNums = 0;

	int* pSelectIdx = m_DetecterModule.mGridSelectIdx;
	int iSelectNum  =0;
	for (int i=0; i<m_DetectFiltNum; i++)
	{
		int iBboxIdx = m_DetectFilter[i].mIdx;
		float fOverlap  = m_DetecterModule.mGridOverlapData[iBboxIdx];
		float fFernConf = m_DetecterModule.mGridFernConf[iBboxIdx];
		if ( (fOverlap < pObj->m_Npar.mOverlap) && (fFernConf >= m_DetecterModule.m_ThresholdN) )
		{
			pSelectIdx[iSelectNum ++] = iBboxIdx;
		}
	}

	// pX -- M(10)*N(10)*Num(var)  pY -- 1*size(pX)
	int iTrainNumP = m_DetecterModule.m_FernTrainP.mNums;
	int iFeatureN = m_DetecterModule.m_TreesNum; //  == mFERN.nTREES
	if (iTrainNumP+iSelectNum > 0)
	{
		// 准备负样本数据
		int* pGridFernPatt = m_DetecterModule.mGridFernPatt;
		int* pTrainDataN = m_DetecterModule.m_FernTrainN.mData;
		for (int i=0; i<iSelectNum; i++)
		{
			memcpy(pTrainDataN, pGridFernPatt+iFeatureN*pSelectIdx[i], iFeatureN*sizeof(int));
			m_DetecterModule.m_FernTrainN.mDataIdx[m_DetecterModule.m_FernTrainN.mNums++] = pTrainDataN;
			pTrainDataN += iFeatureN;
		}

		// update the Ensemble Classifier (reuses the computation made by detector)
		m_DetecterModule.ObjectFernUpDate(pObj, &(m_DetecterModule.m_FernTrainP), &(m_DetecterModule.m_FernTrainN), 2, false);
	}

	return;
}


//generate positive examples from all bounding boxes that are highly overlapping with current bounding box
AmtBbox EvTrackCore::GeneratePositiveData(AmtSingleObject* pObj, float* pGridOverlap, IplImage* pImg, IplImage* pImgBlur, AmtPpar pPar)
{
	int* pSelectIdx = m_DetecterModule.mGridSelectIdx;
	int iSelectNum = 0;

	// for overlapping bound-boxes
	int iScanBboxN = pObj->mAnalyROI.mScanBboxN;
	int* pScanBboxIdx = pObj->mAnalyROI.mScanBboxIdx;
	for (int i=0; i<iScanBboxN; i++)
	{
		int iBboxIdx = pScanBboxIdx[i];
		if (pGridOverlap[iBboxIdx] > 0.6)
		{
			pSelectIdx[iSelectNum ++] = iBboxIdx;
		}
	}

	// for closest bound-box
	AmtBbox tClosestBbox = amtBbox(0, 0, 0, 0);
	if (iSelectNum > 0)
	{
		int iIdxP = 0;
		int iMaxIdxP = 0;
		float fMaxOverlap = 0;
		for (int i=0; i<iSelectNum; i++)
		{
			iIdxP = pSelectIdx[i];
			if (fMaxOverlap < pGridOverlap[ iIdxP ])
			{
				fMaxOverlap = pGridOverlap[ iIdxP ];
				iMaxIdxP    = iIdxP;
			}
		}
		// Get closest bound-box
		tClosestBbox = pObj->mGridBboxData[iMaxIdxP].mBbox;
	}

	// ========================= Fern Positive =============================
	m_DetecterModule.CalcPositivePattern(pObj, pImg, pImgBlur, pSelectIdx, iSelectNum, pPar);

	// ========================= Patch Positive ============================
	m_ExpertsModule.CalcPositivePattern(pImg, &tClosestBbox);

	return tClosestBbox;
}


void EvTrackCore::GenerateNegativeData(AmtSingleObject* pObj, float* pGridOverlap, IplImage* pImg, IplImage* pImgBlur)
{
	//1. Measure patterns on all bound-boxes that are far from initial bound-box
	int* pSelectIdx = m_DetecterModule.mGridSelectIdx;
	int iSelectNum = 0;
	int iScanBboxN = pObj->mAnalyROI.mScanBboxN;
	int* pScanBboxIdx = pObj->mAnalyROI.mScanBboxIdx;
	for (int i=0; i<iScanBboxN; i++)
	{
		int iBboxIdx = pScanBboxIdx[i];

		if (pGridOverlap[iBboxIdx] < pObj->m_Npar.mOverlap)
		{
			pSelectIdx[iSelectNum] = iBboxIdx;
			iSelectNum ++;
		}
	}

	// ======== Get Negative Fern =========
	// "pSelectIdx" select again by Object's Variance in DetectModule, so iSelectNum maybe change
	m_DetecterModule.CalcNegativePattern(pObj, pImg, pImgBlur, pSelectIdx, iSelectNum, pObj->m_Npar);

	// ======== Get Negative Patch =========
	m_ExpertsModule.CalcNegativePattern(pObj, pImg, pSelectIdx, iSelectNum, pObj->m_Npar);

	return;
}


void EvTrackCore::DrawResult(AmtSingleObject* pObj, IplImage* pFrame)
{
	//1----------------------- box result -----------------------
	// ROI Rect
	cvRectangle(pFrame,
		        cvPoint(pObj->mAnalyROI.mROI.x, pObj->mAnalyROI.mROI.y),
		        cvPoint(pObj->mAnalyROI.mROI.x+pObj->mAnalyROI.mROI.width, pObj->mAnalyROI.mROI.y+pObj->mAnalyROI.mROI.height),
				cvScalar(255), 1, 8, 0);
	// Last Result
	cvRectangle(pFrame,
		        cvPoint(pObj->mAnalyROI.mObjRect.x, pObj->mAnalyROI.mObjRect.y),
		        cvPoint(pObj->mAnalyROI.mObjRect.x+pObj->mAnalyROI.mObjRect.width, pObj->mAnalyROI.mObjRect.y+pObj->mAnalyROI.mObjRect.height),
		        cvScalar(255), 1, CV_AA, 0);
	sprintf(m_Text, "ER[%.2f]X[%.2f]Y[%.2f]", pObj->mAnalyROI.mExpRate, pObj->mAnalyROI.mShiftRX, pObj->mAnalyROI.mShiftRY);
	cvPutText(pFrame, m_Text, cvPoint(pObj->mOutPut.mPointLT.x, pObj->mOutPut.mPointLT.y-10), &m_Font, CV_RGB(255, 255, 0));

	if (true == pObj->mUnstable)
		cvRectangle(pFrame, cvPointFrom32f(pObj->mOutPut.mPointLT), cvPointFrom32f(pObj->mOutPut.mPointRD), cvScalar(125, 0, 125), 2, 8, 0);
	else
		cvRectangle(pFrame, cvPointFrom32f(pObj->mOutPut.mPointLT), cvPointFrom32f(pObj->mOutPut.mPointRD), pObj->m_ColorTag, 2, 8, 0);
	sprintf(m_Text, "ID[%d]_%d*%d", pObj->m_ID, (int)(fabs(pObj->mOutPut.mPointLT.x - pObj->mOutPut.mPointRD.x)), (int)(fabs(pObj->mOutPut.mPointLT.y - pObj->mOutPut.mPointRD.y)));
	cvPutText(pFrame, m_Text, cvPoint(pObj->mOutPut.mPointLT.x, pObj->mOutPut.mPointLT.y), &m_Font, CV_RGB(255, 255, 0));

	// ----------------------- detector's result -----------------------
	for (int i=0; i<pObj->m_DetectResult.mMatchNum; i++)
	{
		cvRectangle(pFrame, cvPointFrom32f(pObj->m_DetectResult.mBB[i].mPointLT), cvPointFrom32f(pObj->m_DetectResult.mBB[i].mPointRD), cvScalar(60, 0, 0), 1, 8, 0);
	}

	// ----------------------- tracker's result -----------------------
	if (true == pObj->m_TrackResult.mFlag)
	{
		cvRectangle(pFrame, cvPointFrom32f(pObj->m_TrackResult.mBB.mPointLT), cvPointFrom32f(pObj->m_TrackResult.mBB.mPointRD), cvScalar(0, 0, 60), 1, 8, 0);
		sprintf(m_Text, "%.3f", pObj->m_TrackResult.mConf);
		cvPutText(pFrame, m_Text, cvPoint((int)(pObj->mOutPut.mPointLT.x), (int)(pObj->mOutPut.mPointRD.y)), &m_Font, CV_RGB(255, 255, 0));
	}

	//3. ----------------------- trajectory result -----------------------
	CvPoint Traject[RECORD_POINTS_NUM], *pTraject = Traject;
	int iTracjNum = pObj->mTraceNum;
	for (int i=0; i<iTracjNum; i++)
	{
		pTraject[i].x = pObj->mTraceResult[i].mPos.x;
		pTraject[i].y = pObj->mTraceResult[i].mPos.y;
	}
	cvPolyLine(pFrame, &pTraject, &iTracjNum, 1, 0, cvScalar(0,255,0), 2, 8, 0);

	//4. ----------------------- number of PEX and NEX -----------------------
	sprintf(m_Text, "P: %d", pObj->mPatchNumP);
	cvPutText(pFrame, m_Text, cvPoint(pObj->mOutPut.mPointRD.x, pObj->mOutPut.mPointLT.y+15), &m_Font, CV_RGB(255, 255, 0));
	sprintf(m_Text, "N: %d", pObj->mPatchNumN);
	cvPutText(pFrame, m_Text, cvPoint(pObj->mOutPut.mPointRD.x, pObj->mOutPut.mPointLT.y+30), &m_Font, CV_RGB(255, 255, 0));

	//5. ----------------------- data -----------------------
	sprintf(m_Text, "T[%d]D[%d]DF[%d]", (int)(pObj->m_TrackResult.mValid), pObj->m_DetectResult.mMatchNum, pObj->m_DetectFiltNum);
	cvPutText(pFrame, m_Text, cvPoint(pObj->mOutPut.mPointLT.x, pObj->mOutPut.mPointRD.y+15), &m_Font, CV_RGB(0, 255, 0));
}


// Clustering of tracker and detector responses
// First cluster returned corresponds to the tracker
void EvTrackCore::BboxClusterConfidence(AmtDetectResult* pDetectCluster, AmtDetectResult* pDetectResult)
{
	float spaceThreshold = 0.5;
	pDetectCluster->mMatchNum  = 0;

	if (0 == pDetectResult->mMatchNum)
		return;

	int pSampleTag[DETECT_NUM_PER_FRAME];
	int iSampleNum;
	switch (pDetectResult->mMatchNum)
	{
	case 0:
		break;
	case 1:
		iSampleNum = 1;
		pSampleTag[0] = 0;
		break;
	case 2:
		iSampleNum = 2;
		pSampleTag[0] = 0;
		pSampleTag[1] = 0;
		if ( (1 - BboxOverlapOne(pDetectResult->mBB[0], pDetectResult->mBB[1])) > spaceThreshold )
		{
			pSampleTag[1] = 1;
		}
		break;
	default:
		iSampleNum = pDetectResult->mMatchNum;
		BboxPartition(pSampleTag, pDetectResult->mBB, pDetectResult->mMatchNum);
		break;
	}

	int pClusterIdx[DETECT_NUM_PER_FRAME];
	int iClusterNum = 0;
	for (int i=1; i<iSampleNum; i++)
	{
		bool flag = true;

		//判断idx_cluster中是否存在与T[i]相同的值
		for (int j=0; j<iClusterNum; j++)
		{
			if (pSampleTag[i] == pClusterIdx[j])
			{
				flag = false;
				break;
			}
		}

		//如果idx_cluster中没有与T[i]相同的值，则保存T[i]
		if (flag == true)
		{
			pClusterIdx[iClusterNum] = pSampleTag[i];  // 记录聚类标签
			iClusterNum ++;
		}
	}

	pDetectCluster->mMatchNum = iClusterNum;

	//对于每一个聚类进行一次中值计算
	for (int i=0; i<iClusterNum; i++)
	{
		//对聚类成员求中值
		pDetectCluster->mBB[i]   = amtBbox(0, 0, 0, 0);
		pDetectCluster->mConf[i] = 0;
		pDetectCluster->mSize[i] = 0;
		for (int k=0; k<iSampleNum; k++)
		{
			if (pClusterIdx[i] == pSampleTag[k])
			{
				pDetectCluster->mBB[i].mPointLT.x += pDetectResult->mBB[k].mPointLT.x;
				pDetectCluster->mBB[i].mPointLT.y += pDetectResult->mBB[k].mPointLT.y;
				pDetectCluster->mBB[i].mPointRD.x += pDetectResult->mBB[k].mPointRD.x;
				pDetectCluster->mBB[i].mPointRD.y += pDetectResult->mBB[k].mPointRD.y;

				pDetectCluster->mConf[i] += pDetectResult->mConf[k];
				pDetectCluster->mSize[i] += 1;
			}
		}

		assert(pDetectCluster->mSize[i] > 0);

		pDetectCluster->mBB[i].mPointLT.x = pDetectCluster->mBB[i].mPointLT.x / pDetectCluster->mSize[i];
		pDetectCluster->mBB[i].mPointLT.y = pDetectCluster->mBB[i].mPointLT.y / pDetectCluster->mSize[i];
		pDetectCluster->mBB[i].mPointRD.x = pDetectCluster->mBB[i].mPointRD.x / pDetectCluster->mSize[i];
		pDetectCluster->mBB[i].mPointRD.y = pDetectCluster->mBB[i].mPointRD.y / pDetectCluster->mSize[i];

		pDetectCluster->mConf[i] = pDetectCluster->mConf[i] / pDetectCluster->mSize[i];
	}
}


// ========================  bb_overlap  ===================================
float EvTrackCore::BboxOverlapOne(const AmtBbox& srcBB, const AmtBbox& baseBB)
{
	float bb1[4] = {srcBB.mPointLT.x,  srcBB.mPointLT.y,  srcBB.mPointRD.x,  srcBB.mPointRD.y};
	float bb2[4] = {baseBB.mPointLT.x, baseBB.mPointLT.y, baseBB.mPointRD.x, baseBB.mPointRD.y};

	if (bb1[0]>bb2[2] || bb1[1]>bb2[3] || bb1[2]<bb2[0] || bb1[3]<bb2[1])
	{
		return 0.0;
	}

	float colInt =  MIN(bb1[2], bb2[2]) - MAX(bb1[0], bb2[0]) + 1;
	float rowInt =  MIN(bb1[3], bb2[3]) - MAX(bb1[1], bb2[1]) + 1;

	float intersection = colInt * rowInt;
	float area1 = (bb1[2]-bb1[0]+1)*(bb1[3]-bb1[1]+1);
	float area2 = (bb2[2]-bb2[0]+1)*(bb2[3]-bb2[1]+1);

	return intersection / (area1 + area2 - intersection);
//	return MIN(intersection/area1, intersection/area2);
}

void EvTrackCore::BboxOverlap(const AmtBbox& srcBB, AmtGridBbox* baseBB, int iBboxNum, float* pGridOverlap, AmtObjectROI* pObjROI)
{
	// srcBB
	float bb1[4] = {srcBB.mPointLT.x,  srcBB.mPointLT.y,  srcBB.mPointRD.x,  srcBB.mPointRD.y};
	float area1  = (bb1[2]-bb1[0]+1)*(bb1[3]-bb1[1]+1);//输入框的面积

	// baseBB
	float bb2[4];
	float area2;

	// intersection
	float colInt, rowInt, intersection;

	if (NULL == pObjROI)
	{
		for (int i=0; i<iBboxNum; i++)
		{
			bb2[0] = baseBB[i].mBbox.mPointLT.x;
			bb2[1] = baseBB[i].mBbox.mPointLT.y;
			bb2[2] = baseBB[i].mBbox.mPointRD.x;
			bb2[3] = baseBB[i].mBbox.mPointRD.y;
			area2  = baseBB[i].mArea;

			if (bb1[0]>bb2[2] || bb1[1]>bb2[3] || bb1[2]<bb2[0] || bb1[3]<bb2[1])//这样判定没有重合， 合适吗? add by Lu Dai
			{
				pGridOverlap[i] = 0;
				continue;
			}

			colInt =  MIN(bb1[2], bb2[2]) - MAX(bb1[0], bb2[0]) + 1;
			rowInt =  MIN(bb1[3], bb2[3]) - MAX(bb1[1], bb2[1]) + 1;
			intersection = colInt * rowInt;

			pGridOverlap[i] = intersection / (area1 + area2 - intersection);
		}
	}
	else
	{
		int  iScanBboxN = pObjROI->mScanBboxN;
		int* pScanBboxIdx = pObjROI->mScanBboxIdx;
		for (int i=0; i<iScanBboxN; i++)
		{
			int iBboxIdx = pScanBboxIdx[i];

			bb2[0] = baseBB[iBboxIdx].mBbox.mPointLT.x;
			bb2[1] = baseBB[iBboxIdx].mBbox.mPointLT.y;
			bb2[2] = baseBB[iBboxIdx].mBbox.mPointRD.x;
			bb2[3] = baseBB[iBboxIdx].mBbox.mPointRD.y;
			area2  = baseBB[iBboxIdx].mArea;

			if (bb1[0]>bb2[2] || bb1[1]>bb2[3] || bb1[2]<bb2[0] || bb1[3]<bb2[1])
			{
				pGridOverlap[iBboxIdx] = 0;
				continue;
			}

			colInt =  MIN(bb1[2], bb2[2]) - MAX(bb1[0], bb2[0]) + 1;
			rowInt =  MIN(bb1[3], bb2[3]) - MAX(bb1[1], bb2[1]) + 1;
			intersection = colInt * rowInt;

			pGridOverlap[iBboxIdx] = intersection / (area1 + area2 - intersection);

			assert(pGridOverlap[iBboxIdx] <= 1 && pGridOverlap[iBboxIdx] >= 0);
		}
	}
}


// ========================================================================
int EvTrackCore::BboxPartition(int* labels, const AmtBbox* vec, int num)
{
	const int N = num;
	const int PARENT = 0;
	const int RANK = 1;

	int _nodes[DETECT_NUM_PER_FRAME*2];
	int (*nodes)[2] = (int(*)[2])_nodes;

	// The first O(N) pass: create N single-vertex trees
	for(int i = 0; i < N; i++)
	{
		nodes[i][PARENT] = -1;
		nodes[i][RANK]   =  0;
	}

	// The main O(N^2) pass: merge connected components
	for(int i = 0; i < N; i++ )
	{
		int root = i;

		// find root
		while( nodes[root][PARENT] >= 0 )
			root = nodes[root][PARENT];

		for(int j = 0; j < N; j++ )
		{
			if( i == j || (BboxOverlapOne(vec[i], vec[j]) < 0.5) )
				continue;
			int root2 = j;

			while( nodes[root2][PARENT] >= 0 )
				root2 = nodes[root2][PARENT];

			if( root2 != root )
			{
				// unite both trees
				int rank = nodes[root][RANK], rank2 = nodes[root2][RANK];
				if( rank > rank2 )
					nodes[root2][PARENT] = root;
				else
				{
					nodes[root][PARENT] = root2;
					nodes[root2][RANK] += rank == rank2;
					root = root2;
				}
				assert( nodes[root][PARENT] < 0 );

				int k = j, parent;

				// compress the path from node2 to root
				while( (parent = nodes[k][PARENT]) >= 0 )
				{
					nodes[k][PARENT] = root;
					k = parent;
				}

				// compress the path from node to root
				k = i;
				while( (parent = nodes[k][PARENT]) >= 0 )
				{
					nodes[k][PARENT] = root;
					k = parent;
				}
			}
		}
	}

	// Final O(N) pass: enumerate classes
	int nclasses = 0;
	for(int i = 0; i < N; i++ )
	{
		int root = i;
		while( nodes[root][PARENT] >= 0 )
			root = nodes[root][PARENT];
		// re-use the rank as the class label
		if( nodes[root][RANK] >= 0 )
			nodes[root][RANK] = ~nclasses++;
		labels[i] = ~nodes[root][RANK];
	}

	return nclasses;
}


bool EvTrackCore::CheckTrackDrift(AmtBbox &TrackResult, CvRect &BoundRect)
{
	bool Re = false;
	if (BoundRect.width > 0 && BoundRect.height > 0)
	{
		if (TrackResult.mPointRD.x > BoundRect.x+BoundRect.width ||
			TrackResult.mPointRD.y > BoundRect.y+BoundRect.height ||
			TrackResult.mPointLT.x < BoundRect.x ||
			TrackResult.mPointLT.y < BoundRect.y)
		{
			Re = true;
		}
	}

	return Re;
}

int EvTrackCore::GetObjDirection(AmtTraceResult* pTraceResult, int iTraceNum)
{
	int iDirect = 100; //方向不明确

	if (iTraceNum >= 2)
	{
		int iObjectH = fabs(pTraceResult[0].mBB.mPointRD.y - pTraceResult[0].mBB.mPointLT.y);
		int iShift_Y = pTraceResult[0].mPos.y - pTraceResult[iTraceNum-1].mPos.y;

		if (iShift_Y > 0 && iShift_Y > iObjectH) // 从近到远
		{
			iDirect = 0; //AMT_NEAR_TO_FAR;
		}
		else if(iShift_Y < 0 && abs(iShift_Y) > iObjectH) // 从远到近
		{
			iDirect = 1; //AMT_FAR_TO_NEAR;
		}
	}

	return iDirect;
}

void EvTrackCore::ConstrainTrackResult(AmtTraceResult* pTrackResult, AmtTraceResult* pLastTrace, AmtTraceResult* pInitResult, int iConstrainType)
{
	if (2 == iConstrainType) // ALL_AROUND
	{
		return;
	}

	bool  bIsConstrain = false;
	float fNewW = fabs(pTrackResult->mBB.mPointLT.x - pTrackResult->mBB.mPointRD.x);
	float fNewH = fabs(pTrackResult->mBB.mPointLT.y - pTrackResult->mBB.mPointRD.y);
	float fOldW = fabs(pLastTrace->mBB.mPointLT.x - pLastTrace->mBB.mPointRD.x);
	float fOldH = fabs(pLastTrace->mBB.mPointLT.y - pLastTrace->mBB.mPointRD.y);
	float fIniW = fabs(pInitResult->mBB.mPointLT.x - pInitResult->mBB.mPointRD.x);
	float fIniH = fabs(pInitResult->mBB.mPointLT.y - pInitResult->mBB.mPointRD.y);
	// EvAMT.h : enum{NEAR_TO_FAR = 0, FAR_TO_NEAR = 1, ALL_AROUND = 2};
	switch (iConstrainType)
	{
	case 0: // NEAR_TO_FAR
		// 由近到远，则目标为缩小趋势
	//	if (fNewW*fNewH > fOldW*fOldH)
		if (fNewW*fNewH > fIniW*fIniH)
		{
			bIsConstrain = true;
		}
		break;
	case 1: // FAR_TO_NEAR
		// 由远到近，则目标为变大趋势
	//	if (fNewW*fNewH < fOldW*fOldH)
		if (fNewW*fNewH < fIniW*fIniH)
		{
			bIsConstrain = true;
		}
		break;
	case 100: //方向不明确
		// 方向不明确，则目标缩放尺度控制在有限范围内
		{
			int iSmlW = fIniW*0.9;
			int iSmlH = fIniH*0.9;
			int iBigW = fIniW*1.1;
			int iBigH = fIniH*1.1;

			if (fNewH > iBigH || fNewW > iBigW)
			{
				pTrackResult->mBB.mPointLT.x = pTrackResult->mPos.x - iBigW/2;
				pTrackResult->mBB.mPointLT.y = pTrackResult->mPos.y - iBigH/2;
				pTrackResult->mBB.mPointRD.x = pTrackResult->mPos.x + iBigW/2;
				pTrackResult->mBB.mPointRD.y = pTrackResult->mPos.y + iBigH/2;
			}
			else if(fNewH < iSmlH || fNewW < iSmlW)
			{
				pTrackResult->mBB.mPointLT.x = pTrackResult->mPos.x - iSmlW/2;
				pTrackResult->mBB.mPointLT.y = pTrackResult->mPos.y - iSmlH/2;
				pTrackResult->mBB.mPointRD.x = pTrackResult->mPos.x + iSmlW/2;
				pTrackResult->mBB.mPointRD.y = pTrackResult->mPos.y + iSmlH/2;
			}
		}
	default:
		break;
	}

	if ( true == bIsConstrain )
	{
		// 以新结果中心点为中点，以初始目标框大小限制
		pTrackResult->mBB.mPointLT.x = pTrackResult->mPos.x - fIniW/2;
		pTrackResult->mBB.mPointLT.y = pTrackResult->mPos.y - fIniH/2;
		pTrackResult->mBB.mPointRD.x = pTrackResult->mPos.x + fIniW/2;
		pTrackResult->mBB.mPointRD.y = pTrackResult->mPos.y + fIniH/2;
	}
}

// 目标纹理类型-0为正常，1为纹理弱，2为灰度低
int EvTrackCore::CheckTrackTextureType(AmtBbox &TrackResult)
{
	// 方差过小
	m_ExpertsModule.GetPatchPattern(m_ExpertsModule.m_PatchPattern, m_CurrImage, &TrackResult);
	float fPatchVar = m_ExpertsModule.PatchVariance(m_ExpertsModule.m_PatchPattern, m_ExpertsModule.m_PatternDim);
	if (fPatchVar < 20)
	{
		return 1;
	}

	// 目标灰度低-全黑
	int iLowCount = 0;
	uchar* srcData = (uchar*)(m_ExpertsModule.m_PatchImage->imageData);
	int iStepY = m_ExpertsModule.m_PatchImage->widthStep;
	int iWidth = m_ExpertsModule.m_PatchImage->width;
	int iHeight = m_ExpertsModule.m_PatchImage->height;
	for (int y=0; y<iHeight; y++, srcData+=iStepY)
	{
		for (int x=0; x<iWidth; x++)
		{
			iLowCount += srcData[x] <= 10 ? 1 : 0;
		}
	}

	if ( (float)iLowCount/(float)m_ExpertsModule.m_PatternDim > 0.90 )
	{
		return 2;
	}

	return 0;
}

void EvTrackCore::CorrectShadowResult(AmtBbox &TrackResult)
{
	// 由目标框底端行开始扫描，寻找阴影区域 For m_CurrImage
	CvRect tRect = cvRect(TrackResult.mPointLT.x,
		                  TrackResult.mPointLT.y,
						  TrackResult.mPointRD.x-TrackResult.mPointLT.x+1,
						  TrackResult.mPointRD.y-TrackResult.mPointLT.y+1);

	if (tRect.width <= 0 || tRect.height <= 0)
	{
		return;
	}

	int iConfirmY = (int)TrackResult.mPointRD.y;
	int iHighlightCount = 0;
	int iHighlightThresh = tRect.width*0.2; // 有效前景像素阈值-目标宽度0.2
	uchar* pData = NULL;
	bool bStop = false;
	for (int i=(int)TrackResult.mPointRD.y; i>=(int)TrackResult.mPointLT.y; i--)
	{
		iConfirmY = i;
		iHighlightCount = 0; // 每行清零
		pData = (uchar*)(m_CurrImage->imageData + m_CurrImage->widthStep*i + (int)TrackResult.mPointRD.x);

		for (int j=(int)TrackResult.mPointRD.x; j>=(int)TrackResult.mPointLT.x; j--)
		{
			if (*pData > 50)
			{
				// 灰度值大-光亮区域
				iHighlightCount ++;
			}
			else
			{
				// 存在边缘梯度
				int iGradientThreshold = 20;
				int iDataU = *(pData - m_CurrImage->widthStep); // 上方像素点
				int iDataB = *(pData + m_CurrImage->widthStep); // 下方像素点
				int iDataL = *(pData - 1); // 左边像素点
				int iDataR = *(pData + 1); // 右边像素点
				if ( abs(iDataU - *pData) > iGradientThreshold ||
					 abs(iDataB - *pData) > iGradientThreshold ||
					 abs(iDataL - *pData) > iGradientThreshold ||
					 abs(iDataR - *pData) > iGradientThreshold)
				{
					iHighlightCount ++;
				}
			}
			pData --; // 数据指针前移

			if ( iHighlightCount > iHighlightThresh )
			{
				bStop = true;
				break;
			}
		}

		if ( true == bStop )
		{
			break;
		}
	}

	if (iConfirmY != (int)TrackResult.mPointRD.y)
	{
		// ---------------------------------- Debug ---------------------------------------
		// 之前
		cvRectangle(m_DrawImage, cvPointFrom32f(TrackResult.mPointLT), cvPointFrom32f(TrackResult.mPointRD), cvScalar(255, 0, 0), 1, 8, 0);
		// ---------------------------------------------------------------------------------

		// 下端存在切割情况，则上端相应上移
		int iOffset = TrackResult.mPointRD.y - iConfirmY;
		TrackResult.mPointLT.y -= iOffset;
		TrackResult.mPointRD.y -= iOffset;

		// ---------------------------------- Debug ---------------------------------------
		// 之后
		cvRectangle(m_DrawImage, cvPointFrom32f(TrackResult.mPointLT), cvPointFrom32f(TrackResult.mPointRD), cvScalar(0, 0, 255), 1, 8, 0);
		// ---------------------------------------------------------------------------------
	}
}


// ---------- 创建日志路径 ----------
int EvTrackCore::CreateLogDir(AmtLogRecord* pLog, const char* pParentDir, const char* pChildDir, const char* pChannelDir)
{
	int Re = 0;
	if(NULL == pParentDir)
	{
		return (Re+1);
	}

#ifdef  WIN32 || WIN64
	TCHAR pTempDir[MAX_PATH];
	memset(pTempDir, 0, MAX_PATH * sizeof(TCHAR));
	GetModuleFileName(NULL, pTempDir, MAX_PATH);
	(_tcsrchr(pTempDir, _T('\\')))[1] = 0;
	WideCharToMultiByte(CP_ACP, 0, pTempDir, -1, pLog->mLogPath, MAX_PATH, NULL, NULL);

	strcat(pLog->mLogPath, pParentDir);
	MultiByteToWideChar(CP_ACP, 0, pLog->mLogPath, -1, pTempDir, MAX_PATH);
	if ( !CreateDirectory(pTempDir, NULL) )
	{
		Re += 2;
	}

	if (pChildDir != NULL)
	{
		strcat(pLog->mLogPath, "\\");
		strcat(pLog->mLogPath, pChildDir);
		MultiByteToWideChar(CP_ACP, 0, pLog->mLogPath, -1, pTempDir, MAX_PATH);
		if ( !CreateDirectory(pTempDir, NULL) )
		{
			Re += 3;
		}
	}

	if (pChannelDir != NULL)
	{
		strcat(pLog->mLogPath, "\\");
		strcat(pLog->mLogPath, pChannelDir);
		MultiByteToWideChar(CP_ACP, 0, pLog->mLogPath, -1, pTempDir, MAX_PATH);
		if ( !CreateDirectory(pTempDir, NULL) )
		{
			Re += 4;
		}
	}
#else  // for Linux
	char link[256];
	sprintf(link, "/proc/%d/exe", getpid());
	readlink(link, pLog->mLogPath, MAX_PATH);
	(strrchr(pLog->mLogPath, '/'))[1] = 0;
	strcat(pLog->mLogPath, pParentDir);
	int iR = mkdir(pLog->mLogPath, S_IRWXU | S_IRWXG | S_IRWXO);

	if (pChildDir != NULL)
	{
		strcat(pLog->mLogPath, "/");
		strcat(pLog->mLogPath, pChildDir);
		iR = mkdir(pLog->mLogPath, S_IRWXU | S_IRWXG | S_IRWXO);
	}
	if (pChannelDir != NULL)
	{
		strcat(pLog->mLogPath, "/");
		strcat(pLog->mLogPath, pChannelDir);
		iR = mkdir(pLog->mLogPath, S_IRWXU | S_IRWXG | S_IRWXO);
	}
#endif
	return Re;
}


// ---------- 打印日志 ----------
int EvTrackCore::PrintfLog(AmtLogRecord* pLog, long long timeStamp, const char* pstrLog)
{
	if (NULL == pstrLog)
	{ return 1; }

	if (strlen(pLog->mLogPath) < 1)
	{ return 2; }

	int iMilliseconds = timeStamp % 1000;
	time_t tAlarmTime = timeStamp / 1000;
	struct tm * pLocalTmTime = localtime(&tAlarmTime);
	int iYear, iMon, iDay, iHour, iMin, iSec;
	iYear = pLocalTmTime->tm_year+1900;
	iMon  = pLocalTmTime->tm_mon+1;
	iDay  = pLocalTmTime->tm_mday;
	iHour = pLocalTmTime->tm_hour;
	iMin  = pLocalTmTime->tm_min;
	iSec  = pLocalTmTime->tm_sec;

	char strTemp[80];

	//----新的1天-----
	if(pLog->mLogFileDay != iDay)
	{
		char strLogPath[MAX_PATH];
		if(NULL != pLog->mLogFile)
		{
			fclose(pLog->mLogFile);
			pLog->mLogFile = NULL;
		}

		strcpy(strLogPath, pLog->mLogPath);
		sprintf(strTemp,"//EvAMT_%s_%04d_%02d_%02d.log", m_szChannelID, iYear, iMon, iDay);
		strcat(strLogPath, strTemp);

		if((pLog->mLogFile = fopen(strLogPath,"at+")) != NULL)
		{
			sprintf(strTemp,"[%04d-%02d-%02d %02d:%02d:%02d %03d] ", iYear, iMon, iDay, iHour, iMin, iSec, iMilliseconds);
			fwrite(strTemp, (int)(strlen(strTemp)), 1, pLog->mLogFile);
			fwrite(pstrLog, (int)(strlen(pstrLog)), 1, pLog->mLogFile);
			fflush(pLog->mLogFile);
		}
		else
		{ return 3; }
		pLog->mLogFileDay = iDay;
	}
	//----当前天------
	else
	{
		if(NULL != pLog->mLogFile)
		{
			sprintf(strTemp,"[%04d-%02d-%02d %02d:%02d:%02d %03d] ", iYear, iMon, iDay, iHour, iMin, iSec, iMilliseconds);
			fwrite(strTemp, (int)(strlen(strTemp)), 1, pLog->mLogFile);
			fwrite(pstrLog, (int)(strlen(pstrLog)), 1, pLog->mLogFile);
			fflush(pLog->mLogFile);
		}
		else
		{ return 4; }
	}

	return 0;
}


// ---------- 打印配置日志 ----------
int EvTrackCore::ConfigLog(AmtLogRecord* pLog, bool bReSet)
{
	char sTemp[256];
	long long lTimeStamp;

	if (false == bReSet)
	{
		sprintf(mLogRecord.mLogInfor, "\r\n========== Algo Create! ===========\r\n");
		lTimeStamp = (long long)time(NULL)*1000;
	}
	else
	{
		if (0 == m_CurrTimeStamp)
		{
			sprintf(mLogRecord.mLogInfor, "\r\n======== Algo ReSet(Init)! ========\r\n");
			lTimeStamp = (long long)time(NULL)*1000;
		}
		else
		{
			sprintf(mLogRecord.mLogInfor, "\r\n======= Algo ReSet(Modify)! =======\r\n");
			lTimeStamp = m_CurrTimeStamp;
		}
	}

	sprintf(sTemp, " AlgoVer:[%s]\r\n ImgSize:[%d*%d]\r\n Param:[%d-%d-%d-%.1f-%d-%d]\r\n",
		             g_EvTrackCoreVer,
		             m_ImageSize.width, m_ImageSize.height,
					 m_AlgoParam.mMinWin,
					 m_AlgoParam.mLowScaleLevel,
					 m_AlgoParam.mHighScaleLevel,
					 m_AlgoParam.mShift,
					 m_AlgoParam.mPatchsize,
					 m_AlgoParam.mDrawResults);
	strcat(mLogRecord.mLogInfor, sTemp);

	sprintf(sTemp, "===================================\r\n\r\n");
	strcat(mLogRecord.mLogInfor, sTemp);

	PrintfLog(&mLogRecord, lTimeStamp, mLogRecord.mLogInfor);

	return 0;
}


int EvTrackCore::GetObjTrajectory(int iObjID, CvPoint** pPoints, int* pNum)
{
	*pPoints = NULL;
	*pNum = 0;

	AmtSingleObject *pObj = m_ObjectSets;
	int iObjIndex = -1;
	for(int i=0; i<m_MaxiObjNum; i++)
	{
		// 目标处于激活状态，且目标ID对应
		if(true == pObj->m_IsActive && iObjID == pObj->m_ID)
		{
			iObjIndex = i;
			break;
		}
		pObj ++;
	}
	if (iObjIndex < 0)
		return 0;

	//找到目标进行填充轨迹点
	int iObjNum = m_ObjectSets[iObjIndex].mTraceNum;
	for (int i=0; i<iObjNum; i++)
	{
		m_ObjTraj[i].x = m_ObjectSets[iObjIndex].mTraceResult[i].mPos.x;
		m_ObjTraj[i].y = m_ObjectSets[iObjIndex].mTraceResult[i].mPos.y;
	}
	*pNum = iObjNum;
	*pPoints = m_ObjTraj;

	return iObjNum;
}
