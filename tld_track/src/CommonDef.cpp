// ================================
// Common Define
// ================================

#include "CommonDef.h"


void amtRandPermutation(int n, int* perm, bool IsEmpty)
{
	int i, j, t;

	if (true == IsEmpty)
	{
		for(i=0; i<n; i++)
			perm[i] = i;
	}

	for(i=0; i<n; i++)
	{
		j = rand()%(n-i)+i;
		t = perm[j];
		perm[j] = perm[i];
		perm[i] = t;
	}
}

// % Computes all possible ntupples.
CvMat* amtNtuples(CvMat* col, CvMat* row, bool colUp)
{
	assert(col->rows == 1 && row->rows == 1);

	int numCol   = col->cols;
	int numItem  = row->cols;

	/*	assert(col->type == CV_32FC1 && row->type == CV_32FC1);*/
	CvMat* out = cvCreateMat(2, numCol*numItem, CV_32FC1);
	float* row_1 = (float*)(out->data.ptr+out->step*0); // for repmat Cvmat* col
	float* row_2 = (float*)(out->data.ptr+out->step*1); // for repmat Cvmat* row
	for (int i=0; i<numCol; i++)
	{
		if (colUp == false)
		{
			// for a=[1 2 3] b=[5 6 7 8] --> c = ntuples(a,b, false)
			// c=[5 6 7 8 5 6 7 8 5 6 7 8
			//    1 1 1 1 2 2 2 2 3 3 3 3]
			memcpy(row_1, row->data.fl, numItem*sizeof(float));
			for (int j=0; j<numItem; j++)
			{
				row_2[j] = (col->data.fl)[i];
			}
		}
		else
		{
			// for a=[1 2 3] b=[5 6 7 8] --> c = ntuples(a,b, true)
			// c=[1 1 1 1 2 2 2 2 3 3 3 3  
			//    5 6 7 8 5 6 7 8 5 6 7 8]
			for (int j=0; j<numItem; j++)
			{
				row_1[j] = (col->data.fl)[i];
			}
			memcpy(row_2, row->data.fl, numItem*sizeof(float));
		}
		row_1 += numItem;
		row_2 += numItem;
	}

	return out;
}


void evCopy(IplImage* pSrc, IplImage* pDst, CvRect* pROI, int algoType)
{
	assert(pSrc->nChannels == pDst->nChannels);
	assert(pSrc->depth == pDst->depth);

	if (!pROI)
	{
		// 如果没有感兴趣区域，则直接进行全部内存拷贝
		memcpy(pDst->imageData, pSrc->imageData, pSrc->widthStep*pSrc->height);
	}
	else
	{
		char *_tempSrc, *_tempDst;
		int beginY, endinY;
		int srcStep, dstStep, copyStep;

		beginY = pROI->y;
		endinY = pROI->y+pROI->height;
		srcStep = pSrc->widthStep;                       // 数据源偏移步长
		dstStep = pDst->widthStep;                       // 结果源偏移步长
		copyStep= pROI->width;                           // 数据复制步长

		switch (algoType)
		{
		case EV_COPY_ROI_TO_ALL:
			_tempSrc = pSrc->imageData+beginY*pSrc->widthStep + pROI->x;  // 数据源偏移起点
			_tempDst = pDst->imageData;                                   // 结果源起点
			break;
		case EV_COPY_ROI_TO_ROI:
			_tempSrc = pSrc->imageData+beginY*pSrc->widthStep + pROI->x;  // 数据源偏移起点
			_tempDst = pDst->imageData+beginY*pDst->widthStep + pROI->x;  // 结果源偏移起点
			break;
		case EV_COPY_ALL_TO_ROI:
			_tempSrc = pSrc->imageData;                                   // 数据源起点
			_tempDst = pDst->imageData+beginY*pDst->widthStep + pROI->x;  // 结果源偏移起点
			break;
		default:
			break;
		}

		for (int y=beginY; y<endinY; y++)
		{
			memcpy(_tempDst, _tempSrc, copyStep);
			_tempSrc += srcStep;
			_tempDst += dstStep;
		}
	}
}

// ----------------- AmtMutex -------------------------
#ifdef WIN32 || WIN64
AmtMutex::AmtMutex()
{
}

AmtMutex::~AmtMutex()
{
}

int AmtMutex::Lock()
{
	return m_Mutex.Lock();
}

int AmtMutex::Unlock()
{
	return m_Mutex.Unlock();
}

#else

AmtMutex::AmtMutex()
{
	pthread_mutex_init(&m_Mutex, NULL);
}
AmtMutex::~AmtMutex()
{
	pthread_mutex_destroy(&m_Mutex);
}

int AmtMutex::Lock()
{
	return pthread_mutex_lock(&m_Mutex);
}

int AmtMutex::Unlock()
{
	return pthread_mutex_unlock(&m_Mutex);
}

#endif

