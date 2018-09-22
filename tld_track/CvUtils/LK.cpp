// ******** LK.cpp ********

#include <cv.h>
#include <float.h>
#include <stdio.h>
#include "LK.h"
#include "CommonDef.h"

static void intersect( CvPoint2D32f pt, CvSize win_size, CvSize img_size, CvPoint* min_pt, CvPoint* max_pt )
{
	CvPoint ipt;

	ipt.x = EvFloor( pt.x );
	ipt.y = EvFloor( pt.y );

	ipt.x -= win_size.width;
	ipt.y -= win_size.height;

	win_size.width = win_size.width * 2 + 1;
	win_size.height = win_size.height * 2 + 1;

	min_pt->x = MAX( 0, -ipt.x );
	min_pt->y = MAX( 0, -ipt.y );
	max_pt->x = MIN( win_size.width, img_size.width - ipt.x );
	max_pt->y = MIN( win_size.height, img_size.height - ipt.y );
}


static EvStatus
ievInitPyramidalAlgorithm( const uchar * imgA, const uchar * imgB,
						  int imgStep, CvSize imgSize,
						  uchar * pyrA, uchar * pyrB,
						  int level,
						  CvTermCriteria * criteria,
						  int max_iters, int flags,
						  uchar *** imgI, uchar *** imgJ,
						  int **step, CvSize** size,
						  double **scale, uchar ** buffer )
{
	int pyrBytes, bufferBytes = 0;
	int level1 = level + 1;

	int i;
	CvSize levelSize;

	*buffer = 0;
	*imgI = *imgJ = 0;
	*step = 0;
	*scale = 0;
	*size = 0;

	/* check input arguments */
	if( !imgA || !imgB )
		return CV_NULLPTR_ERR;

	if( (flags & CV_LKFLOW_PYR_A_READY) != 0 && !pyrA ||
		(flags & CV_LKFLOW_PYR_B_READY) != 0 && !pyrB )
		return CV_BADFLAG_ERR;

	if( level < 0 )
		return CV_BADRANGE_ERR;

	switch (criteria->type)
	{
	case CV_TERMCRIT_ITER:
		criteria->epsilon = 0.f;
		break;
	case CV_TERMCRIT_EPS:
		criteria->max_iter = max_iters;
		break;
	case CV_TERMCRIT_ITER | CV_TERMCRIT_EPS:
		break;
	default:
		assert( 0 );
		return CV_BADFLAG_ERR;
	}

	/* compare squared values */
	criteria->epsilon *= criteria->epsilon;

	/* set pointers and step for every level */
	pyrBytes = 0;

#define ALIGN 8

	levelSize = imgSize;

	for( i = 1; i < level1; i++ )
	{
		levelSize.width = (levelSize.width + 1) >> 1; //
		levelSize.height = (levelSize.height + 1) >> 1; // 尺寸减半

		int tstep = evAlignFill(levelSize.width,ALIGN) * sizeof( imgA[0] );//算出每行长度 
		pyrBytes += tstep * levelSize.height;//将所有子图像的大小相加
	}

	assert( pyrBytes <= imgSize.width * imgSize.height * (int) sizeof( imgA[0] ) * 4 / 3 );

	/* buffer_size = <size for patches> + <size for pyramids> */
	bufferBytes = (int)((level1 >= 0) * ((pyrA == 0) + (pyrB == 0)) * pyrBytes +
		                (sizeof( imgI[0][0] ) * 2 + sizeof( step[0][0] ) + sizeof(size[0][0]) + sizeof( scale[0][0] )) * level1);

	*buffer = (uchar *)cvAlloc( bufferBytes ); //分配内存空间
	if( !buffer[0] )
		return CV_OUTOFMEM_ERR;

	*imgI = (uchar **) buffer[0];//
	*imgJ = *imgI + level1;
	*step = (int *) (*imgJ + level1);
	*scale = (double *) (*step + level1);
	*size = (CvSize *)(*scale + level1);

	imgI[0][0] = (uchar*)imgA;
	imgJ[0][0] = (uchar*)imgB;
	step[0][0] = imgStep;
	scale[0][0] = 1;
	size[0][0] = imgSize;

	if( level > 0 )
	{
		uchar *bufPtr = (uchar *) (*size + level1);
		uchar *ptrA = pyrA;
		uchar *ptrB = pyrB;

		if( !ptrA )
		{
			ptrA = bufPtr;
			bufPtr += pyrBytes;
		}

		if( !ptrB )
			ptrB = bufPtr;

		levelSize = imgSize;

		/* build pyramids for both frames */
		for( i = 1; i <= level; i++ )
		{
			int levelBytes;
			CvMat prev_level, next_level;

			levelSize.width = (levelSize.width + 1) >> 1;
			levelSize.height = (levelSize.height + 1) >> 1;

			size[0][i] = levelSize;
			step[0][i] = evAlignFill( levelSize.width, ALIGN ) * sizeof( imgA[0] );
			scale[0][i] = scale[0][i - 1] * 0.5;

			levelBytes = step[0][i] * levelSize.height;
			imgI[0][i] = (uchar *) ptrA;
			ptrA += levelBytes;

			if( !(flags & CV_LKFLOW_PYR_A_READY) )
			{
				prev_level = cvMat( size[0][i-1].height, size[0][i-1].width, CV_8UC1 );
				next_level = cvMat( size[0][i].height, size[0][i].width, CV_8UC1 );
				cvSetData( &prev_level, imgI[0][i-1], step[0][i-1] );
				cvSetData( &next_level, imgI[0][i], step[0][i] );
				cvPyrDown( &prev_level, &next_level );
			}

			imgJ[0][i] = (uchar *) ptrB;
			ptrB += levelBytes;

			if( !(flags & CV_LKFLOW_PYR_B_READY) )
			{
				prev_level = cvMat( size[0][i-1].height, size[0][i-1].width, CV_8UC1 );
				next_level = cvMat( size[0][i].height, size[0][i].width, CV_8UC1 );
				cvSetData( &prev_level, imgJ[0][i-1], step[0][i-1] );
				cvSetData( &next_level, imgJ[0][i], step[0][i] );
				cvPyrDown( &prev_level, &next_level );
			}
		}
	}

	return CV_OK;
}


/* compute dI/dx and dI/dy */
static void
ievCalcIxIy_32f( const float* src, int src_step, float* dstX, float* dstY, int dst_step,
				 CvSize src_size, const float* smooth_k, float* buffer0 )
{
	int src_width = src_size.width;
	int dst_width = src_size.width-2;
	int x;
	int height = src_size.height - 2;
	float* buffer1 = buffer0 + src_width;

	src_step /= sizeof(src[0]);
	dst_step /= sizeof(dstX[0]);

	for( ; height--; src += src_step, dstX += dst_step, dstY += dst_step )
	{
		const float* src2 = src + src_step;
		const float* src3 = src + src_step*2;

		for( x = 0; x < src_width; x++ )
		{
			float t0 = (src3[x] + src[x])*smooth_k[0] + src2[x]*smooth_k[1];
			float t1 = src3[x] - src[x];
			buffer0[x] = t0; 
			buffer1[x] = t1;
		}

		for( x = 0; x < dst_width; x++ )
		{
			float t0 = buffer0[x+2] - buffer0[x];
			float t1 = (buffer1[x] + buffer1[x+2])*smooth_k[0] + buffer1[x+1]*smooth_k[1];
			dstX[x] = t0; 
			dstY[x] = t1;
		}
	}
}


static EvStatus
ievCalcOpticalFlowPyrLK_8uC1R( const uchar* imgA, const uchar* imgB,
							  int imgStep, CvSize imgSize,
							  uchar* pyrA, uchar* pyrB,
							  const CvPoint2D32f* featuresA,
							  CvPoint2D32f* featuresB,
							  int count, CvSize winSize,
							  int level, char* status,
							  float *error, CvTermCriteria criteria, int flags )
{
#define MAX_SIZE  100
#define MAX_LEVEL 10
#define MAX_ITERS 100

	static const float smoothKernel[] = { 0.09375, 0.3125, 0.09375 };  /* 3/32, 10/32, 3/32 */

	uchar *pyrBuffer = 0;
	uchar *buffer = 0;
	int bufferBytes = 0;
	float* _error = 0;
	char* _status = 0;

	uchar **imgI = 0;
	uchar **imgJ = 0;
	int *step = 0;
	double *scale = 0;
	CvSize* size = 0;

	int threadCount = 1;//cvGetNumThreads();
	float* _patchI[CV_MAX_THREADS];
	float* _patchJ[CV_MAX_THREADS];
	float* _Ix[CV_MAX_THREADS];
	float* _Iy[CV_MAX_THREADS];

	int i, l;

	CvSize patchSize = cvSize( winSize.width * 2 + 1, winSize.height * 2 + 1 );//winSize是搜索窗半径
	int patchLen = patchSize.width * patchSize.height;//图像J搜索区域的点数
	int srcPatchLen = (patchSize.width + 2)*(patchSize.height + 2);//图像I搜索区域的点数

	EvStatus result = CV_OK;

	/* check input arguments */
	if( !featuresA || !featuresB )//输入的特征点
		return CV_NULLPTR_ERR;
	if( winSize.width <= 1 || winSize.height <= 1 )//搜索窗半径太小
		return CV_BADSIZE_ERR;

	if( (flags & ~7) != 0 )
		return CV_BADFLAG_ERR;
	if( count <= 0 )
		return CV_BADRANGE_ERR;

	for( i = 0; i < threadCount; i++ )
		_patchI[i] = _patchJ[i] = _Ix[i] = _Iy[i] = 0;

	result = ievInitPyramidalAlgorithm( imgA, imgB, imgStep, imgSize,
		                                pyrA, pyrB, level, &criteria, MAX_ITERS, flags,
										&imgI, &imgJ, &step, &size, &scale, &pyrBuffer );

	if( result < 0 )
		goto func_exit;

	if( !status )
	{
		_status = (char*)cvAlloc( count*sizeof(_status[0]) );
		if( !_status )
		{
			result = CV_OUTOFMEM_ERR;
			goto func_exit;
		}
		status = _status;
	}


	/* buffer_size = <size for patches> + <size for pyramids> */
	bufferBytes = (srcPatchLen + patchLen * 3) * sizeof( _patchI[0][0] ) * threadCount;//？

	buffer = (uchar*)cvAlloc( bufferBytes );
	if( !buffer )
	{
		result = CV_OUTOFMEM_ERR;
		goto func_exit;
	}

	for( i = 0; i < threadCount; i++ )//多线程的情况下挨着分配空间
	{
		_patchI[i] = i == 0 ? (float*)buffer : _Iy[i-1] + patchLen;
		_patchJ[i] = _patchI[i] + srcPatchLen;
		_Ix[i] = _patchJ[i] + patchLen;
		_Iy[i] = _Ix[i] + patchLen;
	}

	memset( status, 1, count );
	if( error )
		memset( error, 0, count*sizeof(error[0]) );

	if( !(flags & CV_LKFLOW_INITIAL_GUESSES) )
		memcpy( featuresB, featuresA, count*sizeof(featuresA[0]));

	/* do processing from top pyramid level (smallest image)
	to the bottom (original image) */
	for( l = level; l >= 0; l-- )//金字塔迭代
	{
		CvSize levelSize = size[l];//本层的图像 宽 高
		int levelStep = step[l];//

		{
			/* find flow for each given point */
			for( i = 0; i < count; i++ ) //遍历每个特征点
			{
				CvPoint2D32f v;
				CvPoint minI, maxI, minJ, maxJ;
				CvSize isz, jsz;
				int pt_status;
				CvPoint2D32f u;
				CvPoint prev_minJ = { -1, -1 }, prev_maxJ = { -1, -1 };
				double Gxx = 0, Gxy = 0, Gyy = 0, D = 0;
				float prev_mx = 0, prev_my = 0;
				int j, x, y;
				int threadIdx = 0;//cvGetThreadNum();
				float* patchI = _patchI[threadIdx];// I 搜索块地址
				float* patchJ = _patchJ[threadIdx];// J 搜索块地址
				float* Ix = _Ix[threadIdx];
				float* Iy = _Iy[threadIdx];

				v.x = featuresB[i].x;//当前特征点
				v.y = featuresB[i].y;
				if( l < level )//如果不是最高层
				{
					v.x += v.x;//特征点
					v.y += v.y;
				}
				else//金字塔最高层
				{
					v.x = (float)(v.x * scale[l]);
					v.y = (float)(v.y * scale[l]);
				}

				pt_status = status[i];
				if( !pt_status )
					continue;

				minI = maxI = minJ = maxJ = cvPoint( 0, 0 );

				u.x = (float) (featuresA[i].x * scale[l]);
				u.y = (float) (featuresA[i].y * scale[l]);

				intersect( u, winSize, levelSize, &minI, &maxI );//确定了minI 和 maxI的值
				isz = jsz = cvSize(maxI.x - minI.x + 2, maxI.y - minI.y + 2);
				u.x += (minI.x - (patchSize.width - maxI.x + 1))*0.5f;
				u.y += (minI.y - (patchSize.height - maxI.y + 1))*0.5f;

				if( isz.width < 3 || isz.height < 3 ||
					ievGetRectSubPix_8u32f_C1R( imgI[l], levelStep, levelSize, patchI, isz.width*sizeof(patchI[0]), isz, u ) < 0 )
				{
					/* point is outside the image. take the next */
					status[i] = 0;
					continue;
				}

				ievCalcIxIy_32f( patchI, isz.width*sizeof(patchI[0]), Ix, Iy, (isz.width-2)*sizeof(patchI[0]), isz, smoothKernel, patchJ );

				for( j = 0; j < criteria.max_iter; j++ )
				{
					double bx = 0, by = 0;
					float mx, my;
					CvPoint2D32f _v;

					intersect( v, winSize, levelSize, &minJ, &maxJ );

					minJ.x = MAX( minJ.x, minI.x );
					minJ.y = MAX( minJ.y, minI.y );

					maxJ.x = MIN( maxJ.x, maxI.x );
					maxJ.y = MIN( maxJ.y, maxI.y );

					jsz = cvSize(maxJ.x - minJ.x, maxJ.y - minJ.y);

					_v.x = v.x + (minJ.x - (patchSize.width - maxJ.x + 1))*0.5f;
					_v.y = v.y + (minJ.y - (patchSize.height - maxJ.y + 1))*0.5f;

					if( jsz.width < 1 || jsz.height < 1 ||
						ievGetRectSubPix_8u32f_C1R( imgJ[l], levelStep, levelSize, patchJ, jsz.width*sizeof(patchJ[0]), jsz, _v ) < 0 )
					{
						/* point is outside image. take the next */
						pt_status = 0;
						break;
					}

					if( maxJ.x == prev_maxJ.x && maxJ.y == prev_maxJ.y &&
						minJ.x == prev_minJ.x && minJ.y == prev_minJ.y )
					{
						for( y = 0; y < jsz.height; y++ )
						{
							const float* pi = patchI + (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
							const float* pj = patchJ + y*jsz.width;
							const float* ix = Ix + (y + minJ.y - minI.y)*(isz.width-2) + minJ.x - minI.x;
							const float* iy = Iy + (ix - Ix);

							for( x = 0; x < jsz.width; x++ )
							{
								double t0 = pi[x] - pj[x];
								bx += t0 * ix[x];
								by += t0 * iy[x];
							}
						}
					}
					else
					{
						Gxx = Gyy = Gxy = 0;
						for( y = 0; y < jsz.height; y++ ) //对积分窗口遍历  此窗口宽jsz.width, 高jsz.height
						{
							const float* pi = patchI + (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1; //pi是I图像中对应点
							const float* pj = patchJ + y*jsz.width;	//pj是J图像中对应点
							const float* ix = Ix + (y + minJ.y - minI.y)*(isz.width-2) + minJ.x - minI.x;
							const float* iy = Iy + (ix - Ix);

							for( x = 0; x < jsz.width; x++ )
							{
								double t = pi[x] - pj[x];  // I图像和J图像中差
								bx += (double) (t * ix[x]);
								by += (double) (t * iy[x]);
								Gxx += ix[x] * ix[x]; // Ix
								Gxy += ix[x] * iy[x];
								Gyy += iy[x] * iy[x];
							}
						}

						D = Gxx * Gyy - Gxy * Gxy; //G的行列式值
						if( D < DBL_EPSILON ) //如果G的行列式值为0
						{
							pt_status = 0;
							break;
						}
						D = 1. / D;

						prev_minJ = minJ;
						prev_maxJ = maxJ;
					}

					mx = (float) ((Gyy * bx - Gxy * by) * D);//本次迭代求得的运动向量
					my = (float) ((Gxx * by - Gxy * bx) * D);

					v.x += mx;
					v.y += my;

					if( mx * mx + my * my < criteria.epsilon ) //如果运动向量已经很小
						break;

					if( j > 0 && fabs(mx + prev_mx) < 0.01 && fabs(my + prev_my) < 0.01 )
					{
						v.x -= mx*0.5f;
						v.y -= my*0.5f;
						break;
					}
					prev_mx = mx;
					prev_my = my;
				}

				featuresB[i] = v;//该点的运动向量
				status[i] = (char)pt_status;
				if( l == 0 && error && pt_status )
				{
					/* calc error */
					double err = 0;

					for( y = 0; y < jsz.height; y++ )
					{
						const float* pi = patchI + (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
						const float* pj = patchJ + y*jsz.width;

						for( x = 0; x < jsz.width; x++ )
						{
							double t = pi[x] - pj[x];
							err += t * t;
						}
					}
					error[i] = (float)sqrt(err);
				}
			} // end of point processing loop (i)
		}
	} // end of pyramid levels loop (l)

func_exit:

	cvFree( &pyrBuffer );
	cvFree( &buffer );
	cvFree( &_error );
	cvFree( &_status );

	return result;
#undef MAX_LEVEL
}


static int ievMinimalPyramidSize( CvSize img_size )
{
	return evAlignFill(img_size.width,8) * img_size.height / 3;
}

int evCalcOpticalFlowPyrLK( const void* arrA, const void* arrB,
						   void* pyrarrA, void* pyrarrB,
						   const CvPoint2D32f * featuresA,
						   CvPoint2D32f * featuresB,
						   int count, CvSize winSize, int level,
						   char *status, float *error,
						   CvTermCriteria criteria, int flags )
{
	CvMat stubA, *imgA = (CvMat*)arrA;
	CvMat stubB, *imgB = (CvMat*)arrB;
	CvMat pstubA, *pyrA = (CvMat*)pyrarrA;
	CvMat pstubB, *pyrB = (CvMat*)pyrarrB;

	imgA = cvGetMat( imgA, &stubA );
	imgB = cvGetMat( imgB, &stubB );

	if( CV_MAT_TYPE( imgA->type ) != CV_8UC1 )
		return (CV_StsUnsupportedFormat);

	if( !CV_ARE_TYPES_EQ( imgA, imgB ))
		return ( CV_StsUnmatchedFormats );

	if( !CV_ARE_SIZES_EQ( imgA, imgB ))
		return ( CV_StsUnmatchedSizes );

	if( imgA->step != imgB->step )
		return ( CV_StsUnmatchedSizes );

	CvSize img_size = {imgA->width, imgA->height}; // cvGetMatSize( imgA );

	if( pyrA )
	{
		pyrA = cvGetMat( pyrA, &pstubA );

		if( pyrA->step*pyrA->height < ievMinimalPyramidSize( img_size ) )
			return ( CV_StsBadArg );
	}
	else
	{
		pyrA = &pstubA;
		pyrA->data.ptr = 0;
	}


	if( pyrB )
	{
		pyrB = cvGetMat( pyrB, &pstubB );

		if( pyrB->step*pyrB->height < ievMinimalPyramidSize( img_size ) )
			return ( CV_StsBadArg );
	}
	else
	{
		pyrB = &pstubB;
		pyrB->data.ptr = 0;
	}

	ievCalcOpticalFlowPyrLK_8uC1R( imgA->data.ptr, imgB->data.ptr, imgA->step,
		                           img_size, pyrA->data.ptr, pyrB->data.ptr,
								   featuresA, featuresB,
								   count, winSize, level, status,
								   error, criteria, flags );
	return CV_OK;
}

static const void*
ievAdjustRect( const void* srcptr, int src_step, int pix_size,
			   CvSize src_size, CvSize win_size,
			   CvPoint ip, CvRect* pRect )
{
	CvRect rect;
	const char* src = (const char*)srcptr;

	if( ip.x >= 0 )
	{
		src += ip.x*pix_size;
		rect.x = 0;
	}
	else
	{
		rect.x = -ip.x;
		if( rect.x > win_size.width )
			rect.x = win_size.width;
	}

	if( ip.x + win_size.width < src_size.width )
		rect.width = win_size.width;
	else
	{
		rect.width = src_size.width - ip.x - 1;
		if( rect.width < 0 )
		{
			src += rect.width*pix_size;
			rect.width = 0;
		}
		assert( rect.width <= win_size.width );
	}

	if( ip.y >= 0 )
	{
		src += ip.y * src_step;
		rect.y = 0;
	}
	else
		rect.y = -ip.y;

	if( ip.y + win_size.height < src_size.height )
		rect.height = win_size.height;
	else
	{
		rect.height = src_size.height - ip.y - 1;
		if( rect.height < 0 )
		{
			src += rect.height*src_step;
			rect.height = 0;
		}
	}

	*pRect = rect;
	return src - rect.x*pix_size;
}


EvStatus  ievGetRectSubPix_8u32f_C1R( const uchar* src, int src_step, CvSize src_size,
                                      float* dst, int dst_step, CvSize win_size, CvPoint2D32f center )
{
	CvPoint ip;
	float  a12, a22, b1, b2;
	float a, b;
	double s = 0;
	int i, j;

	center.x -= (win_size.width-1)*0.5f;
	center.y -= (win_size.height-1)*0.5f;

	ip.x = EvFloor( center.x );
	ip.y = EvFloor( center.y );

	if( win_size.width <= 0 || win_size.height <= 0 )
		return CV_BADRANGE_ERR;

	a = center.x - ip.x;
	b = center.y - ip.y;
	a = MAX(a,0.0001f);
	a12 = a*(1.f-b);
	a22 = a*b;
	b1 = 1.f - b;
	b2 = b;
	s = (1. - a)/a;

	src_step /= sizeof(src[0]);
	dst_step /= sizeof(dst[0]);

	if( 0 <= ip.x && ip.x + win_size.width < src_size.width &&
		0 <= ip.y && ip.y + win_size.height < src_size.height )
	{
		// extracted rectangle is totally inside the image
		src += ip.y * src_step + ip.x;

		for( ; win_size.height--; src += src_step, dst += dst_step )
		{
			float prev = (1 - a)*(b1*EV_8TO32F(src[0]) + b2*EV_8TO32F(src[src_step]));
			for( j = 0; j < win_size.width; j++ )
			{
				float t = a12*EV_8TO32F(src[j+1]) + a22*EV_8TO32F(src[j+1+src_step]);
				dst[j] = prev + t;
				prev = (float)(t*s);
			}
		}
	}
	else
	{
		CvRect r;

		src = (const uchar*)ievAdjustRect( src, src_step*sizeof(*src), sizeof(*src), src_size, win_size,ip, &r);

		for( i = 0; i < win_size.height; i++, dst += dst_step )
		{
			const uchar *src2 = src + src_step;

			if( i < r.y || i >= r.height )
				src2 -= src_step;

			float fSideLeft = EV_8TO32F(src[r.x])*b1 + EV_8TO32F(src2[r.x])*b2;
			for( j = 0; j < r.x; j++ )
			{
		//		float s0 = EV_8TO32F(src[r.x])*b1 + EV_8TO32F(src2[r.x])*b2;
				dst[j] = fSideLeft;
			}

			if( j < r.width )
			{
				float prev = (1 - a)*(b1*EV_8TO32F(src[j]) + b2*EV_8TO32F(src2[j]));
				for( ; j < r.width; j++ )
				{
					float t = a12*EV_8TO32F(src[j+1]) + a22*EV_8TO32F(src2[j+1]);
					dst[j] = prev + t;
					prev = (float)(t*s);
				}
			}

			float fSideRight = EV_8TO32F(src[r.width])*b1 + EV_8TO32F(src2[r.width])*b2;
			for( ; j < win_size.width; j++ )
			{
			//	float s0 = EV_8TO32F(src[r.width])*b1 + EV_8TO32F(src2[r.width])*b2;
				dst[j] = fSideRight;
			}

			if( i < r.height )
				src = src2;
		}
	}

	return CV_OK;
}

EvStatus  ievGetRectPix_8u_C1R( const uchar* src, int src_step, CvSize src_size,
							    uchar* dst, int dst_step, CvSize win_size, CvPoint2D32f center )
{
	CvPoint ip;
	int i, j;

	center.x -= (win_size.width-1)*0.5f;
	center.y -= (win_size.height-1)*0.5f;

	ip.x = EvFloor( center.x );
	ip.y = EvFloor( center.y );

	if( win_size.width <= 0 || win_size.height <= 0 )
		return CV_BADRANGE_ERR;

	src_step /= sizeof(src[0]);
	dst_step /= sizeof(dst[0]);

	if( 0 <= ip.x && ip.x + win_size.width < src_size.width &&
		0 <= ip.y && ip.y + win_size.height < src_size.height )
	{
		// extracted rectangle is totally inside the image
		src += ip.y * src_step + ip.x;

		for( ; win_size.height--; src += src_step, dst += dst_step )
		{
			memcpy(dst, src, win_size.width*sizeof(uchar));
		}
	}
	else
	{
		CvRect r;

		src = (const uchar*)ievAdjustRect( src, src_step*sizeof(*src), sizeof(*src), src_size, win_size,ip, &r);

		for( i = 0; i < win_size.height; i++, dst += dst_step )
		{
			const uchar *src2 = src + src_step;

			if( i < r.y || i >= r.height )
				src2 -= src_step;

			for( j = 0; j < r.x; j++ )
			{
				dst[j] = src[r.x];
			}

			if( j < r.width )
			{
				for( ; j < r.width; j++ )
				{
					dst[j] = src[j+1];
				}
			}

			for( ; j < win_size.width; j++ )
			{
				dst[j] = src[r.width];
			}

			if( i < r.height )
				src = src2;
		}
	}

	return CV_OK;
}

int evGetRectPix(const IplImage* pSrc, IplImage* pDst, CvPoint2D32f center)
{
	assert(IPL_DEPTH_8U == pSrc->depth && IPL_DEPTH_8U == pDst->depth);

	int iRectW   = pDst->width;
	int iRectH   = pDst->height;
	assert(iRectW > 0 && iRectH > 0);

	int iRe = 0;

	uchar* pSrcData = (uchar*)(pSrc->imageData);
	uchar* pDstData = (uchar*)(pDst->imageData);
	int iSrcWStep = pSrc->widthStep;
	int iDstWStep = pDst->widthStep;
	CvSize sSrcSize = cvGetSize(pSrc);
	CvSize sDstSize = cvGetSize(pDst);

	if( !ievGetRectPix_8u_C1R(pSrcData, iSrcWStep, sSrcSize, pDstData, iDstWStep, sDstSize, center) )
		iRe = 1;

	return iRe;
}


/* End of file. */