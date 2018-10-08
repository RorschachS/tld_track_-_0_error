// Test.cpp : Defines the entry point for the console application.
//

// --- memory check ---
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
inline void EnableMemLeakCheck()
{
	_CrtSetDbgFlag(_CrtSetDbgFlag(_CRTDBG_REPORT_FLAG) | _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
}

#include "AMT.h"
#include <cv.h>
#include <cxcore.h>
//#include <highgui.h>
#include <stdio.h>
#include <time.h>
#include<Windows.h>

#include <iostream>
#include <io.h>
#include <fstream>
#include <string>
#include <direct.h>
#include <conio.h>
#include <Windows.h>
#include<vector>
#include "tinyxml2.h"
#include<opencv2\core\mat.hpp>


//#include<opencv.hpp>
//#include<opencv\cxcore.h>
//#include<opencv\cv.h>
#include<opencv2\highgui.hpp>
#include<opencv\cxcore.h>
#include<cvaux.h>
#include<cv.hpp>





using namespace std;
using namespace tinyxml2;
using namespace cv;


//#pragma comment(lib, "cv.lib")
//#pragma comment(lib, "cxcore.lib")
//#pragma comment(lib, "highgui.lib")

// -------------------

#define window_name "SrcImg"
#define MAX_OBJ_TRACKING 20
#define MIN_RECT_WIDTH 10
#define MIN_RECT_HEIGHT 10

void On_Mouse( int event, int x, int y, int flags, void* param);
CvPoint pre_pt, end_pt;
CvRect SelectRect;


// add by Lu Dai
typedef struct RectArray
{
	CvRect RectElement;
	int iIsActive;
	int iIsNew;
	int iObjID;
	


	int pos[4] = { };
	//Mat img;
	string label="";
	bool isAnnotated;
	string path="";
	string name="";
	RectArray()   //���캯��
	{
		isAnnotated = false;
	}

}
RectArray;

typedef struct RectArray_List
{
	RectArray mRectArrayList[MAX_OBJ_TRACKING];
	int iTotal;
	int iNewRectGenerated;
}RectArray_List;
RectArray_List* pRectList = new RectArray_List;

typedef struct Line_type
{
	CvScalar LineColor[3];
	int iThickness[3];
}Line_type;  



//struct metaData
//{
//	vector<int*> pos;
//	Mat img;
//	vector<string> labels;
//	bool isAnnotated;
//	string path;
//	string name;
//	metaData()
//	{
//		isAnnotated = false;
//	}
//};
//
//vector<metaData> mydata;



Line_type LineTypeList;

IplImage* CapImg = NULL;
IplImage* FrameImg  = NULL;
IplImage* FrameGray = NULL;
IplImage* TempImg = NULL;
clock_t TimeStamp;

int track_object = 0;
int object_num = 0;
CvRect track_window;
CvRect sROI;
CvBox2D track_box;
CvConnectedComp track_comp;
int backproject_mode = 0;

void run_tld(char* argc)
{
	//cvNamedWindow("avi");
	char* filename = "D:\\yolo\\videos\\test_video1.avi";
	CvCapture* pCapture = cvCreateFileCapture(filename);
	//CvCapture* pCapture = cvCaptureFromCAM(0);

	EvAMT Amt;
	int FrameCount;
	CvRect initBox;

	if (!pCapture)
	{
		fprintf(stderr, "pCapture is Null!\n");
		return;
	}
	int iScale = 1.0;
	double BeginFrameIdx = 0;
	int iID = 0;

	FrameCount = (int)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_COUNT);// ��Ƶ��֡��
	cvSetCaptureProperty(pCapture, CV_CAP_PROP_POS_FRAMES, BeginFrameIdx);//�ӵ���֡��ʼ

	AmtStructInit(&Amt);
	AmtAlgoParam Param = AmtParamInit(24, 5, 0, 0.1, 15, true);

	char* szChannleID = "AMT-1";
	CvPoint* pPoints;
	int iPointNum;

	if (!(CapImg = cvQueryFrame(pCapture)))
	{
		fprintf(stderr, "pFrame is Null!\n");
		return;
	}

	if (FrameImg == NULL)
	{
		FrameImg = cvCreateImage(cvSize(CapImg->width / iScale, CapImg->height / iScale), CapImg->depth, CapImg->nChannels);
		FrameGray = cvCreateImage(cvGetSize(FrameImg), FrameImg->depth, 1);
		TempImg = cvCreateImage(cvGetSize(FrameImg), FrameImg->depth, FrameImg->nChannels);
		AmtSetConfig(&Amt, szChannleID, FrameImg->width, FrameImg->height, Param, NULL);
	}

	int iKey = 0, frameNum = 0;
	CvFont font = cvFont(1, 1);   // init font
	//char text[256];
	cvNamedWindow(window_name, 1);
	cvSetMouseCallback(window_name, On_Mouse);
	bool bPause = false; // ��ͣ��ʶ
	int iPauseKey;
	double dProcTime = 0, dTime;
	int iProcCount = 0;
	for (;;)  //����ѭ��ֱ�����¡�ESC���˳�
	{
#if 0
		cvWaitKey(100); //����Ƶ����
#endif
		if (frameNum == 1)
		{
			cvWaitKey(0);
		}
		if (!(CapImg = cvQueryFrame(pCapture)))
		{
			fprintf(stderr, "pFrame is Null!\n");
			break;
		}
		cvResize(CapImg, FrameImg);

		//cvFlip(FrameImg);

		cvCvtColor(FrameImg, FrameGray, CV_RGB2GRAY);

		iKey = cvWaitKey(10);  // "ESC"
		if (iKey == 27)
		{
			break;
		}

		frameNum++;

		if (pRectList->iNewRectGenerated == 1) // if there is new rectangle generated 
		{
			for (int i = 0; i < MAX_OBJ_TRACKING; i++)
			{
				if ((pRectList->mRectArrayList[i].iIsNew == 1) && (pRectList->mRectArrayList[i].iIsActive == 1))
				{
					TimeStamp = clock();
					pRectList->mRectArrayList[i].iObjID = iID;  //��ID ���������   ID�����Ӹ���Ŀ��Ϳ��Ψһ��ʶ

					AmtCreateObject(&Amt, pRectList->mRectArrayList[i].RectElement, iID++, (long long)TimeStamp);
					pRectList->mRectArrayList[i].iIsNew = -1;
				}
			}
			object_num++;
			pRectList->iNewRectGenerated = -1;
		}

		dTime = GetTickCount();
		// ======================  TLD����ÿһ֡  ================================
		AmtExecute(&Amt, FrameGray, (long long)time(NULL) * 1000, AMT_ALL_AROUND);//�˴���θ�����ʼλ��
		dTime = GetTickCount() - dTime;
		dProcTime += dTime;
		iProcCount++;

		fprintf(stdout, "֡��: %d, time = %g ms\r", frameNum, dTime);
		
		int boundLineY = 2;
		int boundLineX = 2;
		// ���ٹ���Ŀ��
		for (int i = 0; i < MAX_OBJECT_NUM; i++)
		{
			if (Amt.m_ObjStatus[i].mStatus) //���Ŀ�괦�ڼ���״̬
			{
				int iObjID = Amt.m_ObjStatus[i].mID;
				int iCountMiss = Amt.m_ObjStatus[i].mCountMiss;
				int iUnStableN = Amt.m_ObjStatus[i].mUnstableNum;
				CvPoint &mPos = Amt.m_ObjStatus[i].mPos;
				if (iCountMiss > 2 || iUnStableN > 4 || mPos.y < boundLineY || mPos.x < boundLineX || mPos.x > FrameImg->width - boundLineX)
				{
					int iRectIndex = -1;
					for (int j = 0; j < MAX_OBJ_TRACKING; j++)
					{
						if (pRectList->mRectArrayList[j].iObjID == iObjID)
						{
							iRectIndex = j;
							break;
						}
					}
					if (iRectIndex > 0)
					{
						pRectList->mRectArrayList[iRectIndex].iIsActive = -1;
						pRectList->mRectArrayList[iRectIndex].iIsNew = -1;
						pRectList->mRectArrayList[iRectIndex].iObjID = -1;
						pRectList->mRectArrayList[iRectIndex].RectElement.x = -1;
						pRectList->mRectArrayList[iRectIndex].RectElement.y = -1;
						pRectList->mRectArrayList[iRectIndex].RectElement.width = -1;
						pRectList->mRectArrayList[iRectIndex].RectElement.height = -1;
					}

					AmtCleanObject(&Amt, iObjID);




					continue;
				}

				//����Ŀ��켣
				AmtGetObjTrajectory(&Amt, iObjID, &pPoints, &iPointNum);

				for (int k = 0; k < iPointNum - 1; ++k)
				{
					cvLine(FrameImg, cvPoint(pPoints[k].x, pPoints[k].y), cvPoint(pPoints[k + 1].x, pPoints[k + 1].y), LineTypeList.LineColor[iObjID % 3], LineTypeList.iThickness[iObjID % 3]);
				}
				CvRect* pEndRect = &(Amt.m_ObjStatus[i].mBbox);
				cvRectangle(FrameImg, cvPoint(pEndRect->x, pEndRect->y), cvPoint(pEndRect->x + pEndRect->width, pEndRect->y + pEndRect->height), cvScalar(0, 255, 0), 1);
				pRectList->mRectArrayList[i].pos[0] = pEndRect->x ;
				pRectList->mRectArrayList[i].pos[1] = pEndRect->y;
				pRectList->mRectArrayList[i].pos[2] = pEndRect->x + pEndRect->width;
				pRectList->mRectArrayList[i].pos[3] = pEndRect->y + pEndRect->height;

				//				cvPolyLine(FrameImg, &pPoints, &iPointNum, 1, 0, CV_RGB(0,255,0), 1);
			}
		}
		cvLine(FrameImg, cvPoint(boundLineX, 0), cvPoint(boundLineX, FrameImg->height - 1), cvScalar(255), 2, 8, 0);
		cvLine(FrameImg, cvPoint(FrameImg->width - boundLineX, 0), cvPoint(FrameImg->width - boundLineX, FrameImg->height - 1), cvScalar(255), 2, 8, 0);
		cvLine(FrameImg, cvPoint(0, boundLineY), cvPoint(FrameImg->width - 1, boundLineY), cvScalar(255), 2, 8, 0);
		cvShowImage(window_name, FrameImg);
		if (bPause == false)
			iPauseKey = cvWaitKey(10);
		else
			iPauseKey = cvWaitKey(0);

		// ������ͣ  P��F1
		if (80 == iPauseKey || 112 == iPauseKey)
			bPause = true;
		else
			bPause = false;


		tinyxml2::XMLDocument xmlDoc;
		//size_t cnt = 0;
		//for (int j = 0; j < MAX_OBJ_TRACKING; ++j)
		//{
		//	if (pRectList->mRectArrayList[j].isAnnotated)
		//	{
		//		cnt++;
		//pRectList->mRectArrayList[frameNum].name = to_string(frameNum);
		char name[6];
		sprintf(name, "%06d", frameNum);

		XMLNode * annotation = xmlDoc.NewElement("annotation");
		xmlDoc.InsertFirstChild(annotation);

		XMLElement * pElement = xmlDoc.NewElement("folder");
		pElement->SetText("VOCType");
		annotation->InsertFirstChild(pElement);

		pElement = xmlDoc.NewElement("filename");
		pElement->SetText(name); //pRectList->mRectArrayList[frameNum].name.c_str());
		annotation->InsertEndChild(pElement);

		pElement = xmlDoc.NewElement("source");
		XMLElement * pElement_sub = xmlDoc.NewElement("database");
		pElement_sub->SetText("VOC");
		pElement->InsertFirstChild(pElement_sub);
		annotation->InsertEndChild(pElement);

		pElement = xmlDoc.NewElement("size");
		pElement_sub = xmlDoc.NewElement("width");
		pElement_sub->SetText(FrameImg->width);//pRectList->mRectArrayList[frameNum].RectElement.width);
		pElement->InsertFirstChild(pElement_sub);
		pElement_sub = xmlDoc.NewElement("height");
		pElement_sub->SetText(FrameImg->height);//pRectList->mRectArrayList[frameNum].RectElement.height);
		pElement->InsertEndChild(pElement_sub);
		pElement_sub = xmlDoc.NewElement("depth");
		pElement_sub->SetText(3);
		pElement->InsertEndChild(pElement_sub);
		annotation->InsertEndChild(pElement);

		pElement = xmlDoc.NewElement("segmented"); // �Ƿ�ָ�
		pElement->SetText(0);
		annotation->InsertEndChild(pElement);

		for (int k = 0; k < object_num; ++k)
		{
			pElement = xmlDoc.NewElement("object");
			pElement_sub = xmlDoc.NewElement("name"); // ���
			pElement_sub->SetText(pRectList->mRectArrayList[k].label.c_str());
			pElement->InsertFirstChild(pElement_sub);

			pElement_sub = xmlDoc.NewElement("pose"); // ��̬
			pElement_sub->SetText("Unspecified");
			pElement->InsertEndChild(pElement_sub);

			pElement_sub = xmlDoc.NewElement("truncated");
			pElement_sub->SetText(0);
			pElement->InsertEndChild(pElement_sub);

			pElement_sub = xmlDoc.NewElement("difficult");
			pElement_sub->SetText(0);
			pElement->InsertEndChild(pElement_sub);

			pElement_sub = xmlDoc.NewElement("bndbox");
			XMLElement* pElement_sub_sub = xmlDoc.NewElement("xmin");
			pElement_sub_sub->SetText(pRectList->mRectArrayList[k].pos[0]);
			pElement_sub->InsertFirstChild(pElement_sub_sub);
			pElement_sub_sub = xmlDoc.NewElement("ymin");
			pElement_sub_sub->SetText(pRectList->mRectArrayList[k].pos[1]);
			pElement_sub->InsertEndChild(pElement_sub_sub);
			pElement_sub_sub = xmlDoc.NewElement("xmax");
			pElement_sub_sub->SetText(pRectList->mRectArrayList[k].pos[2]);
			pElement_sub->InsertEndChild(pElement_sub_sub);
			pElement_sub_sub = xmlDoc.NewElement("ymax");
			pElement_sub_sub->SetText(pRectList->mRectArrayList[k].pos[3]);
			pElement_sub->InsertEndChild(pElement_sub_sub);
			pElement->InsertEndChild(pElement_sub);

			annotation->InsertEndChild(pElement);

		}

	

	string savefile = "D:\\tld_track\\tld_track_��Ŀ��\\tld_track\\Annotations/";
	//for (int x = 0; x < pRectList->mRectArrayList[j].name.length() - 4; ++x)
	//{
	//	filename += pRectList->mRectArrayList[j].name[x];

	savefile += name;
	//}
	savefile += ".xml";
	xmlDoc.SaveFile(savefile.c_str());

	

}
			


	



	// #ifdef  EV_JPEG
	// 	evReleaseCapture(&pCapture); 
	// #else
	cvReleaseCapture(&pCapture);
	// #endif
	cvReleaseImage(&FrameImg);
	cvReleaseImage(&FrameGray);
	cvReleaseImage(&TempImg);
	cvDestroyAllWindows();
	AmtStructUnInit(&Amt);
}


int main(int argc, char* argv[])
{
	EnableMemLeakCheck();
	// 	_CrtSetBreakAlloc(68);

	for (int i =0;i<MAX_OBJ_TRACKING;i++)
	{
		pRectList->mRectArrayList[i].iIsActive = -1;
		pRectList->mRectArrayList[i].iIsNew = -1;
		pRectList->mRectArrayList[i].iObjID = -1;
	}
	pRectList->iTotal = -1;
	pRectList->iNewRectGenerated = -1;

	LineTypeList.iThickness[0] = 2; 
	LineTypeList.LineColor[0]=cvScalar(255,0,0);

	LineTypeList.iThickness[1] = 3; 
	LineTypeList.LineColor[1]=cvScalar(0,255,0);

	LineTypeList.iThickness[2] = 4; 
	LineTypeList.LineColor[2]=cvScalar(0,0,255);


	run_tld(argv[1]);

	//	TestMeanShiftTrack();

	_CrtDumpMemoryLeaks();

	return 0;
}


void On_Mouse( int event, int x, int y, int flags, void* param)
{
	
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		pre_pt = cvPoint(x, y);
		printf("*************************Left Button Down, point is (%d %d)\n\n",pre_pt.x,pre_pt.y);
	}
	if (event == CV_EVENT_MOUSEMOVE && (flags && CV_EVENT_FLAG_LBUTTON))
	{
		end_pt = cvPoint(x, y);
		printf("*************************Mouse Move, point is (%d %d)\r\r",end_pt.x,end_pt.y);
		
		SelectRect.x = MIN(pre_pt.x, end_pt.x);
		SelectRect.y = MIN(pre_pt.y, end_pt.y);
		SelectRect.width = abs(pre_pt.x - end_pt.x);
		SelectRect.height = abs(pre_pt.y - end_pt.y);
		cvCopy(FrameImg,TempImg);
		//		cvLine(temp, pre_pt, end_pt, CV_RGB(255, 0, 0), 2, 8, 0);
		cvRectangle(TempImg, pre_pt, end_pt, CV_RGB(255, 0, 0), 2, 8, 0);
		cvShowImage(window_name,TempImg);
	}
	if (event == CV_EVENT_LBUTTONUP)
	{
		end_pt = cvPoint(x, y);
		printf("*************************Left Mouse Up, point is (%d %d)\n\n",end_pt.x,end_pt.y);

		SelectRect.x = MIN(pre_pt.x, end_pt.x);
		SelectRect.y = MIN(pre_pt.y, end_pt.y);
		SelectRect.width = abs(pre_pt.x - end_pt.x);
		SelectRect.height = abs(pre_pt.y - end_pt.y);

		

		int isfound = -1;

		if ((SelectRect.x >= 0)&&(SelectRect.x<FrameImg->width)&&(SelectRect.y>=0)&&(SelectRect.y<FrameImg->height)&&(SelectRect.width>=MIN_RECT_WIDTH)&&(SelectRect.height>=MIN_RECT_HEIGHT))
		{
			for (int i=0;i<MAX_OBJ_TRACKING;i++)
			{
				if (pRectList->mRectArrayList[i].iIsActive == -1)   
				{
					pRectList->mRectArrayList[i].RectElement.x = SelectRect.x;
					pRectList->mRectArrayList[i].RectElement.y = SelectRect.y;
					pRectList->mRectArrayList[i].RectElement.width = SelectRect.width;
					pRectList->mRectArrayList[i].RectElement.height = SelectRect.height;
					pRectList->mRectArrayList[i].iIsActive = 1;
					pRectList->mRectArrayList[i].iIsNew = 1;
					isfound = i;
					pRectList->iNewRectGenerated = 1;//indicating there is new Rect generated from windows input
					pRectList->mRectArrayList[i].isAnnotated = 1;


					string label ="";
					printf("����label:");
					cin >> label;
					
					pRectList->mRectArrayList[i].label.assign(label, 0, sizeof(label));

					//pRectList->mRectArrayList[i].pos[0] = SelectRect.x + 0.5*SelectRect.width;
					//pRectList->mRectArrayList[i].pos[1] = SelectRect.y + 0.5*SelectRect.height;
					//pRectList->mRectArrayList[i].pos[2] = SelectRect.x + 0.5*SelectRect.width;
					//pRectList->mRectArrayList[i].pos[3] = SelectRect.x + 0.5*SelectRect.height;

					break;
				}
			}
			if (isfound < 0)
			{
				printf("There are too many boxes in executing!\n");
			}
		}

		cvCopy(FrameImg,TempImg); //��FrameImg��ͼ�����ݿ�����TempImg��ȥ
		cvRectangle(TempImg, pre_pt, end_pt, CV_RGB(0, 255, 0), 2, 8, 0);
		cvShowImage(window_name,TempImg);
		cvWaitKey(1);
		track_object = -1;


	}

	
}




