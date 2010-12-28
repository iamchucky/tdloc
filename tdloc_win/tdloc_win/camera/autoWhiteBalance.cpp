#include "autoWhiteBalance.h"


CAutoWhiteBalance::CAutoWhiteBalance(void)
{

}
CAutoWhiteBalance::~CAutoWhiteBalance(void)
{

}

int CAutoWhiteBalance::DoAutoWhiteBalance(IplImage *im, int *wr, int *wb)
{
	/*int curr_wr=*wr;
	int curr_wb=*wb;
	int i,j,ptr;
	float rSum=0;
	float gSum=0;
	float bSum=0;
	float rAve=0;
	float gAve=0;
	float bAve=0;
	int pixCount=0;
	const int thresh = 10;
	const int gain=5;

	if(im->nChannels!=3){
		printf("White Balance error. Colour image not detected!\n");
		return -1;
	}
	//create region of interest for white balance probing
	const int halfSize=10;
	const CvPoint topLeft= cvPoint(im->width/2-halfSize,im->height/2-halfSize);
	const CvPoint botRight=cvPoint(im->height/2+halfSize,im->height/2+halfSize);

	//get average values
	for(i=topLeft.y;i<botRight.y;i++){	//horizontal
		for(j=topLeft.x;j<botRight.x;j++){	//vertical
			ptr=3*(2*j*halfSize+i);
			rSum+=im->imageData[ptr];
			gSum+=im->imageData[ptr+1];
			bSum+=im->imageData[ptr+2];
			pixCount++;
		}	
	}
	//get average
	rAve=rSum/pixCount;
	gAve=gSum/pixCount;
	bAve=bSum/pixCount;

	//control color balance
	if(rAve-gAve<thresh){
		*wr += gain;
	}
	else if(rAve-gAve>thresh){
		*wr -= gain;
	}

	if(bAve-gAve<thresh){
		*wb += gain;
	}
	else if(bAve-gAve>thresh){
		*wb -= gain;
	}

	//Display region
	//cvRectangle(im,
	//*/
	return 0;
}




#define BIN_NUM 255

#define HIST_IM_WIDTH 300
#define HIST_IM_HEIGHT 100

#define BIN_WIDTH 1 //12.75//255/BIN_SIZE
#define EXPOSURE_STEPS 2048	//defined by firewire camera

CAutoExposure::CAutoExposure(void)
{
}

CAutoExposure::~CAutoExposure(void)
{
}

void CAutoExposure::InitInstance()
{
	imsizerz=cvSize(320,160);
	imLrz = cvCreateImage(imsizerz,8,3);
	imCrz = cvCreateImage(imsizerz,8,3);
	imRrz = cvCreateImage(imsizerz,8,3);

	redval=0;
	greenval=0;
	blueval=0;

	
	cvNamedWindow("hist");
	cvNamedWindow("imL",0);
	cvResizeWindow("imL",160,120);
	cvNamedWindow("imC",0);
	cvResizeWindow("imC",160,120);
	cvNamedWindow("imR",0);
	cvResizeWindow("imR",160,120);

	cvCreateTrackbar("red_L","hist",&redval,EXPOSURE_STEPS,NULL);
	cvCreateTrackbar("green_C","hist",&greenval,EXPOSURE_STEPS,NULL);
	cvCreateTrackbar("blue_R","hist",&blueval,EXPOSURE_STEPS,NULL);


}

void CAutoExposure::ExitInstance()
{
	cvReleaseImage(&imLrz);
	cvReleaseImage(&imCrz);
	cvReleaseImage(&imRrz);
}

int CAutoExposure::Run(IplImage *imL, IplImage *imC, IplImage *imR, 
						   int *shutterL, int *shutterC, int *shutterR)
{
	//resize image
	//InitInstance();
	
	//get trackbar position
	//GetTrackbarPos(shutterL,shutterC,shutterR);

	//Compute histogram for all three images
	GetHistogram(imL,imC,imR);
	
	//Access histogram levels
	

	//output shutter results
	*shutterL;
	*shutterC;
	*shutterR;

	//ExitInstance();
	return 0;
}

void CAutoExposure::GetHistogram(IplImage *imL, IplImage *imC, IplImage *imR)
{
	FILE *pfile = fopen("hist.txt","w");
	const int threshold=1;
	int i;
	
	maxL=0;	minL=1000;
	maxC=0;	minC=1000;
	maxR=0;	minR=1000;
	
	binL.assign(256,0);//BIN_NUM
	binC.assign(256,0);
	binR.assign(256,0);

	
	unsigned char val=0;
	int idx;


	//build up histogram
	//int imgarea = imL->width*imL->height;
	int imgarea = 640*480; //640/4 by 480/4
	int rzFactor = 2;
	for(i=0;i<imgarea;i+=2){
		val=(unsigned char)(imL->imageData[i]);
		//idx = val>255?255:val;
		binL[val]++;

		val=(unsigned char)(imC->imageData[i]);
		//idx = val>255?255:val;
		binC[val]++;

		val=(unsigned char)(imR->imageData[i]);
		//idx = val>255?255:val;
		binR[val]++;


		/*
		val=((unsigned char)(imL->imageData[rzFactor*3*i]) + 
			 (unsigned char)(imL->imageData[rzFactor*3*i+1]) + 
			 (unsigned char)(imL->imageData[rzFactor*3*i+2]))*0.33333f ;
		idx = (int)(val/BIN_WIDTH);
		binL[idx]++;
	
		val=((unsigned char)(imC->imageData[rzFactor*3*i]) + 
			 (unsigned char)(imC->imageData[rzFactor*3*i+1]) + 
			 (unsigned char)(imC->imageData[rzFactor*3*i+2]))*0.33333f ;
		idx = (int)(val/BIN_WIDTH);
		binC[idx]++;

		val=((unsigned char)(imR->imageData[rzFactor*3*i]) + 
			 (unsigned char)(imR->imageData[rzFactor*3*i+1]) + 
			 (unsigned char)(imR->imageData[rzFactor*3*i+2]))*0.33333f ;
		idx = (int)(val/BIN_WIDTH);
		binR[idx]++;//*/	
	}
	
	
	//find min/max
	for(i=0;i<BIN_NUM;i++){
		if(binL[i]>maxL)
			maxL=binL[i];
		else if(binL[i]<minL)
			minL=binL[i];
		
		if(binC[i]>maxC)
			maxC=binC[i];
		else if(binC[i]<minC)
			minC=binC[i];

		if(binR[i]>maxR)
			maxR=binR[i];
		else if(binR[i]<minR)
			minR=binR[i];
	}
 	//fclose(pfile);
	//maxL=maxL;
	
	minbound_L=-1;	maxbound_L=-1;
	minbound_C=-1;	maxbound_C=-1;
	minbound_R=-1;	maxbound_R=-1;

	flagL=0;
	flagC=0;
	flagR=0;


	//normalise histogram
	for(i=0;i<BIN_NUM;i++){		
		val=(binL[i]/(float)maxL)*HIST_IM_HEIGHT;
		binL[i]=(int)(val+0.5f);
		if(binL[i]>HIST_IM_HEIGHT)
			binL[i]=HIST_IM_HEIGHT;

		val=(binC[i]/(float)maxC)*HIST_IM_HEIGHT;
		binC[i]=(int)(val+0.5f);
		if(binC[i]>HIST_IM_HEIGHT)
			binC[i]=HIST_IM_HEIGHT;

		val=(binR[i]/(float)maxR)*HIST_IM_HEIGHT;
		binR[i]=(int)(val+0.5f);
		if(binR[i]>HIST_IM_HEIGHT)
			binR[i]=HIST_IM_HEIGHT;
	
		//printf("%d  %d %d %d\n",i, binL[i],binC[i],binR[i]);
		//A Simple State Machine to Find left bound and right bound
		//do for left image
		if(minbound_L==-1 && binL[i]>threshold){		//search for lower bound
			minbound_L=i;				
		}
		if(maxbound_L==-1 && binL[BIN_NUM-1-i]>threshold){	//search for upper bound
			maxbound_L=BIN_NUM-1-i;
		}
		//do for center image
		if(minbound_C==-1 && binC[i]>threshold){		//search for lower bound
			minbound_C=i;
		}
		if(maxbound_C==-1 && binC[BIN_NUM-1-i]>threshold){	//search for upper bound
			maxbound_C=BIN_NUM-1-i;
		}
		//do for right image
		if(minbound_R==-1 && binR[i]>threshold){		//search for lower bound
			minbound_R=i;
		}
		else if(maxbound_R==-1 && binR[BIN_NUM-1-i]>threshold){	//search for upper bound
			maxbound_R=BIN_NUM-1-i;
		}	
	}
	//printf("min_L: %d max_L: %d\n",minbound_L,maxbound_L);
	//printf("min_C: %d max_C: %d\n",minbound_C,maxbound_C);
	//printf("min_R: %d max_R: %d\n",minbound_R,maxbound_R);

	//draw histogram
	IplImage *histimg = cvCreateImage(cvSize(HIST_IM_WIDTH,HIST_IM_HEIGHT),8,3);
	DrawHist(histimg,binL,binC,binR);
	
	cvShowImage("hist",histimg);
	cvShowImage("imL",imL);
	cvShowImage("imC",imC);
	cvShowImage("imR",imR);
	cvWaitKey(1);
}

void CAutoExposure::DrawHist(IplImage *image, std::vector<int> &histL, std::vector<int> &histC, std::vector<int> &histR)
{
	//assume hist is normalized to 0 ~ 100
	int widthStep = (int)(HIST_IM_WIDTH/BIN_NUM);
	cvZero(image);

	int i;
	int x_val=0;

	for(i=0;i<256;i++){
		memcpy(&ptCurr,&ptNext,sizeof(ptCurr));
		x_val+=widthStep;
		
		ptNext[0]=cvPoint(x_val,HIST_IM_HEIGHT-histL[i]);
		ptNext[1]=cvPoint(x_val,HIST_IM_HEIGHT-histC[i]);
		ptNext[2]=cvPoint(x_val,HIST_IM_HEIGHT-histR[i]);
		
		cvLine( image,
				ptCurr[0],
				ptNext[0],
				CV_RGB(255,0,0),
				2,8);
		cvLine( image,
				ptCurr[1],
				ptNext[1],
				CV_RGB(0,255,0),
				2,8);
		cvLine( image,
				ptCurr[2],
				ptNext[2],
				CV_RGB(0,0,255),
				2,8);


		/*cvLine( image,
				cvPoint(x_val,HIST_IM_HEIGHT),
				cvPoint(x_val,HIST_IM_HEIGHT-histL[i]),
				CV_RGB(255,0,0),
				2,8);
		cvLine( image,
				cvPoint(x_val,HIST_IM_HEIGHT),
				cvPoint(x_val,HIST_IM_HEIGHT-histC[i]),
				CV_RGB(255,0,0),
				2,8);
		cvLine( image,
				cvPoint(x_val,HIST_IM_HEIGHT),
				cvPoint(x_val,HIST_IM_HEIGHT-histR[i]),
				CV_RGB(255,0,0),
				2,8);*/


	}//end for loop
}
void CAutoExposure::ResizeImages(	IplImage *imL, IplImage *imC, IplImage *imR,
						IplImage *imLrz, IplImage *imCrz, IplImage *imRrz)
{
	int imgarea=imCrz->width*imCrz->height;
	int rzFactor=2;
	for(int i=0;i<imgarea;i++){
		imLrz->imageData[3*i]=  imL->imageData[rzFactor*3*i];
		imLrz->imageData[3*i+1]=imL->imageData[rzFactor*3*i+1];
		imLrz->imageData[3*i+2]=imL->imageData[rzFactor*3*i+2];

		imCrz->imageData[3*i]=  imC->imageData[rzFactor*3*i];
		imCrz->imageData[3*i+1]=imC->imageData[rzFactor*3*i+1];
		imCrz->imageData[3*i+2]=imC->imageData[rzFactor*3*i+2];

		imRrz->imageData[3*i]=  imR->imageData[rzFactor*3*i];
		imRrz->imageData[3*i+1]=imR->imageData[rzFactor*3*i+1];
		imRrz->imageData[3*i+2]=imR->imageData[rzFactor*3*i+2];

	}

}
void CAutoExposure::GetTrackbarPos(int *left_shutter, int *midd_shutter, int *righ_shutter)
{
	cvGetTrackbarPos("red_L","hist");
	cvGetTrackbarPos("green_C","hist");
	cvGetTrackbarPos("blue_R","hist");

	left_shutter=&redval;
	midd_shutter=&greenval;
	righ_shutter=&blueval;
}
