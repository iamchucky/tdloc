#include "AutoWhiteBal.h"
#include <math.h>
CAutoWhiteBal::CAutoWhiteBal(void)
{
	isinit=0;
}

CAutoWhiteBal::~CAutoWhiteBal(void)
{
}

void CAutoWhiteBal::InitInstance(CvSize size)
{
	if(!isinit){
		imsize=size;
		imyuv=cvCreateImage(imsize,8,1);
		isinit=1;
	}
}
void CAutoWhiteBal::ExitInstance()
{
	cvReleaseImage(&imyuv);
}
int CAutoWhiteBal::RunAWB(IplImage *im, unsigned short *red_gain, unsigned short *blue_gain)
{
	//cvShowImage("original",im); 
	//cvWaitKey();
	const float wr=0.299f;
	const float wg=0.114f;
	const float wb=0.587f;
	int R,G,B;
	unsigned char uR,uB,uG;
	float Y,U,V,T;
	
	float Usum=0,Vsum=0;
	float Uave=0,Vave=0;
	float devThresh=0.01;	//color deviation threshold
	int N=0;
	float error;

	const float a=0.8f;
	const float b=0.1f;
	const float d=0;
	const float mu = 0.0312f*25.0f;
	float threshold=0.2753;

	//convert to YUV, extract Y channel

	for(int i=0;i<(im->width*im->height);i+=4){
		B=(unsigned char)(im->imageData[3*i]);
		G=(unsigned char)(im->imageData[3*i+1]);
		R=(unsigned char)(im->imageData[3*i+2]);

				 
		Y = (0.299f*R + 0.587f*G + 0.114f*B);
		U = (-0.299f*R - 0.587f*G + 0.886f*B);
		V = (0.701f*R - 0.587f*G -0.114f*B);
		if(Y!=0){
			T = (abs(U)+abs(V)) / Y;
		}
		else{
			T = 650000;
		}
				
		if(T<threshold){
			//grey point
			Usum+=U;
			Vsum+=V;
			N++;
		}
	}
	if(N!=0){
		Uave=Usum/N;
		Vave=Vsum/N;
	}
	else{
		//*red_gain=0;
		//*blue_gain=0;
		return -1;
	}	

	//if(abs(Uave)<devThresh && abs(Vave)<devThresh)
	if(Uave==Vave && Uave==0)
	{
		//not change the gains
		//perfect balance
		//red_gain=red_gain;
		//blue_gain=blue_gain;
	}
	else if(abs(Uave)>abs(Vave))
	{
		//Need to adjust B_gain
		error=d-Uave;
		*blue_gain += mu*error_weight_fn(error,a,b);
		if(*blue_gain>WB_ADJ_MAX) *blue_gain=WB_ADJ_MAX;
		else if(*blue_gain<WB_ADJ_MIN) *blue_gain=WB_ADJ_MIN;
	}
	else if(abs(Uave)<abs(Vave))
	{
		//need to adjust R_gain
		error=d-Vave;
		*red_gain += mu*error_weight_fn(error,a,b);
		if(*red_gain>WB_ADJ_MAX) *red_gain=WB_ADJ_MAX;
		else if(*red_gain<WB_ADJ_MIN) *red_gain=WB_ADJ_MIN;
	}
	else{
		//not change the gains
		//red_gain=red_gain;
		//blue_gain=blue_gain;
	}
	//cvShowImage("test",imyuv);
	//cvWaitKey();
	return 0;
}
inline float CAutoWhiteBal::error_weight_fn(float error, float a, float b)
{
	float ret=0;
	float errorabs=abs(error);
	if(errorabs>a){
		ret = 2*sign(error);
	}
	else if(errorabs>=b && errorabs<a){
		ret = sign(error);
	}
	else{
		ret = 0;
	}
	return ret;
}

inline int CAutoWhiteBal::sign(float x)
{
	if(x==0){
		return 0;
	}
	else if(x>0){
		return 1;
	}
	else{
		return -1;
	}
}