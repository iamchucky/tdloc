#pragma once
#include "opencv\cv.h"
#include "opencv\highgui.h"

#define WB_ADJ_MAX 800
#define WB_ADJ_MIN 0

class CAutoWhiteBal
{
public:
	CAutoWhiteBal(void);
	~CAutoWhiteBal(void);
	void InitInstance(CvSize size);
	void ExitInstance();
	int RunAWB(IplImage *im, unsigned short *red_gain, unsigned short *blue_gain);

protected:
	CvSize imsize;
	IplImage *imyuv;
	inline float error_weight_fn(float error, float a, float b);
	inline int sign(float x);
	bool isinit;
	
};
