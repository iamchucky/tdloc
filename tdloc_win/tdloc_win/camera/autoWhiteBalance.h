#pragma once
#include "opencv\cv.h"
#include "opencv\highgui.h"
#include <vector>

class CAutoWhiteBalance
{
public:
	CAutoWhiteBalance(void);
	~CAutoWhiteBalance(void);
	
	int DoAutoWhiteBalance(IplImage *im, int *wr, int *wb);

private:

};
class CAutoExposure
{
public:
	CAutoExposure(void);
	~CAutoExposure(void);

	void InitInstance();
	void ExitInstance();
	int Run(IplImage *imL, IplImage *imC, IplImage *imR, int *shutterL, int *shutterC, int *shutterR);
	void GetHistogram(IplImage *imL, IplImage *imC, IplImage *imR);
	void DrawHist(IplImage *image, std::vector<int> &histL, std::vector<int> &histC, std::vector<int> &histR);
	void ResizeImages(	IplImage *imL, IplImage *imC, IplImage *imR,
						IplImage *imLrz, IplImage *imCrz, IplImage *imRrz);
	void GetTrackbarPos(int *left_shutter, int *midd_shutter, int *righ_shutter);

protected:
	std::vector<int> binL;
	std::vector<int> binC;
	std::vector<int> binR;

	int maxL;
	int minL;
	int maxC;
	int minC;
	int maxR;
	int minR;

	int minbound_L;
	int maxbound_L;
	int minbound_C;
	int maxbound_C;
	int minbound_R;
	int maxbound_R;

	int flagL;
	int flagC;
	int flagR;

	CvPoint ptCurr[3];
	CvPoint ptNext[3];

	IplImage *imLrz;
	IplImage *imCrz;
	IplImage *imRrz;
	CvSize imsizerz;

	int redcount;
	int greencount;
	int bluecount;
	int redval;
	int greenval;
	int blueval;

};
