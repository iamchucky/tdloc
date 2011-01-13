#pragma once
#ifdef _DEBUG
#pragma comment(lib,"ARToolKitPlusd.lib")
#pragma comment(lib,"ARToolKitPlusDlld.lib")
#else
#pragma comment(lib,"ARToolKitPlus.lib")
#pragma comment(lib,"ARToolKitPlusDll.lib")
#endif

#include "ARtag.h"
#include <ARToolKitPlus/ARToolKitPlus.h>
class ARtagLocalizer
{
public:
	ARtagLocalizer();
	~ARtagLocalizer();
	int initARtagPose(int width, int height, float markerWidth);
	bool getARtagPose(IplImage * src, IplImage * dst);
	int cleanupARtagPose(void);

	std::vector<ARtag> tags;

private:
	int imgwidth;
	int imgheight;
	bool useBCH;
	bool init;

	float patternWidth_;
	float patternCenter_[2];
	ARToolKitPlus::TrackerSingleMarker *tracker;
};