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
	int initARtagPose(int width, int height, float markerWidth, float x_offset, float y_offset, float yaw_offset, float ffactor = 0.97);
	bool getARtagPose(IplImage * src, IplImage * dst, int camID);
	ARtag * getARtag(int index);
	int getARtagSize();
	void setARtagOffset(float x_offset, float y_offset, float yaw_offset);
	int cleanupARtagPose(void);

	static ARtag * tags[50];
	static CRITICAL_SECTION tags_mutex;
	static bool allStop;
	
private:
	int imgwidth;
	int imgheight;
	bool useBCH;
	bool init;
	float xoffset;
	float yoffset;
	float yawoffset;
	float fudge;

	std::vector<ARtag> mytag;
	float patternWidth_;
	float patternCenter_[2];
	ARToolKitPlus::TrackerSingleMarker *tracker;
};