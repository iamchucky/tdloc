#include "ARtagLocalizer.h"
#include "ARToolKitPlus/TrackerSingleMarkerImpl.h"

using namespace std;

const float THIN_PATTERN_BORDER = 0.125;
const float THICK_PATTERN_BORDER = 0.25;

ARtagLocalizer::ARtagLocalizer()
{
	imgwidth = 640;
	imgheight = 480;
	init = false;
	useBCH = true;
	patternCenter_[0] = patternCenter_[1] = 0.0;
	patternWidth_ = 80.0;
}

int ARtagLocalizer::initARtagPose(int width, int height, float markerWidth)
{
    size_t numPixels = width*height;
    cameraBuffer = new unsigned char[numPixels];
	// create a tracker that does:
    //  - 6x6 sized marker images
    //  - samples at a maximum of 6x6
    //  - works with luminance (gray) images
    //  - can load a maximum of 1 pattern
    //  - can detect a maximum of 8 patterns in one image
    tracker = new ARToolKitPlus::TrackerSingleMarkerImpl<6,6,6, 1, 8>(width,height);
	imgwidth = width;
	imgheight = height;
	patternCenter_[0] = patternCenter_[1] = 0.0;

	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
	// load a camera file. 
    if(!tracker->init("..\\..\\ARToolKitPlus\\data\\no_distortion.cal", 1.0f, 1000.0f))
	{
		printf("ERROR: init() failed\n");
		delete cameraBuffer;
		delete tracker;
		return -1;
	}

	patternWidth_ = markerWidth;
	// define size of the marker
    tracker->setPatternWidth(patternWidth_);

	// the marker in the BCH test image has a thin border...
	tracker->setBorderWidth(THIN_PATTERN_BORDER);

    // set a threshold. alternatively we could also activate automatic thresholding
    tracker->setThreshold(150);

    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

    // RPP is more robust than ARToolKit's standard pose estimator
    tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);

	init = true;
	return 0;
}

bool ARtagLocalizer::getARtagPose(IplImage * src)
{
	if (!init)
	{
		printf("Did not initalize the ARtagLocalizer!!\n");
		return NULL;
	}
	if (src->width != imgwidth || src->height != imgheight)
	{
		printf("Image passed in does not match initialized image size!!\n");
		return NULL;
	}
	if (src->nChannels != 1)
	{
		printf("Please pass in grayscale image into ARtagLocalizer! \n");
		return NULL;
	}
	
	int n = 0;
	for(int i = 0; i < src->height; ++i)
		for(int j = 0; j < src->width; ++j)
			cameraBuffer[n++] = CV_IMAGE_ELEM(src,uchar,i,j);

	/*const char* description = tracker->getDescription();
	printf("ARToolKitPlus compile-time information:\n%s\n\n", description);*/

	int numMarkers = 0;
	ARToolKitPlus::ARMarkerInfo* markers = NULL;
	if (tracker->arDetectMarker(const_cast<unsigned char*>(cameraBuffer), 150, &markers, &numMarkers) < 0) 
	{
		return false;
	}

	if (numMarkers == 0 || markers[0].id == -1)
		return false;

	float modelViewMatrix_[16];
	for(int m = 0; m < numMarkers; ++m) {
		if(markers[m].id != -1 && markers[m].cf >= 0.5) {
			tracker->calcOpenGLMatrixFromMarker(&markers[m], patternCenter_, patternWidth_, modelViewMatrix_);

			ARtag ar;
			ar.setId(markers[m].id);

			float x = modelViewMatrix_[12] / 1000.0;
			float y = modelViewMatrix_[13] / 1000.0;
			float z = modelViewMatrix_[14] / 1000.0;
			float yaw = atan2(modelViewMatrix_[1], modelViewMatrix_[0]);

			if ((x == 0 && y == 0 && yaw == 0) || (x > 10000 && y > 10000) || (x < -10000 && y < -10000))
			{
				// ARTKPlus bug that occurs sometimes
				continue;
			}

			printf("Id: %d\t Conf: %.2f\n", markers[m].id, markers[m].cf);
			printf("x: %.2f \t y: %.2f \t z: %.2f \t yaw: %.2f\n", x,y,z,yaw);
			printf("\n");

			char str[30];
			sprintf(str,"%d",markers[m].id);
			cvRectangle(src,cvPoint(markers[m].pos[0]-10,markers[m].pos[1]+10),
				cvPoint(markers[m].pos[0]+10,markers[m].pos[1]-10),cvScalar(150,150,150),-1);
			cvPutText (src,str,cvPoint( markers[m].pos[0]-10,markers[m].pos[1]+5),&cvFont(1,1),cvScalar(0,0,0));

			cv::Mat PoseM(4, 4, CV_32F, modelViewMatrix_);
			cv::transpose(PoseM,PoseM);
			CvMat pose = PoseM;
			ar.setPose(&pose);
			ar.setPoseAge(0);
			tags.push_back(ar);
		}
	}
	printf("\n\n");

	return true;
}

int ARtagLocalizer::cleanupARtagPose(void)
{
	delete [] cameraBuffer;
	delete tracker;
	return 0;
}