//amn32 - October 19, 2009
//Provides a memory mapped file interface for the 1394 camera. 
//Accurately synchronizes the camera with
//the pulse coming from the timing system.
#define DISPLAY_ON 1

#ifndef _WIN32_WINNT		// Allow use of features specific to Windows XP or later.                   
#define _WIN32_WINNT 0x0501	// Change this to the appropriate value to target other versions of Windows.
#endif			

#define UNDIST true		// Setting to undistort the camera to account for lens distortion.

#include <stdio.h>
#include <tchar.h>
#include <string>
#include <sstream>

#include "camera\sync1394camera.h"
//#include "camera\actuationInterface.h"
//#include "camera\externTrigger.h"
#include "camera\autowhitebalance.h"
#include "opencv\cv.h"
#include "opencv\cxcore.h"
#include "opencv\highgui.h"


using namespace std;

//the name of the memory mapped file
char szName[]="Global\\CamMappingObject";

//undistortion map location for unibrain
char mapxloc[]="..\\Unibrain\\mapx.xml";
char mapyloc[]="..\\Unibrain\\mapy.xml";

HANDLE hMapFile;
HANDLE ghMutex;
HANDLE instanceMutex;
HANDLE close_event;
HANDLE main_thread;

enum CameraType
{
	CAM_BASLER622, CAM_BASLER311, CAM_PROSILICA, CAM_UNIBRAIN, CAM_FIREFLY
};

camera_adjust_param_t camSetting[3];	//this is the settings for all 3 cameras
CRITICAL_SECTION csGui;

volatile bool running=true;
void* pBuf;
int WIDTH;
int HEIGHT;
int syncID=0;
int camOrder=12;
int AGCbot;
int AGCtop;
bool isSlave;
bool upsidedown=false;
CameraType camtype=CAM_UNIBRAIN;

int numChannels=3;
unsigned int frameNum=0;		
double lasttime=0;

//member variables
//CAutoWhiteBalance m_whitebal;

//captures the closing of the console window and closes the app appropriately
BOOL WINAPI handler_routine(DWORD dwCtrlType) {
	BOOL ret = FALSE;
	SetEvent(close_event);
	running = false;
	if (WaitForSingleObject(main_thread, 4000) == WAIT_OBJECT_0) 
	{
		printf("Main thread terminated OK\n");
		ret = TRUE;
	}
	CloseHandle(main_thread);
	ExitProcess(3);
	return ret;
}

//ActInterfaceReceiver* actuation; 
//double timestampACT = 0;
//void ActCallback(double timestamp, ActInterfaceReceiver* rx, void* data)
//{
//	timestampACT = timestamp;
//}
//
//eTrigReceiver* externTrigger; 
//double timestampETG = 0;
//void EtrigCallback(double timestamp, eTrigReceiver* rx, void* data)
//{
//	timestampETG = timestamp;
//}

//The GUI displays for adjusting camera parameters
int currentCamSelect = 0;
void camID_Track_Callback(int cam_id)
{
	currentCamSelect=cam_id;
	cvSetTrackbarPos("Gain", "CameraServer_GUI", camSetting[currentCamSelect].setGain);
	cvSetTrackbarPos("White Bal", "CameraServer_GUI", camSetting[currentCamSelect].setWhiteBal);
	cvSetTrackbarPos("Shutter", "CameraServer_GUI", camSetting[currentCamSelect].setShutter);
}

void camgain_Track_Callback(int gain_new)
{
	EnterCriticalSection(&csGui);
	camSetting[currentCamSelect].setGain = gain_new;
	camSetting[currentCamSelect].updateCamera=1;	
	LeaveCriticalSection(&csGui);
//	if (cam!=NULL)
//		cam->SetGain(currentCamSelect, gain_new);
}

void camwhite_Track_Callback(int wb_new)
{
	EnterCriticalSection(&csGui);
	camSetting[currentCamSelect].setWhiteBal = wb_new;
	camSetting[currentCamSelect].updateCamera=1;
	LeaveCriticalSection(&csGui);
}

void camshuttle_Track_Callback(int shuttle_new)
{
	EnterCriticalSection(&csGui);
	camSetting[currentCamSelect].setShutter = shuttle_new;
	camSetting[currentCamSelect].updateCamera=1;
	LeaveCriticalSection(&csGui);

//	if (cam!=NULL)
//		cam->SetShutter(currentCamSelect, shuttle_new);
}
void InitGUI()
{
	int tmpThres;
	//cvShowImage("GUI", img);
	//cvNamedWindow ("GUI");
	//cvResizeWindow("GUI",200,0);
	cvNamedWindow("CameraServer_GUI",0);
	//cvResizeWindow("CameraServer_GUI",(int)(WIDTH*0.5),(int)(HEIGHT*0.5));

	tmpThres = (int)currentCamSelect;	 //default value
	cvCreateTrackbar("Camera ID", "CameraServer_GUI", &tmpThres, 2, 0);//;camID_Track_Callback
	
	tmpThres = camSetting[currentCamSelect].setGain;	 //default value
	cvCreateTrackbar("Gain", "CameraServer_GUI", &tmpThres, GAIN_MAX -GAIN_MIN +1, 0);//camgain_Track_Callback
	
	tmpThres = camSetting[currentCamSelect].setWhiteBal;	 //default value
	cvCreateTrackbar("White Bal", "CameraServer_GUI", &tmpThres, WB_MAX -WB_MIN +1, 0);//camwhite_Track_Callback
	
	tmpThres = camSetting[currentCamSelect].setShutter;	 //default value
	cvCreateTrackbar("Shutter", "CameraServer_GUI", &tmpThres, SHUTTER_MAX -SHUTTER_MIN +1, 0);//camshuttle_Track_Callback
}
void DestroyGUI()
{
	cvDestroyWindow("CameraServer_GUI");
}
int AutoWhiteBalance(int camType, int camId, IplImage *img)
{
	
	return 0;
}

int main(int argc, const char* argv[])
{
	//check there is only one instance of this running (checking InstanceMutex)
	instanceMutex = CreateMutexA(NULL,TRUE,"InstanceMutex");   
	if (instanceMutex == NULL) return -1;

	//now try to create the camera mutex
	ghMutex = CreateMutexA(NULL,FALSE,"CamMutex");           
	if (ghMutex == NULL) printf("CreateMutex error: %d\n", GetLastError());

	//create a global event to let clients know when this closes
	close_event = CreateEventA(NULL, TRUE, FALSE, "camMMF_quit");

	//duplicate the handle to this thread so we can detect when the app closes
	if (!DuplicateHandle(GetCurrentProcess(), GetCurrentThread(), GetCurrentProcess(),  &main_thread, 0, FALSE, DUPLICATE_SAME_ACCESS)) 
		printf("could not duplicate app thread handle\n");

	//setup the handler for when the console is closed
	SetConsoleCtrlHandler(handler_routine, TRUE);


	//finally, create a new camera
	Sync1394Camera* cam = new Sync1394Camera ();
	SyncCamParams s;

	if (argc>1)
	{
		if (strcmp(argv[1],"FLIP")==0) 
		{
			upsidedown = true;			
			printf ("RUNNING UPSIDEDOWN.\n");			
		}
		else
		{
			upsidedown = false;			
			printf ("RUNNING RIGHTSIDEUP.\n");			
		}
		if (strcmp(argv[2],"SLAVE")==0) 
		{
			isSlave = true;			
			printf ("RUNNING AS SLAVE.\n");			
		}
		else
		{
			isSlave = false;
			printf ("RUNNING AS MASTER.\n");			
		}

		if (strcmp(argv[3],"A311")==0) 
		{
			camtype = CAM_BASLER311;
			printf ("USING A311.\n");			
		}
		else if (strcmp(argv[3],"PROSILICA") ==0 )
		{
			camtype = CAM_PROSILICA;
			printf ("USING prosilica");
		}
		else if (strcmp(argv[3],"UNIBRAIN") == 0)
		{
			camtype = CAM_UNIBRAIN;
			printf("Using unibrain");
		}
		else if (strcmp(argv[3],"FIREFLY") == 0)
		{
			camtype = CAM_FIREFLY;
			printf("Using PGR Firefly");
		}
		else if (strcmp(argv[3],"A622") == 0)
		{
			camtype = CAM_BASLER622;
			printf("Using CAM_BASLER622");
		}
		else
		{
			camtype = CAM_UNIBRAIN;
			printf("Could not decifer camera type, so using unibrain");	
		}
		syncID = atoi(argv[4]);		
		//printf("\nSyncID is %d\n",syncID);	
		camOrder = atoi(argv[5]);
	}
	else
		printf("Warning: No Command Line Arguments Defined. Using Defaults.\n");


	if (camtype == CAM_BASLER622)
	{
		cout<<"Using Basler A622"<<endl;
		WIDTH	=	1280;
		HEIGHT=	1024;
		numChannels=  1;
		s.isColor = false; 
		s.width = WIDTH; 
		s.height = HEIGHT; 
		s.videoMode = 0; 
		s.videoFormat = 7; 
		s.BitDepth16 = false;
		s.usePartialScan = true; 
		s.syncEnabled = true;
		s.partialHeight = 1024; 
		s.partialWidth = 1280;
		s.syncFPS = 16; 
		s.AGC = true; 
		s.bytesPerPacket = 3280; 
		s.AGCbot = AGCbot; 
		s.AGCtop = AGCtop; 
		s.syncID = syncID; 
		s.isSlave = isSlave;
		printf("SyncID is %d\n",syncID);
		if (syncID ==0) 
			s.syncCamInput = 1;
		else			
			s.syncCamInput = 0;
	}
	else if (camtype == CAM_BASLER311)
	{
		cout<<"Using Basler A311"<<endl;
		WIDTH	=	640;
		HEIGHT=	480;
		numChannels=3;
		s.isColor = true; 
		s.width = WIDTH; 
		s.height = HEIGHT; 
		s.videoMode = 3; 
		s.videoFormat = 0; 
		s.BitDepth16 = false;
		s.usePartialScan = false; 
		s.syncEnabled = true; 
		s.syncFPS = 20; 
		s.AGC = true; 		 
		s.AGCbot = 480; 
		s.AGCtop = 0; 
		s.syncID = syncID; 
		s.isSlave = isSlave;
		printf("SyncID is %d\n",syncID);
		if (syncID ==0) 
			s.syncCamInput = 1;
		else			
			s.syncCamInput = 0;
		cam->idealMedian = 50;
	}
	else if (camtype == CAM_PROSILICA)
	{
		cout<<"Using Prosilica"<<endl;
		WIDTH	=	1024;
		HEIGHT=	768;
		numChannels= 3;
		s.isColor = true; 
		s.width = WIDTH; 
		s.height = HEIGHT; 
		s.videoMode = 2; 
		s.videoFormat = 7; 
		s.BitDepth16 = false;
		s.usePartialScan = true; 
		s.syncEnabled = true;  //DEBUG DEBUG DEBUG
		s.partialHeight = 768; 
		s.partialWidth = 1024;
		s.syncFPS = 16; 
		s.AGC = false; 
		s.bytesPerPacket = 3280; 
		s.AGCbot = AGCbot; 
		s.AGCtop = AGCtop; 
		s.syncID = syncID; 
		s.isSlave = isSlave;
		printf("SyncID is %d\n",syncID);
		if (syncID ==0) 
			s.syncCamInput = 1;
		else			
			s.syncCamInput = 0;
	}
	else if (camtype == CAM_UNIBRAIN)
	{
		cout<<"Using Unibrain"<<endl;
		WIDTH=640;
		HEIGHT=	480;
		numChannels= 3;
		s.isColor = true; 
		s.width = WIDTH; 
		s.height = HEIGHT; 
		s.videoMode = 3; 
		s.videoFormat = 0; 
		s.videoFrameRate = 3;
		s.BitDepth16 = false;
		s.usePartialScan = false; 
		s.syncEnabled = false;  //DEBUG DEBUG DEBUG
		s.partialHeight = 480; 
		s.partialWidth = 640;
		s.syncFPS = 30; 
		s.AGC = false; 
		s.bytesPerPacket = 0; 
		s.AGCbot = 0; 
		s.AGCtop = 640; 
		s.syncID = syncID; 
		s.isSlave = isSlave;
		printf("SyncID is %d\n",syncID);
		if (syncID ==0) 
			s.syncCamInput = 1;
		else			
			s.syncCamInput = 0;
	}
	else if (camtype == CAM_FIREFLY)
	{
		cout<<"Using PGR Firefly"<<endl;
		WIDTH = 3*640;
		HEIGHT = 480;
		numChannels = 1;
		s.isColor = false; 
		s.width = 640; 
		s.height = HEIGHT; 
		s.videoMode = 5; 
		s.videoFormat = 0; 
		s.videoFrameRate = 4;
		s.BitDepth16 = false;
		s.usePartialScan = false; 
		s.syncEnabled = false;  //DEBUG DEBUG DEBUG
		s.eTrigEnabled = true;
		s.AutoShutter = false;
		s.AutoGain = false;
		s.AutoBrightness = false;
		s.AutoWB = true;
		s.gammaEnable = false;
		s.adjWB = true;
		s.partialHeight = 480; 
		s.partialWidth = 640;
		s.syncFPS = 30; 
		s.AGC = true; 
		s.bytesPerPacket = 0; 
		s.AGCbot = 480; 
		s.AGCtop = 0; 
		s.syncID = syncID; 
		s.isSlave = isSlave;
		printf("Sync with External Trigger\n");
	}
	
	//// Initialize timestamp receiver
	//if(s.eTrigEnabled)
	//{
	//	externTrigger = new eTrigReceiver();
	//	externTrigger->SetFeedbackCallback (EtrigCallback, NULL);
	//}
	//else
	//{
	//	actuation = new ActInterfaceReceiver();
	//	actuation->SetFeedbackCallback (ActCallback, NULL);
	//}

	IplImage* img;
	IplImage* imgrz;
	IplImage* imgBW;
	IplImage* imrgb;

	if (s.isColor)
	{
		img = cvCreateImage (cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,3);
		imgrz = cvCreateImage (cvSize(640,480),IPL_DEPTH_8U,3);
	}
	else
	{
		if (camtype != CAM_FIREFLY)
		{
			img = cvCreateImage (cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,1);
			imgrz = cvCreateImage (cvSize(640,480),IPL_DEPTH_8U,1);
		}
		else
		{
			/*imgBW = cvCreateImage (cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,1);
			img = cvCreateImage (cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,3);
			imgrz = cvCreateImage (cvSize(WIDTH/2,HEIGHT/2),IPL_DEPTH_8U,3);*/
			img = cvCreateImage (cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,1);
			
			imgrz = cvCreateImage (cvSize(WIDTH/2,HEIGHT/2),IPL_DEPTH_8U,3);
			imrgb = cvCreateImage (cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,3);
		}
	}		
	IplImage* mapx;
	IplImage* mapy;

	if (UNDIST)
	{
		// Build the undistort map which we will use for all 
		// subsequent frames.
		//
		mapx = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
		mapy = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
		mapx = (IplImage*)cvLoad(mapxloc);
		mapy = (IplImage*)cvLoad(mapyloc);
	}
	printf("Initializing camera....\n\n");
	
	
	cam->InitCamera(0, s);
	if (camtype == CAM_FIREFLY)
	{
		cam->InitCamera(1, s);
		cam->InitCamera(2, s);
	}
	Sleep(10);

	hMapFile = CreateFileMappingA(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security 
		PAGE_READWRITE,          // read/write access
		0,                       // max. object size 
		(WIDTH*HEIGHT*numChannels) + sizeof(double), // buffer size  
		szName);                 // name of mapping object

	if (hMapFile == NULL) 
	{ 
		printf("Could not create file mapping object (%d).\n", 
			GetLastError());

	}
	pBuf = MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,                   
		0,                   
		0);            //goto max size

	if (pBuf == NULL) 
	{ 
		printf("Could not map view of file (%d).\n", GetLastError()); 
	}
	running=true;
	//printf("\n\nPress q to quit\n\n");
	//*
	cvNamedWindow ("CameraServer. Press q to quit");
	cvResizeWindow("CameraServer. Press q to quit",200,0);//*/
	
	//initialise GUI
	//InitializeCriticalSection(&csGui);
	//InitGUI();
	bool display_image=0;
	//camera_adjust_param_t mycamSetting[3];
	//memset(&mycamSetting,0,sizeof(mycamSetting));

	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	while(running)
	{	

		//wait for an image
		WaitForSingleObject (cam->cameraEvent,2000);
		if (camtype == CAM_FIREFLY)
		{
			WaitForSingleObject (cam->cameraEvent1,2000);
			WaitForSingleObject (cam->cameraEvent2,2000);
		}
		if (((cam->buf) == NULL)||((camtype == CAM_FIREFLY)&&(((cam->buf1) == NULL)||((cam->buf2) == NULL))))
		{
			if (cvWaitKey (1)=='q')
				running=false;
			if (running)
				printf("WARNING: No Image. Check cam sync is on and connected, then restart.\n");
			else
				printf("WARNING: No Image TERMINATING.\n");
			WaitForSingleObject(ghMutex,5000); //steal the mutex to prevent further processing in host apps
			continue;
		}
		
		//Pass in camera adjustments parameters
/*		EnterCriticalSection(&csGui);
		memcpy(&mycamSetting,&camSetting,sizeof(mycamSetting));
		LeaveCriticalSection(&csGui);
		cam->DoManualAdjustments(mycamSetting);//*/

		EnterCriticalSection(&cam->camgrab_cs);
		if (s.isColor)
		{
			for (int i=0; i<WIDTH*HEIGHT; i++)
			{				
				img->imageData[(i*3)] =  (cam->buf[(i*3)+2]);
				img->imageData[(i*3)+1] =(cam->buf[(i*3)+1]);
				img->imageData[(i*3)+2] =(cam->buf[(i*3)]);
			}
		}		
		else 
		{
			if (camtype != CAM_FIREFLY)
			{
				for (int i=0; i<WIDTH*HEIGHT; i++)
				{
					if ((cam->buf) == NULL) {/*printf("WARNING BAD POINTER 2!\n");*/ continue;}
					char shit = i[((char*)(cam->buf))];				
					img->imageData[i] = shit>255?255:shit;
				}
			}
			else
			{
				// combine all 3 camera images
				int col = 0;
				int row = 0;
				for (int i=0; i<640*480; i++)
				{
					if ((cam->buf) == NULL) { continue;}
					char shit = i[((char*)(cam->buf2))];	
					char shit1 = i[((char*)(cam->buf1))];
					char shit2 = i[((char*)(cam->buf))];
					switch(camOrder)
					{
						case 12:
							shit = i[((char*)(cam->buf))];
							shit1 = i[((char*)(cam->buf1))];
							shit2 = i[((char*)(cam->buf2))];
							break;
						case 21:
							shit = i[((char*)(cam->buf))];
							shit1 = i[((char*)(cam->buf2))];
							shit2 = i[((char*)(cam->buf1))];
							break;
						case 120:
							shit = i[((char*)(cam->buf1))];
							shit1 = i[((char*)(cam->buf2))];
							shit2 = i[((char*)(cam->buf))];
							break;
						case 210:
							shit = i[((char*)(cam->buf2))];
							shit1 = i[((char*)(cam->buf1))];
							shit2 = i[((char*)(cam->buf))];
							break;
						case 102:
							shit = i[((char*)(cam->buf1))];
							shit1 = i[((char*)(cam->buf))];
							shit2 = i[((char*)(cam->buf2))];
							break;
						case 201:
							shit = i[((char*)(cam->buf2))];
							shit1 = i[((char*)(cam->buf))];
							shit2 = i[((char*)(cam->buf1))];
							break;
						default:
							shit = i[((char*)(cam->buf))];
							shit1 = i[((char*)(cam->buf1))];
							shit2 = i[((char*)(cam->buf2))];
							break;
					}

					/*((uchar*)(imgBW->imageData + imgBW->widthStep*row))[col] = shit>255?255:shit;
					((uchar*)(imgBW->imageData + imgBW->widthStep*row))[col+640] = shit1>255?255:shit1;
					((uchar*)(imgBW->imageData + imgBW->widthStep*row))[col+2*640] = shit2>255?255:shit2;*/

					((uchar*)(img->imageData + img->widthStep*row))[col] = shit>255?255:shit;
					((uchar*)(img->imageData + img->widthStep*row))[col+640] = shit1>255?255:shit1;
					((uchar*)(img->imageData + img->widthStep*row))[col+2*640] = shit2>255?255:shit2;

					if (col < 639)
					{
						col++;
					}
					else
					{
						col = 0;
						row++;
					}
				}
			}
		}

		double	timestamp = cam->curtimestamp;
		//double	timestamp;
		//if(s.eTrigEnabled)
		//	timestamp = timestampETG;
		//else
		//	timestamp = timestampACT;	
		
		LeaveCriticalSection(&cam->camgrab_cs);


//		if (WaitForSingleObject(ghMutex,5000)!=WAIT_OBJECT_0)
//		{
//			printf("Warning: Did not receive global map handle in 5 seconds...");
//			continue;
//		}
//		/*if (camtype == CAM_FIREFLY)
//			cvCvtColor(imgBW,img,CV_BayerBG2BGR);		// convert from Bayer to Color RGB*/
//
//		//dst src size
//		if (upsidedown)
//			cvFlip (img,img,-1);
		if (UNDIST)
		{
			IplImage *t = cvCloneImage(img);
			cvRemap( t, img, mapx, mapy );
			cvReleaseImage(&t);
		}
////*		//cvShowImage("Test", img);
//		CopyMemory(pBuf, img->imageData, WIDTH*HEIGHT*numChannels);		
//		CopyMemory((char*)pBuf+(WIDTH*HEIGHT*numChannels), &timestamp,sizeof(double));
//		ReleaseMutex(ghMutex);//*/

		//GUI stuffs
/*		if(display_image){
			if(camtype == CAM_FIREFLY){
				//cvResize(img,imgrz,1);
				//cvCvtColor(imgrz,imrgb,CV_BayerBG2BGR);
				//cvShowImage("CameraServer_GUI", imrgb);
			}
			else{
				//cvResize(img,imgrz,1);
				//cvShowImage("CameraServer_GUI", imgrz);
			}
		}	//*/
		int wb_wr=0;
		int wb_wb=0;
		const int wbStep=5;
		switch(cvWaitKey (10)){
			case 'q':
				running=false;
				break;
			case 0x20:	//space-bar
				if(!display_image){
					display_image = !display_image;
					cvResizeWindow("CameraServer. Press q to quit",img->width, img->height);
					
				}
				else{
					display_image = !display_image;
					cvResizeWindow("CameraServer. Press q to quit",200, 0);
					
				}				
				break;
			case '1':
				currentCamSelect=0;
				break;
			case '2':
				currentCamSelect=1;
				break;
			case '3':
				currentCamSelect=2;
				break;
			case 'n':
				cam->GetWhiteBal(currentCamSelect,&wb_wr,&wb_wb);
				wb_wr-=wbStep;
				if(wb_wr<WB_MIN) wb_wr=WB_MIN;
				cam->SetWhiteBal(currentCamSelect,wb_wr,wb_wb);
				printf("White balance: cam%d: %d, %d\n",currentCamSelect,wb_wr,wb_wb);
				break;
			case 'm':
				cam->GetWhiteBal(currentCamSelect,&wb_wr,&wb_wb);
				wb_wr+=wbStep;
				if(wb_wr>WB_MAX) wb_wr=WB_MAX;
				cam->SetWhiteBal(currentCamSelect,wb_wr,wb_wb);
				printf("White balance: cam%d: %d, %d\n",currentCamSelect,wb_wr,wb_wb);
				break;
			case ',':
				cam->GetWhiteBal(currentCamSelect,&wb_wr,&wb_wb);
				wb_wb-=wbStep;
				if(wb_wb<WB_MIN) wb_wb=WB_MIN;
				cam->SetWhiteBal(currentCamSelect,wb_wr,wb_wb);
				printf("White balance: cam%d: %d, %d\n",currentCamSelect,wb_wr,wb_wb);
				break;
			case '.':
				cam->GetWhiteBal(currentCamSelect,&wb_wr,&wb_wb);
				wb_wb+=wbStep;
				if(wb_wb>WB_MAX) wb_wb=WB_MAX;
				cam->SetWhiteBal(currentCamSelect,wb_wr,wb_wb);
				printf("White balance: cam%d: %d, %d\n",currentCamSelect,wb_wr,wb_wb);
				break;
			case 'b':
				AutoWhiteBalance(camtype, currentCamSelect, img);
				//cam->DoAutoWhiteBalance(camtype,currentCamSelect,&wb_wr,&wb_wb);

		}
		
		frameNum++;	

		if(DISPLAY_ON && display_image){
			if(frameNum%3!=0) continue; 
		/*
		cvNamedWindow ("CameraServer. q to quit");
		cvResizeWindow("Click Me + q to quit",200,0);//*/
		
		//*
		//cvNamedWindow ("img", 0);
		
		//cvLine (img,cvPoint(0,s.AGCbot),cvPoint(WIDTH,s.AGCbot),cvScalar(0,0,255));
		//cvLine (img,cvPoint(0,s.AGCtop),cvPoint(WIDTH,s.AGCtop),cvScalar(0,0,255));
		//IplImage *imrz = cvCreateImage(cvSize(img->width/2,img->height/2),8,1);
		//IplImage *imrgb = cvCreateImage(cvGetSize(img),8,3);
		//cvResize (img,imrz);
		
		switch(camtype){
			case CAM_FIREFLY:
				cvCvtColor(img,imrgb,CV_BayerBG2BGR);
				cvResize(imrgb,imgrz,1);
				cvShowImage("CameraServer. Press q to quit",imgrz );
				break;
			case CAM_UNIBRAIN:
				cvResize(img,imgrz,1);
				cvShowImage("CameraServer. Press q to quit",imgrz );
				break;
			default:
				cvShowImage("CameraServer. Press q to quit",img );
				break;
		}
		//cvReleaseImage(&imrz);
		//cvReleaseImage(&imrgb);//*/
		}
		
	}
	UnmapViewOfFile(pBuf);
	CloseHandle(hMapFile);
	CloseHandle(close_event);
	DeleteCriticalSection(&csGui);
	delete cam;
	Sleep(1000);
}