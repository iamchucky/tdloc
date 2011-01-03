//amn32 - October 19, 2009
//Provides a memory mapped file interface for the 1394 camera. 
//Accurately synchronizes the camera with
//the pulse coming from the timing system.
#define DISPLAY_ON 1

#ifndef _WIN32_WINNT		// Allow use of features specific to Windows XP or later.                   
#define _WIN32_WINNT 0x0501	// Change this to the appropriate value to target other versions of Windows.
#endif			

#include <stdio.h>
#include <tchar.h>
#include <string>
#include <sstream>

#include "camera\sync1394camera.h"
#include "opencv\cv.h"
#include "opencv\cxcore.h"
#include "opencv\highgui.h"


using namespace std;

//the name of the memory mapped file
char szName[]="CamMappingObject";

HANDLE hMapFile;
HANDLE ghMutex;
HANDLE instanceMutex;
HANDLE close_event;
HANDLE main_thread;

enum CameraType
{
	CAM_BASLER622, CAM_BASLER311, CAM_PROSILICA, CAM_UNIBRAIN, CAM_FIREFLY
};

volatile bool running=true;
void* pBuf;
int WIDTH;
int HEIGHT;
int camOrder=12;
bool upsidedown=false;
CameraType camtype=CAM_UNIBRAIN;

int numChannels=3;
unsigned int frameNum=0;		
double lasttime=0;

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
	
	//grab input command line arguments
	if (argc>1)
	{
		if (strcmp(argv[1],"UNIBRAIN") == 0)
		{
			camtype = CAM_UNIBRAIN;
			printf("Using Unibrain");
		}
		else if (strcmp(argv[1],"FIREFLY") == 0)
		{
			camtype = CAM_FIREFLY;
			printf("Using PGR Firefly");
		}
		else
		{
			camtype = CAM_UNIBRAIN;
			printf("Could not decifer camera type, so using unibrain");	
		}
		camOrder = atoi(argv[2]);
	}
	else
		printf("Warning: No Command Line Arguments Defined. Using Defaults.\n");

	SyncCamParams s;
	if (camtype == CAM_UNIBRAIN)
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
		s.syncID = 0; 
		s.isSlave = false;
		s.syncCamInput = 0;
	}
	else
	{
	}
	
	IplImage* img;
	IplImage* imgrz;
	if (s.isColor)
	{
		img = cvCreateImage (cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,3);
		imgrz = cvCreateImage (cvSize(WIDTH/2,HEIGHT/2),IPL_DEPTH_8U,3);
	}
	else
	{
		img = cvCreateImage (cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,1);
		imgrz = cvCreateImage (cvSize(WIDTH/2,HEIGHT/2),IPL_DEPTH_8U,1);
	}		

	printf("Initializing camera....\n\n");
	
	//finally, create a new camera
	Sync1394Camera* cam[50];			// support up to 50 cam pointers
	cam[0] = new Sync1394Camera ();
	cam[0]->InitCamera(0, s);
	if (camtype == CAM_FIREFLY)
	{
		cam[1] = new Sync1394Camera ();
		cam[2] = new Sync1394Camera ();
		cam[1]->InitCamera(1, s);
		cam[2]->InitCamera(2, s);
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
		printf("Could not create file mapping object (%d).\n", GetLastError());
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
	
	//cvNamedWindow ("CameraServer. Press q to quit");
	//cvResizeWindow("CameraServer. Press q to quit",200,0);//*/
	
	bool display_image=0;

	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	while(running)
	{	

		//wait for an image
		if (WaitForSingleObject (cam[0]->cameraEvent,2000) == WAIT_TIMEOUT)
		{
			continue;
		}
		if (camtype == CAM_FIREFLY)
		{
			WaitForSingleObject (cam[1]->cameraEvent,2000);
			WaitForSingleObject (cam[2]->cameraEvent,2000);
		}
		if (((cam[0]->buf) == NULL)||((camtype == CAM_FIREFLY)&&(((cam[1]->buf) == NULL)||((cam[2]->buf) == NULL))))
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
		
		EnterCriticalSection(&cam[0]->camgrab_cs);
		if (camtype == CAM_FIREFLY)
		{
			EnterCriticalSection(&cam[1]->camgrab_cs);
			EnterCriticalSection(&cam[2]->camgrab_cs);
		}
		if (s.isColor)
		{
			for (int i=0; i<WIDTH*HEIGHT; i++)
			{				
				img->imageData[(i*3)] =  (cam[0]->buf[(i*3)+2]);
				img->imageData[(i*3)+1] =(cam[0]->buf[(i*3)+1]);
				img->imageData[(i*3)+2] =(cam[0]->buf[(i*3)]);
			}
		}		
		else 
		{
			if (camtype != CAM_FIREFLY)
			{
				for (int i=0; i<WIDTH*HEIGHT; i++)
				{
					if ((cam[0]->buf) == NULL) {/*printf("WARNING BAD POINTER 2!\n");*/ continue;}
					char buf = i[((char*)(cam[0]->buf))];				
					img->imageData[i] = buf>255?255:buf;
				}
			}
			else
			{
				// combine all 3 camera images
				int col = 0;
				int row = 0;
				for (int i=0; i<640*480; i++)
				{
					if ((cam[0]->buf) == NULL) { continue;}
					char buf0 = i[((char*)(cam[2]->buf))];	
					char buf1 = i[((char*)(cam[1]->buf))];
					char buf2 = i[((char*)(cam[0]->buf))];
					switch(camOrder)
					{
						case 12:
							buf0 = i[((char*)(cam[0]->buf))];
							buf1 = i[((char*)(cam[1]->buf))];
							buf2 = i[((char*)(cam[2]->buf))];
							break;
						case 21:
							buf0 = i[((char*)(cam[0]->buf))];
							buf1 = i[((char*)(cam[2]->buf))];
							buf2 = i[((char*)(cam[1]->buf))];
							break;
						case 120:
							buf0 = i[((char*)(cam[1]->buf))];
							buf1 = i[((char*)(cam[2]->buf))];
							buf2 = i[((char*)(cam[0]->buf))];
							break;
						case 210:
							buf0 = i[((char*)(cam[2]->buf))];
							buf1 = i[((char*)(cam[1]->buf))];
							buf2 = i[((char*)(cam[0]->buf))];
							break;
						case 102:
							buf0 = i[((char*)(cam[1]->buf))];
							buf1 = i[((char*)(cam[0]->buf))];
							buf2 = i[((char*)(cam[2]->buf))];
							break;
						case 201:
							buf0 = i[((char*)(cam[2]->buf))];
							buf1 = i[((char*)(cam[0]->buf))];
							buf2 = i[((char*)(cam[1]->buf))];
							break;
						default:
							buf0 = i[((char*)(cam[0]->buf))];
							buf1 = i[((char*)(cam[1]->buf))];
							buf2 = i[((char*)(cam[2]->buf))];
							break;
					}

					((uchar*)(img->imageData + img->widthStep*row))[col] = buf0>255?255:buf0;
					((uchar*)(img->imageData + img->widthStep*row))[col+640] = buf1>255?255:buf1;
					((uchar*)(img->imageData + img->widthStep*row))[col+2*640] = buf2>255?255:buf2;

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

		double	timestamp = cam[0]->curtimestamp;
		
		LeaveCriticalSection(&cam[0]->camgrab_cs);
		if (camtype == CAM_FIREFLY)
		{
			LeaveCriticalSection(&cam[1]->camgrab_cs);
			LeaveCriticalSection(&cam[2]->camgrab_cs);
		}


		if (WaitForSingleObject(ghMutex,5000)!=WAIT_OBJECT_0)
		{
			printf("Warning: Did not receive global map handle in 5 seconds...");
			continue;
		}

		//dst src size
		if (upsidedown)
			cvFlip (img,img,-1);

		CopyMemory(pBuf, img->imageData, WIDTH*HEIGHT*numChannels);		
		CopyMemory((char*)pBuf+(WIDTH*HEIGHT*numChannels), &timestamp,sizeof(double));
		ReleaseMutex(ghMutex);

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
		}
		
		frameNum++;	

		if(DISPLAY_ON && display_image)
		{
			/*if(frameNum%3!=0) continue; */
		
			cvResize(img,imgrz);
			cvShowImage("CameraServer. Press q to quit",imgrz );
		}
		
	}

	// exit this program
	UnmapViewOfFile(pBuf);
	CloseHandle(hMapFile);
	CloseHandle(close_event);
	delete cam[0];
	if (camtype == CAM_FIREFLY)
	{
		delete cam[1];
		delete cam[2];
	}
	Sleep(1000);
}