//amn32 - October 19, 2009
//Provides a memory mapped file interface for the 1394 camera. 
//Accurately synchronizes the camera with
//the pulse coming from the timing system.
#define DISPLAY_ON 1
#define NUM_CAM 4
#define ORIGINX 0.34
#define ORIGINY -0.05

#ifndef _WIN32_WINNT		// Allow use of features specific to Windows XP or later.                   
#define _WIN32_WINNT 0x0501	// Change this to the appropriate value to target other versions of Windows.
#endif			

#include <stdio.h>
#include <tchar.h>
#include <string>
#include <sstream>
//#include <windows.h>

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

struct cam_offset
{
	float xoffset;
	float yoffset;
};

volatile bool running=true;
void* pBuf;
int WIDTH;
int HEIGHT;
int numCam = 1;
int camOrder=12;
bool upsidedown=false;
CameraType camtype=CAM_UNIBRAIN;

int numChannels=3;
unsigned int frameNum=0;		
double lasttime=0;
unsigned int framecount = 0;
char filename[50];
struct cam_offset coff[50];

void calibCam();
void ClearScreen();

//captures the closing of the console window and closes the app appropriately
BOOL WINAPI handler_routine(DWORD dwCtrlType) 
{
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
		s.videoFrameRate = 2;
		s.BitDepth16 = false;
		s.usePartialScan = false; 
		s.syncEnabled = false;
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

	for (int n = 0; n < 50; ++n)
	{
		/*coff[n].xoffset = 1.20*(n%4) + ORIGINX;
		coff[n].yoffset = 1.60*(floor(n/4.f)) + ORIGINY;*/
		coff[n].xoffset = 0.f;
		coff[n].yoffset = 0.f;
	}
	
	//finally, create a new camera
	Sync1394Camera* cam[50];			// support up to 50 cam pointers
	for (int n = 0; n < 50; ++n)
	{
		cam[n] = new Sync1394Camera ();
		if (n == 0)
		{
			//numCam = cam[n]->GetNumberOfCameras();
			numCam = 1;
			printf("Initializing %d camera(s)....\n\n", numCam);
		}
		if (numCam == 0)	break;

		cam[n]->InitCamera(n, s, coff[n].xoffset, coff[n].yoffset);

		if (n == numCam-1)	break;
	}
	Sync1394Camera::allCamInit = true;
	Sleep(10);

	IplImage* img;
	IplImage* imgrz;
	int camWidth = WIDTH*numCam;
	int camHeight = HEIGHT*((int)ceil(numCam/4.f));
	if (numCam >= 4)
	{
		camWidth = WIDTH*4;
	}
	if (s.isColor)
	{
		img = cvCreateImage (cvSize(camWidth,camHeight),IPL_DEPTH_8U,3);
		imgrz = cvCreateImage (cvSize(camWidth*2/3,camHeight*2/3),IPL_DEPTH_8U,3);
	}
	else
	{
		img = cvCreateImage (cvSize(camWidth,camHeight),IPL_DEPTH_8U,1);
		imgrz = cvCreateImage (cvSize(camWidth/2,camHeight/2),IPL_DEPTH_8U,1);
	}		

	hMapFile = CreateFileMappingA(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security 
		PAGE_READWRITE,          // read/write access
		0,                       // max. object size 
		(camWidth*camHeight*numChannels) + sizeof(double), // buffer size  
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
	
	cvNamedWindow ("CameraServer. Press q to quit");
	cvResizeWindow("CameraServer. Press q to quit",200,0);//*/
	
	bool display_image=0;

	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	//---------------------MAIN LOOP---------------------------------
	while(running)
	{	
		bool ready = true;
		//wait for an image
		for (int n = 0; n < numCam; ++n)
		{
			if (WaitForSingleObject (cam[n]->cameraEvent,2000) != WAIT_OBJECT_0)
			{
				ready = false;
			}
		}

		if (!ready)
		{
			continue;
		}

		if ((cam[0]->buf) == NULL)
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
		for (int n = 0; n < numCam; ++n)
		{
			EnterCriticalSection(&cam[n]->camgrab_cs);
		}
		
		if (s.isColor)
		{
			// combine all 3 camera images
			int col = 0;
			int row = 0;
			
			for (int i=0; i<WIDTH*HEIGHT*3; i+=3)
			{
				for (int k = 0; k < 3; k++)
				{
					if ((cam[0]->buf) == NULL) { continue;}
		
					for (int n = 0; n < numCam; ++n)
					{
						char buf = i[((char*)(cam[n]->buf)+k)];	
						((uchar*)(img->imageData + img->widthStep*(row+HEIGHT*(n/4))+2-k))[(col+(n%4)*WIDTH)*img->nChannels] = buf>255?255:buf;
					}
				}

				if (col < WIDTH-1)
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
		
		double	timestamp = cam[0]->curtimestamp;
		
		for (int n = 0; n < numCam; ++n)
		{
			LeaveCriticalSection(&cam[n]->camgrab_cs);
		}

		if (WaitForSingleObject(ghMutex,5000)!=WAIT_OBJECT_0)
		{
			printf("Warning: Did not receive global map handle in 5 seconds...");
			continue;
		}

		//dst src size
		if (upsidedown)
			cvFlip (img,img,-1);

		CopyMemory(pBuf, img->imageData, camWidth*camHeight*numChannels);		
		CopyMemory((char*)pBuf+(camWidth*camHeight*numChannels), &timestamp,sizeof(double));
		ReleaseMutex(ghMutex);

		ClearScreen();
		EnterCriticalSection(&ARtagLocalizer::tags_mutex);
		for (int n = 0; n < 50; ++n)
		{
			ARtag * ar = ARtagLocalizer::tags[n];
			if (ar->getId() == n)
			{
				cv::Mat pose = ar->getPose();
				int camId = ar->getCamId();
				float x = pose.at<float>(0,3)/1000.0*0.96 + coff[camId].xoffset;
				float y = pose.at<float>(1,3)/1000.0*0.96 + coff[camId].yoffset;
				float z = pose.at<float>(2,3)/1000.0;
				float yaw = atan2(pose.at<float>(1,0), pose.at<float>(0,0));
				printf("Id: %d\n", ar->getId());
				printf("x: %.2f \t y: %.2f \t z: %.2f \t yaw: %.2f\n", x,y,z,yaw);
				printf("\n");
				ar->setId(-1);
			}
		}
		LeaveCriticalSection(&ARtagLocalizer::tags_mutex);

		switch(cvWaitKey (10)){
			case 'q':
				running=false;
				Sync1394Camera::allStop = true;
				ARtagLocalizer::allStop = true;
				break;
			case 'c':
				sprintf(filename, "image%d.jpg", framecount);
				cvSaveImage (filename,img);
				printf("Current image saved to %s\n", filename);
				framecount++;
				break;
			case 0x20:	//space-bar
				if(!display_image){
					display_image = !display_image;
					cvResizeWindow("CameraServer. Press q to quit",imgrz->width, imgrz->height);
					
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
	for (int n = 0; n < numCam; ++n)
	{
		delete cam[n];
	}
	
	Sleep(1000);
}

void calibCam()
{

}

void ClearScreen()
{
	HANDLE                     hStdOut;
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	DWORD                      count;
	DWORD                      cellCount;
	COORD                      homeCoords = { 0, 0 };

	hStdOut = GetStdHandle( STD_OUTPUT_HANDLE );
	if (hStdOut == INVALID_HANDLE_VALUE) return;

	/* Get the number of cells in the current buffer */
	if (!GetConsoleScreenBufferInfo( hStdOut, &csbi )) return;
	cellCount = csbi.dwSize.X *csbi.dwSize.Y;

	/* Fill the entire buffer with spaces */
	if (!FillConsoleOutputCharacter(
	hStdOut,
	(TCHAR) ' ',
	cellCount,
	homeCoords,
	&count
	)) return;

	/* Fill the entire buffer with the current colors and attributes */
	if (!FillConsoleOutputAttribute(
	hStdOut,
	csbi.wAttributes,
	cellCount,
	homeCoords,
	&count
	)) return;

	/* Move the cursor home */
	SetConsoleCursorPosition( hStdOut, homeCoords );
}
