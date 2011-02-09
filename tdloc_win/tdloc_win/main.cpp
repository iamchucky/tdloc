//amn32 - October 19, 2009
//Provides a memory mapped file interface for the 1394 camera. 
//Accurately synchronizes the camera with
//the pulse coming from the timing system.
#define DISPLAY_ON 1
#define NUM_CAM 0
#define FUDGE_FACTOR 0.97
#define SHARE_MEM_PROC 0

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

enum OperationMode
{
	opmode_CALIB, opmode_NORMAL, opmode_IDLE
};

struct cam_offset
{
	float xoffset;
	float yoffset;
	float yawoffset;
};

struct bcast_msg
{
	char tag_id;
	char pose_x[4];
	char pose_y[4];
	char pose_yaw[4];
	char timestamp[8];
};

typedef struct bcast_msg * bcast_msg_t;

volatile bool running=true;
void* pBuf;
int WIDTH;
int HEIGHT;
int numCam = 1;
int camOrder=12;
int subx = 0;
int suby = 0;
bool upsidedown=false;
CameraType camtype=CAM_UNIBRAIN;

int numChannels=3;
unsigned int frameNum=0;		
double lasttime=0;
unsigned int framecount = 0;
char filename[50];
struct cam_offset coff[50];
OperationMode opmode = opmode_IDLE;
bool calib_capture = false;
char viewWindowName[] = "CameraServer. Press Q to quit. Press C to calibrate. Press V to start broadcasting, B to toggle broadcasting. Press I to go into idle.";
bool broadcast_this = true;
bool showsub = false;
udp_connection* udp_msgTX;
std::vector<bcast_msg> my_msg;
char transmit_msg[1024];

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
			printf("Using Unibrain Fire-i");
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
	{
		//printf("Warning: No Command Line Arguments Defined. Using Defaults.\n");
	}

	SyncCamParams s;
	if (camtype == CAM_UNIBRAIN)
	{
		cout<<"Using Unibrain Fire-i"<<endl;
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
		//initialize camera offsets here
		coff[n].xoffset = 0.f;
		coff[n].yoffset = 0.f;
		coff[n].yawoffset = 0.f;
	}
	struct cam_offset ooffset;	// origin offset
	ooffset.xoffset = 0.f;
	ooffset.yoffset = 0.f;
	ooffset.yawoffset = 0.f;
	
	//finally, create a new camera
	Sync1394Camera* cam[50];			// support up to 50 cam pointers
	for (int n = 0; n < 50; ++n)
	{
		cam[n] = new Sync1394Camera ();
		if (n == 0)
		{
			if (NUM_CAM == 0)
			{
				numCam = cam[n]->GetNumberOfCameras();
			}
			else
			{
				numCam = NUM_CAM;
			}
			
			printf("Initializing %d camera(s)....\n\n", numCam);
		}
		if (numCam == 0)	break;

		cam[n]->InitCamera(n, s, coff[n].xoffset, coff[n].yoffset, coff[n].yawoffset, FUDGE_FACTOR);

		if (n == numCam-1)	break;
	}
	Sync1394Camera::allCamInit = true;
	Sleep(10);

	//Initialize UDP transmit port
	udp_params paramsTX  =  udp_params(8866, UDP_BROADCAST_IP, UDP_BROADCAST_PORT); 
	try
	{		
		paramsTX.no_listen = true;
		udp_msgTX = new udp_connection(paramsTX);  
	}
	catch (exception)
	{
		printf("Couldn't init UDP TX on port %d\n",paramsTX.local_port);
	}

	printf("Press spacebar with view window selected to expand it\n");

	IplImage* img;
	IplImage* imgrz;
	IplImage* imgfull;
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
		imgfull = cvCreateImage (cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,3);
	}
	else
	{
		img = cvCreateImage (cvSize(camWidth,camHeight),IPL_DEPTH_8U,1);
		imgrz = cvCreateImage (cvSize(camWidth/2,camHeight/2),IPL_DEPTH_8U,1);
	}		

	if (SHARE_MEM_PROC)
	{
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
	}

	running=true;
	
	cvNamedWindow (viewWindowName);
	cvResizeWindow(viewWindowName,200,0);
	
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

			if (SHARE_MEM_PROC)
			{
				WaitForSingleObject(ghMutex,5000); //steal the mutex to prevent further processing in host apps
			}
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

		//dst src size
		if (upsidedown)
			cvFlip (img,img,-1);

		if (SHARE_MEM_PROC)
		{
			if (WaitForSingleObject(ghMutex,5000)!=WAIT_OBJECT_0)
			{
				printf("Warning: Did not receive global map handle in 5 seconds...");
				continue;
			}
			CopyMemory(pBuf, img->imageData, camWidth*camHeight*numChannels);		
			CopyMemory((char*)pBuf+(camWidth*camHeight*numChannels), &timestamp,sizeof(double));
			ReleaseMutex(ghMutex);
		}

		switch(opmode)
		{
			case opmode_CALIB:
				// Calibration function starts here *************************************************
				if (calib_capture && numCam > 1)
				{
					int thereiszero = -1;
					ClearScreen();
					for (int n = 0; n < numCam; ++n)
					{
						EnterCriticalSection(&cam[n]->camgrab_cs);
					}
					for (int n = 1; n < numCam; ++n)
					{
						struct cam_offset myoffset;
						myoffset.xoffset = 0.f;
						myoffset.yoffset = 0.f;
						myoffset.yawoffset = 0.f;

						int tagcount = 0;
						int origin = n-1;
						if (n%4 == 0)
						{
							origin = n - 4;
						}
						int cam1tagSize = cam[n]->artagLoc->getARtagSize();
						int cam0tagSize = cam[origin]->artagLoc->getARtagSize();
						if (cam1tagSize > 0 && cam0tagSize > 0)
						{
							for (int i = 0; i < cam1tagSize; ++i)
							{
								for (int j = 0; j < cam0tagSize; ++j)
								{
									cv::Mat pose0 = cam[origin]->artagLoc->getARtag(j)->getPose();
									cv::Mat pose1 = cam[n]->artagLoc->getARtag(i)->getPose();
									float x0 = pose0.at<float>(0,3)/1000.0*FUDGE_FACTOR;
									float x1 = pose1.at<float>(0,3)/1000.0*FUDGE_FACTOR;
									
									float y0 = pose0.at<float>(1,3)/1000.0*FUDGE_FACTOR;
									float y1 = pose1.at<float>(1,3)/1000.0*FUDGE_FACTOR;

									// find match ARtag ID
									if (cam[origin]->artagLoc->getARtag(j)->getId() == cam[n]->artagLoc->getARtag(i)->getId())
									{
										myoffset.xoffset = myoffset.xoffset + (x0 - x1);
										myoffset.yoffset = myoffset.yoffset + (y0 - y1);
										tagcount++;
									}
									if (cam[origin]->artagLoc->getARtag(j)->getId() == 0)
									{
										ooffset.xoffset = x0;
										ooffset.yoffset = y0;
										thereiszero = origin;
									}
									else if (cam[n]->artagLoc->getARtag(i)->getId() == 0)
									{
										ooffset.xoffset = x1;
										ooffset.yoffset = y1;
										thereiszero = n;
									}
								}
							}
							if (tagcount != 0)
							{
								myoffset.xoffset /= tagcount;				// Averaging offsets from matched ARtags
								myoffset.yoffset /= tagcount;
								myoffset.yawoffset /= tagcount;
								myoffset.xoffset += coff[origin].xoffset;	// Accumulate offsets from previous cameras
								myoffset.yoffset += coff[origin].yoffset;
								
								coff[n].xoffset = myoffset.xoffset;
								coff[n].yoffset = myoffset.yoffset;
							}
							else	// not all camera got more than 0 matched ARtag
							{
								calib_capture = false;
								printf("Trying again...\n");
								break;
							}
						}
						else	// not all camera got more than 0 matched ARtag
						{
							calib_capture = false;
							printf("Trying again...\n");
							break;
						}
					}
					for (int n = 0; n < numCam; ++n)
					{
						LeaveCriticalSection(&cam[n]->camgrab_cs);
					}

					// calibrated successfully, go into idle mode to wait for user to verify 
					// and press 'v' to change to normal mode
					if (calib_capture)	
					{
						if (thereiszero != -1)
						{
							ooffset.xoffset += coff[thereiszero].xoffset;	
							ooffset.yoffset += coff[thereiszero].yoffset;
						}
						else
						{
							ooffset.xoffset = 0.f;
							ooffset.yoffset = 0.f;
						}
						for (int n = 0; n < numCam; ++n)
						{
							cam[n]->artagLoc->setARtagOffset(coff[n].xoffset-ooffset.xoffset, coff[n].yoffset-ooffset.yoffset, 0.f);
							printf("CAM%d\t xoffset: %.2f\t yoffset: %.2f\t yawoffset: %.2f\n", n, coff[n].xoffset-ooffset.xoffset, -(coff[n].yoffset-ooffset.yoffset), 0.f);
						}
						opmode = opmode_IDLE;
						calib_capture = false;
					}
					else
					{
						calib_capture = true;
					}
				}
				// Calibration function ends here *************************************************
				break;
			case opmode_NORMAL:
				ClearScreen();
				my_msg.clear();
				EnterCriticalSection(&ARtagLocalizer::tags_mutex);
				for (int n = 0; n < 50; ++n)
				{
					ARtag * ar = ARtagLocalizer::tags[n];
					if (ar->getId() == n)
					{
						cv::Mat pose = ar->getPose();
						int camId = ar->getCamId();
						float x = pose.at<float>(0,3)/1000.0*FUDGE_FACTOR + coff[camId].xoffset - ooffset.xoffset;
						float y = -(pose.at<float>(1,3)/1000.0*FUDGE_FACTOR + coff[camId].yoffset - ooffset.yoffset);
						float z = pose.at<float>(2,3)/1000.0;
						float yaw = -atan2(pose.at<float>(1,0), pose.at<float>(0,0));
						if (yaw < 0)
						{
							yaw += 6.28;
						}
						printf("ARtag ID: %d\n", ar->getId());
						printf("x: %.2f \t y: %.2f \t z: %.2f \t yaw: %.2f \t time: %.4f\n", x,y,z,yaw + coff[camId].yawoffset - ooffset.yawoffset, cam[camId]->curtimestamp);
						printf("\n");

						if (broadcast_this)
						{
							struct bcast_msg bmsg;
							bmsg.tag_id = (char)ar->getId();
							memcpy(bmsg.pose_x, &x, sizeof(float));
							memcpy(bmsg.pose_y, &y, sizeof(float));
							memcpy(bmsg.pose_yaw, &yaw, sizeof(float));
							memcpy(bmsg.timestamp, &cam[camId]->curtimestamp, sizeof(double));
							my_msg.push_back(bmsg);
						}

						ar->setId(-1);
					}
				}
				LeaveCriticalSection(&ARtagLocalizer::tags_mutex);
				//broadcast msg here
				if (broadcast_this)
				{
					if (my_msg.size() > 0)
					{
						transmit_msg[0] = (char)my_msg.size();
						for (int i = 0; i < my_msg.size(); ++i)
						{
							memcpy(transmit_msg+1 + sizeof(struct bcast_msg)*i, &my_msg[i], sizeof(struct bcast_msg));
						}
						udp_msgTX->send_message (transmit_msg, sizeof(struct bcast_msg)*((int)transmit_msg[0])+1, UDP_BROADCAST_IP, UDP_BROADCAST_PORT);
					}
					printf("Total of %d ARtags broadcasted.\n", my_msg.size());
				}				
				break;
			case opmode_IDLE:
				break;
		}

		switch(cvWaitKey (10))
		{
			case '0':
				if (numCam > 0)
				{
					subx = 0;
					suby = 0;
					showsub = true;
				}
				break;
			case '1':
				if (numCam > 1)
				{
					subx = 640;
					suby = 0;
					showsub = true;
				}
				break;
			case '2':
				if (numCam > 2)
				{
					subx = 1280;
					suby = 0;
					showsub = true;
				}
				break;
			case '3':
				if (numCam > 3)
				{
					subx = 1920;
					suby = 0;
					showsub = true;
				}
				break;
			case '4':
				if (numCam > 4)
				{
					subx = 0;
					suby = 480;
					showsub = true;
				}
				break;
			case '5':
				if (numCam > 5)
				{
					subx = 640;
					suby = 480;
					showsub = true;
				}
				break;
			case '6':
				if (numCam > 6)
				{
					subx = 1280;
					suby = 480;
					showsub = true;
				}
				break;
			case '7':
				if (numCam > 7)
				{
					subx = 1920;
					suby = 480;
					showsub = true;
				}
				break;
			case 'q':
				running=false;
				Sync1394Camera::allStop = true;
				ARtagLocalizer::allStop = true;
				break;
			case 's':
				sprintf(filename, "image%d.jpg", framecount);
				cvSaveImage (filename,img);
				printf("Current image saved to %s\n", filename);
				framecount++;
				break;
			case 'c':
				opmode = opmode_CALIB;
				calib_capture = true;
				break;
			case 'v':
				opmode = opmode_NORMAL;
				break;
			case 'b':
				broadcast_this = !broadcast_this;
				break;
			case 'i':
				opmode = opmode_IDLE;
				break;
			case 0x20:	//space-bar
				if(!display_image){
					display_image = !display_image;
					cvResizeWindow(viewWindowName,imgrz->width, imgrz->height);
					
				}
				else{
					display_image = !display_image;
					cvResizeWindow(viewWindowName,200, 0);
					
				}				
				break;
		}
		
		frameNum++;	

		if(DISPLAY_ON && display_image)
		{
			/*if(frameNum%3!=0) continue; */
			if (showsub)
			{
				cvSetImageROI(img,cvRect(subx,suby,WIDTH,HEIGHT));
				cvCopy(img,imgfull);
				cvResetImageROI(img);
				cvShowImage("camera subimage",imgfull);
			}
			cvResize(img,imgrz);
			cvShowImage(viewWindowName,imgrz );
		}
		
	}

	// exit this program
	if (SHARE_MEM_PROC)
	{
		UnmapViewOfFile(pBuf);
		CloseHandle(hMapFile);
	}
	CloseHandle(close_event);
	for (int n = 0; n < numCam; ++n)
	{
		delete cam[n];
	}
	
	Sleep(1000);
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
