#ifndef _SYNCCAM1394_H
#define _SYNCCAM1394_H

#define WIN32_LEAN_AND_MEAN 
#include <windows.h>
#include <iostream>
#include <memory.h>
#include <queue>
#include <math.h>
#ifdef _DEBUG
#pragma comment(lib,"1394camerad.lib")
#else
#pragma comment(lib,"1394camera.lib")
#endif
#include <1394Camera.h>
#include <vector>
#include "..\network\udp_connection.h"

#include "..\camera\AutoWhiteBal.h"

#include "opencv\cv.h"
#include "opencv\highgui.h"

#define ENABLE_WHITEBALANCE 1

#define CAMERA_MSG_REGISTER 0x00 //Set register listener
#define CAMERA_MSG_SETFPS   0x01 //Set frame rate
#define CAMERA_MSG_SETDUTY  0x02 
#define CAMERA_START        0x04 //resets the counter, initializes shit
#define CAMERA_STOP         0x05 //resets the counter, initializes shit

#define UDP_CONTROL_PORT 20
#define UDP_CONTROL_IP "192.168.1.8"

#define UDP_BROADCAST_IP "232.132.1.8"
#define UDP_BROADCAST_PORT 30008

#define AUTOGAIN_USE_MEDIAN		1
#define AUTOGAIN_MEDIAN_IDEAL	120
#define AUTOGAIN_MAX_IDEAL		50000
#define AUTOGAIN_KP				0.02
#define AUTOGAIN_MAX8BIT		250
#define AUTOGAIN_MAX16BIT		1000
#define MAX_FPS_LIMITED_SHUTTER 900
#define USESHUTTERGAIN
#define AUTOSHUTTER_KP			0.1
#define AUTOBRIGHT_KP			0.005
#define TOTAL_KP				1.5
#define GAINVAL				17
#define SHUTTERVAL			200
#define WHITEBALANCE_VALUE1	510
#define WHITEBALANCE_VALUE2	800

#define UNDIST true		// Setting to undistort the camera to account for lens distortion.

using namespace std;
//Encapsulates the UDP connection class for syncronized images delivered
//from a 1394 camera, using the CMU cam driver

#define GAIN_MAX 100
#define GAIN_MIN 0
#define WB_MIN 0
#define WB_MAX 800
#define BRIGHT_MIN 0
#define BRIGHT_MAX 100
#define SHUTTER_MIN 0
#define SHUTTER_MAX 100

struct SyncCamParams
{
	int		width; 
	int		height;
	bool	isColor;
	int		videoMode;				// 3 for 640x480 YUV 422
	int		videoFormat;			// 0 for 640x480 YUV 422  
	int		videoFrameRate;			// 4 for normal stuff ,3 for unibrain
	bool	isSlave;

	bool	usePartialScan;	  
	unsigned short bytesPerPacket;	//needed on the a622f, set to < 4000, > 3500
	bool	BitDepth16;							
	bool	AGC;					//enabled auto gain control (software)
	int		AGCtop, AGCbot;			//the top and bottom of the frame used for agc control
	bool	syncEnabled;  			//set to -1 for no use!
	int		syncID;					//this is the ID on the MCU that provides the sync signal, NOT the firewire ID
	int		syncFPS;				//desired FPS from the sync mcu
	bool	syncKillOnClose;		//turns off timing when this app closes...
	bool	eTrigEnabled;
	bool	AutoShutter;
	bool	AutoGain;
	bool	AutoBrightness;
	bool	AutoWB;
	bool	gammaEnable;
	bool	adjWB;
	bool	ghettoSync;				//dont ever use this please
	unsigned short syncCamInput;	//the physical port on the camera used for sync
	int		partialHeight;
	int		partialWidth;
	int		partialTop;
	int		partialLeft;
	
	SyncCamParams()
	{
		width = 640; 
		height = 480;
		isColor = true;
		videoMode = 3;				// 3 for 640x480 YUV 422
		videoFormat = 0;			// 0 for 640x480 YUV 422 
		videoFrameRate = 4;			// 4 for baslers, 3 for unibrain
		usePartialScan = false;					 
		bytesPerPacket = 0;			//needed on the a622f, set to < 4000, > 3500
		BitDepth16 = false;							
		AGC = true;					//enabled auto gain control (software)
		AGCtop = 640;
		AGCbot = 480;				//the top and bottom of the frame used for agc control
		syncEnabled = false;		//set to -1 for no use!
		syncID = 0;					//this is the ID on the MCU that provides the sync signal, NOT the firewire ID
		syncFPS = 30;				//desired FPS from the sync mcu
		syncKillOnClose = false;	//turns off timing when this app closes...
		eTrigEnabled = false;
		AutoShutter = true;
		AutoGain = true;
		AutoBrightness = true;
		AutoWB = true;
		gammaEnable = false;
		adjWB = false;
		ghettoSync = false;
		syncCamInput = 0;			//the physical port on the camera used for sync
		partialHeight = height;
		partialWidth = width;
		partialLeft = 0;
		partialTop = 0;
	}
};

#pragma pack (1)
struct SyncCamPacket
{
	unsigned short seconds;
	unsigned long	 ticks;
	unsigned long	 seqNum;
	unsigned char  id;
};
#pragma pack ()

DWORD WINAPI CamThreadWrap(LPVOID t);

class Sync1394Camera
{
public:
	Sync1394Camera();
	~Sync1394Camera();
	bool InitCamera (int cameraID, SyncCamParams config);
	int GetNumberOfCameras ();
	int SetBright(C1394Camera* camptr,int val);
	int SetGain(C1394Camera* camptr,int val);
	int SetShutter (C1394Camera* camptr,int val);
	unsigned short GetBright(C1394Camera* camptr);
	unsigned short GetGain(C1394Camera* camptr);
	unsigned short GetShutter (C1394Camera* camptr);
	void GetShutterMinMax(C1394Camera* camptr, unsigned short*, unsigned short*);
	void GetGainMinMax(C1394Camera* camptr, unsigned short*, unsigned short*);
	void DoAutoShutter(C1394Camera* camptr, unsigned char* buf, unsigned short targetGain);
	int  SetAutoWhiteBal(C1394Camera* camptr);
	
	int GetWhiteBal(C1394Camera* camptr, unsigned short* val0, unsigned short* val1);
	int SetWhiteBal(C1394Camera* camptr, unsigned short val0, unsigned short val1);
	int GetWhiteBal(int camId, int *val0, int *val1);
	int SetWhiteBal(int camId, int val0, int val1);

	int DoAutoWhiteBal(C1394Camera* camptr, unsigned char* buf);
	int DoAutoWhiteBalance(C1394Camera* camPtr, IplImage *im, unsigned short *wr, unsigned short *wb);

	DWORD CamThread();
	HANDLE cameraEvent;
	unsigned char* buf;
	double curtimestamp;
	int lastMedian;
	int lastMaxAcc;
	float AGCerror;
	float AGCeffort;
	float kp;
	int idealMedian;
	CRITICAL_SECTION camgrab_cs;
	
private:
	static int shortComp (const void* a, const void* b);
	static int charComp (const void* a, const void* b);
	static C1394Camera camera;

	//CAutoWhiteBal m_wbal[3];

	int camId;
	bool isRunning;	
	HANDLE cameraHandle;
	SyncCamParams config;
	int GetNumMaxedPixelsInBuf (unsigned char* buf, int top, int bottom);
	int GetMedianPixelValue(unsigned char* buf, int top, int bottom);
	int size;
	udp_connection* udpRX;
	udp_connection* udpTX;
	void UDPCallback(udp_message& msg, udp_connection* conn, void* arg);
	unsigned short maxGain;
	unsigned short minGain;
	unsigned short maxShutter;
	unsigned short minShutter;
	int curSeqNumber;
	int expSeqNumber;

	//camera_adjust_param_t camSettings[3];	//settings for shutter, white balance, gain

};

#endif
