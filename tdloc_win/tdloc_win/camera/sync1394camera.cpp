#include "sync1394camera.h"

C1394Camera Sync1394Camera::camera;

DWORD WINAPI CamThreadWrap(LPVOID t)
{
	return ((Sync1394Camera*)t)->CamThread();
}

int bullshit= 0;
DWORD Sync1394Camera::CamThread ()
{
	//init white balance************************************************
	bool iswbconverged=0;
	int wbFreq = 10;
	IplImage* mapx;
	IplImage* mapy;

	if (UNDIST)
	{
		// Build the undistort map which we will use for all 
		// subsequent frames.
		//
		mapx = cvCreateImage( cvSize(config.width,config.height), IPL_DEPTH_32F, 1 );
		mapy = cvCreateImage( cvSize(config.width,config.height), IPL_DEPTH_32F, 1 );
		mapx = (IplImage*)cvLoad("..\\Unibrain\\mapx.xml");
		mapy = (IplImage*)cvLoad("..\\Unibrain\\mapy.xml");
	}

	CAutoWhiteBal m_wbal;
	unsigned short r_gain,b_gain;
	unsigned short r_gain_old,b_gain_old;
	IplImage *wbim = cvCreateImage(cvSize(config.width,config.height),8,1);	
	IplImage *wbrgb = cvCreateImage(cvSize(config.width,config.height),8,3);
	IplImage *undist_src = cvCreateImage(cvSize(config.width,config.height),8,3);	
	IplImage *undist_dst = cvCreateImage(cvSize(config.width,config.height),8,3);
	m_wbal.InitInstance(cvSize(config.width,config.height));	
	//******************************************************************
	printf("Cam Thread Started\n\n");
	while(isRunning) 
	{	
		// camera 0
		C1394Camera* camptr = &camera;
		bullshit++;
		unsigned long dlength=0;			
		//float pgain = .01f;
		int dFrames=0;

		if (CAM_SUCCESS != camptr->AcquireImageEx(TRUE,&dFrames))
		{
			if (bullshit %100==0)
				printf("COULD NOT AQUIRE AN IMAGE FROM THE CAMERA 0.\n");			
			//this->buf = NULL;			
			//while(1);
		}
		if (dFrames>0) printf ("DROPPED %d FRAMES! 0\n",dFrames);

		EnterCriticalSection(&camgrab_cs);		
		if (config.isColor)
		{
			camptr->getRGB(buf,config.height * config.width * 3); dlength=1;
		}
		else
		{
			camptr = &camera;
			buf = camptr->GetRawData (&dlength);
		}
		LeaveCriticalSection(&camgrab_cs);
		if (0 == dlength) 
		{			
			continue;
		}

		if ((bullshit %100==0) && (!config.AGC))
		{
			printf(".");
		}
		if (UNDIST)
		{
			memcpy(undist_src->imageData,buf,config.width*config.height*3);
			cvRemap( undist_src, undist_dst, mapx, mapy );
			memcpy(buf,undist_dst->imageData,config.width*config.height*3);
		}
		SetEvent (cameraEvent);	

		if ((config.AGC) && (config.isSlave == false))
		{
			if (bullshit%3 == 0)
			{

#if AUTOGAIN_USE_MEDIAN
				int median = GetMedianPixelValue(buf,config.AGCtop,config.AGCbot);

				//too bright, positive error, 
				//so pos error means decrease gain
				float error = ((float)median - (float)idealMedian); 
				float effort = 1;
				error = error*TOTAL_KP;

				effort = (abs(error * kp));
				if (error<0) effort*=-1;
				AGCerror = error;
				AGCeffort = effort;	

				int newgain = (int)GetGain(camptr) - (int)effort;
				if ((int)GetGain(camptr) != newgain)
					SetGain (camptr,newgain);

#ifdef USESHUTTERGAIN
				float seffort = AUTOSHUTTER_KP;
				int newShutter = (int) GetShutter (camptr)  - (int)(error*seffort);
				if ((int)GetShutter(camptr) != newShutter)
					SetShutter (camptr,newShutter);		


#endif

				double beffort = AUTOBRIGHT_KP;
				int newbright = (int)GetBright(camptr) - (int) (error*beffort);
				/*if ((int)GetBright(camptr) != newbright)
					SetBright (camptr,newbright);*/
				if (bullshit %100==0)
				{
					printf("CAM0 AGC: Median: %d Brightness: %d Gain: %d Shutter: %d Error: %f\n",median,newbright,newgain,newShutter,error);
				}
			}
				/*camSettings[0].getMedian=median;
				camSettings[0].getBrightness = newbright;
				camSettings[0].getGain = newgain;
				camSettings[0].getShutter=newShutter;
				camSettings[0].getError=error;//*/
		
			if(config.AutoWB)
			{
				float thres=1;
				if(iswbconverged )
				{
					wbFreq=10;
				}
				else
				{
					wbFreq=1;
				}
				if(bullshit%wbFreq==0)
				{
					//cvShowImage("teest",wbim);cvWaitKey(10);
					memcpy(wbim->imageData,buf,config.width*config.height);
					cvCvtColor(wbim,wbrgb,CV_BayerBG2BGR);
					GetWhiteBal(camptr, &r_gain, &b_gain);
					r_gain_old=r_gain;
					b_gain_old=b_gain;
					m_wbal.RunAWB(wbrgb, &r_gain, &b_gain);
					SetWhiteBal(camptr, r_gain, b_gain);
					if(abs(r_gain-r_gain_old)<thres && abs(b_gain-b_gain_old)<thres)
					{	//check for converge1
						iswbconverged=1;
					}
					else
					{
						iswbconverged=0;
					}
				}
			}
			

			
#else
			int maxedOutPixels = GetNumMaxedPixelsInBuf(config.AGCtop,config.AGCbot);
			if (maxedOutPixels != -1)
			{
				//too many pixels, + error
				int error = (maxedOutPixels - AUTOGAIN_MAX_IDEAL); 
				if ((error>0) && (GetGain()>0))	 //too bright
					SetGain (GetGain() - 1);
				else if (error<0)
					SetGain (GetGain() + 1);
			}
#endif
		}

		/*if (config.syncEnabled) //are we using timing?
		{
			if ((expSeqNumber != 0) && (curSeqNumber != expSeqNumber))
			{
				//interpolate the correct timestamp
				float expInterval = 1.0f/(float)config.syncFPS;				
				//printf("WARNING: Sequence number mismatch: Got: %d Expected %d \nCurTime: %f Interval: %f ProjTime: %f \n",curSeqNumber, expSeqNumber,curtimestamp,expInterval,expInterval + curtimestamp);
				curtimestamp += expInterval;		
			}
			expSeqNumber = curSeqNumber + 1;

		}	*/
		

		//output data
//		camSettings[0].getWhiteBal = SetAutoWhiteBal(camptr);
		
	}
	cvReleaseImage(&undist_src);
	cvReleaseImage(&undist_dst);
	printf("\nCam Thread Ended\n");
	return 0;
}

///This is a little ghetto but will be useful. Call this function at the beginning of your app.
// It will block until the auto gain setlles to the target gain value by adjusting shutter. It can
// take several seconds so be careful how you use it. Its important the scene is relatively constant
// while this is executing
void Sync1394Camera::DoAutoShutter(C1394Camera* camptr, unsigned char* buf, unsigned short targetGain)
{

	printf("Attempting to adjust shutter....\n");
	bool gotOKFrame = false;
	while (gotOKFrame==false)
	{
		WaitForSingleObject (cameraEvent,5000);
		if ((buf) == NULL)				
			printf("waiting for first image...\n");
		else
			gotOKFrame = true;
	}
	unsigned short minShutter =0;
	unsigned short maxShutter =0;
	GetShutterMinMax (camptr, &minShutter,&maxShutter);
	printf("max shutter: %d min shutter: %d",(int)maxShutter,(int)minShutter);
	bool targetOK = false;	
	bool oldAGCVal = this->config.AGC;
	this->config.AGC = false;
	SetGain (camptr,targetGain);
	while (targetOK == false)	
	{
		int median = GetMedianPixelValue(buf,config.AGCtop,config.AGCbot);		
		//too bright, positive error, 
		//so pos error means decrease gain
		float error = ((float)median - (float)idealMedian); 
		float effort = 1;

		effort = (abs(error * kp));
		if (error<0) effort*=-1;
		AGCerror = error;
		AGCeffort = effort;		

		int newShutter = (int) GetShutter (camptr)  - (int) effort;
		if ((newShutter < 900) && (newShutter > 0))
			SetShutter (camptr,newShutter);
		else if (newShutter > MAX_FPS_LIMITED_SHUTTER)
		{	
			SetShutter (camptr,MAX_FPS_LIMITED_SHUTTER);
			targetOK = true;
			printf("WARNING: Max shutter!\n");
		}
		else 
		{
			SetShutter (camptr,1);
			targetOK = true;
			printf("WARNING: Min shutter!\n");
		}

		if (error < 5) targetOK = true;
	}
	this->config.AGC = oldAGCVal;
	printf("Settled on shutter of: %d\n",(int)GetShutter (camptr));
}

unsigned short Sync1394Camera::GetShutter(C1394Camera* camptr)
{

	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_SHUTTER);
	unsigned short val = 0;
	ctrl->GetValue(&val,NULL);
	return (int)val;
}

void Sync1394Camera::GetShutterMinMax (C1394Camera* camptr, unsigned short* min, unsigned short* max)
{
	if (config.isSlave) return;
	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_SHUTTER);
	ctrl->GetRange(min,max);
	if (*min < 1) *min = 1;

}


int Sync1394Camera::SetShutter (C1394Camera* camptr,int val)
{
	if (config.isSlave) return 0 ;
	if (val>maxShutter) val = maxShutter;
	if (val<minShutter) val = minShutter;
	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_SHUTTER);
	if (ctrl->StatusAutoMode() == true)
	{	
		printf("I HAVE AUTOSHUTTER ASSHOLE");
		return (-1);
	}
	else
		return(ctrl->SetValue(val));
}

void Sync1394Camera::GetGainMinMax (C1394Camera* camptr, unsigned short* min, unsigned short* max)
{
	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_GAIN);
	ctrl->GetRange(min,max);
}

unsigned short Sync1394Camera::GetGain(C1394Camera* camptr)
{
	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_GAIN);
	unsigned short val = 0;
	ctrl->GetValue(&val,NULL);
	return val;
}

int Sync1394Camera::SetGain (C1394Camera* camptr, int val)
{
	if (config.isSlave) return 0;
	if (val>maxGain) val = maxGain;
	if (val<minGain) val = minGain;
	//printf("G : %d\n",val);
	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_GAIN);
	if (ctrl->StatusAutoMode() == true)
	{	
		printf("I HAVE AUTOGAIN ASSHOLE");
		return -1;
	}
	else
	{
		/*unsigned short read = 0;
		ctrl->GetValue(&read,NULL);
		printf("gain set to %d\n",read);*/
		return(ctrl->SetValue(val));
	}
}

unsigned short Sync1394Camera::GetBright(C1394Camera* camptr)
{
	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_BRIGHTNESS);
	unsigned short val = 0;
	ctrl->GetValue(&val,NULL);
	return val;
}

int Sync1394Camera::SetBright (C1394Camera* camptr, int val)
{
	unsigned short max = 0;
	unsigned short min = 0;
	if (config.isSlave) return 0;
	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_BRIGHTNESS);
	ctrl->GetRange(&min,&max);
	if (val>max) val = max;
	if (val<min) val = min;
	//printf("G : %d\n",val);
	
	if (ctrl->StatusAutoMode() == true)
	{	
		printf("I HAVE AUTOGAIN ASSHOLE");
		return -1;
	}
	else
	{
		/*unsigned short read = 0;
		ctrl->GetValue(&read,NULL);
		printf("gain set to %d\n",read);*/
		return(ctrl->SetValue(val));
	}
}


Sync1394Camera::Sync1394Camera()
{
	curSeqNumber = 0;
	expSeqNumber = 0;
	lastMedian=0;
	lastMaxAcc=0;
	kp = AUTOGAIN_KP;
	idealMedian = AUTOGAIN_MEDIAN_IDEAL;
	curtimestamp  = 0;
	cameraEvent = CreateEvent ( NULL , false , false , NULL);
}

int Sync1394Camera::GetNumberOfCameras ()
{
	if( camera.RefreshCameraList() == 0 ) 
	{ printf( "RefreshCameraList failed. Check that any cameras are connected.\n"); return false; }
	return camera.GetNumberCameras();
}

int Sync1394Camera::GetMedianPixelValue(unsigned char* buf, int top, int bottom)
{
	int start = config.width * top;
	int end = config.width * bottom;

	//ugh sort the pixels, find the middle
	if (config.isColor)
	{
		int tmp [256]={0};
		for (int i=start; i<end; i++)
		{	
			tmp[(((unsigned char*)buf )[i*3])]++;			
			tmp[(((unsigned char*)buf )[i*3+1])]++;			
			tmp[(((unsigned char*)buf )[i*3+2])]++;			
		}
		int dashits = 0;
		int i=0;
		int mid = ((end-start)*3) /2;						

		while (dashits < mid)
			dashits += tmp[i++];			
		i--;				
		lastMedian = i;
		return i;
	}

	else
	{
		if (config.BitDepth16)
		{
			int tmp [1024]={0};
			for (int i=start; i<end; i++)
				tmp[(((unsigned short*)buf )[i])]++;			
			int dashits = 0;
			int i=0;
			int mid = (end-start) /2;						
			i=0;
			while (dashits < mid)
				dashits += tmp[i++];			
			i--;					
			lastMedian = i;
			return i;
		}
		else
		{
			int tmp [256]={0};
			for (int i=start; i<end; i++)
				tmp[(((unsigned char*)buf )[i])]++;			
			int dashits = 0;
			int i=0;
			int mid = (end-start) /2;						
			i=0;
			while (dashits < mid)
				dashits += tmp[i++];			
			i--;					
			lastMedian = i;
			return i;
		}
	}
}


int Sync1394Camera::shortComp (const void* a, const void* b)
{
	unsigned short shita = *(unsigned short*)a;
	unsigned short shitb = *(unsigned short*)b;
	return (shita-shitb);
}

int Sync1394Camera::charComp (const void* a, const void* b)
{
	unsigned char shita = *(unsigned char*)a;
	unsigned char shitb = *(unsigned char*)b;
	return (shita-shitb);
}

int Sync1394Camera::GetNumMaxedPixelsInBuf (unsigned char* buf, int top, int bottom)
{
	int thisMax=0; 
	int acc = 0;
	int MAX = AUTOGAIN_MAX8BIT;
	int start = config.width * top;
	int end = config.width * bottom;
	if (config.BitDepth16) MAX = AUTOGAIN_MAX16BIT;
	if (config.isColor)
	{
		for (int i=start; i<end; i++)
		{
			if (((unsigned char*)buf )[i*3+0] >= MAX) acc++;			
			if (((unsigned char*)buf )[i*3+1] >= MAX) acc++;
			if (((unsigned char*)buf )[i*3+2] >= MAX) acc++;
		}
	}
	else
	{
		for (int i=start; i<end; i++)
		{
			if (config.BitDepth16)
			{
				if (((unsigned short*)buf )[i] >= MAX) acc++;
			}
			else
			{
				if (((unsigned char*)buf )[i] >= MAX)	acc++;
			}
		}
	}
	lastMaxAcc = acc;
	return acc;
}


Sync1394Camera::~Sync1394Camera()
{	
	isRunning = false;

	WaitForSingleObject (cameraHandle,INFINITE);
	printf("Terminating SyncCam\n");
	if (config.syncEnabled )
	{
		if ((config.syncKillOnClose) && (config.isSlave==false))
		{
			char killmsg[] = {0x05, config.syncID & 0xFF}; //stop message
			udpTX->send_message (killmsg,2,UDP_CONTROL_IP,UDP_CONTROL_PORT);
		}
		delete udpRX;
		delete udpTX;
	}
	DeleteCriticalSection (&camgrab_cs);
}
bool Sync1394Camera::InitCamera(int cameraID, SyncCamParams m_config) 
{
	C1394Camera* camptr;
	
	isRunning = true;
	Sync1394Camera::config = m_config;	
	int effWidth = config.partialWidth;
	int effHeight = config.partialHeight;
	
	camptr = &camera;

	if (config.isColor) 
		size = effWidth * effHeight * 3;
	else	
		size = effWidth * effHeight;
	if (config.BitDepth16) size *= 2;

	buf = new unsigned char[size];

	if(camptr->RefreshCameraList() == 0 ) 
	{ 
		printf("RefreshCameraList failed.\n");
		return false; 
	} 
	if(camptr->SelectCamera(cameraID)!=CAM_SUCCESS)	
	{ 
		printf("Could not select camera\n" ); 
		return false; 
	}
	camId = cameraID;
	if(camptr->InitCamera()!=CAM_SUCCESS)	
	{	
		printf("Could not init camera\n" ); 
		return false; 
	}	
	if(camptr->SetVideoFormat(config.videoFormat)!=CAM_SUCCESS)								
	{ 
		printf("Could not SetVideoFormat on camera\n" ); 
	}
	if(camptr->SetVideoMode(config.videoMode)!=CAM_SUCCESS)										
	{ 
		printf("Could not SetVideoMode on camera\n"); 
	}
	if (config.usePartialScan == false)
	{
		if(camptr->SetVideoFrameRate(config.videoFrameRate)!=CAM_SUCCESS)														
		{ 
			printf("Could not set video frame rate!");	
			return false;
		} //30 fps
	}
	else if (config.usePartialScan)
	{
		unsigned short w, h; 
		C1394CameraControlSize* p_size_control = camptr->GetCameraControlSize();
		p_size_control->GetSizeLimits(&w, &h);
		if ((w>effWidth) || (h>effHeight))																			
		{ 
			printf("FATAL: Bad Partial Scan Size Specified! Specified w:%d h:%d, Max: w:%d h:%d",effWidth,effHeight,w,h);	
			return false;
		} 
		if (config.isColor)
		{
			if( p_size_control->SetColorCode(COLOR_CODE_YUV422) != CAM_SUCCESS ) 	
			{ 
				printf("SetColorCode failed.\n"); 
				return false; 
			} 
		}
		else
		{
			if( p_size_control->SetColorCode(config.BitDepth16?COLOR_CODE_Y16:COLOR_CODE_Y8) != CAM_SUCCESS ) 
			{ 
				printf("SetColorCode failed (BW).\n"); 
				return false; 
			} 
		}
		if( p_size_control->SetSize(effWidth,effHeight) != CAM_SUCCESS )	
		{ 
			printf("SetSize failed.\n"); 
			return false; 
		} 
		if( p_size_control->SetPos(config.partialLeft,config.partialTop) != CAM_SUCCESS )
		{ 
			printf("SetPos failed.\n"); 
			return false; 
		} 
		if (p_size_control->SetBytesPerPacket (config.bytesPerPacket) != CAM_SUCCESS)
		{ 
			printf("SetBytesPerPacket failed.\n"); 
			return false; 
		} 

		float interval=0.0; p_size_control->GetFrameInterval (&interval);
		if (((float)config.syncFPS) > (1.0f/interval))													
		{ 
			printf("WARNING: SyncFPS (%f) is greater than the actual camera FPS (%f)\n",(float)config.syncFPS,(1.0f/interval));
		}
		printf("Frame Interval is: %f",interval);
	}
	if (config.isSlave)	
	{ 
		printf("Using SLAVE configuration.\n");
	}

	if (!config.AutoGain)
	{
		C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_GAIN);
		if (ctrl->SetAutoMode(false) != CAM_SUCCESS)
		{	
			printf("Gain SetManualMode failed.\n");
		}
		ctrl->SetValue(GAINVAL);
		if (ctrl->StatusAutoMode() == true)
		{	
			printf("I HAVE AUTOGAIN ASSHOLE");
		}
		unsigned short gainval = 0;
		ctrl->GetValue(&gainval);
		printf("gain: %d\n",gainval);
	}
	else
	{
		C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_GAIN);
		ctrl->SetValue(GAINVAL);
		if (ctrl->SetAutoMode(true) != CAM_SUCCESS)
		{	
			printf("Gain SetAutoMode failed.\n");
		}
		if (ctrl->StatusAutoMode() == true)
		{	
			printf("AutoGain successfully set!\n");
		}
	}
	
	minGain=0;
	maxGain=0;
	GetGainMinMax( camptr, &minGain, &maxGain);
	printf("Min gain: %d, Max gain: %d\n",minGain,maxGain);
	
	if (!config.AutoShutter)
	{
		C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_SHUTTER);
		if (ctrl->SetAutoMode(false) != CAM_SUCCESS)
		{	
			printf("Shutter SetManualMode failed.\n");
		}
		ctrl->SetValue(SHUTTERVAL);
		/*if (ctrl->SetValue(530) != CAM_SUCCESS)
		{	
			printf("Shutter SetValue failed.\n");
		}*/
		if (ctrl->StatusAutoMode() == true)
		{	
			printf("I HAVE AUTOSHUTTER ASSHOLE");
		}
	}
	else
	{
		C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_SHUTTER);
		ctrl->SetValue(SHUTTERVAL);
		if (ctrl->SetAutoMode(true) != CAM_SUCCESS)
		{	
			printf("Shutter SetAutoMode failed.\n");
		}
		if (ctrl->StatusAutoMode() == true)
		{	
			printf("AutoShutter successfully set!\n");
		}
	}

	if (!config.AutoBrightness)
	{
		C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_BRIGHTNESS);
		if (ctrl->SetAutoMode(false) != CAM_SUCCESS)
		{	
			printf("Brightness SetManualMode failed.\n");
		}
		ctrl->SetValue(154);
		if (ctrl->StatusAutoMode() == true)
		{	
			printf("I HAVE AUTOBRIGHTNESS ASSHOLE");
		}
		unsigned short bval = 0;
		ctrl->GetValue(&bval);
		printf("brightness: %d\n",bval);
	}
	else
	{
		C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_BRIGHTNESS);
		ctrl->SetValue(154);
		if (ctrl->SetAutoMode(true) != CAM_SUCCESS)
		{	
			printf("Brightness SetAutoMode failed.\n");
		}
		if (ctrl->StatusAutoMode() == true)
		{	
			printf("AutoBrightness successfully set!\n");
		}
	}
	
	if (config.gammaEnable)
	{
		C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_GAMMA);
		if (ctrl->SetOnOff(true) != CAM_SUCCESS)
		{	
			printf("Gamma SetOnOff failed.\n");
		}
		if (ctrl->SetValue(1) != CAM_SUCCESS)
		{
			printf("Gamma SetValue failed.\n");
		}
	}

	minShutter=0;
	maxShutter=0;
	GetShutterMinMax(camptr, &minShutter, &maxShutter);
	printf("Min shutter: %d, Max shutter: %d\n",minShutter,maxShutter);

	if (config.adjWB)
	{
		C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_WHITE_BALANCE);
		if (ctrl->SetValue(WHITEBALANCE_VALUE1,WHITEBALANCE_VALUE2) != CAM_SUCCESS)
		{
			printf("WhiteBalance SetValue failed.\n");
		}
	}
	
	printf("Completed Init of Camera\n");

	if (config.eTrigEnabled)
	{
		C1394CameraControlTrigger* trig = camptr->GetCameraControlTrigger();
		if (trig->SetTriggerSource (config.syncCamInput)!=CAM_SUCCESS)
			printf("Could Not Set Trigger Source!\n");
		else
		{
			if (trig->SetMode (0))
				printf("Could Not Set Trigger Mode!\n");
			else
			{
				if (trig->SetOnOff (true))
					printf("Could Not Set Trigger ON!\n");
				else
				{
					trig->Status();
					printf("External Trigger Initialized.\n");
				}
			}
		}
	}
	else
	{
		if (config.syncEnabled)
		{
			udp_params paramsRX  = udp_params(); 
			paramsRX.remote_ip = inet_addr(UDP_BROADCAST_IP);
			paramsRX.local_port = UDP_BROADCAST_PORT;
			paramsRX.reuse_addr = 1;
			try
			{		
				udpRX = new udp_connection(paramsRX);  
			}
			catch (exception)
			{
				printf("Couldn't init UDP RX on  %s:%d  \n",UDP_BROADCAST_IP,paramsRX.local_port);
				return false;
			}

			udpRX->set_callback(MakeDelegate(this,&Sync1394Camera::UDPCallback), udpRX);

			udp_params paramsTX  =  udp_params(); 

			try
			{		
				udpTX = new udp_connection(paramsTX);  
			}
			catch (exception)
			{
				printf("Couldn't init UDP TX on port %d\n",paramsTX.local_port);
				return false;
			}

			if (config.isSlave == false)
			{
				char regmsg[] = {CAMERA_MSG_REGISTER, config.syncID & 0xFF, 0x00, 0x00, 0x00, 0x00,(paramsRX.local_port>>8)&0xff, paramsRX.local_port&0xff}; //register message
				udpTX->send_message (regmsg,8,UDP_CONTROL_IP,UDP_CONTROL_PORT);
				Sleep(100);
				char fpsmsg[] = {CAMERA_MSG_SETFPS, config.syncID & 0xff, config.syncFPS &0xff};
				udpTX->send_message (fpsmsg,3,UDP_CONTROL_IP,UDP_CONTROL_PORT);
				Sleep(100);
				char initmsg[] = {CAMERA_START, config.syncID & 0xFF}; //start message
				udpTX->send_message (initmsg,2,UDP_CONTROL_IP,UDP_CONTROL_PORT);
				if (config.ghettoSync == false)
				{
					C1394CameraControlTrigger* trig = camptr->GetCameraControlTrigger ();

					unsigned short modein=0; unsigned short modeparam =0; unsigned short trigsrc=0;
					if (trig->SetTriggerSource (config.syncCamInput)!=CAM_SUCCESS)
						printf("Could Not Set Trigger Source!\n");
					if (trig->SetMode (0))
						printf("Could Not Set Trigger Mode!\n");
					if (trig->SetOnOff (true))
						printf("Could Not Set Trigger ON!\n");
				}
			}
			printf("Timing Initialized.\n");	
		}
		else
			printf("Timing has been disabled.\n");
	}

	unsigned long ulFlags = 0;  
	ulFlags |= ACQ_START_VIDEO_STREAM;

	if(config.isSlave)
	{
		ulFlags |= ACQ_SUBSCRIBE_ONLY;
		printf("Using slave configuration... make sure to start a master or no images will come in.\n");
		if (camptr->StartImageAcquisitionEx(6,1000,ulFlags))
		{
			printf("WARNING: Could not start image aquisition! Bailing....\n");
			return false;
		}
	}
	else
	{
		ulFlags |= ACQ_ALLOW_PGR_DUAL_PACKET;
		if(camptr->StartImageAcquisitionEx (6,1000,ulFlags))
		{
			printf("WARNING: Could not start image aquisition! Bailing....\n");
			return false;
		}
	}

	InitializeCriticalSection(&camgrab_cs);

	cameraHandle = CreateThread(NULL, 0, CamThreadWrap, this, 0, NULL);


	//Sleep(2000);//starting...
	//SetThreadPriority(cameraHandle, THREAD_PRIORITY_HIGHEST);
	return true;
}


void Sync1394Camera::UDPCallback(udp_message& msg, udp_connection* conn, void* arg)
{ 	
	if (msg.len != sizeof (SyncCamPacket))
	{
		printf("Warning: bad packet size. Packet is: %d bytes Expected %d bytes\n", msg.len,  sizeof (SyncCamPacket));
		return;
	}
	SyncCamPacket packet = *((SyncCamPacket*)(msg.data)); //very high sketchfactor
	if (config.syncID != packet.id) return;	
	packet.seconds = ntohs(packet.seconds);
	packet.seqNum = ntohl(packet.seqNum);
	packet.ticks = ntohl(packet.ticks);

	curtimestamp = (double)packet.seconds + (double)packet.ticks/10000.0;
	if (packet.seqNum > (unsigned int)curSeqNumber)
		curSeqNumber = packet.seqNum;

	//printf ("sec: %d ticks: %d sync: %d\n",packet.seconds,packet.ticks,packet.seqNum);
}

int Sync1394Camera::SetAutoWhiteBal(C1394Camera* camptr)
{
	unsigned short val = 0;
/*
	if (config.isSlave) return 0 ;
	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_WHITE_BALANCE);
	
	if(!ctrl->StatusAutoMode())
		ctrl->SetAutoMode(1);

	
	ctrl->GetValue(&val,NULL);//*/
//*/
	return (int)val;
}

int Sync1394Camera::GetWhiteBal(C1394Camera* camptr, unsigned short *val0, unsigned short *val1)
{
	if (config.isSlave) return 0 ;

	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_WHITE_BALANCE);
	
	ctrl->GetValue((unsigned short*)val0,(unsigned short*)val1);
	return 0;
//*/
}

int Sync1394Camera::GetWhiteBal(int camId, int *val0, int *val1)
{
	//C1394Camera* mycamPtr=NULL;
	//switch(camId){
	//	case 0:
	//		mycamPtr=&camera;
	//		break;
	//	case 1:
	//		mycamPtr=&camera1;
	//		break;
	//	case 2:
	//		mycamPtr=&camera2;
	//		break;
	//}
	return GetWhiteBal(&camera, (unsigned short*)val0, (unsigned short*)val1);
}



int Sync1394Camera::SetWhiteBal(C1394Camera* camptr, unsigned short val0, unsigned short val1)
{
//*
	if (config.isSlave) return 0 ;
	C1394CameraControl* ctrl = camptr->GetCameraControl (FEATURE_WHITE_BALANCE);
	return ctrl->SetValue(val0,val1);	
//*/	
}

int Sync1394Camera::SetWhiteBal(int camId, int val0, int val1)
{
	//C1394Camera* mycamPtr=NULL;
	//switch(camId){
	//	case 0:
	//		mycamPtr=&camera;
	//		break;
	//	case 1:
	//		mycamPtr=&camera1;
	//		break;
	//	case 2:
	//		mycamPtr=& camera2;
	//		break;
	//}
	return SetWhiteBal(&camera,(unsigned short)val0, (unsigned short)val1);
}

void Sync1394Camera::DoManualAdjustments(camera_adjust_param_t *cam_param)
{
	/*static unsigned long count;
	count ++;
	if(count%100 !=0)
		return;

	camera_adjust_param_t mycam_param[3];
	memcpy(&mycam_param, cam_param, sizeof(mycam_param));

	for(int camId=0;camId<3;camId++){
		if(cam_param[camId].updateCamera){
			cam_param[camId].updateCamera=0;
			//***set white balance***
			if(cam_param[camId].setWhiteBal==0){
				switch(camId){
					case 0:
						SetAutoWhiteBal(&camera);
						break;
					case 1:
						SetAutoWhiteBal(&camera1);
						break;
					case 2:
						SetAutoWhiteBal(&camera2);
						break;
				}//end switch
				
			}
			else{
				switch(camId){
					case 0:
						SetWhiteBal(&camera,mycam_param[camId].setWhiteBal);
						break;
					case 1:
						SetWhiteBal(&camera1,mycam_param[camId].setWhiteBal);
						break;
					case 2:
						SetWhiteBal(&camera2,mycam_param[camId].setWhiteBal);
						break;
				}//end switch
				
			}
			//***set shutter***
			if(cam_param[camId].setShutter==0){
				//set auto shutter
			}
			else{
				//set shutter
			}
			//***set gain***
			if(cam_param[camId].setShutter==0){
				//set auto gain
			}
			else{
				//set gain
			}
		}//end if(cam_param[camId].updateCamera){
	}//end for(int camId=0;camId<3;camId++){//*/
}

int Sync1394Camera::DoAutoWhiteBal(C1394Camera* camptr, unsigned char* buf)
{
	

	return 0;
}
//*


int Sync1394Camera::DoAutoWhiteBalance(C1394Camera* camPtr, IplImage *im, unsigned short *wr, unsigned short *wb)
{
	/*if(im->nChannels!=3){
		printf("White Balance error. Colour image not detected!\n");
		return -1;
	}
	unsigned short r_gain,b_gain;
	unsigned short r_gain_old,b_gain_old;
	m_wbal[0].InitInstance(cvSize(im->width,im->height));


	GetWhiteBal(camPtr, &r_gain, &b_gain);
	r_gain_old=r_gain;
	b_gain_old=b_gain;
	m_wbal[0].RunAWB(im, &r_gain, &b_gain);
	SetWhiteBal(camPtr, r_gain, b_gain);

	//printf("white balance: %d->%d %d->%d\n",r_gain_old,r_gain,b_gain_old,b_gain);
*/
	return 0;


	unsigned short old_wr;
	unsigned short old_wb;
	int i,j,ptr;
	float rSum=0;
	float gSum=0;
	float bSum=0;
	float rAve=0;
	float gAve=0;
	float bAve=0;
	int pixCount=0;
	const float thresh = 10.0f;
	const float gain=1;
	const int halfSize = 100;
	bool bupdateWhiteBalance=0;
	CvRect roi;
	float fwr,fwb;

	
	if(im->nChannels!=3){
		printf("White Balance error. Colour image not detected!\n");
		return -1;
	}

	
	//create region of interest for white balance probing
	roi.x=config.width/2-halfSize;
	roi.y=config.height/2-halfSize;
	roi.width=halfSize*2;
	roi.height=halfSize*2;

	CvPoint topLeft= cvPoint(roi.x,roi.y);
	CvPoint botRight=cvPoint(roi.x+roi.width,roi.y+roi.height);
	
	//cvShowImage("test",im); cvWaitKey(10);
	//return 0;
	//get average values
	for(i=topLeft.y;i<botRight.y;i++){	//horizontal
		for(j=topLeft.x;j<botRight.x;j++){	//vertical
			ptr=3*(j*roi.width+i);
			rSum+= (unsigned char)im->imageData[ptr];
			gSum+= (unsigned char)im->imageData[ptr+1];
			bSum+= (unsigned char)im->imageData[ptr+2];
			pixCount++;
		}	
	}
	//get average
	rAve=rSum/pixCount;
	gAve=gSum/pixCount;
	bAve=bSum/pixCount;

	//get current white balance
	GetWhiteBal(camPtr,wr,wb);
	old_wr=*wr;
	old_wb=*wb;

	//control color balance
	if((rAve-gAve)<-thresh){
		//fwbr += wr + gain;
		*wr += (int)(*wr - gain);
		bupdateWhiteBalance=1;
	}
	else if((rAve-gAve)>thresh){
		//fwbr += fwbr-gain;

		*wr += (int)*wr + gain;
		bupdateWhiteBalance=1;
	}

	if(bAve-gAve<-thresh){
		*wb = *wb + gain;
		bupdateWhiteBalance=1;
	}
	else if(bAve-gAve>thresh){
		*wb = *wb - gain;
		bupdateWhiteBalance=1;
	}

	
	SetWhiteBal(camPtr,*wr,*wb);
	printf("white balance: %d->%d %d->%d\n",old_wr,*wr,old_wb,*wb);

	
	return 0;
}
//*