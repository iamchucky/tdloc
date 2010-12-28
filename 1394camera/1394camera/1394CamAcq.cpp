/**\file 1394CamAcq.cpp
 * \brief Implements Acquisition functionality for the 1394Camera class
 * \ingroup camacq
 */

//////////////////////////////////////////////////////////////////////
//
//	Version 6.4
//
//  Copyright 8/2006
//
//  Christopher Baker
//  Robotics Institute
//  Carnegie Mellon University
//  Pittsburgh, PA
//
//	Copyright 5/2000
// 
//	Iwan Ulrich
//	Robotics Institute
//	Carnegie Mellon University
//	Pittsburgh, PA
//
//  This file is part of the CMU 1394 Digital Camera Driver
//
//  The CMU 1394 Digital Camera Driver is free software; you can redistribute 
//  it and/or modify it under the terms of the GNU Lesser General Public License 
//  as published by the Free Software Foundation; either version 2.1 of the License,
//  or (at your option) any later version.
//
//  The CMU 1394 Digital Camera Driver is distributed in the hope that it will 
//  be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with the CMU 1394 Digital Camera Driver; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//////////////////////////////////////////////////////////////////////

#include "pch.h"

/** \defgroup camacq Frame Acquisition
 *  \brief This is the primary means of grabbing camera frames via the 1394 bus.
 *  \ingroup camcore
 *
 * The I/O model for image acquisition is as follows:
 * - A handful of image buffers are maintained as a circularly-linked list.
 *
 * - The buffer pointed to by m_pCurrentBuffer is the "Current Buffer" and
 *     is guaranteed to not be attached to the isochronous buffer queue.
 *
 * - The buffer pointed to by m_pLastBuffer is the last buffer in the list
 *     to have undergone the IOTCL_ATTACH_BUFFER prodecure.  Under most 
 *     circumstances, the buffer immediately following m_pLastBuffer will
 *     be m_pCurrentBuffer
 *
 * - As each buffer is "acquired", that buffer becomes the "Current Buffer"
 *     all buffers between m_pLastBuffer and the new m_pCurrentBuffer are 
 *     reattached to the isochronous buffer queue
 */

/** 
 * \defgroup acqflags Acquisition Options
 * \ingroup camacq
 * These flags allow alternative uses of the frame acquisition system, 
 * such as using OneShot(), MultiShot(), or Hardware Triggering 
 * \see C1394CameraControlTrigger For information on Hardware Triggering.
 */

/**
 * \brief Initialize the Image Acquisition Process
 * \ingroup camacq
 * \param nBuffers The number of buffers to allocate.  Minimum of 1
 * \param FrameTimeout Timeout, in milliseconds for blocking calls to AcquireImageEx();
 * \param Flags Acquisition Flags, \see acqflags
 * \return
 * - <b>CAM_SUCCESS</b> The camera is ready. Call AcquireImage() to grab frames 
 * - <b>CAM_ERROR_NOT_INITIALIZED</b> No camera has been selected and/or InitCamera has not been successfully called 
 * - <b>CAM_ERROR_INVALID_VIDEO_SETTINGS</b> The current video settings (format, mode, and/or rate) are not supported. 
 * - <b>CAM_ERROR_BUSY</b> The camera is already capturing or acquiring images
 * - <b>CAM_ERROR_INSUFFICIENT_RESOURCES</b> Not enough 1394 bus resoures are available to complete the operation 
 * - <b>ERROR_OUTOFMEMORY</b> GlobalAlloc for the descriptors or for the buffers has failed
 * - <i>Any other windows error code</i> Bad things happened on DeviceIoControl or on GetOverlappedResult. The particular error code indicates just what
 * 
 * Allocates and Attaches Frame Buffers, configures kernel-side 1394 resources, and (optionally) starts image streaming
 */
int C1394Camera::StartImageAcquisitionEx(int nBuffers, int FrameTimeout, int Flags)
{
	PACQUISITION_BUFFER				pAcqBuffer = NULL;
	DWORD							dwRet;
	HANDLE							hdev;
	int							i;
	int retval = CAM_ERROR;
	
	DllTrace(DLL_TRACE_ENTER,"ENTER StartImageAcquisition (nBuffers = %d)\n",nBuffers);
	
	if(m_hDeviceAcquisition != INVALID_HANDLE_VALUE)
	{
		DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: The Camera is already acquiring images, call StopImageAcquisition first\n");
		retval =  CAM_ERROR_BUSY;
		goto _exit;
	}
	
	if(!m_cameraInitialized)
	{
		DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: Call InitCamera() first.\n");
		retval =  CAM_ERROR_NOT_INITIALIZED;
		goto _exit;
	}
	
	if(nBuffers < 1)
	{
		DllTrace(DLL_TRACE_ERROR,"AcquireImage: Invalid number of buffers: %d\n",nBuffers);
		retval = CAM_ERROR_PARAM_OUT_OF_RANGE;
		goto _exit;
	}
	
	if(!CheckVideoSettings())
	{
		DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: CheckVideoSettings Failed\n");
		retval =  CAM_ERROR_INVALID_VIDEO_SETTINGS;
		goto _exit;
	}
	
	// adopt the incoming timeout and flags
	this->m_AcquisitionTimeout = FrameTimeout;
	this->m_AcquisitionFlags = Flags;
	this->m_AcquisitionBuffers = nBuffers;
	
	if(!InitResources())
	{
		DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: InitResources Failed\n");
		retval =  CAM_ERROR_INSUFFICIENT_RESOURCES;
		goto _exit;
	}
	
	///////////////////////////////////////////
	// allocate and set up the frame buffers //
	///////////////////////////////////////////
	
	DllTrace(DLL_TRACE_CHECK,"StartImageAcquisition: Initializing Buffers...\n");
	for(i=0; i<nBuffers; i++)
	{
		// allocate the buffer header (stores data about a buffer)
		pAcqBuffer = (PACQUISITION_BUFFER) GlobalAlloc(LPTR,sizeof(ACQUISITION_BUFFER));
		if(!pAcqBuffer)
		{
			DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: Error Allocating AcqBuffer %d\n",i);
			retval = CAM_ERROR_INSUFFICIENT_RESOURCES;
			goto _exit;
		}
		
		// add it to our list of buffers
		if(i == 0)
		{
			m_pLastBuffer = m_pFirstBuffer = pAcqBuffer;
			m_pLastBuffer->pNextBuffer = m_pCurrentBuffer = NULL;
		} else {
			m_pFirstBuffer->pNextBuffer = pAcqBuffer;
			m_pFirstBuffer = pAcqBuffer;
		}
		
		// allocate the actual frame buffer
		// the buffer passed to ATTACH_BUFFER must be aligned on a page boundary
		// thus, allocate an extra 4K and generate a pointer to the first page boundary
		pAcqBuffer->pDataBuf = (unsigned char *)GlobalAlloc(LPTR,m_maxBufferSize + 4096);
		if(!pAcqBuffer->pDataBuf)
		{
			DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: Error Allocating Data Buffer %d\n",i);
			retval = ERROR_OUTOFMEMORY;
			goto _exit;
		}
		
		// HACK HACK HACK Something about DMA and 1394 gets angry if the Isoch buffers aren't page-aligned.
		// point pFrameStart at the first page boundary
		pAcqBuffer->pFrameStart = pAcqBuffer->pDataBuf + (4096 - (((int)(pAcqBuffer->pDataBuf)) & 0xfff));
		
		// set the index (mostly for debugging purposes)
		pAcqBuffer->index = i;
		
		// give the overlapped structure an event
		pAcqBuffer->overLapped.hEvent = CreateEvent(NULL,TRUE,FALSE,NULL);
		if(pAcqBuffer->overLapped.hEvent == INVALID_HANDLE_VALUE)
		{
			DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: Error Creating Overlapped Event %d\n",i);
			retval = GetLastError();
			goto _exit;
		}
		pAcqBuffer->ulBufferSize = m_maxBufferSize;
		DllTrace(DLL_TRACE_VERBOSE,"StartImageAcquisition: Allocated buffer %d\n",i);
	}
	
	// all done making buffers
	// open our long term device handle
	if((hdev = OpenDevice(m_pName, TRUE)) == INVALID_HANDLE_VALUE)
	{
		DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: error opening device (%s)\n",m_pName);
		retval = CAM_ERROR;
		goto _exit;
	}
	
	// attach all our buffers
	pAcqBuffer = m_pLastBuffer;
	while(pAcqBuffer)
	{
		DllTrace(DLL_TRACE_VERBOSE,
			"StartImageAcquisition: Attaching buffer: index:%d, size:%d, FrameStart:%08x\n",
			pAcqBuffer->index,
			pAcqBuffer->ulBufferSize,
			pAcqBuffer->pFrameStart);
		
		dwRet = t1394IsochAttachBuffer(
			hdev,
			pAcqBuffer->pFrameStart,
			pAcqBuffer->ulBufferSize,
			&(pAcqBuffer->overLapped)
			);
		
		if ((dwRet != ERROR_IO_PENDING) && (dwRet != ERROR_SUCCESS)) 
		{
			// bad things have happened
			// what exactly needs done here isn't very well defined, so let's do nothing
			// for now
			DllTrace(DLL_TRACE_ERROR,
				"StartImageAcquisition: Error %08x while Attaching Buffer %d\n",
				dwRet,pAcqBuffer->index);
			retval =  CAM_ERROR;
			goto _exit;
		}
		pAcqBuffer = pAcqBuffer->pNextBuffer;
	}
	
	// isoch listen
	if((dwRet = t1394IsochListen(m_pName)) != ERROR_SUCCESS)
	{
		DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: Error %08x on IOCTL_ISOCH_LISTEN\n",dwRet);
		retval =  CAM_ERROR;
		goto _exit;
	}
	
	// start streaming if necessary
	if(this->m_AcquisitionFlags & ACQ_START_VIDEO_STREAM)
	{
		if((retval = StartVideoStream()) != CAM_SUCCESS)
			goto _exit;
	}
	
	// if we get here, everything is cool
	m_hDeviceAcquisition = hdev; // assigning this here makes things a little more thread-safe;
	retval =  CAM_SUCCESS;
	
_exit:
	
	if(retval != CAM_SUCCESS)
	{
		// we go here if something breaks and we need to clean up anything we've allocated thus far
		DllTrace(DLL_TRACE_ERROR,"StartImageAcquisition: Error on setup, Cleaning up...\n");
		this->StopImageAcquisition();
	}
	
	DllTrace(DLL_TRACE_EXIT,"EXIT StartImageAcquisition (%d)\n",retval);
	return retval;
}

/**
 * \brief Initialize the Image Acquisition Process
 * \ingroup camacq
 * \return same as StartImageAcquisitionEx()
 *
 * As of Version 6.3, this wraps StartImageAcquisitionEx() for backwards compatiblility.
 * It is equivalent to StartImageAcquisitionEx(6,1000,ACQ_START_VIDEO_STREAM)
 */
int C1394Camera::StartImageAcquisition()
{
	return StartImageAcquisitionEx(6,1000,ACQ_START_VIDEO_STREAM);
}

/**
 * \brief Grab the most recent frame from the queue, dropping stale frames.
 * \ingroup camacq
 * \return same as AcquireImageEx()
 *
 * As of Version 6.3, this wraps AcquireImageEx() for backwards compatiblility.
 * It is equivalent to AcquireImageEx(TRUE,NULL)
 */
int C1394Camera::AcquireImage()
{
	return AcquireImageEx(TRUE,NULL);
}

/**
 * \brief Grab a frame off the queue.
 * \ingroup camacq
 * \param DropStaleFrames Boolean: whether to skip stale frames in the list
 * \param lpnDroppedFrames: where to put the  number of dropped frames
 * \return
 * - <b>CAM_SUCCESS</b> An image was successfully grabbed and the data awaits 
 * - <b>CAM_ERROR_NOT_INITIALIZED</b> StartImageAcquisitionEx() has not been successfully called.
 * - <i>Any other windows error code</i> Bad things happened on DeviceIoControl or on GetOverlappedResult.<i>GetLastError</i> will indicate what.
 *
 * According to the model, this reattaches all buffers that have fallen off up to, but not including the
 * buffer described by C1394camera::m_pCurrentBuffer, thus maintaining all invaraints
 *   
 * Also, this will block until at least the next buffer in the isochronous queue is ready.  To avoid Blocking, see GetFrameEvent()
 */
int C1394Camera::AcquireImageEx(BOOL DropStaleFrames, int *lpnDroppedFrames)
{
	DWORD dwRet, dwBytesRet;
	BOOL ready,bWaited=FALSE;
	int blockedframe = -1;
	int	dropped = 0;
	int n=0;
	int ret = CAM_ERROR; // default return value is CAM_ERROR
	
	DllTrace(DLL_TRACE_ENTER,"ENTER AcquireImage\n");
	
	if(m_hDeviceAcquisition == INVALID_HANDLE_VALUE)
	{
		DllTrace(DLL_TRACE_ERROR,"AcquireImage: Not Acquiring Images: Call StartImageAcquisition First");
		ret = CAM_ERROR_NOT_INITIALIZED;
		goto _exit;
	}
	
	// this loop is basically: attach the "current" buffer and snag the next until we have something or we time out
	do
	{
		// first things first, attach the current buffer, if possible
		if(m_pCurrentBuffer != NULL)
		{
			// reset the overlapped event
			ResetEvent(m_pCurrentBuffer->overLapped.hEvent);
			
			DllTrace(DLL_TRACE_VERBOSE,"AcquireImage: Attaching buffer: index:%d, size:%d, FrameStart:%08x\n",
				m_pCurrentBuffer->index,
				m_pCurrentBuffer->ulBufferSize,
				m_pCurrentBuffer->pFrameStart);
			
			dwRet = t1394IsochAttachBuffer(
				m_hDeviceAcquisition,
				m_pCurrentBuffer->pFrameStart,
				m_pCurrentBuffer->ulBufferSize,
				&(m_pCurrentBuffer->overLapped)
				);
			
			if ((dwRet != ERROR_IO_PENDING) && (dwRet != ERROR_SUCCESS)) 
			{
				DllTrace(DLL_TRACE_ERROR,"AcquireImage: Error %08x while reattaching Buffer %d\n",dwRet,m_pCurrentBuffer->index);
				goto _exit;
			}
			
			// push m_pCurrentBuffer onto the Buffer Queue
			if(m_pFirstBuffer == NULL)
			{
				m_pLastBuffer = m_pCurrentBuffer;
			} else {
				m_pFirstBuffer->pNextBuffer = m_pCurrentBuffer;
			}
			m_pFirstBuffer = m_pCurrentBuffer;
			m_pCurrentBuffer = m_pFirstBuffer->pNextBuffer = NULL;
		}
		
		// Check the Last frame on the queue for completion
		if(!(ready = GetOverlappedResult(m_hDeviceAcquisition, &m_pLastBuffer->overLapped, &dwBytesRet, FALSE)))
		{
			dwRet = GetLastError();
			if(dwRet == ERROR_IO_INCOMPLETE) 
			{
				// try waiting
				DllTrace(DLL_TRACE_VERBOSE,"AcquireImage: First Frame Incomplete, blocking...\n");
				dwRet = WaitForSingleObject(
					m_pLastBuffer->overLapped.hEvent,
					this->m_AcquisitionTimeout >= 0 ? this->m_AcquisitionTimeout : INFINITE );
				if(dwRet == WAIT_OBJECT_0)
				{
					// the wait worked
					if(!(ready = GetOverlappedResult(m_hDeviceAcquisition, &m_pLastBuffer->overLapped, &dwBytesRet, TRUE)))
					{
						DllTrace(DLL_TRACE_ERROR,"AcquireImage: Error %08x while Getting Overlapped Result (Block)\n",GetLastError());
						goto _exit;
					} else {
						bWaited = TRUE;
					}
				} else {
					if(dwRet == WAIT_TIMEOUT)
					{
						DllTrace(DLL_TRACE_VERBOSE,"AcquireImage: Timeout waiting for frame %d\n",m_pLastBuffer->index);
						ret = CAM_ERROR_FRAME_TIMEOUT;
						goto _exit;
					} else {
						DllTrace(DLL_TRACE_ERROR,"AcquireImage: Error %08x on WaitForSingleObject\n",GetLastError());
						goto _exit;
					}
				}        
			} else {
				DllTrace(DLL_TRACE_ERROR,"AcquireImage: Error %08x while Getting Overlapped Result (Non-Block)\n",GetLastError());
				goto _exit;
			}
		}
		
		if(ready)
		{
			DllTrace(DLL_TRACE_VERBOSE,"AcquireImage: Frame %d is ready\n",m_pLastBuffer->index);
			m_pCurrentBuffer = m_pLastBuffer;
			m_pLastBuffer = m_pLastBuffer->pNextBuffer;
			if(m_pLastBuffer == NULL)
				m_pFirstBuffer = NULL;
			n++;
		}
	} while(DropStaleFrames && !bWaited);
	
	DllTrace(DLL_TRACE_VERBOSE,"AcquireImage: Current Buffer is now %d, data at %08x\n",m_pCurrentBuffer->index, m_pCurrentBuffer->pFrameStart);
	
	if(lpnDroppedFrames)
		*lpnDroppedFrames = n-1;
	
	ret = CAM_SUCCESS;
_exit:
	DllTrace(DLL_TRACE_EXIT,"EXIT AcquireImage (%d)\n",ret);
	return ret;
}

/**
 * \brief Halt frame transfer and free bus resources.
 * \ingroup camacq
 * \return
 * - <b>CAM_SUCCESS</b> This is slightly deceptive in that it will return "success" 
 *    regardless of whether is successfully turned things off. If it does encounter 
 *    an error in the process, it traces it, but then continues on to free whatever remaining resource it can. 
 * - <b>CAM_ERROR_NOT_INITIALIZED</b> StartImageAcquisitionEx() has not been successfully called.
 *
 *  This function will block until all acquisition buffers are filled (upper limit is ~10 seconds for timeout)
 *  It also frees all resources (memory, events, handles, etc.) allocated by StartImageAcquisitionEx()
 */
int C1394Camera::StopImageAcquisition()
{
	DWORD							dwBytesRet;
	PACQUISITION_BUFFER				pAcqBuffer = NULL;
	int ret;
	
	DllTrace(DLL_TRACE_ENTER,"ENTER StopImageAcquisition\n");
	
	if(m_hDeviceAcquisition == INVALID_HANDLE_VALUE)
		DllTrace(DLL_TRACE_WARNING,"StopImageAcquisition: Called with invalid device handle\n");
	
	if(this->m_AcquisitionFlags & ACQ_START_VIDEO_STREAM)
		StopVideoStream();
	
	// Tear down the stream
	if(!FreeResources())
		DllTrace(DLL_TRACE_WARNING,"StartmageAcquisition: Cleanup Warning: FreeResources() failed\n");
	
	// put m_pCurrentBuffer on the list for the sake of cleanup
	if(m_pCurrentBuffer != NULL)
	{
		m_pCurrentBuffer->pNextBuffer = m_pLastBuffer;
		m_pLastBuffer = m_pCurrentBuffer;
	}
	
	while(m_pLastBuffer)
	{
		DllTrace(DLL_TRACE_VERBOSE,"StopImageAcquisition: Removing buffer %d\n",m_pLastBuffer->index);
		if(m_pLastBuffer != m_pCurrentBuffer)
		{
			// check the IO status, just in case
			DllTrace(DLL_TRACE_CHECK,"StopImageAcquisition: Checking on buffer %d\n",m_pLastBuffer->index);
			if(!GetOverlappedResult(m_hDeviceAcquisition, &m_pLastBuffer->overLapped, &dwBytesRet, TRUE))
			{
				DllTrace(DLL_TRACE_WARNING,"StopImageAcqisition: Warning Buffer %d has not been detached, error = %d\n",
					m_pLastBuffer->index,GetLastError());
			}
		}
		
		// close event
		if(m_pLastBuffer->overLapped.hEvent)
			CloseHandle(m_pLastBuffer->overLapped.hEvent);
		// free data buffer
		if(m_pLastBuffer->pDataBuf)
			GlobalFree(m_pLastBuffer->pDataBuf);
		
		// advance to next buffer
		pAcqBuffer = m_pLastBuffer;
		m_pLastBuffer = m_pLastBuffer->pNextBuffer;
		
		// free buffer struct
		GlobalFree(pAcqBuffer);
	}
	
	// clean up our junk
	if(m_hDeviceAcquisition != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hDeviceAcquisition);
		m_hDeviceAcquisition = INVALID_HANDLE_VALUE;
	}

	m_pFirstBuffer = m_pLastBuffer = m_pCurrentBuffer = NULL;
	this->m_AcquisitionTimeout = 0;
	this->m_AcquisitionFlags = 0;
	this->m_AcquisitionBuffers = 0;

	ret = CAM_SUCCESS;
	DllTrace(DLL_TRACE_EXIT,"EXIT StopImageAcquisition (%d)\n",ret);
	return ret;
}

/**
 * \brief Retrieve the synchronization event for the next pending frame
 * \ingroup camacq
 * \return
 *  - Asynchronous Event Handle for the next pending frame on the list
 *  - INVALID_HANDLE_VALUE if there is no pending frame (not Acquiring images, or else the single buffer case)
 *
 *  This is a minor hack to allow blocking on multiple cameras via WaitForMultipleObjects.
 *  Once this event is triggered, the next call to AcquireImageEx() is guaranteed not to block.
 *  This must be called after each call to AcquireImageEx() to retrieve the proper handle for the next pending frame.
 *
 *  If StartImageAcquisitionEx() was called with nBuffers=1, then this will only return a valid handle after a call to 
 *  AcquireImageEx() that returns CAM_ERROR_FRAME_TIMEOUT
 */
 HANDLE C1394Camera::GetFrameEvent()
 {
	 if(m_pLastBuffer)
		 return m_pLastBuffer->overLapped.hEvent;
	 else
		 return INVALID_HANDLE_VALUE;
 }

/**\brief Get a pointer to the raw frame data from the current frame
 * \ingroup camacq
 * \param pLength If non-NULL, receives the length (in bytes) of the frame data
 * \return Pointer to the first byte of frame data, or NULL if none is available
 *
 * This is a safety-checked accessor for m_pCurrentBuffer
 */
unsigned char *C1394Camera::GetRawData(unsigned long *pLength)
{
	if(this->m_pCurrentBuffer)
	{
		if(pLength)
			*pLength = m_pCurrentBuffer->ulBufferSize;
		return m_pCurrentBuffer->pFrameStart;
	} else {
		if(pLength)
			*pLength = 0;
		return NULL;
	}
}
