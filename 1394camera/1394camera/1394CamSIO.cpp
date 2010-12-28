/**\file 1394CamSIO.cpp
 * \brief Implements SIO Advanced Functionality
 * \ingroup sio
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

/**\defgroup sio Serial I/O Functionality
 * \ingroup camoptional
 *
 * The simplest optional extension is 32 bits each of bitwise I/O,
 * accessed via a handful of registers.
 */

/**\brief Configure the Serial Port
 * \ingroup sio
 * \param baud The data rate, in bps.
 * \param databits The number of data bits (7 or 8)
 * \param stopbits The number of stop bits (0,1,2 -> 1,1.5,2)
 * \param parity (0,1,2 -> None, Odd, Even)
 * \return CAM_SUCCESS on success, or else:
 *  - CAM_ERROR_UNSUPPORTED if the serial functionality is not available
 *  - CAM_ERROR_INVALID_PARAMETER if the arguments are broken
 *  - CAM_ERROR if an underlying WriteQuadlet() failed
 */
int C1394Camera::SIOConfigPort(unsigned long baud, unsigned long databits, unsigned long stopbits, unsigned long parity)
{
	int ret = CAM_ERROR;
	unsigned long ulData = 0;

	DllTrace(DLL_TRACE_ENTER,"ENTER SIOConfigPort(%d,%d,%d,%d)\n",
		baud,databits,stopbits,parity);

	if(!this->HasSIO())
	{
		DllTrace(DLL_TRACE_ERROR,"SIOConfigPort: Camera at %08x does not support SIO\n",this);
		ret = CAM_ERROR_UNSUPPORTED;
		goto _exit;
	}

	if((databits != 7 && databits != 8) ||
		stopbits > 2 || parity > 2)
	{
		DllTrace(DLL_TRACE_ERROR,"SIOConfigPort: Invalid data format parameter: %d(7-8),%d(0-2),%d(0-2)\n",
			databits,stopbits,parity);
		ret = CAM_ERROR_PARAM_OUT_OF_RANGE;
		goto _exit;
	}


	switch(baud)
	{
	case 230400:
		ulData++;
	case 115200:
		ulData++;
	case 57600:
		ulData++;
	case 38400:
		ulData++;
	case 19200:
		ulData++;
	case 9600:
		ulData++;
	case 4800:
		ulData++;
	case 2400:
		ulData++;
	case 1200:
		ulData++;
	case 600:  
		ulData++;
	case 300:
		DllTrace(DLL_TRACE_CHECK,"SIOConfigPort: Baud %d -> %d\n",baud,ulData);
		break;
	default:
		DllTrace(DLL_TRACE_ERROR,"SIOConfigPort: Invalid Baud %d\n",baud);
		ret = CAM_ERROR_PARAM_OUT_OF_RANGE;
		goto _exit;
	}

	// pack it in
	ulData <<= 24;
	ulData |= databits << 16;
	ulData |= parity << 14;
	ulData |= stopbits << 12;

	// write it out
	ret = this->WriteQuadlet(this->m_SIOFuncOffset,ulData);

_exit:
	DllTrace(DLL_TRACE_EXIT,"EXIT SIOConfigPort (%d)\n",ret);
	return ret;
}

/**\brief Enable the Transmission and/or reception of data at the serial port
 * \ingroup sio
 * \param bReceive Whether to enable the receiver
 * \param bTransmit Whether to enable the transmitter
 * \return CAM_SUCCESS on success, or else:
 *  - CAM_ERROR_UNSUPPORTED if the serial functionality is not available
 *  - CAM_ERROR if an underlying WriteQuadlet() fails
 */
int C1394Camera::SIOEnable(BOOL bReceive, BOOL bTransmit)
{
	unsigned long ulData;
	if(!this->HasSIO())
		return CAM_ERROR_UNSUPPORTED;

	ulData = (bReceive == TRUE);
	ulData <<= 1;
	ulData |= (bTransmit == TRUE);
	ulData <<= 30;
	return this->WriteQuadlet(this->m_SIOFuncOffset + 0x0004,ulData);
}

/**\brief Read Some Serial Data
 * \ingroup sio
 * \param data The data buffer
 * \param datalen how many bytes to read
 * \return The non-negative number of bytes read, or else:
 *  - CAM_ERROR_UNSUPPORTED if the serial functionality is not available
 *  - CAM_ERROR if an underlying ReadQuadlet() or WriteQuadlet() fails
 *
 * Note: we are stuck using 4-byte reads and writes for now.  This may be boosted
 * by re-enabling the generic Asynchronous I/O, but that might be a bit of a hassle
 */
int C1394Camera::SIOReadBytes(unsigned char *data, unsigned long datalen)
{
	int ret;
	unsigned long i,nreads,ulData, bytes = 0;

	if(data == NULL || !this->HasSIO())
		return CAM_ERROR_UNSUPPORTED;

	if((ret = this->ReadQuadlet(this->m_SIOFuncOffset + 0x0008,&ulData)) != CAM_SUCCESS)
		return ret;

	ulData >>= 22;
	DllTrace(DLL_TRACE_CHECK,"SIOReadBytes: %d bytes are available\n",ulData);
	
	if(ulData < datalen)
		datalen = ulData;

	nreads = (datalen + 3) /4;

	for(i=0; i<nreads; i++)
	{
		// for now, we read four at a time
		ulData = 4 << 16;
		if((ret = this->WriteQuadlet(this->m_SIOFuncOffset + 0x0008,ulData)) != CAM_SUCCESS)
			return ret;
		
		// read them out
		if((ret = this->ReadQuadlet(this->m_SIOFuncOffset + 0x0100,&ulData)) != CAM_SUCCESS)
			return ret;
	
		for(bytes = (i<<2); bytes < datalen && bytes < ((i+1)<<2); bytes++)
		{
			data[bytes] = (unsigned char)((ulData >> 24) & 0x00FF);
			ulData <<= 8;
		}
	}
	return bytes;
}

/**\brief Write Some Serial Data
 * \ingroup sio
 * \param data The data buffer
 * \param datalen how many bytes to write
 * \return The non-negative number of bytes written, or else:
 *  - CAM_ERROR_UNSUPPORTED if the serial functionality is not available
 *  - CAM_ERROR if an underlying ReadQuadlet() or WriteQuadlet() fails
 *
 * Note: we are stuck using 4-byte reads and writes for now.  This may be boosted
 * by re-enabling the generic Asynchronous I/O, but that might be a bit of a hassle
 */
int C1394Camera::SIOWriteBytes(unsigned char *data, unsigned long datalen)
{
	int ret;
	unsigned long i,nreads,ulData, bytes = 0;

	if(data == NULL || !this->HasSIO())
		return CAM_ERROR_UNSUPPORTED;

	if((ret = this->ReadQuadlet(this->m_SIOFuncOffset + 0x000C,&ulData)) != CAM_SUCCESS)
		return ret;

	ulData >>= 22;
	DllTrace(DLL_TRACE_CHECK,"SIOWriteBytes: %d bytes are available\n",ulData);
	
	if(ulData < datalen)
		datalen = ulData;

	nreads = (datalen + 3) /4;

	for(i=0; i<nreads; i++)
	{
		// for now, we write four at a time
		ulData = 4 << 16;
		if((ret = this->WriteQuadlet(this->m_SIOFuncOffset + 0x000C,ulData)) != CAM_SUCCESS)
			return ret;
		
		// pack them in
		ulData = 0;
		for(bytes = (i<<2); bytes < datalen && bytes < ((i+1)<<2); bytes++)
		{
			ulData += data[bytes];
			ulData <<= 8;
		}

		// write them out
		if((ret = this->WriteQuadlet(this->m_SIOFuncOffset + 0x0100,ulData)) != CAM_SUCCESS)
			return ret;
	
	}
	return bytes;
}

/**\brief Retrieve the Status byte for the SIO controller
 * \ingroup sio
 * \param byte Where to put the byte
 * \return CAM_SUCCESS on success, else:
 *  - CAM_ERROR_UNSUPPORTED if the serial functionality is not available
 *  - CAM_ERROR if an underlying ReadQuadlet() or WriteQuadlet() fails
 *
 * From High to low, the bits in the returned byte indicate:
 *  - 0: Transmit data buffer ready (not full)
 *  - 1: reserved
 *  - 2: Receive data buffer ready (not empty)
 *  - 3: reserved
 *  - 4: Receive Buffer Overrun Error (clear with SIOEnable())
 *  - 5: Receive Data Framing Error (clear with SIOEnable())
 *  - 6: Receive Data Parity Error (clear with SIOEnable())
 *  - 7: reserved
 */
int C1394Camera::GetSIOStatusByte(unsigned char *byte)
{
	int ret;
	unsigned long ulData;

	if(byte == NULL || !this->HasSIO())
		return CAM_ERROR_UNSUPPORTED;

	if((ret = this->ReadQuadlet(this->m_SIOFuncOffset + 0x0004,&ulData)) != CAM_SUCCESS)
		return ret;

	*byte = (unsigned char)((ulData>>16) & 0x00FF);
	return CAM_SUCCESS;
}