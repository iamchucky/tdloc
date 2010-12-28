#include "udpCameraBroadcaster.h"

//TRANSMITTER----------------------------------------------------------------------------------------------------

UDPCameraBroadcaster::UDPCameraBroadcaster()
{
	udp_params params = udp_params();
	params.remote_ip = inet_addr(UDP_CAMERA_ADDR);	
	//bind to the .10 subnet
	unsigned int outIP=0;
	//this means to send it on the wireless network (i.e. something with ip address 10.x.x.x)

	if (find_subnet_adapter(10,outIP))
		printf("Bound OK\n");

	params.local_ip = outIP;
	params.local_port = 30099;
	params.multicast_loopback = true;
	params.multicast_ttl=10;
	conn = new udp_connection (params);	
	
	UDP_CAMERA_ADDR_SEND = UDP_CAMERA_ADDR;
	UDP_CAMERA_PORT_SEND = UDP_CAMERA_PORT;
	printf("UDP Camera Broadcast TX Interface Initialized. %s:%d\r\n", UDP_CAMERA_ADDR_SEND, UDP_CAMERA_PORT_SEND);
}
UDPCameraBroadcaster::UDPCameraBroadcaster(const char* ip_addr, unsigned short port)
{
	udp_params params = udp_params();
	params.remote_ip = inet_addr(ip_addr);
	
	//bind to the .10 subnet
	unsigned int outIP=0;
	//this means to send it on the logging network (i.e. something with ip address 239.132.1.99)

	if (find_subnet_adapter(192,outIP))
		printf("Bound OK\n");			// Socket not bounded? debug here!

	//params.local_ip = inet_addr("192.168.1.101");
	params.local_ip = outIP;
	params.local_port = port;
	params.multicast_loopback = true;
	params.multicast_ttl=10;
	conn = new udp_connection (params);		
	UDP_CAMERA_ADDR_SEND = ip_addr;
	UDP_CAMERA_PORT_SEND = port;

	printf("UDP Camera Broadcast TX Interface Initialized. %s:%d\r\n", ip_addr,port);
}

UDPCameraBroadcaster::~UDPCameraBroadcaster()
{
	printf("UDP Camera Broadcast Shutdown...\r\n");
}


void UDPCameraBroadcaster::SendImage (const uchar* img, int step, int width, int height, double ts, int index, int id)
{
	WorstStreamEver s = WorstStreamEver (65000);
	SimpleJPEG jpeg;
	//jpeg.WriteImage (img,width,height,s);
	jpeg.WriteImage(img, step, width, height, 8, 3, s);
	int jpegSize = s.GetLength ();
	
	UDPCameraMsg hdr;
	hdr.height = height; hdr.width = width; hdr.index = index; hdr.timestamp = ts; hdr.size =jpegSize; hdr.id = id;

	int rawmsgSize= jpegSize + sizeof(UDPCameraMsg);
	void* rawmsg = malloc (rawmsgSize);
	memcpy(rawmsg,&hdr,sizeof(UDPCameraMsg));
	memcpy(((unsigned char*)rawmsg)+sizeof(UDPCameraMsg),s.GetBuffer(),s.GetLength());
	conn->send_message(rawmsg,rawmsgSize,inet_addr(UDP_CAMERA_ADDR_SEND),UDP_CAMERA_PORT_SEND);
	free (rawmsg);		
}



//RECIEVER-----------------------------------------------------------------------------------------------------------
void UDPCameraReceiver::SetCallback(UDPCamera_Msg_handler handler, void* arg)
{
	cbk = handler;
	cbk_arg = arg;
}

UDPCameraReceiver::UDPCameraReceiver() 
{
	udp_params params = udp_params();
	params.remote_ip = inet_addr(UDP_CAMERA_ADDR);
	params.local_port = UDP_CAMERA_PORT;
	params.reuse_addr = 1;
	conn = new udp_connection (params);	
	conn->set_callback (MakeDelegate(this,&UDPCameraReceiver::UDPCallback),conn);	
	sequenceNumber=0;
	packetCount=0;
	dropCount=0;
	printf("UDP Camera RX Interface Initialized. %s:%d\r\n",UDP_CAMERA_ADDR,UDP_CAMERA_PORT);
}

UDPCameraReceiver::~UDPCameraReceiver ()
{
	delete conn;
	printf("UDP Camera Shutdown...\r\n");
}

void UDPCameraReceiver::UDPCallback(udp_message& msg, udp_connection* conn, void* arg)
{ 
	//messages come in like this:
	// ID + memory mapped message, ID is 1 byte	
	UDPCameraMsg umsg = *((UDPCameraMsg*)msg.data);
	
	int rxSequenceNumber=	umsg.index;
	dropCount += (rxSequenceNumber-(sequenceNumber+1));
	sequenceNumber = rxSequenceNumber;
	packetCount++;
	
	//raise event				
	if (!(cbk.empty()))	cbk(umsg, this, cbk_arg);


	if (packetCount%100==0)	
	{
		#ifdef PRINT_PACKET_COUNT_DEBUG
		printf("UC: Packets: %d Seq: %d Dropped: %d Drop Rate: %f \r\n",packetCount,sequenceNumber,dropCount,((float)dropCount/(float)packetCount)*100.0f);	
		#endif
		packetCount=0; dropCount=0;
	}
}