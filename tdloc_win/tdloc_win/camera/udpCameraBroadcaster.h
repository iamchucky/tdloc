#pragma once

#include "..\network\udp_connection.h"
#include "..\network\net_utility.h"
#include "..\utility\fastdelegate.h"
#include "..\camera\simplejpeg.h"
#include <fstream>

#define UDP_CAMERA_PORT 30099
// YES MULTIPLEXER
#define UDP_CAMERA_ADDR "10.0.0.8"

// NO MULTIPLEXER
//#define UDP_CAMERA_ADDR "239.132.1.99"

#pragma pack(1)
struct UDPCameraMsg
{
	int index;
	double timestamp;
	int width;
	int height;
	int size;
	int id;
};

#pragma pack()

class UDPCameraBroadcaster
{
public:

	UDPCameraBroadcaster();
	UDPCameraBroadcaster(const char* ip_addr, unsigned short port);
	~UDPCameraBroadcaster();
	void SendImage (const uchar* img, int step, int height, int width, double ts, int index, int id);			

private:
	udp_connection *conn;
	udp_connection *connFast;
	const char* UDP_CAMERA_ADDR_SEND;
	int UDP_CAMERA_PORT_SEND;
	};

class UDPCameraReceiver;

typedef FastDelegate3<UDPCameraMsg, UDPCameraReceiver*, void*> UDPCamera_Msg_handler;

using namespace std;
class UDPCameraReceiver
{
private:
	udp_connection *conn;		
	void UDPCallback(udp_message& msg, udp_connection* conn, void* arg);
	UDPCamera_Msg_handler cbk;		
	void* cbk_arg;		

public:
	UDPCameraReceiver(void);
	~UDPCameraReceiver(void);
	void SetCallback(UDPCamera_Msg_handler handler, void* arg);		
	int sequenceNumber;
	int packetCount;
	int dropCount;
};