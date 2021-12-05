//
// AE483GroundStation
// David Hanley
//
// main.c
// Entry point to program. Retrieves data from the motion capture system
// using the NatNet SDK and transmits the data over an XBee radio to the
// robot. An XBee must be connected to the PC and the robot. The robot
// must be expecting data over Ascending Technologies ACI. NatNet 
// compliant server must be running (e.g. Motive, Tracking Tools, etc..).
//
// Options:
//
//
// Usage:
//    AE483GroundStation.exe
//

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*--------------- Includes ---------------*/
//windows sockets, must go before windows.h
#include <winsock2.h>

//windows
#include <windows.h>

//system
#include <io.h>
#include <stdio.h>
#include <fcntl.h>  // File control definitions
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <time.h>
#include <tchar.h>
#include <strsafe.h>
#include <conio.h>
#include <ws2tcpip.h>
#include <stdint.h> // Dave's Addition

//asctec
#include "ftd2xx.h"
#include "asctecCommIntf.h"

//project created
#include "planner.h"
/*------------- End Includes -------------*/

/*--------------- Pragmas ---------------*/
/*------------- End Pragmas -------------*/

/*--------------- Defines ---------------*/
// NATNET message ids
#define NAT_PING                    0 
#define NAT_PINGRESPONSE            1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100
#define UNDEFINED                   999999.9999

//max packet size (actual size is dynamic)
#define MAX_PACKETSIZE				100000

//max lengths
#define MAX_NAMELENGTH              256
#define MAX_ADDLENGTH               128
#define MAX_MSGLENGTH                64

//socket information
#define MULTICAST_ADDRESS		"239.255.42.99"     // IANA, local network
#define PORT_COMMAND            1510
#define PORT_DATA  			    1511                // Default multicast group

//mechanism ID
#define QUAD_ID					2 //Set to Quadrotor ID (should ALWAYS be 1)

#define OBST_ID					1// the id you must assign to the obst while init MOCAP

//flight data recorder
#define FDR_DIRPATH				"U:\\AE 483 Lab\\Workspace\\Project\\Follower\\"	// path to directory where flight data will be saved

/*------------- End Defines -------------*/

/*--------------- Globals ---------------*/

/*int ndes = 37;
int ides;
float xdes[37] = {  0.0,  0.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  0.0};
float ydes[37] = {  0.0,  0.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  0.0};
float zdes[37] = {  0.0,  0.0, -1.0,  -1.0, -1.5,  -1.5, -1.0,  -1.0, -1.5,  -1.5, -1.0,  -1.0, -0.5,  -0.5, -1.0,  -1.0, -0.5,  -0.5, -1.0,  -1.0, -1.5,  -1.5, -1.0,  -1.0, -1.5,  -1.5, -1.0,  -1.0, -0.5,  -0.5, -1.0,  -1.0, -0.5,  -0.5, -1.0,  -1.0,  0.0};
float tdes[37] = {  0.0,  5.0,  8.0,  10.0, 13.0,  15.0, 18.0,  20.0, 23.0,  25.0, 28.0,  30.0, 33.0,  35.0, 38.0,  40.0, 43.0,  45.0, 48.0,  50.0, 53.0,  55.0, 58.0,  60.0, 63.0,  65.0, 68.0,  70.0, 73.0,  75.0, 78.0,  80.0, 83.0,  85.0, 88.0,  90.0, 93.0};
*/
//Take-off and Land
int ndes = 5;
int ides;
float xdes[5] = {  0.0,  0.0,  0.0,   0.0,  0.0 };
float ydes[5] = {  0.0,  0.0,  0.0,   0.0,  0.0 };
float zdes[5] = {  0.0,  0.0, -2.0,  -2.0,  0.0 };
float tdes[5] = {  0.0,  5.0, 15.0,  30.0, 40.0 };

/*//Back and Forth
int ndes = 9;
int ides;
float xdes[9] = {  0.0,  0.0,  0.0,   2.0,  2.0,  -2.0,  -2.0,  0.0,  0.0 };
float ydes[9] = {  0.0,  0.0,  0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.0 };
float zdes[9] = {  0.0,  0.0, -0.66,  -0.66, -0.66,  -0.66,  -0.66, -0.66,  0.0 };
float tdes[9] = {  0.0,  5.0, 10.0,  20.0, 25.0,  45.0,  50.0, 60.0, 65.0 };
*/

/*----- Primitives -----*/
// Handlers for XBee file descriptor
FT_HANDLE ftHandle;
FT_STATUS ftStatus;

// Control Handling Flag
int run_program = 1;
int var_getted = 0;
int cmd_ready = 0;
int data_updated = 0;

// Flight Data Recorder Start
int StartREC = 0;

// NatNet
int NatNetVersion[4] = { 0, 0, 0, 0 };
int ServerVersion[4] = { 0, 0, 0, 0 };

//MOCAP Counter
int32_t mocap_counter = 0;

// Variables for receiving ACI data sent from on-board over XBee
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
int32_t gyro_x;
int32_t gyro_y;
int32_t gyro_z;
uint32_t onboard_counter;
/*--- End Primitives ---*/

/*----- Structs -----*/
// Goal Position and yaw angle of quarotor
PosYaw_f QuadGoal;
// Desired position and yaw angle of quadrotor
PosYaw_f QuadDesired;
// Current position and orientation (yaw, pitch, roll) of quadrotor
State6_f QuadCurrent;
// Position and orientation (yaw, pitch, roll) of obstacle
State6_f ObstCurrent;

//networking
typedef struct sockaddr sockaddr;
typedef struct sockaddr_in sockaddr_in;
typedef struct in_addr in_addr;
typedef struct addrinfo addrinfo;
SOCKET CommandSocket;
SOCKET DataSocket;
sockaddr_in HostAddr;

// sender
typedef struct
{
	char szName[MAX_NAMELENGTH];            // sending app's name
	unsigned char Version[4];               // sending app's version [major.minor.build.revision]
	unsigned char NatNetVersion[4];         // sending app's NatNet version [major.minor.build.revision]

} sSender;

//packet
typedef struct
{
	unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
	unsigned short nDataBytes;              // Num bytes in payload
	union
	{
		unsigned char  cData[MAX_PACKETSIZE];
		char           szData[MAX_PACKETSIZE];
		unsigned long  lData[MAX_PACKETSIZE / 4];
		float          fData[MAX_PACKETSIZE / 4];
		sSender        Sender;
	} Data;                                 // Payload

} sPacket;
/*--- End Structs ---*/
/*------------- End Globals -------------*/

/*--------------- Function Prototypes ---------------*/
/*------------- End Function Prototypes -------------*/

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*------------------------------ Helpers -------------------------------*/
/*----------------------------------------------------------------------*/

/*--------------- Print Message ---------------*/
void PrintMessage(const char* msg)
{
	size_t msg_sz = sizeof(msg);
	int width = 70 - (int)msg_sz;
	printf("= %-*s =\n", width, msg);
}
/*------------- End Print Message -------------*/

/*--------------- Print Break ---------------*/
void PrintBreak() { printf("======================================================================\n"); };
/*------------- End Print Break -------------*/

/*--------------- Print Intro Prompt ---------------*/
void PrintIntroPrompt()
{
	printf("\n======================================================================\n");
	printf("=====                    AE 483 Ground Station                   =====\n");
	printf("= This program reads data from an OptiTrack Motion Capture system    =\n");
	printf("= using the NatNet SDK and sends data over an XBee radio. This       =\n");
	printf("= requires that an XBee is plugged into the PC and the robot. Also,  =\n");
	printf("= the robot must be expecting data over Ascending Technolgies' ACI.  =\n");
	printf("= Lastly, the PC must be running a NatNet compliant server running   =\n");
	printf("= (e.g Tracking Tool, Motive, etc...).                               =\n");
	printf("=                                                                    =\n");
	printf("= Run \"AE483GroundStation.exe -h\" for more information             =\n");
	printf("=====                                                            =====\n");
	printf("= David Hanley                                                       =\n");
	printf("= University of Illinois                                             =\n");
	printf("= May 12, 2015                                                       =\n");
	printf("======================================================================\n");
	PrintMessage("");
}
/*------------- End Print Intro Prompt -------------*/

/*--------------- Print Exit Prompt ---------------*/
void PrintExitPrompt()
{
	PrintMessage("");
	printf("======================================================================\n");
	printf("=====                Exiting AE 483 Ground Station               =====\n");
	printf("======================================================================\n\n");
}
/*------------- End Print Exit Prompt -------------*/

/*----------------------------------------------------------------------*/
/*---------------------------- End Helpers -----------------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*----------------------------- ACI Callbacks --------------------------*/
/*----------------------------------------------------------------------*/
/*---------------- transmit -------------------*/
// callback function for aciSetSendDataCallback(). Called by ACI
// when data should be written. In this function, we write the data
// to the Xbee.
//
// input:
//		byte - bytes to be written
//		cnt - number of bytes to be written
void transmit(void* byte, unsigned short cnt)
{
	DWORD dwBytesRead = 0;
	FT_Write(ftHandle, byte, cnt, &dwBytesRead);
}
/*---------------- End transmit----------------*/

/*---------------- versions -------------------*/
void versions(struct ACI_INFO aciInfo)
{
	printf("**************** Versions ****************\n");
	printf("* Type \t\t\tDevice\t\tRemote\t*\n");
	printf("* Major version\t\t%d\t=\t\%d\t*\n", aciInfo.verMajor, ACI_VER_MAJOR);
	printf("* Minor version\t\t%d\t=\t\%d\t*\n", aciInfo.verMinor, ACI_VER_MINOR);
	printf("* MAX_DESC_LENGTH\t%d\t=\t\%d\t*\n", aciInfo.maxDescLength, MAX_DESC_LENGTH);
	printf("* MAX_NAME_LENGTH\t%d\t=\t\%d\t*\n", aciInfo.maxNameLength, MAX_NAME_LENGTH);
	printf("* MAX_UNIT_LENGTH\t%d\t=\t\%d\t*\n", aciInfo.maxUnitLength, MAX_UNIT_LENGTH);
	printf("* MAX_VAR_PACKETS\t%d\t=\t\%d\t*\n", aciInfo.maxVarPackets, MAX_VAR_PACKETS);
	printf("***************************************************\n");
}
/*--------------- End versions ----------------*/

/*--------------- aciThread ---------------*/
// Function ran by the thread created in main. This function
// listens for incoming data and reads it. 
//
// input:!G
//    lpParameter - parameters passed by function that initializes thread
DWORD WINAPI aciThread(__in LPVOID lpParameter)
{
	//variables
	int result = 0;
	unsigned char data = 0;

	//run
	while (1)
	{
		//listen
		BOOL readed;
		DWORD dwBytesRead = 0;
		readed = FT_Read(ftHandle, &data, 1, &dwBytesRead);

		//bytes received
		while (dwBytesRead != 0)
		{
			aciReceiveHandler(data);
			readed = FT_Read(ftHandle, &data, 1, &dwBytesRead);
		}
		aciSynchronizeVars();

		//sleep
		aciEngine();
		Sleep(1);
	}
}
/*--------------- End aciThread -----------------------*/

/*--------------- cmdListUpdateFinished ---------------*/
// callback function called when the cmd list update is finished
// by the ACI engine.
void cmdListUpdateFinished()
{
	//prompt
	PrintMessage("Command List Received.");
	PrintMessage("");

	//defining packet structure: aciAddContentToCmdPacket(packet number, id, pointer to variable)
	aciAddContentToCmdPacket(0, 0x0511, &mocap_counter);
	aciAddContentToCmdPacket(0, 0x0512, &(QuadCurrent.Pos.x));
	aciAddContentToCmdPacket(0, 0x0513, &(QuadCurrent.Pos.y));
	aciAddContentToCmdPacket(0, 0x0514, &(QuadCurrent.Pos.z));
	aciAddContentToCmdPacket(0, 0x0515, &(QuadCurrent.Ori.roll));
	aciAddContentToCmdPacket(0, 0x0516, &(QuadCurrent.Ori.pitch));
	aciAddContentToCmdPacket(0, 0x0517, &(QuadCurrent.Ori.yaw));
	aciAddContentToCmdPacket(0, 0x0518, &(QuadDesired.Pos.x));
	aciAddContentToCmdPacket(0, 0x0519, &(QuadDesired.Pos.y));
	aciAddContentToCmdPacket(0, 0x0520, &(QuadDesired.Pos.z));
	aciAddContentToCmdPacket(0, 0x0521, &(QuadDesired.yaw));
	

	//configure packet
	aciSendCommandPacketConfiguration(0, 0);

	//Initialize
	QuadCurrent.Pos.x = 0.0;
	QuadCurrent.Pos.y = 0.0;
	QuadCurrent.Pos.z = 0.0;
	QuadCurrent.Ori.roll = 0.0;
	QuadCurrent.Ori.pitch = 0.0;
	QuadCurrent.Ori.yaw = 0.0;
	QuadDesired.Pos.x = 0.0;
	QuadDesired.Pos.y = 0.0;
	QuadDesired.Pos.z = -1.0;
	QuadDesired.yaw = 0.0;

	//aciUpdateCmdPacket(Packet #)
	aciUpdateCmdPacket(0);
	cmd_ready = 1;
}
/*------------- End cmdListUpdateFinished -------------*/

/*--------------- varListUpdateFinished ---------------*/
// callback function called when the variable list update is finished
// by the ACI engine.
void varListUpdateFinished()
{
	// Prompt
	printf("Variable List Received.\n");

	// Define packet structure: aciAddContentToVarPacket(packet number, id, pointer to variable);
	aciAddContentToVarPacket(1, 0x0203, &accel_x);
	aciAddContentToVarPacket(1, 0x0204, &accel_y);
	aciAddContentToVarPacket(1, 0x0205, &accel_z);
	aciAddContentToVarPacket(1, 0x0201, &gyro_x);
	aciAddContentToVarPacket(1, 0x0200, &gyro_y);
	aciAddContentToVarPacket(1, 0x0202, &gyro_z);
	//aciAddContentToVarPacket(1, 0x0206, &onboard_counter);

	// Initialize
	//
	// NOTE: the number "20" in aciSetVarPacketTransmissionRate means that the ground station
	// should receive ACI data at a rate of 50 Hz (1000 / 20). You can change this number - e.g.,
	// "10" is another common choice - but you will have to use the ACI Tool (perhaps with a
	// power cycle, etc.) to set the transmission rate to a matching number in your network
	// configuration.
	//
	aciSetVarPacketTransmissionRate(1, 1000);
	aciVarPacketUpdateTransmissionRates();
	aciSendVariablePacketConfiguration(1);

	var_getted = 1;
}
/*------------- End varListUpdateFinished -------------*/
/*----------------------------------------------------------------------*/
/*----------------------------- End ACI Callbacks ----------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*----------------------------- Networking -----------------------------*/
/*----------------------------------------------------------------------*/
/*--------------- Unpack ---------------*/
//
// NOTE: This function is what parses the MOCAP data.
//
void Unpack(char* pData)
{
	char MsgText[MAX_MSGLENGTH];
	char szName[MAX_NAMELENGTH];
	int major = NatNetVersion[0];
	int minor = NatNetVersion[1];
	int frameNumber = 0;
	int nMarkerSets = 0;
	int nOtherMarkers = 0;
	int nRigidBodies = 0;
	int nRigidMarkers = 0;
	int nMarkers = 0;
	int ID = 0;
	int i = 0;
	int j = 0;
	int MessageID = 0;
	int nBytes = 0;
	int nDataBytes = 0;
	short params = 0;
	int bTrackingValid = 0;
	float x = 0.0f, y = 0.0f, z = 0.0f;
	float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 0.0f;
	char *ptr = pData;
	Quat_f Quaternion;
	Pos_f  Position;
	DWORD mutexR;

	// message ID
	memcpy(&MessageID, ptr, 2); ptr += 2;
	
	// size
	memcpy(&nBytes, ptr, 2); ptr += 2;
	
	if (MessageID == 7) {     // FRAME OF MOCAP DATA packet
		// frame number
		memcpy(&frameNumber, ptr, 4); ptr += 4;
		
		/*----- Marker Sets -----*/
		// number of markersets
		memcpy(&nMarkerSets, ptr, 4); ptr += 4;
		
		for (i = 0; i < nMarkerSets; i++)
		{
			// Markerset name
			strcpy_s(szName, MAX_NAMELENGTH, ptr);
			nDataBytes = (int)strlen(szName) + 1;
			ptr += nDataBytes;

			//skip marker data
			memcpy(&nMarkers, ptr, 4); ptr += 4;
			ptr += nMarkers * 12;
		}
		/*--- End Marker Sets ---*/

		/*----- Unidentified Markers -----*/
		memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
		
		//move passed nOtherMarkers data, 12 bytes per Marker
		ptr += nOtherMarkers * 12;
		/*--- End Unidentified Markers ---*/

		/*----- Rigid Bodies -----*/
		memcpy(&nRigidBodies, ptr, 4); ptr += 4;
		for (j = 0; j < nRigidBodies; j++)
		{
			// rigid body pos/ori
			memcpy(&ID, ptr, 4); ptr += 4;
			memcpy(&(Position.x), ptr, 4); ptr += 4;
			memcpy(&(Position.y), ptr, 4); ptr += 4;
			memcpy(&(Position.z), ptr, 4); ptr += 4;
			memcpy(&(Quaternion.qx), ptr, 4); ptr += 4;
			memcpy(&(Quaternion.qy), ptr, 4); ptr += 4;
			memcpy(&(Quaternion.qz), ptr, 4); ptr += 4;
			memcpy(&(Quaternion.qw), ptr, 4); ptr += 4;
			
			//populate current state
			mutexR = WAIT_OBJECT_0;
			if (mutexR == WAIT_OBJECT_0)
			{
				//update Quadrotor state
				if (QUAD_ID > 0 && ID == QUAD_ID)
				{
					Planner_MakeState6f_PosQuat(&QuadCurrent, &Position, &Quaternion);
					data_updated = 1;
				}else if (OBST_ID > 0 && ID == OBST_ID){
					//update obstacle state
					Planner_MakeState6f_PosQuat(&ObstCurrent, &Position, &Quaternion);
					data_updated = 1;
				}
			}
			
			// associated marker positions
			memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
			ptr += nRigidMarkers * 3 * sizeof(float);
			major = 2;
			minor = 7;
			if (major >= 2)
			{
				// associated marker IDs
				ptr += nRigidMarkers*sizeof(int);;

				// associated marker sizes
				ptr += nRigidMarkers*sizeof(float);

				// Mean marker error
				ptr += 4;
			}

			// 2.6 and later
			if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
			{
				// params
				ptr += 2;
			}
		}
	} else if (MessageID == 5) {
		PrintMessage("Ignoring Data Description Packet.");
	} else {
		PrintMessage("Unrecognized Packet Type.");
	}

}
/*------------- End Unpack -------------*/

/*--------------- GetLocalIPAddresses ---------------*/
int GetLocalIPAddresses(unsigned long Addresses[], int nMax)
{
	//variables
	unsigned long  NameLength = 128;
	char szMyName[1024];
	addrinfo aiHints;
	addrinfo *aiList = NULL;
	sockaddr_in addr;
	int retVal = 0;
	char* port = "0";

	//get computer name
	if (GetComputerNameA(szMyName, &NameLength) != TRUE) { PrintMessage("Failed to get computer name."); return 0; }

	//getaddrinfo
	memset(&aiHints, 0, sizeof(aiHints));
	aiHints.ai_family = AF_INET;
	aiHints.ai_socktype = SOCK_DGRAM;
	aiHints.ai_protocol = IPPROTO_UDP;
	if ((retVal = getaddrinfo(szMyName, port, &aiHints, &aiList)) != 0)	{ PrintMessage("Failed to getaddrinfo()."); return 0; }

	//copy address info
	memcpy(&addr, aiList->ai_addr, aiList->ai_addrlen);
	freeaddrinfo(aiList);
	Addresses[0] = addr.sin_addr.S_un.S_addr;

	//return
	return 1;
}
/*------------- End GetLocalIPAddresses -------------*/

/*--------------- CreateCommandSocket ---------------*/
SOCKET CreateCommandSocket(unsigned long IP_Address, unsigned short uPort)
{
	//variables
	sockaddr_in my_addr;
	static unsigned long ivalue;
	static unsigned long bFlag;
	int nlengthofsztemp = 64;
	SOCKET sockfd;

	// Create a blocking, datagram socket
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) { return 1; }

	// bind socket
	memset(&my_addr, 0, sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(uPort);
	my_addr.sin_addr.S_un.S_addr = IP_Address;
	if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == SOCKET_ERROR) { closesocket(sockfd); return 1; }

	// set to broadcast mode
	ivalue = 1;
	if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (char *)&ivalue, sizeof(ivalue)) == SOCKET_ERROR) { closesocket(sockfd); return 1; }

	//return
	return sockfd;
}
/*------------- End CreateCommandSocket -------------*/

/*--------------- CommandListenThread ---------------*/
DWORD WINAPI CommandListenThread(void* dummy)
{
	int addr_len;
	int nDataBytesReceived;
	int i;
	sockaddr_in TheirAddress;
	sPacket PacketIn;
	addr_len = sizeof(struct sockaddr);

	while (1)
	{
		// blocking
		int nDataBytesReceived = recvfrom(CommandSocket, (char *)&PacketIn, sizeof(sPacket), 0, (struct sockaddr *)&TheirAddress, &addr_len);

		//if no bytes or error, continue
		if ((nDataBytesReceived == 0) || (nDataBytesReceived == SOCKET_ERROR)) { continue; }

		// handle command - Doing Nothing, Uncomment to start handling commands
		switch (PacketIn.iMessage)
		{
		case NAT_PINGRESPONSE:
			for (i = 0; i<4; i++)
			{
				NatNetVersion[i] = (int)PacketIn.Data.Sender.NatNetVersion[i];
				ServerVersion[i] = (int)PacketIn.Data.Sender.Version[i];
			}
			break;
		default: PrintMessage("    Command Packet Received. Ignoring."); break;
		}
	}

	return 0;
}
/*------------- End CommandListenThread -------------*/

/*--------------- DataListenerThread ---------------*/
DWORD WINAPI DataListenThread(void* dummy)
{
	//variables
	char  szData[20000];
	int addr_len = sizeof(struct sockaddr);
	sockaddr_in TheirAddress;

	//run and listen for packets
	while (1)
	{
		// Block until we receive a datagram from the network (from anyone including ourselves)
		int nDataBytesReceived = recvfrom(DataSocket, szData, sizeof(szData), 0, (sockaddr *)&TheirAddress, &addr_len);
		Unpack(szData);

		//Increment mocap counter
		mocap_counter++;
	}

	//return
	return 0;
}
/*------------- End DataListenerThread -------------*/
/*----------------------------------------------------------------------*/
/*----------------------------- Networking -----------------------------*/
/*----------------------------------------------------------------------*/


/*----------------------------------------------------------------------*/
/*-------------------------- Signal Handling ---------------------------*/
/*----------------------------------------------------------------------*/

/*--------------- Ctrl Handler ---------------*/
BOOL CtrlHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
		// Handle the CTRL-C signal. 
	case CTRL_C_EVENT:
		PrintMessage("Quit Signal (Ctrl-C). Quitting.");
		run_program = 0;
		return(TRUE);

		// CTRL-CLOSE: confirm that the user wants to exit. 
	case CTRL_CLOSE_EVENT:
		PrintMessage("Quit Signal (Ctrl-C). Quitting.");
		run_program = 0;
		return(TRUE);

	case CTRL_LOGOFF_EVENT:
		PrintMessage("Quit Signal (Ctrl-C). Quitting.");
		run_program = 0;
		return(TRUE);

	case CTRL_SHUTDOWN_EVENT:
		PrintMessage("Quit Signal (Ctrl-C). Quitting.");
		run_program = 0;
		return(TRUE);

	default:
		return FALSE;
	}
}
/*------------- End Ctrl Handler -------------*/

/*----------------------------------------------------------------------*/
/*-------------------------- Signal Handling ---------------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*-------------------------- Flight Data Record ------------------------*/
/*----------------------------------------------------------------------*/
/*--------------- RecData ---------------*/
// Function ran by the thread created in main. This function
// listens for incoming data and reads it. 
//
// input:
//    lpParameter - parameters passed by function that initializes thread
DWORD WINAPI RecData(__in LPVOID lpParameter)
{
	char enter = 0;
	printf("Record Data [Hit Enter to Begin]\n");

	while (enter != '\r' && enter != '\n') { enter = getchar(); }

	StartREC = 1;
	printf("Recording Data!\n");
}
/*--------------- End RecData -----------------------*/
/*----------------------------------------------------------------------*/
/*-------------------------- End Flight Data Record --------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*-------------------------------- Main --------------------------------*/
/*----------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
	//variables
	HANDLE thHandle;						//aci thread handler
	DWORD  thID;							//aci thread id
	HANDLE RecHandle;	// RecData handler
	DWORD Record;		// RecData identifier

	//Networking variables
	WSADATA WsaData;								//winsock handler
	in_addr ServerAddress;							//server addresses
	char szServerIPAddress[MAX_ADDLENGTH] = "";		//ServerIPAddress
	in_addr MyAddress;								//client address
	char szMyIPAddress[MAX_ADDLENGTH] = "";			//ClientIPAddress
	in_addr MultiCastAddress;						//multicast address
	char MsgChar[MAX_NAMELENGTH];					//Message String
	int port;										//Port Number
	int optval;										//socket option value
	int optval_size;								//socket option value
	SECURITY_ATTRIBUTES security_attribs;			//socker security attributes
	HANDLE CommandListenThread_Handle;				//command socket thread handler
	DWORD CommandListenThread_ID;					//command socket thread id
	int value;										//socket option value
	int retval;										//integer return value from socket functions
	sockaddr_in MySocketAddr;						//client sockaddr struct
	struct ip_mreq Mreq;							//multicast request struct
	HANDLE DataListenThread_Handle;					//data socket thread handler
	DWORD DataListenThread_ID;						//data socket thread id
	sPacket PacketOut;								//Packet to send
	int nTries;										//counter for ping attempts
	FILE *FDR;
	char text[200];
	time_t now;
	struct tm *t;
	char *filename;

	// Initialize flight clock
	float timer = 0.0;	
	int start = 1;
	clock_t begin;

	//prompt
	PrintIntroPrompt();

	/*---------- SIGNAL HANDLING ----------*/
	if (SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE)) { PrintMessage("Signal Handler Installed - Ctrl-C to quit."); }
	else { PrintMessage("Could not set control handler. Exiting."); return 0; }
	/*-------- END SIGNAL HANDLING --------*/

	/*---------- SETUP CLIENT ----------*/
	//status text
	PrintMessage("Setting Up Networking...");

	/*----- IP Address -----*/
	//check socket version
	if (WSAStartup(0x202, &WsaData) == SOCKET_ERROR)	{ PrintMessage("WSAStartup failed. Exiting..."); PrintExitPrompt(); WSACleanup(); return 0; }
	
	//get local server ip
	if (!GetLocalIPAddresses((unsigned long *)&ServerAddress, 1)) { PrintMessage("Failed to get local server ip. Exiting..."); PrintExitPrompt(); WSACleanup(); return 0; }
	ServerAddress.S_un.S_un_b.s_b1 = 192;
	ServerAddress.S_un.S_un_b.s_b2 = 168;
	ServerAddress.S_un.S_un_b.s_b3 = 1;
	ServerAddress.S_un.S_un_b.s_b4 = 97;
	


	sprintf_s(szServerIPAddress, MAX_ADDLENGTH, "%d.%d.%d.%d", ServerAddress.S_un.S_un_b.s_b1, ServerAddress.S_un.S_un_b.s_b2, ServerAddress.S_un.S_un_b.s_b3, ServerAddress.S_un.S_un_b.s_b4);
	
	//get local client ip
	if (!GetLocalIPAddresses((unsigned long *)&MyAddress, 1)) { PrintMessage("Failed to get local client ip. Exiting..."); PrintExitPrompt(); WSACleanup(); return 0; }
	MyAddress.S_un.S_un_b.s_b1 = 192;
	MyAddress.S_un.S_un_b.s_b2 = 168;
	MyAddress.S_un.S_un_b.s_b3 = 1;
	MyAddress.S_un.S_un_b.s_b4 = 97;
	
	sprintf_s(szMyIPAddress, MAX_ADDLENGTH, "%d.%d.%d.%d", MyAddress.S_un.S_un_b.s_b1, MyAddress.S_un.S_un_b.s_b2, MyAddress.S_un.S_un_b.s_b3, MyAddress.S_un.S_un_b.s_b4);
	
	//multicast address
	MultiCastAddress.S_un.S_addr = inet_addr(MULTICAST_ADDRESS);

	//ipaddress status text
	sprintf_s(MsgChar, MAX_NAMELENGTH, "    Client: %s", szMyIPAddress);     PrintMessage(MsgChar);
	sprintf_s(MsgChar, MAX_NAMELENGTH, "    Server: %s", szServerIPAddress); PrintMessage(MsgChar);
	sprintf_s(MsgChar, MAX_NAMELENGTH, "    Multicast Group: %s", MULTICAST_ADDRESS); PrintMessage(MsgChar);
	/*--- End IP Address ---*/

	/*----- Command Socket -----*/
	//port variable
	port = 0;

	//create command socket
	CommandSocket = CreateCommandSocket(MyAddress.S_un.S_addr, port);

	//check for success
	if (CommandSocket == -1) { PrintMessage("    Error Creating Command Socket. Exiting..."); PrintExitPrompt(); WSACleanup(); return 0; }
	else
	{
		//Set Socket Options and Check for Success
		optval = 0x100000;
		optval_size = 4;
		setsockopt(CommandSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
		getsockopt(CommandSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, &optval_size);
		if (optval != 0x100000) { PrintMessage("    Error Setting CommandSocket SO_RCVBUF to 4. Skipping..."); }

		// startup our "Command Listener" thread
		security_attribs.nLength = sizeof(SECURITY_ATTRIBUTES);
		security_attribs.lpSecurityDescriptor = NULL;
		security_attribs.bInheritHandle = TRUE;
		CommandListenThread_Handle = CreateThread(&security_attribs, 0, CommandListenThread, NULL, 0, &CommandListenThread_ID);

		//status text
		PrintMessage("    Command Socket Created.");
	}
	/*--- End Command Socket ---*/

	/*----- Data Socket -----*/

	//create data socket
	DataSocket = socket(AF_INET, SOCK_DGRAM, 0);

	//allow multiple clients on same machine to use address/port
	value = 1;
	retval = setsockopt(DataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(value));
	if (retval == SOCKET_ERROR) { closesocket(DataSocket); PrintMessage("    Error Creating Data Socket. Exiting..."); PrintExitPrompt(); WSACleanup(); return -1; }

	//bind
	memset(&MySocketAddr, 0, sizeof(MySocketAddr));
	MySocketAddr.sin_family = AF_INET;
	MySocketAddr.sin_port = htons(PORT_DATA);
	MySocketAddr.sin_addr = MyAddress;
	if (bind(DataSocket, (struct sockaddr *)&MySocketAddr, sizeof(struct sockaddr)) == SOCKET_ERROR) { PrintMessage("    Error Binding Data Socket. Exiting..."); PrintExitPrompt(); WSACleanup(); return 0; }

	// join multicast group
	Mreq.imr_multiaddr = MultiCastAddress;
	Mreq.imr_interface = MyAddress;
	retval = setsockopt(DataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&Mreq, sizeof(Mreq));
	if (retval == SOCKET_ERROR) { PrintMessage("    Error Joining Multicast. Exiting..."); PrintExitPrompt(); WSACleanup(); return -1; }

	// create a 1MB buffer
	setsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
	getsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, &optval_size);
	if (optval != 0x100000)
	{
		sprintf_s(MsgChar, MAX_NAMELENGTH, "    Data Socket ReceiveBuffer Size = %d.", optval);
		PrintMessage(MsgChar);
	}

	// startup our "Data Listener" thread
	security_attribs.nLength = sizeof(SECURITY_ATTRIBUTES);
	security_attribs.lpSecurityDescriptor = NULL;
	security_attribs.bInheritHandle = TRUE;
	DataListenThread_Handle = CreateThread(&security_attribs, 0, DataListenThread, NULL, 0, &DataListenThread_ID);

	PrintMessage("    Data Socket Created.");
	/*--- End Data Socket ---*/

	/*----- Final Setup -----*/
	// server address for commands
	memset(&HostAddr, 0, sizeof(HostAddr));
	HostAddr.sin_family = AF_INET;
	HostAddr.sin_port = htons(PORT_COMMAND);
	HostAddr.sin_addr = ServerAddress;

	// send initial ping command
	PacketOut.iMessage = NAT_PING;
	PacketOut.nDataBytes = 0;
	nTries = 3;
	while (nTries--)
	{
		retval = sendto(CommandSocket, (char *)&PacketOut, 4 + PacketOut.nDataBytes, 0, (sockaddr *)&HostAddr, sizeof(HostAddr));
		if (retval != SOCKET_ERROR) { break; }
	}
	PrintMessage("    Client Started...");
	/*--- End Final Setup ---*/
	/*-------- END SETUP CLIENT --------*/

	/*---------- OPEN DEVICE ----------*/
	//status text
	PrintMessage(""); PrintBreak(); PrintMessage(""); PrintMessage("Setting Up XBee...");

	//open device
	ftStatus = FT_Open(0, &ftHandle);	//opening USB Device 0, may need changed to whatever number corresponds to XBee
	if (ftStatus != FT_OK) { PrintMessage("    Failed to open device with FT_OPEN. Exiting..."); PrintExitPrompt(); return 0; }
	else { PrintMessage("    Successfully opened device with FT_OPEN."); }

	//setup device
	FT_SetBaudRate(ftHandle, 57600);
	FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
	FT_SetTimeouts(ftHandle, 1, 1);
	/*-------- END OPEN DEVICE --------*/

	/*---------- ACI INIT ----------*/
	aciInit();									//Required initial ACI function call
	aciSetSendDataCallback(&transmit);			//Call when ACI wants to send a data packet
	aciInfoPacketReceivedCallback(&versions);	//call if you request ACI Info packet and it was received
	if (1) {
		aciSetCmdListUpdateFinishedCallback(&cmdListUpdateFinished);
	}
	aciSetVarListUpdateFinishedCallback(&varListUpdateFinished);
	aciSetEngineRate(50, 10);					//ACI Engine called at 100 Hz. Heart beat rate is 10 Hz.
	/*-------- END ACI INIT --------*/

	/*---------- RUN THREAD ----------*/
	//status text
	PrintMessage(""); PrintBreak(); PrintMessage(""); PrintMessage("Running...");

	//create thread
	thHandle = CreateThread(0, 0, aciThread, 0, 0, &thID);

	//get version information
	aciCheckVerConf();

	//get aci command list
	if (1) {
		aciGetDeviceCommandsList();
		PrintMessage("    Waiting for command list...");
		while (!cmd_ready) Sleep(1);
	}

	//get aci variable list
	aciGetDeviceVariablesList();
	PrintMessage("    Waiting for variable list...");
	while (!var_getted) Sleep(1);
	
	/*----- Open Data File -----*/
	PrintMessage("    Opening Flight Data Recorder...");

	// insert the date into the char array
	now = time(NULL);
	t = localtime(&now);
	strftime(text, sizeof(text) - 1, "%Y%m%d-%H%M%S", t);
	text[199] = 0;

	// concat the date to file name
	if ((filename = malloc(strlen(FDR_DIRPATH) + strlen(text) + 5)) != NULL){
		filename[0] = '\0';   // ensures the memory is an empty string
		strcat(filename, FDR_DIRPATH);
		strcat(filename, text);
		strcat(filename, ".csv");
	}
	
	// use the file
	FDR = fopen(filename, "w+");
	fprintf(FDR, "time (s), accel_x (m/s^2), accel_y (m/s^2), accel_z (m/s^2), gyro_x (rad/s), gyro_y (rad/s), gyro_z (rad/s), x (m), y (m), z (m), yaw (rad), pitch (rad), roll (rad), mocap_counter, counter, desired x, desired y, desired z, desired yaw\n");
	PrintMessage("Flight Data Recorder Opened");
	RecHandle = CreateThread(0, 0, RecData, 0, 0, &Record);
	/*--- End Open Data File ---*/

	while (run_program){
		if (data_updated)
		{

			if (1) {
				// Choose desired position and yaw angle
				QuadGoal.Pos.x = 0;
				QuadGoal.Pos.y = 0;
				QuadGoal.Pos.z = 0;
				QuadGoal.yaw = 0.0;

				planner(QuadCurrent, ObstCurrent, &QuadDesired, QuadGoal);
				aciUpdateCmdPacket(0);
				Sleep(1);
			}

			/*if (1) {
				QuadGoal.yaw = 0.0;
				if (timer <= tdes[0]) {
					QuadGoal.Pos.x = xdes[0];
					QuadGoal.Pos.y = ydes[0];
					QuadGoal.Pos.z = zdes[0];
				} else if (timer >= tdes[ndes-1]) {
					QuadGoal.Pos.x = xdes[ndes-1];
					QuadGoal.Pos.y = ydes[ndes-1];
					QuadGoal.Pos.z = zdes[ndes-1];
				} else {
					int i;    // we have to initialize the loop variable before the loop, because
							// the compiler we're using follows the C89 standard (very old)
					for (i=1; i<ndes; i++) {
						if (timer <= tdes[i]) {
							float ratio = (timer - tdes[i-1]) / (tdes[i] - tdes[i-1]);
							QuadGoal.Pos.x = ((1.0 - ratio) * xdes[i-1]) + (ratio * xdes[i]);
							QuadGoal.Pos.y = ((1.0 - ratio) * ydes[i-1]) + (ratio * ydes[i]);
							QuadGoal.Pos.z = ((1.0 - ratio) * zdes[i-1]) + (ratio * zdes[i]);
							break;
						}
					}
				}

				// run the planner to update the desired position
				planner(QuadCurrent, ObstCurrent, &QuadDesired, QuadGoal);

				// send the desired position to the drone
				aciUpdateCmdPacket(0);
				Sleep(1);
			}*/
			if (StartREC == 1){
				//Compute Flight Time
				if (start == 1) {
					begin = clock();
					start = 0;
				}
				else {
					timer = (float)(clock() - begin) / CLOCKS_PER_SEC;
				}
				fprintf(FDR, "%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %d, %d, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\n",
							 timer,
							 ((float)accel_x)*9.80665 / 10000, ((float)accel_y)*9.80665 / 10000, ((float)accel_z)*9.80665 / 10000,
							 ((float)gyro_x)*0.0154*PI_f / 180.0, ((float)gyro_y)*0.0154*PI_f / 180.0, ((float)gyro_z)*0.0154*PI_f / 180.0,
							 QuadCurrent.Pos.x, QuadCurrent.Pos.y, QuadCurrent.Pos.z,
							 QuadCurrent.Ori.yaw, QuadCurrent.Ori.pitch, QuadCurrent.Ori.roll, mocap_counter, onboard_counter, QuadDesired.Pos.x, QuadDesired.Pos.y, QuadDesired.Pos.z, QuadDesired.yaw, ObstCurrent.Pos.x, ObstCurrent.Pos.y, ObstCurrent.Pos.z, QuadGoal.Pos.x, QuadGoal.Pos.y, QuadGoal.Pos.z);
				printf("Position (x, y, z): %0.4f %0.4f %0.4f\tOrientation (yaw, pitch, roll): %0.4f %0.4f %0.4f\r",
					   QuadCurrent.Pos.x, QuadCurrent.Pos.y, QuadCurrent.Pos.z,
					   QuadCurrent.Ori.yaw, QuadCurrent.Ori.pitch, QuadCurrent.Ori.roll);
			}
			data_updated = 0;
		}
	}

	/*-------- END RUN THREAD --------*/

	fclose(FDR);
	PrintExitPrompt();
	return 0;
}
/*----------------------------------------------------------------------*/
/*------------------------------ End Main ------------------------------*/
/*----------------------------------------------------------------------*/