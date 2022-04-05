#include "make_tls_server.h"
#include "tls_common_lib.h"
#include "netconstants.h"
#include "constants.h"
#include "packet.h"
#include "serial.h"
#include "serialize.h"

#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600
#define SERVER_PORT			5000
        
#define Alexs_private_key "alex.key"
#define certificate "alex.crt"
#define CA_cert "signing.pem"
#define Common_name "SU_CS1231"

// Our network buffer consists of 1 byte of packet type, and 128 bytes of data
#define BUF_LEN				129

// This variable shows whether a network connection is active
// We will also use this variable to prevent the server from serving
// more than one connection, to keep connection management simple.
static volatile int networkActive;

// This variable is used by sendNetworkData to send back responses
// to the TLS connection.  It is sent by handleNetworkData
static void *tls_conn = NULL;

/*
 * Alex Serial Routines to the Arduino
 */
// i think we dont need this
// void sendNetworkData(const char *, int);

void handleErrorResponse(TPacket *packet)
{
	printf("UART ERROR: %d\n", packet->command);
	char buffer[2];
	buffer[0] = NET_ERROR_PACKET;
	buffer[1] = packet->command;
	sendNetworkData(buffer, sizeof(buffer));
}

void handleMessage(TPacket *packet)
{
	char data[33];
	printf("UART MESSAGE PACKET: %s\n", packet->data);
	data[0] = NET_MESSAGE_PACKET;
	memcpy(&data[1], packet->data, sizeof(packet->data));
	sendNetworkData(data, sizeof(data));
}

void handleStatus(TPacket *packet)
{
	char data[65];
	printf("UART STATUS PACKET\n");
	data[0] = NET_STATUS_PACKET;
	memcpy(&data[1], packet->params, sizeof(packet->params));
	sendNetworkData(data, sizeof(data));
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			char resp[2];
			printf("Command OK\n");
			resp[0] = NET_ERROR_PACKET;
			resp[1] = RESP_OK;
			sendNetworkData(resp, sizeof(resp));
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		default:
		printf("Boo\n");
	}
}


void handleUARTPacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}


void uartSendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));
	printf("sending packet \n");

	serialWrite(buffer, len);
}

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

void *uartReceiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handleUARTPacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				} // result
		} // len > 0
	} // while
}

/*
 * Alex Network Routines
 */
void sendNetworkData(const char *data, int len)
{
	// Send only if network is active
	if(networkActive)
	{
        // Use this to store the number of bytes actually written to the TLS connection.
        int c;

		printf("WRITING TO CLIENT\n");
 
        if(tls_conn != NULL) {
              c = sslWrite(tls_conn,data,len);
        }
		// Network is still active if we can write more then 0 bytes.
		networkActive = (c > 0);
	}
}

void handleCommand(void *conn, const char *buffer)
{
	// The first byte contains the command
	char cmd = buffer[1];
	uint32_t cmdParam[2];

	// Copy over the parameters.
	memcpy(cmdParam, &buffer[2], sizeof(cmdParam));

	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;
	commandPacket.params[0] = cmdParam[0];
	commandPacket.params[1] = cmdParam[1];
	printf("%d, %d",commandPacket.params[0], commandPacket.params[1]);
	printf("COMMAND RECEIVED: %c %d %d\n", cmd, cmdParam[0], cmdParam[1]);
	
	switch(cmd)
	{
		case 'f':
		case 'F':
			commandPacket.command = COMMAND_FORWARD;
			uartSendPacket(&commandPacket);
			break;

		case 'b':
		case 'B':
			commandPacket.command = COMMAND_REVERSE;
			uartSendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			commandPacket.command = COMMAND_TURN_LEFT;
			uartSendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			commandPacket.command = COMMAND_TURN_RIGHT;
			uartSendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			uartSendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			uartSendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			uartSendPacket(&commandPacket);
			break;

		default:
			printf("Bad command\n");

	}
}

void handleNetworkData(void *conn, const char *buffer, int len)
{
	// conn stored in global variable tls_conn
        tls_conn = conn; // This is used by sendNetworkData

	if(buffer[0] == NET_COMMAND_PACKET)
		handleCommand(conn, buffer);
}

void *worker(void *conn)
{
	int len;

	char buffer[BUF_LEN];
	
	while(networkActive)
	{
		len = sslRead(conn,buffer,sizeof(buffer));
		
		// As long as we are getting data, network is active
		networkActive=(len > 0);

		if(len > 0)
			handleNetworkData(conn, buffer, len);
		else
			if(len < 0)
				perror("ERROR READING NETWORK: ");
	}

  	// Reset tls_conn to NULL.
	tls_conn = NULL;
	EXIT_THREAD(conn);
}

void sendHello()
{
	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	uartSendPacket(&helloPacket);
}

int main()
{
	// Start the uartReceiveThread. The network thread is started by
        // createServer

	pthread_t serThread;

	printf("\nALEX REMOTE SUBSYSTEM\n\n");

	printf("Opening Serial Port\n");
	// Open the serial port
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);
	printf("Done. Waiting 3 seconds for Arduino to reboot\n");
	sleep(3);

	printf("DONE. Starting Serial Listener\n");
	pthread_create(&serThread, NULL, uartReceiveThread, NULL);
	
	printf("Starting Alex Server\n");
	networkActive = 1;
	
	// client authentication, send Alex's certificate
	createServer(Alexs_private_key,certificate,5000,&worker,CA_cert,Common_name, 1);

	printf("DONE. Sending HELLO to Arduino\n");
	sendHello();
	printf("DONE.\n");
	
	// Loop while the server is active
	while(server_is_running());
}
