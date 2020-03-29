#include <iostream>
#include <iomanip>
#include <WS2tcpip.h>

// Include the Winsock library (lib) file
#pragma comment (lib, "ws2_32.lib")

// Saves us from typing std::cout << etc. etc. etc.
using namespace std;

// Main entry point into the server
void main()
{

	WSADATA data;

	WORD version = MAKEWORD(2, 2);

	// Start WinSock
	int wsOk = WSAStartup(version, &data);
	if (wsOk != 0)
	{
		// Not ok! Get out quickly
		cout << "Can't start Winsock! " << wsOk;
		return;
	}

	SOCKET in = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in serverHint;
	serverHint.sin_addr.S_un.S_addr = ADDR_ANY; // Us any IP address available on the machine
	serverHint.sin_family = AF_INET; // Address format is IPv4
	serverHint.sin_port = htons(20000); // Convert from little to big endian

	bind(in, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR;

	sockaddr_in client; // Use to hold the client information (port / ip address)
	int clientLength = sizeof(client); // The size of the client information

	char buf[100];
	unsigned short k;
	int output = 0;
	double speed;
	while (true)
	{
		
		// ZeroMemory(buf, 10); // Clear the receive buffer
		int datalength = recvfrom(in, buf, 100, 0, (sockaddr*)&client, &clientLength);
		for (int i = 0; i < datalength; i++)
		{
			cout << buf[i];
		}
		cout << endl;
		/*
		if (output % 20 == 0)
		{
			for (int i = 2; i < 10; i += 2)
			{
				k = 0;
				k |= (int16_t)(unsigned char)buf[i] << 8;
				k |= (int16_t)(unsigned char)buf[i + 1];
				speed = (double)(k - 0x2000) / 8192.0 * 400.0;
				printf("%04X", k);
				printf("	");
			}
			printf("\n");
			output = 0;
		}
		output++;
		*/
		// printf("%d\n", buf);
	}

	// Close socket
	closesocket(in);

	// Shutdown winsock
	WSACleanup();
}
