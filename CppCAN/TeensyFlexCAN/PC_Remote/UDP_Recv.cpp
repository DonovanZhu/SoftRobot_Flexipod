#include <iostream>
#include <iomanip>
#include <WS2tcpip.h>
#include <msgpack.hpp>
#include <vector>
#include <string>
#include <msgpack.hpp>


// Include the Winsock library (lib) file
#pragma comment (lib, "ws2_32.lib")

using namespace std;
class MotorData {
public:
	double angle[4];
	double rpm[4];
	double torque[4];
	MSGPACK_DEFINE(angle, rpm, torque);
};

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
	serverHint.sin_port = htons(2000); // Convert from little to big endian

	bind(in, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR;

	sockaddr_in client; // Use to hold the client information (port / ip address)
	int clientLength = sizeof(client); // The size of the client information

	char buf[200];
	unsigned short k;
	int output = 0;
	int UDPrecv;

	int step = 0;
	while (true)
	{

		int datalength = recvfrom(in, buf, 200, 0, (sockaddr*)&client, &clientLength);

		msgpack::object_handle oh = msgpack::unpack(buf, datalength);

		msgpack::object obj = oh.get();
		// you can convert object to UDPdata class directly
		std::vector<MotorData> rvec;
		obj.convert(rvec);

		/*
		for (int i = 0; i < 4; i++)
		{
			printf("%f", rvec[0].angle[i]);
			printf(" ");
		}
		printf("\n");

		for (int i = 0; i < 4; i++)
		{
			printf("%f", rvec[0].rpm[i]);
			printf(" ");
		}
		printf("\n");

		for (int i = 0; i < 4; i++)
		{
			printf("%f", rvec[0].torque[i]);
			printf(" ");
		}
		printf("\n");
		*/
		
		if (step % 20 == 0)
		{
			for (int i = 0; i < 4; i++)
			{
				printf("%f", rvec[0].rpm[i]);
				printf(" ");
			}
			printf("\n");
			step = 0;
		}
		step++;
		

	}

	// Close socket
	closesocket(in);

	// Shutdown winsock
	WSACleanup();
}
