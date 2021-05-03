#include <iostream>
#include <iomanip>
#include <WS2tcpip.h>
#include <msgpack.hpp>
#include <vector>
#include <string>
#include <msgpack.hpp>
#include <fstream>

// Include the Winsock library (lib) file
#pragma comment (lib, "ws2_32.lib")

using namespace std;

// Using this class to hold data from RPi
class MsgToPC {
public:
	double rpm[4];
	MSGPACK_DEFINE(rpm);
};

void main()
{

	// UDP initializing
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
	serverHint.sin_port = htons(1000); // Convert from little to big endian

	bind(in, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR;

	sockaddr_in client; // Use to hold the client information (port / ip address)
	int clientLength = sizeof(client); // The size of the client information

	char buf[200];
	unsigned short k;
	int output = 0;
	int UDPrecv;

	int step = 0;
	double x = 0.0;
	while (true)
	{
		auto start = chrono::high_resolution_clock::now();
		int datalength = recvfrom(in, buf, 200, 0, (sockaddr*)&client, &clientLength);
		auto end = chrono::high_resolution_clock::now();
		double time_taken = chrono::duration_cast<chrono::nanoseconds>(end - start).count();

		time_taken *= 1e-9;
		printf("%f\t", time_taken);
		if (time_taken > 0.001) {
			x = time_taken;
		}
		printf("%f\n", x);
		msgpack::object_handle oh = msgpack::unpack(buf, datalength);

		msgpack::object obj = oh.get();
		// you can convert object to UDPdata class directly
		std::vector<MsgToPC> rvec;
		obj.convert(rvec);


	}

	// Close socket
	closesocket(in);

	// Shutdown winsock
	WSACleanup();
}
