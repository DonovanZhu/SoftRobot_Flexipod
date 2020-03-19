#include <iostream>
#include <WS2tcpip.h>
#include <string>
#include <windows.h>
// Include the Winsock library (lib) file
#pragma comment (lib, "ws2_32.lib")

// Saves us from typing std::cout << etc. etc. etc.
using namespace std;

void main() 
{
	WSADATA data;

	WORD version = MAKEWORD(2, 2);

	int wsOk = WSAStartup(version, &data);
	if (wsOk != 0)
	{
		// Not ok! Get out quickly
		cout << "Can't start Winsock! " << wsOk;
		return;
	}

	sockaddr_in server;
	server.sin_family = AF_INET; // AF_INET = IPv4 addresses
	server.sin_port = htons(10000); // Little to big endian conversion
	inet_pton(AF_INET, "192.168.0.77", &server.sin_addr); // Convert from string to byte array

	SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);

	string s;
	int sendOk;
	double command = 0.0;
	while (true)
	{
		cout << "Input the command" << endl;
		cin >> s;
		// s = to_string(command);
		sendto(out, s.c_str(), s.size() + 1, 0, (sockaddr*)&server, sizeof(server));
		// cout << x << endl;
		// command += 1.0;
	}

	closesocket(out);

	WSACleanup();
}
