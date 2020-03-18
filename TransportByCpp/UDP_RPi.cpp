#include <iostream>
#include <WS2tcpip.h>
#include <thread>
#include <string>
#include <windows.h>

// Include the Winsock library (lib) file
#pragma comment (lib, "ws2_32.lib")

using namespace std;

void recv_from_PC();
void send_to_PC();

void recv_from_PC()
{
	WSADATA data;

	WORD version = MAKEWORD(2, 2);

	// Start WinSock
	int wsOk = WSAStartup(version, &data);
	if (wsOk != 0)
	{
		cout << "Can't start Winsock! " << wsOk;
		return;
	}

	// Create a socket, notice that it is a user datagram socket (UDP)
	// AF_INET/AF_INET6: IPV4/IPV6 | SOCK_DGRAM/SOCK_STREAM: UDP/TCP | 0: default protocol
	SOCKET in = socket(AF_INET, SOCK_DGRAM, 0);

	// Create a server hint structure for the server
	sockaddr_in serverHint;
	serverHint.sin_addr.S_un.S_addr = ADDR_ANY; // Us any IP address available on the machine
	serverHint.sin_family = AF_INET; // Address format is IPv4
	serverHint.sin_port = htons(55000); // Convert from little to big endian

	// Try and bind the socket to the IP and port
	/*
	if (bind(in, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR)
	{
		cout << "Can't bind socket! " << WSAGetLastError() << endl;
		return;
	}*/

	sockaddr_in client; // Use to hold the client information (port / ip address)
	int clientLength = sizeof(client); // The size of the client information

	char buf[1024];

	// Enter a loop
	int bytesIn;
	while (true)
	{
		ZeroMemory(&client, clientLength); // Clear the client structure
		ZeroMemory(buf, 1024); // Clear the receive buffer

		// Wait for message
		bytesIn = recvfrom(in, buf, 1024, 0, (sockaddr*)&client, &clientLength);
		/*
		if (bytesIn == SOCKET_ERROR)
		{
			cout << "Error receiving from client " << WSAGetLastError() << endl;
			continue;
		}*/

		// Display message and client info
		char clientIp[256]; // Create enough space to convert the address byte array
		ZeroMemory(clientIp, 256); // to string of characters

		// Convert from byte array to chars
		inet_ntop(AF_INET, &client.sin_addr, clientIp, 256);

		// Display the message / who sent it
		cout << "Message recv from " << clientIp << " : " << buf << endl;
	}

	closesocket(in);

	WSACleanup();
}

void send_to_PC()
{
	WSADATA data;

	WORD version = MAKEWORD(2, 2);

	// Start WinSock
	int wsOk = WSAStartup(version, &data);
	/*
	if (wsOk != 0)
	{
		// Not ok! Get out quickly
		cout << "Can't start Winsock! " << wsOk;
		return;
	}*/

	// Create a hint structure for the server
	sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_port = htons(54000);
	inet_pton(AF_INET, "192.168.0.67", &server.sin_addr);

	SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);

	// Write out to that socket
	string s;
	int sendOk;
	int x = 0;
	while (true)
	{
		s = to_string(x);
		sendOk = sendto(out, s.c_str(), s.size() + 1, 0, (sockaddr*)&server, sizeof(server));
		cout << "send:" << x << endl;
		x++;
		//Sleep(1000);
		/*
		if (sendOk == SOCKET_ERROR)
		{
			cout << "That didn't work! " << WSAGetLastError() << endl;
		}*/
	}
	closesocket(out);

	WSACleanup();
}

void main()
{
	thread send(send_to_PC);
	thread recv(recv_from_PC);
	send.join();
	recv.join();
}
