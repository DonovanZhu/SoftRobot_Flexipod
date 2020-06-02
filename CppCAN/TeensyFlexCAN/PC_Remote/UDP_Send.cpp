#include <iostream>
#include <iomanip>
#include <WS2tcpip.h>
#include <string>
#include <windows.h>
#include <conio.h>
#include <vector>
#include <string>
#include <msgpack.hpp>


// Include the Winsock library (lib) file
#pragma comment (lib, "ws2_32.lib")

// Saves us from typing std::cout << etc. etc. etc.
using namespace std;

class MotorSpeed {
public:
	double rpm[4];
	MSGPACK_DEFINE(rpm);
};

MotorSpeed RotationSpeed;

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
	server.sin_port = htons(1000); // Little to big endian conversion
	inet_pton(AF_INET, "192.168.0.77", &server.sin_addr); // Convert from string to byte array

	SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);

	char s[4] = {0.0, 0.0, 0.0, 0.0};

	double motor1_command = 200.0, motor2_command = 0.0, motor3_command = 0.0, motor4_command = 0.0;
	int kbinput;
	int speedcommand;
	double speed;
	vector<MotorSpeed> SendCommand;

	while (true)
	{
		// cout << "enter the number:" << endl;
		// cout << (int16_t)s[0] << endl;
		// cin >> speed_command;
		if (_kbhit())
		{
			kbinput = _getch();
			if (kbinput == 'q')
				motor1_command += 200.0;
			else if (kbinput == 'a')
				motor1_command -= 200.0;
			else if (kbinput == 'w')
				motor2_command += 200.0;
			else if (kbinput == 's')
				motor2_command -= 200.0;
			else if (kbinput == 'e')
				motor3_command += 200.0;
			else if (kbinput == 'd')
				motor3_command -= 200.0;
			else if (kbinput == 'r')
				motor4_command += 200.0;
			else if (kbinput == 'f')
				motor4_command -= 200.0;
		}
		RotationSpeed.rpm[0] = motor1_command;
		RotationSpeed.rpm[1] = motor2_command;
		RotationSpeed.rpm[2] = motor3_command;
		RotationSpeed.rpm[3] = motor4_command;
		SendCommand.clear();
		SendCommand.push_back(RotationSpeed);

		msgpack::sbuffer sbuf;
		msgpack::pack(sbuf, SendCommand);
		for (int i = 0; i < 4; i++)
		{
			printf("%f", RotationSpeed.rpm[i]);
			printf("	");
			// cout << "0x" << setfill('0') << setw(2) << hex << (int16_t)(unsigned char)s[i] << " ";
		}
		sendto(out, sbuf.data(), sbuf.size(), 0, (sockaddr*)&server, sizeof(server));

		cout << endl;
		// cout << x << endl;
		// command += 1.0;
	}
	//Sleep(10);
	closesocket(out);

	WSACleanup();
}
