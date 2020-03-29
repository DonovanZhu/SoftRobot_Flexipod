#include <iostream>
#include <iomanip>
#include <WS2tcpip.h>
#include <string>
#include <windows.h>
#include <conio.h>
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

	char s[10] = {0x80, 0x01, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00};
	int16_t motor1_command = 8192, motor2_command = 8192, motor3_command = 8192, motor4_command = 8192;
	int kbinput;
	int speedcommand;
	double speed;
	while (true)
	{
		// cout << "enter the number:" << endl;
		// cout << (int16_t)s[0] << endl;
		// cin >> speed_command;
		if (_kbhit())
		{
			kbinput = _getch();
			if (kbinput == 'q')
				motor1_command += 256;
			else if (kbinput == 'a')
				motor1_command -= 256;
			else if (kbinput == 'w')
				motor2_command += 256;
			else if (kbinput == 's')
				motor2_command -= 256;
			else if (kbinput == 'e')
				motor3_command += 256;
			else if (kbinput == 'd')
				motor3_command -= 256;
			else if (kbinput == 'r')
				motor4_command += 256;
			else if (kbinput == 'f')
				motor4_command -= 256;
		}

		s[9] = motor4_command & 0x00ff;
		s[8] = (motor4_command >> 8) & 0x00ff;
		s[7] = motor3_command & 0x00ff;
		s[6] = (motor3_command >> 8) & 0x00ff;
		s[5] = motor2_command & 0x00ff;
		s[4] = (motor2_command >> 8) & 0x00ff;
		s[3] = motor1_command & 0x00ff;
		s[2] = (motor1_command >> 8) & 0x00ff;
		//cout << (int16_t)s[8] << endl;
		 
		// for (int i = 0; i < 10; i++)
			//cout << hex << s[i] << " ";
		// cout << endl;
		// cout << "Input the command" << endl;
		// for (int j = 2; j < 10; j++)
			// cin >> hex >> s[j];
		// for (int k = 0; k < 10; k++)
			// s2[k] = s[k];
		// s = to_string(command);
		sendto(out, s, 10, 0, (sockaddr*)&server, sizeof(server));
		for (int i = 2; i < 10; i += 2)
		{
			speedcommand = 0;
			speedcommand |= (int16_t)(unsigned char)s[i] << 8;
			speedcommand |= (int16_t)(unsigned char)s[i + 1];
			speed = (double)(speedcommand - 8192.0) / 8192.0 * 400.0;
			printf("%f", speed);
			printf("	");
			// cout << "0x" << setfill('0') << setw(2) << hex << (int16_t)(unsigned char)s[i] << " ";
		}
		cout << endl;
		// cout << x << endl;
		// command += 1.0;
		Sleep(50);
	}

	closesocket(out);

	WSACleanup();
}
