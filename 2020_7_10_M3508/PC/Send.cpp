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
	double command[2];
	MSGPACK_DEFINE(command);
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
	server.sin_port = htons(10); // Little to big endian conversion
	inet_pton(AF_INET, "192.168.8.136", &server.sin_addr); // Convert from string to byte array

	SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);

	double speed_command_x = 0.0;
	double speed_command_y = 0.0;
	int kbinput;
	vector<MotorSpeed> SendCommand;

	int j = 0;
	RotationSpeed.command[0] = 0;
	RotationSpeed.command[1] = 0;
	while (true)
	{
		if (_kbhit())
		{
			kbinput = _getch();
			if (kbinput == 'w') {
				speed_command_y += 10.0;
			}
			else if (kbinput == 'x') {
				speed_command_y -= 10.0;
			}
			else if (kbinput == 'a') {
				speed_command_x -= 10.0;
			}
			else if (kbinput == 'd') {
				speed_command_x += 10.0;
			}
			else if (kbinput == 's')
			{
				speed_command_x = 0.0;
				speed_command_y = 0.0;
			}
		}

		if (j > 2000)
		{
			system("cls");
			j = 0;
		}
		j++;

		if (speed_command_x > 500.0) {
			speed_command_x = 500.0;
		}

		if (speed_command_x < -500.0) {
			speed_command_x = -500.0;
		}

		if (speed_command_y > 500.0) {
			speed_command_y = 500.0;
		}

		if (speed_command_y < -500.0) {
			speed_command_y = -500.0;
		}

		if (fabs(speed_command_y) >= 0.01 && fabs(speed_command_x) / fabs(speed_command_y) >= 1.0) {
			if (speed_command_x > 0.0) {
				speed_command_x = fabs(speed_command_y);
			}
			else {
				speed_command_x = -fabs(speed_command_y);
			}
		}

		RotationSpeed.command[0] = speed_command_x;
		RotationSpeed.command[1] = speed_command_y;

		SendCommand.clear();
		SendCommand.push_back(RotationSpeed);

		msgpack::sbuffer sbuf;
		msgpack::pack(sbuf, SendCommand);

		if (fabs(speed_command_x) <= 0.01 && fabs(speed_command_y) <= 0.01)
			printf("Stop				");

		else if(fabs(speed_command_x) <= 0.01 && speed_command_y > 0.01)
			printf("Go head				");

		else if (fabs(speed_command_x) <= 0.01 && speed_command_y < -0.01)
			printf("Go back				");

		else if (speed_command_x < - 0.01 && fabs(speed_command_y) <= 0.01)
			printf("Turn left in place	");

		else if (speed_command_x > 0.01 && fabs(speed_command_y) <= 0.01)
			printf("Turn right in place	");

		else if (speed_command_x < -0.01 && speed_command_y > 0.01)
			printf("Go head Turn left	");

		else if (speed_command_x > 0.01 && speed_command_y > 0.01)
			printf("Go head Turn right	");

		else if (speed_command_x < -0.01 && speed_command_y < -0.01)
			printf("Go back Turn left	");

		else if (speed_command_x > 0.01 && speed_command_y < -0.01)
			printf("Go back Turn right	");

		printf("%f\t%f\n", RotationSpeed.command[0], RotationSpeed.command[1]);

		sendto(out, sbuf.data(), sbuf.size(), 0, (sockaddr*)&server, sizeof(server));

	}
	closesocket(out);

	WSACleanup();
}
