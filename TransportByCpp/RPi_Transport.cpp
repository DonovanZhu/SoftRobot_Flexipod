#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <stdio.h>
#include <iostream>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <errno.h>   /* ERROR Number Definitions           */

using namespace std;

void sendUDP(int fd)
{
	int sock_send;
	unsigned int length_send;
	struct sockaddr_in server_send;
	struct hostent *hp;
	double data = 0.0;
	char read_buffer[32];
	int bytes_read;

	sock_send= socket(AF_INET, SOCK_DGRAM, 0);

	server_send.sin_family = AF_INET;
	hp = gethostbyname("192.168.0.67");

	bcopy((char *)hp->h_addr, (char *)&server_send.sin_addr, hp->h_length);
	server_send.sin_port = htons(20000);
	length_send=sizeof(struct sockaddr_in);
	while (true)
	{
		bytes_read = read(fd,&read_buffer,32);
		sendto(sock_send, read_buffer, bytes_read, 0, (const struct sockaddr *)&server_send, length_send);
		printf("%s\n", read_buffer);
	}
	close(sock_send);
	return;
}

void recvUDP(int fd)
{
	int sock_recv, length_recv;
	socklen_t fromlen;
	struct sockaddr_in server_recv;
	struct sockaddr_in from;
	char buf[32];
	
	sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
	
	length_recv = sizeof(server_recv);
	bzero(&server_recv,length_recv);
	
	server_recv.sin_family = AF_INET;
	server_recv.sin_addr.s_addr = INADDR_ANY;
	server_recv.sin_port = htons(10000);
	bind(sock_recv,(struct sockaddr *)&server_recv,length_recv);
		   
	fromlen = sizeof(struct sockaddr_in);
	while (true) 
	{
		recvfrom(sock_recv, buf, 32, 0, (struct sockaddr *)&from, &fromlen);
		write(fd,buf,sizeof(buf));
		// printf("%s\n", buf);
	}
	return;
}

int main()
{	int fd;/*File Descriptor*/

	fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY);		                                        
									
	if(fd == -1)
		cout << "Error! in Opening ttyACM0" << endl;
	else
		cout << "ttyUSB0 Opened Successfully" << endl;
		
	struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
	cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */

	/* 8N1 Mode */
	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
		
	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
		
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

	/* Setting Time outs */
	SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
	SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
	    cout << "ERROR ! in Setting attributes" << endl;
	else
        cout << "BaudRate = 9600 \n  StopBits = 1 \n  Parity = none" << endl;
			

	tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
	pid_t pid = fork();
	if (pid > 0)
	{
		sendUDP(fd);
	}
	else if(pid == 0)
	{
		recvUDP(fd);
	}
	return 0;
}
