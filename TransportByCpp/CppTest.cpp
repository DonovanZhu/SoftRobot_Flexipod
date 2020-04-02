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
#include <iomanip>

using namespace std;
int main()
{
	int fd;/*File Descriptor*/

	fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY);		                                        
									
	if(fd == -1)
		cout << "Error! in Opening ttyACM0" << endl;
	else
		cout << "ttyACM0 Opened Successfully" << endl;
		
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

	if(tcsetattr(fd,TCSANOW,&SerialPortSettings) != 0) /* Set the attributes to the termios structure*/
	    cout << "ERROR ! in Setting attributes" << endl;
	else
        cout << "BaudRate = 9600 \n  StopBits = 1 \n  Parity = none" << endl;
			

	tcsetattr(fd, TCIFLUSH, &SerialPortSettings);   /* Discards old data in the rx buffer            */

	char read_buffer[32];   /* Buffer to store the data received              */
	int  bytes_read = 0;    /* Number of bytes read by the read() system call */

	char readfromTeensy[12];
	//int8_t led; 
	while (1)
	{
		bytes_read = read(fd,&readfromTeensy,12); /* Read the data                   */
		
		for(int i=0;i<bytes_read;i++)	 /*printing only the received characters*/
		 	cout << setfill('0') << setw(2) << hex << (int16_t)(unsigned char)readfromTeensy[i] << " ";
		cout << endl;

	}
	cout << "\n +----------------------------------+\n\n\n" << endl;

	close(fd); /* Close the serial port */
	return 0;
}

