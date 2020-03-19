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

using namespace std;

void sendUDP()
{
   int sock_send;
   unsigned int length_send;
   struct sockaddr_in server_send;
   struct hostent *hp;
   double data = 0.0;
   string udpMessage;
   
   sock_send= socket(AF_INET, SOCK_DGRAM, 0);

   server_send.sin_family = AF_INET;
   hp = gethostbyname("192.168.0.67");

   bcopy((char *)hp->h_addr, (char *)&server_send.sin_addr, hp->h_length);
   server_send.sin_port = htons(20000);
   length_send=sizeof(struct sockaddr_in);
   while (true)
   {
      udpMessage = to_string(data);
      sendto(sock_send, udpMessage.c_str(), udpMessage.size() + 1, 0, (const struct sockaddr *)&server_send, length_send);
      data += 1.0;
   }
   close(sock_send);
   return;
}

void recvUDP()
{
   int sock_recv, length_recv;
   socklen_t fromlen;
   struct sockaddr_in server_recv;
   struct sockaddr_in from;
   char buf[1024];
   
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
      recvfrom(sock_recv, buf, 1024, 0, (struct sockaddr *)&from, &fromlen);
      printf("%s\n", buf);
   }
   return;
}

int main()
{
	pid_t pid = fork();
	if (pid > 0)
	{
		sendUDP();
	}
	else if(pid == 0)
	{
		recvUDP();
	}
	return 0;
}
