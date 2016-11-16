/*
/////////////////////////////////////////////////
ver.0.06版
-不限定從零開始存深度值
-刪除不必要的迴圈
/////////////////////////////////////////////////
*/



#include "UDP.h"

#include<iostream>
#include<fstream>
#include<stdio.h>
#include<winsock2.h>
#include<string>
#include<math.h>

using namespace std;

#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFLEN 1206  //Max length of buffer
#define PORT 2368   //The port on which to listen for incoming data
#define TRFC 4294967040
#define PI 3.14159265359

float UDP(float matrix[16][1800],float (&AZ)[900])
{
	SOCKET s;
	struct sockaddr_in server, si_other;
	int slen, recv_len;
	char buf[BUFLEN];
	WSADATA wsa;

	/////////////////////////////////
	//float matrix[16][1800];
	/////////////////////////////////
	slen = sizeof(si_other);

	//Initialise winsock
	//printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		system("pause");
		exit(EXIT_FAILURE);
	}
	//printf("Initialised.\n");
	
	//Create a socket
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		printf("Could not create socket : %d", WSAGetLastError());
	}
	//printf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT);

	//Bind
	if (bind(s, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
	{
		printf("Bind failed with error code : %d", WSAGetLastError());
		system("pause");
		exit(EXIT_FAILURE);
	}
	//puts("Bind done");
	/////////////////////////////////
	int x = 0;
	int y = 0;
	int z = 0;
	int flag = 0;
	int first = 0;
	float A =0;
	////////////////////////////////
	//keep listening for data
	while (1)
	{
		int ps = 0;
		//printf("Waiting for data...");
		fflush(stdout);

		//clear the buffer by filling null, it might have previously received data
		memset(buf, '\0', BUFLEN);

		//try to receive some data, this is a blocking call
		if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
		{
			printf("recvfrom() failed with error code : %d", WSAGetLastError());
			system("pause");
			exit(EXIT_FAILURE);
		}
		
		//printf("Data length: %d bytes\n", recv_len);
		//print details of the client/peer and the data received
		//printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
		//printf("Data: %s\n", buf);

		//轉換封包成無號整數/////////////////////////////////////////////////////////////////
		int t[1206];
		for (int i = 0; i < 1206; i++)
		{
			if ((unsigned int)buf[i]>40000000)
				t[i] = (unsigned int)buf[i] - TRFC; //將誤判減去FFFFFF00
			else
				t[i] = (unsigned int)buf[i];

		}		
		if (flag == 0)
		{					
			flag = 1;
			ps = 0;			
		}
		if (flag == 1)
		{
			//每100個byte為兩次掃描 傳一次有1200byte
			for (int j = ps; j < 1200; j += 100)
			{
				A = t[3 + j] * 256 + t[2 + j];
				A = A / 100;
				AZ[z] = A;
				z++;
				//printf("A = %.2f j = %d ps = %d \n", A ,j,ps);
				for (int i = 0; i < 94; i += 3) //取出第5個和第6個byte做運算得到距離
				{					
					float tmp = 0;
					tmp = (float)t[j + 4 + i] + ((float)t[j + 5 + i] * 256);
					tmp = tmp / 500; // T*2/1000
					matrix[x][y] = tmp;
					x++;
					if (x == 16)
					{
						x = 0;
						y++;
					}
					if (y == 1800)
					{
						break;
					}
				}			
				if (y == 1800)
				{
					break;
				}

			}
			if (y == 1800)
			{
				break;
			}
		}		
		
	}
	closesocket(s);
	for (int i = 0; i < 1800; i++) //將Layer放到正確的位置
	{
		float swapt;
		swapt = matrix[1][i];
		matrix[1][i] = matrix[13][i];
		matrix[13][i] = matrix[4][i];
		matrix[4][i] = matrix[7][i];
		matrix[7][i] = swapt;

		swapt = matrix[2][i];
		matrix[2][i] = matrix[11][i];
		matrix[11][i] = matrix[8][i];
		matrix[8][i] = matrix[14][i];
		matrix[14][i] = swapt;

		swapt = matrix[3][i];
		matrix[3][i] = matrix[9][i];
		matrix[9][i] = matrix[12][i];
		matrix[12][i] = matrix[6][i];
		matrix[6][i] = swapt;

		swapt = matrix[15][i];
		matrix[15][i] = matrix[0][i];
		matrix[0][i] = swapt;
	}

	return matrix[16][1800];
	
}