
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "dtof_user.h"
#include <pthread.h>
#include <arpa/inet.h>

#define DTOF_BUF_SIZE 4900
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 384

// The buffer stores real-time refreshed UDP packets sent by the dToF
static uint8_t message[DTOF_BUF_SIZE] = {0};
int serv_sock;
struct sockaddr_in serv_addr, clnt_addr;
socklen_t clnt_addr_size;
int str_len = 0;
// float distance = 0.0, distance_prev = 0.0;
uint16_t px, py;    //proportion
char ch;
int status1111111111;

uint8_t dtof_Init(void)
{
    system("cd /ko && insmod usbnet.ko && insmod cdc_ether.ko && insmod rndis_host.ko");
    usleep(50000);
    system("ifconfig usb0 192.168.137.100");
    usleep(50000);

    px = round(CAMERA_WIDTH/40.0);
    py = round(CAMERA_HEIGHT/30.0);

    serv_sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (serv_sock == -1) 
    {
        printf("socket error");
        return 1;
    }

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr("192.168.137.100");   //本地IP
    serv_addr.sin_port = 0x4009;    // 本地端口号

    if (bind(serv_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1) 
    {
        printf("bind socket error\n");
        return 1;
    }

    clnt_addr_size = sizeof(clnt_addr);

    return 0;
}


static uint16_t dtofCalDis(const float cx, const float cy)
{
    uint16_t disData = (40*(round(cy/py) - 1) + round(cx/px) - 1)*4 + 74;
    uint16_t dis = 0;
    str_len = recvfrom(serv_sock, message, DTOF_BUF_SIZE, 0, (struct sockaddr*)&clnt_addr, &clnt_addr_size);

    if(str_len >= 4873)
    {
        message[str_len] = 0;
        dis = (uint16_t)(message[disData]) << 8 | message[disData-1];   //mm
        return dis;
    }
    else if(str_len < 0) 
    {
        printf("recvfrom error\n");
    }
    return 1;
}

float distance = 0.0;
float dtofDistance(uint16_t cx_min, uint16_t cy_min, uint16_t cx_max, uint16_t cy_max)
{
    cx_min = CAMERA_WIDTH - cx_min;
    cx_max = CAMERA_WIDTH - cx_max;
    cy_min = CAMERA_HEIGHT - cy_min;
    cy_max = CAMERA_HEIGHT - cy_max;

    float xPoint = (cx_min+cx_max)/2.0, yPoint = (cy_min+cy_max)/2.0;

    distance = dtofCalDis(xPoint, yPoint)/1000.0;

    // printf("distance: %.2fm\n", distance);

    return distance;
}

// static void* GetDtofUdpData(void)
// {
//     while (1) 
//     {
//         dtofDistance(631,471, 652,495);
//         if(status1111111111)
//         {
//             break;
//         }
//     }

//     close(serv_sock);
//     pthread_exit(NULL);
// }

// int main_dtof(void)
// {
//     int ret;

//     static pthread_t t_dtof;

//     dtof_Init();

//     pthread_create(&t_dtof, NULL, GetDtofUdpData, NULL);

//     while(1)
//     {
//         ch = getchar();
//         if (ch == '\n') 
//         {
//             status1111111111 = 1;
//             break;
//         }
//     }

//     pthread_join(t_dtof, NULL);

//     return ret;
// }
