
/*
 * 该文件实现了在Hi3516DV300端通过Uart串口实现自发自收的功能，在内核配置成功的前提下，
 * 通过打开/dev/ttyAMA1串口，并基于该串口实现自定义字符串的发送，并可在PC端通过串口工具进行查
 * 看发送的字符串，也可以通过PC端串口工具往板Hi3516DV300端进行发送，板端可通打印查看是否收到的报文。
 * 该示例可测试Hi3516DV300端串口收发是否正常。
 *
 * This file realizes the function of self-sending and self-receiving through the Uart serial port
 * on the Hi3516DV300 side. On the premise that the kernel configuration is successful,
 * by opening the /dev/ttyAMA1 serial port, and based on this serial port, the user-defined string is sent,
 * and on the PC side through the serial port tool to view the sent string.
 * It can also be sent to the Hi3516DV300 side of the board through the serial port tool on the PC side,
 * and the board side can print to see if the received message is received.
 * This example can test whether the serial port on the Hi3516DV300 is sending and receiving normally.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "gnss_user.h"
#include <pthread.h>

#define DEBUGMODE  0

char uart1ReadBuff[512] = {0};
uint8_t rcv1_flag = 0;
struct GNSSdata GNSS = {0};
int uart1Fd = 0;

/*
 * Serial port settings
 */
static int8_t Uart1Config(int fd)
{
    struct termios newtio = {0}, oldtio = {0};
    /*
     * 获取原有串口配置
     * Get the original serial port configuration
     */
    if (tcgetattr(fd, &oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    (void)memset_s(&newtio, sizeof(newtio), 0, sizeof(newtio));
    /*
     * CREAD开启串行数据接收，CLOCAL打开本地连接模式
     * CREAD opens serial data reception, CLOCAL opens local connection mode
     */
    newtio.c_cflag  |=  CLOCAL | CREAD;

    /*
     * 设置数据位
     * Set data bit
     */
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;
    /*
     * 设置奇偶校验位
     * Set parity bit
     */
    newtio.c_cflag &= ~PARENB; // 无奇偶校验
    /*
     * 设置波特率9600
     * Set baud rate 9600
     */
    cfsetispeed(&newtio, B9600);    //in
    cfsetospeed(&newtio, B9600);    //out

    /*
     * 设置停止位
     * Set stop bit
     */
    newtio.c_cflag &=  ~CSTOPB; /* 默认为一位停止位 */
    /*
     * 设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时
     *
     * Set the minimum characters and waiting time,
     * when there are no special requirements for receiving characters and waiting time
     */
    newtio.c_cc[VTIME]  = 0; /* 非规范模式读取时的超时时间 */
    newtio.c_cc[VMIN] = 0; /* 非规范模式读取时的最小字符数 */
    /*
     * tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来
     *
     * tcflush clears the unfinished input/output requests and data of the terminal;
     * TCIFLUSH means clearing the data being received and not reading it out
     */
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }
    return 0;
}

int Uart1Send(char *buf, int len)
{
    int ret = 0;
    int count = 0;
    char *sendBuf = buf;
    int sendLen = len;

    tcflush(uart1Fd, TCIFLUSH);

    while (sendLen > 0) {
        ret = write(uart1Fd, (char*)sendBuf + count, sendLen);
        if (ret < 1) {
            printf("write data below 1 byte % d\r\n", ret);
            break;
        }
        count += ret;
        sendLen = sendLen - ret;
    }

    return count;
}


uint8_t Uart1Read(void)
{
    fd_set readfds;
    int ret = 0;
    int reslen = 0;

    FD_ZERO(&readfds);
    FD_SET(uart1Fd, &readfds);

    int ready = select(uart1Fd + 1, &readfds, NULL, NULL, NULL);
    if (ready == 0) 
    {
        return 0;
    }
    else if (FD_ISSET(uart1Fd, &readfds))
    {
        rcv1_flag = 0;
        // memset(uart1ReadBuff, 0, sizeof(uart1ReadBuff));
        // usleep(5000);
        while (1)
        {
            usleep(15000);
            ret = read(uart1Fd, uart1ReadBuff + reslen, sizeof(uart1ReadBuff) - reslen);
            if (ret > 0)
            {
                reslen += ret;
                // printf("Received %d bytes: %s\n", ret, uart1ReadBuff);
                continue;
            }
            else if(ret == 0)
            {
                reslen = 0;
                rcv1_flag = 1;
                // printf("RX: %s.\n", uart1ReadBuff);
                // memset(uart1ReadBuff, 0, sizeof(uart1ReadBuff));
                break;
            }
            else
            {
                printf("uart1 read error: %d !\n", ret);
                return 1;
            }
        }
    }
    return 0;
}

uint8_t Uart1Init(void)
{
    char *uart1 = "/dev/ttyAMA1";

    if ((uart1Fd = open(uart1, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) 
    {
        printf("open %s is failed", uart1);
        return 1;
    }
    else 
    {
        Uart1Config(uart1Fd);
    }
    return 0;
}

uint8_t GNSS_Init(void)
{
    char *PCAS02 = NULL;
    char *PCAS03 = NULL;
    char *PCAS04 = NULL;
    PCAS02 = "$PCAS02,500*1A\r\n";     //$PCAS02,1000*2E
    PCAS03 = "$PCAS03,0,0,0,0,1,0,0,0,0,0,,,0,0*03\r\n";
    PCAS04 = "$PCAS04,3*1A\r\n";

    Uart1Send(PCAS02, strlen(PCAS02));     //2Hz
    usleep(50000);
	Uart1Send(PCAS03, strlen(PCAS03));     //GNRMC
    usleep(50000);
	Uart1Send(PCAS04, strlen(PCAS04));     //GPS+BD
    usleep(50000);

    return 0;
}

uint8_t GNSS_GetData(void)
{
	uint8_t DataCount = 0, temp = 0;
	float lati = 0.0, longi = 0.0;
    double N = 0.0, sinLati = 0.0, cosLati = 0.0, sinLongi = 0.0, cosLongi = 0.0, 
		deltaECEFx = 0.0, deltaECEFy = 0.0, deltaECEFz = 0.0;
	
	if(rcv1_flag == 1)
	{
		rcv1_flag = 0;
		
		if(DEBUGMODE)printf("RX:%s\r\n", uart1ReadBuff);
		
        if(uart1ReadBuff[8] == 'V')
		{
            GNSS.GnssTimeValid = 0;
            GNSS.GnssPosValid = 0;
            if(DEBUGMODE)printf("No signal.\r\n");
			memset(uart1ReadBuff, 0, sizeof(uart1ReadBuff));
			return 1;
		}
		else if(uart1ReadBuff[18] == 'V')
		{
            //$GNRMC,123652.000,V,,,,,,,230624,,,N,V*29
            DataCount = sscanf(uart1ReadBuff, "$GNRMC,%f,V,,,,,,,%f,,,N", 
            &GNSS.hour_minute_second, &GNSS.day_month_year);

            GNSS.hour = (uint8_t)(GNSS.hour_minute_second/10000);
            GNSS.minute = (uint8_t)(GNSS.hour_minute_second/100) - 100*GNSS.hour;
            GNSS.second = (uint8_t)(GNSS.hour_minute_second) - GNSS.hour*10000 - GNSS.minute*100;
            GNSS.hour += 8;
             
            GNSS.day = GNSS.day_month_year/10000;
            GNSS.month = GNSS.day_month_year/100 - GNSS.day*100;
            GNSS.year = GNSS.day_month_year - GNSS.day*10000 - GNSS.month*100;

            GNSS.GnssTimeValid = 1;
            GNSS.GnssPosValid = 0;
			if(DEBUGMODE)printf("No signal, Beijing Time: 20%d-%02d-%02d, %02dh-%02dmin-%02ds\r\n", 
            GNSS.year, GNSS.month, GNSS.day, GNSS.hour, GNSS.minute, GNSS.second);
			memset(uart1ReadBuff, 0, sizeof(uart1ReadBuff));
			return 1;
		}
		else if(uart1ReadBuff[18] == 'A')
		{
            //$GNRMC,052850.500,A,3401.25131,N,10845.21136,E,0.00,0.00,070724,,,A,V*0E
		    //$GNRMC,135408.000,A,3400.99872,N,10844.62337,E,1.03,240.55,180524,,,A,V*0F
			DataCount = sscanf(uart1ReadBuff, "$GNRMC,%f,A,%f,N,%f,E,%f,%f,%d,", &GNSS.hour_minute_second, 
			&lati, &longi, &GNSS.Ground_speed, &GNSS.Course, &GNSS.day_month_year);
			if(DataCount == 6)
            {
                GNSS.hour = (uint8_t)(GNSS.hour_minute_second/10000);
                GNSS.minute = (uint8_t)(GNSS.hour_minute_second/100) - 100*GNSS.hour;
                GNSS.second = (uint8_t)(GNSS.hour_minute_second) - GNSS.hour*10000 - GNSS.minute*100;
                GNSS.hour += 8;
                
                GNSS.day = GNSS.day_month_year/10000;
                GNSS.month = GNSS.day_month_year/100 - GNSS.day*100;
                GNSS.year = GNSS.day_month_year - GNSS.day*10000 - GNSS.month*100;
                
                temp = (uint8_t)(lati/100.0f);
                GNSS.Latitude = (double)(lati - temp*100)/60.0 + temp;
                temp = (uint8_t)(longi/100.0f);
                GNSS.Longitude = (double)(longi - temp*100)/60.0 + temp;
                
                GNSS.GnssPosValid = 1;
                GNSS.GnssTimeValid = 1;

                sinLati = sin(GNSS.Latitude/57.2957805);
                sinLongi = sin(GNSS.Longitude/57.2957805);
                cosLati = cos(GNSS.Latitude/57.2957805);
                cosLongi = cos(GNSS.Longitude/57.2957805);
                N = LongSemiAxis/sqrt(1 - Eccentricity_2 * sinLati*sinLati);
                
                GNSS.ECEF_X = (N+GNSS.Height) * cosLati * cosLongi;
                GNSS.ECEF_Y = (N+GNSS.Height) * cosLati * sinLongi;
                GNSS.ECEF_Z = (N * (1-Eccentricity_2) + GNSS.Height) * sinLati;
                
                deltaECEFx = GNSS.ECEF_X-GNSS.ECEFinitX;
                deltaECEFy = GNSS.ECEF_Y-GNSS.ECEFinitY;
                deltaECEFz = GNSS.ECEF_Z-GNSS.ECEFinitZ;
                
                GNSS.ENU_x = -sinLongi*deltaECEFx + cosLongi*deltaECEFy;
                GNSS.ENU_y = -sinLati*cosLongi*deltaECEFx - sinLati*sinLongi*deltaECEFy + cosLati*deltaECEFz;
    			//GNSS.ENU_z = -cosLati*cosLongi*deltaECEFx - cosLati*sinLongi*deltaECEFy + sinLati*deltaECEFz;
                GNSS.ENU_z = 0.0;

                if(DEBUGMODE)printf("Beijing Time: 20%d.%02d.%02d, %02dh-%02dmin-%02ds\r\n"
                    "North Latitude: %.6f degree, East Longitude: %.6f degree.\r\n", 
                    GNSS.year, GNSS.month, GNSS.day, GNSS.hour, GNSS.minute, GNSS.second, 
                    GNSS.Latitude, GNSS.Longitude);
            }
            else
            {
                GNSS.GnssPosValid = 0;
                GNSS.GnssTimeValid = 0;
                if(DEBUGMODE)printf("DataCount: %d, error!", DataCount);
                memset(uart1ReadBuff, 0, sizeof(uart1ReadBuff));
                return 1;
            }
		}
        else
        {
            printf("GNSS error!\n");
            memset(uart1ReadBuff, 0, sizeof(uart1ReadBuff));
            return 1;
        }
	}
	return 0;
}

void Uart1Close(void)
{
    close(uart1Fd);
}


// char ch;
// void* thread_uart1rcv_gnss(void)
// {
//     while(1)
//     {
//         Uart1Read();
//         usleep(100000);
//         GNSS_GetData();
//         usleep(100000);
//     }
// }
// int main_gnss(void)
// {

//     pthread_t t_uart1r;
//     GNSS_Init();

//     /*Open uart1,uart2 receive thread*/
//     pthread_create(&t_uart1r, NULL, thread_uart1rcv_gnss, NULL);

//     while (1)
//     {
//         ch = getchar();
//         if (ch == '\n') 
//         {
//             Uart1Close();
//             break;
//         }
//     }

//     return 0;
// }



// #define PAGE_SIZE   (0x1000)
// void UartPinConfig(void)
// {
//     int memFd;
//     uint32_t RegTarget = 0x114F005C;  // 目标寄存器的物理地址
//     uint32_t RegWriteValue = 0x000433;    // 要写入的值
//     void *map1_reg;
//     void *map2_reg;

//     if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
//         printf("Failed to open mem\r\n");
//     }else{
//         printf("open mem successed\r\n");
//     }

//     RegTarget = (RegTarget & (~(PAGE_SIZE - 1)));   //页对齐
//     map1_reg = mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, RegTarget);
//     if (map1_reg == ((void *) -1)) {
//         printf("Failed to map1 memory !");
//     }
//     int* data1 = (uint32_t*)map1_reg;
//     *data1 = RegWriteValue;
//     // *(volatile uint32_t *)map1_reg = RegWriteValue;

//     RegTarget = 0x114F0058;
//     RegWriteValue = 0x000533;
//     RegTarget = (RegTarget & (~(PAGE_SIZE - 1)));   //页对齐
//     map2_reg = mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, RegTarget);
//         if (map2_reg == ((void *) -1)) {
//         printf("Failed to map2 memory !");
//     }
//     // *(volatile uint32_t *)map2_reg = RegWriteValue;
//     int* data2 = (uint32_t*)map2_reg;
//     *data2 = RegWriteValue;

//     close(memFd);
// }

// //$GNGGA,153749.000,,,,,0,00,25.5,,,,,,*77
// if(uart1ReadBuff[7] == ',')
// {
//     printf("No time signal.\r\n");
//     memset(uart1ReadBuff, 0, sizeof(uart1ReadBuff));
//     return 1;
// }
// else if(uart1ReadBuff[19] == ','&&uart1ReadBuff[20] == ','&&uart1ReadBuff[21] == ',')
// {
//     for (i = 0; i < 6; i++) 
//     {
//         h_m_s[i] = uart1ReadBuff[7+i];
//     }
//     GNSS.hour_minute_second = atoi(h_m_s);
//     hour = (uint8_t)(GNSS.hour_minute_second/10000);
//     minute = (uint8_t)(GNSS.hour_minute_second/100) - 100*hour;
//     second = (uint8_t)(GNSS.hour_minute_second) - hour*10000 - minute*100;
//     hour += 8;

//     printf("Beijing Time: %dh-%dmin-%ds, No signal.\r\n", hour, minute, second);
//     memset(uart1ReadBuff, 0, sizeof(uart1ReadBuff));
//     return 1;
// }
// else
// {
//     DataCount = sscanf(uart1ReadBuff, "$GNRMC,%f,%f,N,%f,E,%d,%d,%f,%f,M,%f,M,,",
//     &GNSS.hour_minute_second, &lati, &longi, &GNSS.Quality, &GNSS.Satellites,
//     &GNSS.Horizontal_accuracy, &GNSS.Altitude, &GNSS.Geoid_height);
    
//     hour = (uint8_t)(GNSS.hour_minute_second/10000);
//     minute = (uint8_t)(GNSS.hour_minute_second/100) - 100*hour;
//     second = (uint8_t)(GNSS.hour_minute_second) - hour*10000 - minute*100;
//     hour += 8;
    
//     temp = (uint8_t)(lati/100.0f);
//     GNSS.Latitude = (double)(lati - temp*100)/60.0 + temp;

//     temp = (uint8_t)(longi/100.0f);
//     GNSS.Longitude = (double)(longi - temp*100)/60.0 + temp;
    
//     printf(
//     "Beijing Time: %dh-%dmin-%ds, Satellites: %d,\r\n"
//     "North Latitude: %.5f degree, East Longitude: %.5f degree,\r\n"
//     "Altitude: %.5f meter, Geoid height: %.5f meter,\r\n"
//     "Horizontal accuracy: %.5f, Quality: %d, DataCount: %d.\r\n\n", 
//     hour, minute, second, GNSS.Satellites, 
//     GNSS.Latitude, GNSS.Longitude, 
//     GNSS.Altitude, GNSS.Geoid_height, 
//     GNSS.Horizontal_accuracy, GNSS.Quality, DataCount);
// }


