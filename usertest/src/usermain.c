
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <math.h>
#include "sdk.h"
#include "usermain.h"
#include "gpio_user.h"
#include "gnss_user.h"
#include "attitude.h"
#include "imu_user.h"
#include "net_user.h"
#include "sample_audio.h"
#include "dtof_user.h"
#include "sample_media_ai.h"

/*extern variable*/
extern uint8_t audioBusy;
extern uint8_t AiProcessStopFlag;
extern uint8_t obstacleInFiveMeter;
extern uint8_t snapingFlag;
extern int pathNum;
/*TIMESTAMP*/
struct timeval timevalue = {0};
long long Timestamp = 0;
/*LED & KEY*/
int key1Val = 0, key2Val = 0;
uint8_t ledcount = 0;
char keyBoradIn;
/*STATUS & FLAG*/
uint8_t uploadDataFlag = 0, uartStopFlag = 0, obstacleAvoidStopFlag = 0;
uint8_t walkNaviFailFlag = 0, shutDownFlag = 0, naviStartFlag = 0, userStopNaviFlag = 0;
////////////////////////////////////////////////MAIN/////////////////////////////////////////////////
int main(void)
{
    int ret = 0;
    void *status_init1, *status_init2, *status_init3;
    pthread_t t_uart1r, t_uart2r, t_init1, t_init2, t_init3, t_avoiding, t_transfer, 
              t_navi, t_shutDown = {0};

    /*Core part Init*/
    sdk_init();
    ret = allUartInit();
    if(ret != 0){
        printf("open uart error! ret:%d.", ret);
        return 0;
    }
    
    /*Open uart1,uart2 receive thread*/
    pthread_create(&t_uart1r, NULL, thread_uart1Rcv, NULL);
    pthread_create(&t_uart2r, NULL, thread_uart2Rcv, NULL);

    /*Initialization start*/
    pthread_create(&t_init1, NULL, thread_init1, NULL);
    pthread_create(&t_init2, NULL, thread_init2, NULL);
    pthread_create(&t_init3, NULL, thread_init3, NULL);

    /*Determine whether initialization is successful*/
    pthread_join(t_init1, &status_init1);   //4g, gnss
    pthread_join(t_init2, &status_init2);   //camera, dtof
    pthread_join(t_init3, &status_init3);   //audio, gpio, imu
    if(status_init1 == 0 && status_init2 == 0 && status_init3 == 0){
        printf("Initialization complete!\r\n\n");
        printf("-------------------------Helloworld, OHOS!-------------------------\r\n\n");
        Timestamp = Now_Timestamp();    //record timestamp
    }else{
        uartStopFlag = 1;
        aiVision_DeInit();
        // AudioDeInit();
        sdk_exit();
        printf("Initialization failed!\n");
        return 1;
    }

    /*Creating thread: dtof and visual obstacle avoidance*/
    pthread_create(&t_avoiding, NULL, thread_obstacleAvoidance, NULL);
    /*press Enter to stop main thread*/
    pthread_create(&t_shutDown, NULL, thread_shutdown, NULL);

    while (shutDownFlag == 0)
    {
        Location_Algorithm();

        if(Now_Timestamp() - Timestamp  >= 1000000) //1Hz
        {
            /*transfer picture and position*/
            if(uploadDataFlag >= 60)    //30s
            {
                uploadDataFlag = 0;
                if(netBusy == 0 && snapingFlag == 0 && naviStartFlag == 0){
                    statusTransferTrd(&t_transfer);
                }else{
                    printf("network busy.\n");
                }
            }
            uploadDataFlag++;
            if(uploadDataFlag > 200){   //Prevent overflow
                uploadDataFlag = 0;
            }

            /*navigation entrance*/
            if(key1Val == 1){
                if(naviStartFlag == 0) {
                    key1Val = 0;
                    pthread_create(&t_navi, NULL, walkingNavigation, NULL);
                    LED1_ON();
                }
                key1Val = 0;
            }
            if(key2Val == 1){
                if(audioBusy == 0) {
                    key2Val = 0;
                    getDirection();
                }
                key2Val = 0;
            }

            /*update timestamp*/
            Timestamp = Now_Timestamp();
            if(ledcount > 1)   //2s
            {
                if(GNSS.GnssTimeValid)
                {
                    printf("%.1f, %.1f, %.1f\r\n", pitch, roll, yaw);
                    printf("Beijing Time: 20%d-%02d-%02d, %02dh-%02dmin-%02ds\r\n", 
                    GNSS.year, GNSS.month, GNSS.day, GNSS.hour, GNSS.minute, GNSS.second);
                }
				LED2_Blink();
				ledcount = 0;
			}
			ledcount++; 
        }

        if(GetKey1Value() == 0) { //Pull down
            key1Val = 1;
            userStopNaviFlag = 1;
        }
        if(GetKey2Value() == 0) { //Pull down
            key2Val = 1;
        }

    }

    uartStopFlag = 1;
    obstacleAvoidStopFlag = 1;
    pthread_join(t_avoiding, NULL);
    aiVision_DeInit();
    // AudioDeInit();
    allUartClose();
    sdk_exit();
    printf("\n======main thread end======\n");

    return 0;
}


/////function/////////////////////////////////////////////////////////////////////
long long Now_Timestamp(void)
{
    long long nowtime = 0;
    gettimeofday(&timevalue, NULL);
    nowtime = (long long)timevalue.tv_sec*1000000 + timevalue.tv_usec;
    return nowtime;
}

/////part of thread///////////////////////////////////////////////////////////
void* thread_obstacleAvoidance(void)
{
    pthread_t t_aiVision = {0};
    obstacleDetectAiTrd(&t_aiVision);

    while (1) 
    {
        // usleep(1000000);
        // if(obstacleInFiveMeter == 1)
        // {
        //     if(audioBusy)
        //     {
        //         Play_audioFile(21);
        //         printf("obstacle Avoid 5 m.\n");
        //     }
        //     obstacleInFiveMeter = 0;
        // }

        usleep(1000000);
        if(obstacleAvoidStopFlag == 1)
        {
            AiProcessStopFlag = 1;
            printf("obstacle Avoid Process Stop.\n");
            break;
        }
    }

    pthread_join(t_aiVision, NULL);
}

/*
define: chn0 is record audio, 1~23 is system audio.
0:first audio, 1~19:path audio, 20:start audio, 21:avoid audio, 22:wait audio, 23:error audio
24:arrive audio, 25:arrive point audio, 27~30:diraction.
*/
void* walkingNavigation(void)
{
    uint8_t timeOutCount = 0;
    uint8_t pathCount = 0;
    pthread_t t_transGetRouteAudio={0}, t_firstAudio={0};
    void* transferStatus, *path_Status;
    naviStartFlag = 1;

    printf("Start walking planning.\n");
    audioReport_startTalk();       //audio: start navi, please talk destination

    /*recording*/
    LED1_OFF();
    Get_audioFile();
    LED1_ON();
    usleep(200000);
    
    /*transfer audio*/
    audioTransferTrd(&t_transGetRouteAudio);
    usleep(200000);
    audioReport_wait();     //wait
    pthread_join(t_transGetRouteAudio, &transferStatus);
    if(transferStatus != 0){
        printf("walk navi fail: get route fail.\n");
        goto NAVI_ERR;
    }

    audioReport_wait();     //wait
    usleep(10000000);

    /*first audio, path0*/
    firstAudioReceiveTrd(&t_firstAudio);
    pthread_join(t_firstAudio, &path_Status);
    if(path_Status != 0){
        printf("walk navi fail: first audio fail.\n");
        goto NAVI_ERR;
    }
    Play_audioFile(0);     //first audio
    // audioReceiveTrd(&t_getPath1, "path1", 1);    //Obtain audio for path1 in advance

    while (1)
    {
        if(key1Val == 1)
        {
            key1Val = 0;
            LED1_OFF();
            printf("Destination correct.\n");
            break;
        }
        LED1_OFF();
        usleep(200000);
        LED1_ON();
        timeOutCount++;
        if(timeOutCount > 40)  //8s
        {
            printf("Timeout, change destination!\n");
            goto NAVI_ERR;
        }
    }


    for(pathCount = 1; pathCount<=pathNum; pathCount++)
    {
        pthread_t t_getPathN={0};
        printf("path %d over.\n", pathCount - 1);
        usleep(1500000);
        audioReceiveTrd(&t_getPathN, pathCount, pathCount);
        pthread_join(t_getPathN, &path_Status);
        if(path_Status != 0){
            printf("walk navi fail: path%d audio fail.\n", pathCount);
            goto NAVI_ERR;
        }
        usleep(200000);
        Play_audioFile(pathCount);  //pathn audio
        /*check position*/
        userStopNaviFlag = 0;
        while(1)
        {
            if(fabs(GNSS.Latitude - poly.latitude_end)<0.001 && fabs(GNSS.Longitude - poly.longitude_end)<0.001)
            {
                printf("arrive path%d end piont.\n", pathCount);
                break;
            }
            if(userStopNaviFlag == 1)
            {
                printf("user stop navi.\n", pathCount);
                break;
                // goto NAVI_ERR;
            }
        }
    }

    LED1_OFF();
    audioReport_arrive();
    walkNaviFailFlag = 1;
    naviStartFlag = 0;
    pthread_exit(NULL);

NAVI_ERR:
    LED1_OFF();
    audioReport_error();
    walkNaviFailFlag = 1;
    naviStartFlag = 0;
    pthread_exit(NULL);
}

void* thread_init1(void)
{
    int ret = 0;
    ret = GNSS_Init();
    if(ret != 0){
        return (void*)1;
    }
    ret = net_Init();
    if(ret != 0){
        return (void*)1;
    }
    utcTimeUpdate_Beijing();
    usleep(100000);
    positionUpdate();
    usleep(100000);
    printf("thread_init1 ok.\r\n");
    return (void*)0;
}

void* thread_init2(void)
{
    int ret = 0;
    ret = dtof_Init();
    if(ret != 0){
        return (void*)1;
    }
    AiProcessStopFlag = 0;
    ret = aiVision_Init();
    if(ret != 0){
        return (void*)1;
    }
    printf("thread_init2 ok.\r\n");
    return (void*)0;
}

void* thread_init3(void)
{
    int ret = 0;
    ret = GPIO_Init();
    if(ret != 0){
        return (void*)1;
    }
    ret = IMU_Init();
    if(ret != 0){
        return (void*)1;
    }
    printf("thread_init3 ok.\r\n");
}

void* thread_uart1Rcv(void)
{
    uint8_t GnssReportCount = 0;
    while(uartStopFlag == 0)
    {
        Uart1Read();
        GNSS_GetData();
        usleep(400000);
        if(GnssReportCount >= 20)   //10s
        {
            GnssReportCount = 0;
            printf("gnss conditon: TimeValid=%d, PosValid=%d.\r\n", 
                GNSS.GnssTimeValid, GNSS.GnssPosValid);
        }
        GnssReportCount++;
    }
    pthread_exit(NULL);
}

void* thread_uart2Rcv(void)
{
    while(uartStopFlag == 0)
    {
        Uart2Read();
        usleep(20000);
    }
    pthread_exit(NULL);
}

void* thread_shutdown(void)
{
    while(1)
    {
        keyBoradIn = getchar();
        if (keyBoradIn == '\n') 
        {
            shutDownFlag = 1;
            pthread_exit(NULL);
        }
    }
}

void getDirection(void)
{
    if (yaw >= -45 && yaw < 45) //N
    {
        Play_audioFile(27);
    } 
    else if (yaw >= 45 && yaw < 135) //E
    {
        Play_audioFile(28);
    } 
    else if (yaw >= 135 && yaw > -135)   //S
    {
        Play_audioFile(29);
    } 
    else if (yaw >= -135 && yaw < -45)   //W
    {
        Play_audioFile(30);
    }
}

