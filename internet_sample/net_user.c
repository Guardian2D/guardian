
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
#include "net_user.h"
#include <stdlib.h>
#include <pthread.h>
#include "gnss_user.h"
#include "base64_user.h"
#include "sample_audio.h"
#include "sdk.h"

char uart2ReadBuff[98304] = {0};
char send2_buf[2048] = {0};
uint8_t rcv2_flag = 0;
UTCtime BeijingTime = {0};
Position position = {0};
polyLine poly = {0};
int uart2Fd = 0;
uint8_t netBusy = 0;
// char audioFolderName[21] = "2024-07-09 14:22:40";
char audioFolderName[21] = {0};
int pathNum = 0;

#define DEBUGMODE  1

/*
 * Serial 2 port settings
 */
static int8_t Uart2Config(int uart2Fd, int baudrate)
{
    struct termios tty;

    if (tcgetattr(uart2Fd, &tty) != 0) 
    {
        printf("SetupSerial 2\n");
        return -1;
    }
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 data bits
    tty.c_cflag |= CREAD;   // Enable receiver
    tty.c_cflag |= CLOCAL;  // Ignore modem control lines
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_lflag = 0;        // No signaling characters, no echo
    tty.c_cc[VMIN] = 0;     // Read doesn't block
    tty.c_cc[VTIME] = 0;    // No timeout
    if (tcsetattr(uart2Fd, TCSANOW, &tty) != 0) 
    {
        printf("com 2 set error\n");
        return -1;
    }
    usleep(10000);
    return 0;
}

static int Uart2Send(int len)
{
    int ret = 0;
    int count = 0;
    int sendLen = len;

    tcflush(uart2Fd, TCIFLUSH);

    while (sendLen > 0) {
        ret = write(uart2Fd, send2_buf + count, sendLen);
        if (ret < 1) {
            printf("write data below 1 byte % d\r\n", ret);
            break;
        }
        count += ret;
        sendLen = sendLen - ret;
    }
    memset(send2_buf, 0, sizeof(send2_buf));
    return count;
}

static int Uart2BigSend(int len, char *buf)
{
    int ret = 0;
    int count = 0;
    int sendLen = len;

    tcflush(uart2Fd, TCIFLUSH);

    while (sendLen > 0) 
    {
        ret = write(uart2Fd, (char *)buf + count, sendLen);
        // printf("write bytes %d\r\n", ret);
        if (ret < 1) 
        {
            printf("write data below 1 byte % d\r\n", ret);
            break;
        }
        count += ret;
        sendLen = sendLen - ret;
        usleep(20);
    }

    return count;
}

static void Uart2PinConfig(void)
{
    system("./himm 0x114F005C 0x0433");
    usleep(10000);
    system("./himm 0x114F0058 0x0533");
}

int8_t Uart2Read(void)
{
    fd_set rfds;
    int ret = 0;
    int reslen = 0;
    // struct timeval timeout = {0, 500000};
    FD_ZERO(&rfds);
    FD_SET(uart2Fd, &rfds);

    int ready = select(uart2Fd + 1, &rfds, NULL, NULL, NULL);
    if (ready == 0) 
    {
        return 0;
    }
    else if (FD_ISSET(uart2Fd, &rfds))
    {
        rcv2_flag = 0;
        while (1)
        {
            usleep(1000);
            ret = read(uart2Fd, uart2ReadBuff + reslen, sizeof(uart2ReadBuff) - reslen);
            if (ret > 0)
            {
                reslen += ret;
                // printf("Received %d bytes: %s\n", ret, uart2ReadBuff);
                continue;
            }
            else if(ret == 0)
            {
                rcv2_flag = 1;
                // printf("RX: %s", uart2ReadBuff+2);  //test
                reslen = 0;
                break;
            }
            else
            {
                printf("uart2 read error: %d !\r\n", ret);
                return 1;
            }
        }
    }
    return 0;
}

uint8_t CheckAtCommand(const char *command, const char *response)
{
    int32_t timeoutc = 0;
    char *retAt = NULL;

    rcv2_flag = 0;
    Uart2Send(sprintf(send2_buf, "%s", command));

    if(strcmp(response, "null") == 0)
    {
        memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
        return 0;
    }

    while(rcv2_flag == 0)
    {
        usleep(200000);
        timeoutc++;
        if (timeoutc > 25)      //5s
        {
            timeoutc = 0;
            printf("Check AtCommand timeout!\r\n");
            break;
        }
    }

    if(rcv2_flag == 1)
    {
        rcv2_flag = 0;
        while (retAt == NULL)
        {
            retAt = strstr(uart2ReadBuff, response);
            usleep(200000);
            timeoutc++;
            if (timeoutc > 25)      //5s
            {
                printf("Check AtCommand timeout!\r\n");
                break;
            }
        }
        if(retAt != NULL)
        {
            if(DEBUGMODE)printf("AT response correct! Tx: %s\n", command);
            memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
            return 0;
        }
    }

    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    return 1;
}

uint8_t run_status = 0;
void* thread_uart2(void)                                                                                                                             
{
    while(run_status == 0)
    {
        Uart2Read();
        usleep(50000);
    }
}
void* thread_uart1(void)                                                                                                                             
{
    while(run_status == 0)
    {
        Uart1Read();
        usleep(200000);
        GNSS_GetData();
        usleep(200000);
        // printf("%d, %d\r\n", GNSS.GnssPosValid, GNSS.GnssTimeValid);
    }
}

/*
///////////////////////////////////////////transfer file//////////////////////////////////////////////
*/
void* thread_status_transfer(void *arg)                                                                                                                             
{
    uint8_t timeoutc = 0;
    int base64BufLen = 0;
    char *arg_ossname = (char *)arg;
    int base64FirstLen = 0;
    int base64SecondLen = 0;
    int FirstSendLen = 0;
    int SecondSendLen = 0;
    
    /*set url*/
    CheckAtCommand("AT+HTTPSET=\"URL\",\"http://47.109.198.70:8080/states/Half\"\r\n", "OK");
    usleep(150000);

    /*open jpg file*/
    FILE *jpgFile = fopen("status.jpg", "rb");
    if (jpgFile == NULL){
        printf("failed to open jpgFile\r\n");
        goto TRD_ERR;
    }

    /*get file size*/
    fseek(jpgFile, 0, SEEK_END);
    long filesize = ftell(jpgFile);
    fseek(jpgFile, 0, SEEK_SET);
    if (filesize <= 0){
        printf("failed to get file size\r\n");
        fclose(jpgFile);
        goto TRD_ERR;
    }
    printf("jpg file size: %d bytes.\r\n", filesize);

    /*apply buffer*/
    char *binaryBuf = (char*)malloc(filesize);
    if (binaryBuf == NULL){
        printf("failed to allocate memory\r\n");
        fclose(jpgFile);
        goto TRD_ERR;
    }

    /*save binary file to buf*/
    size_t read_size = fread(binaryBuf, 1, filesize, jpgFile);
    if (read_size != filesize) 
    {
        printf("failed to read file\r\n");
        free(binaryBuf);
        fclose(jpgFile);
        goto TRD_ERR;
    }
    fclose(jpgFile);
    jpgFile = NULL;

    /*binary convert to base64*/
    char *base64Buf = (char*)malloc(filesize*2);
    base64_encode(binaryBuf, filesize, base64Buf, &base64BufLen);
    printf("base64Len:%d.\n", base64BufLen);
    free(binaryBuf);
    binaryBuf = NULL;

    /*devide base64Buf*/
    if(base64BufLen > 126800)
    {
        printf("The file is too large, cant not send.\r\n");
        free(base64Buf);
        goto TRD_ERR;
    }
    else if(base64BufLen > 64000)
    {
        base64FirstLen = 63900;
        base64SecondLen = base64BufLen - base64FirstLen;
        if(DEBUGMODE)printf("The file is load.\r\n");
    }
    else
    {
        printf("The file is too small, cant not send.\r\n");
        free(base64Buf);
        goto TRD_ERR;
    }

    /*load first half jpgbase64 file data*/
    char *base64FirstBuf = (char *)malloc(base64FirstLen + 1);
    if (base64FirstBuf == NULL)
    {
        printf("malloc base64FirstBuf fail\r\n");
        free(base64Buf);
        goto TRD_ERR;
    }
    strncpy(base64FirstBuf, base64Buf, base64FirstLen);
    base64FirstBuf[base64FirstLen] = '\0';
    /*formal string*/
    char *sendbuf_first = (char *)malloc(base64FirstLen + 64);
    if (sendbuf_first == NULL)
    {
        printf("malloc sendbuf_first fail\r\n");
        free(base64Buf);
        goto TRD_ERR;
    }
    FirstSendLen = snprintf(sendbuf_first, base64FirstLen + 64,
        "{\"fileName\": \"%s\"}\r\n",base64FirstBuf);

    // printf("\n%s\n", sendbuf_first);     //test
    /*transfer to 4G module*/
    Uart2Send(sprintf(send2_buf, "AT+HTTPDATA=%d\r\n", FirstSendLen));
    usleep(300000);
    Uart2BigSend(FirstSendLen, sendbuf_first);
    rcv2_flag = 0;
    while(rcv2_flag == 0)
    {
        usleep(100000);
        if(timeoutc > 50)  //5s
        {
            printf("transfer timeout!\r\n");
            break;
        }
    }
    free(base64FirstBuf);
    base64FirstBuf = NULL;
    free(sendbuf_first);
    sendbuf_first = NULL;

    /*HTTPPOST*/
    if(rcv2_flag == 1)
    {
        CheckAtCommand("AT+HTTPACT=1\r\n", "HTTPRES");
        if(DEBUGMODE)printf("sending\r\n");
    }
    // Uart2Send(sprintf(send2_buf, "AT+HTTPREAD=0,215\r\n")); //test
    //first transfer end//
/////////////////////////////////////////////////////////////////////////////////////////////
    //second transfer start//
    /*set url*/
    CheckAtCommand("AT+HTTPSET=\"URL\",\"http://47.109.198.70:8080/states/add\"\r\n", "OK");
    usleep(150000);
    /*get position*/
    positionUpdate();
    /*load second jpg file data*/
    char *base64SecondBuf = (char *)malloc(base64SecondLen+1);
    if (base64SecondBuf == NULL)
    {
        printf("malloc base64SecondBuf fail\r\n");
        free(base64Buf);
        goto TRD_ERR;
    }
    memcpy(base64SecondBuf, base64Buf + base64FirstLen, base64SecondLen);
    base64SecondBuf[base64SecondLen] = '\0';

    char *sendbuf_second = (char *)malloc(base64SecondLen + 512);
    if (sendbuf_second == NULL){
        printf("malloc sendbuf_second fail\r\n");
        free(base64Buf);
        goto TRD_ERR;
    }
    SecondSendLen = snprintf(sendbuf_second, base64SecondLen + 512,
    "{\"state\": "
        "{\"latitude\": \"%.6f\",\"longitude\": \"%.6f\",\"dateTime\": \"%s\"},"
    "\"base64File\": "
        "{\"file\": \"%s\",\"fileName\": \"%s\",\"storagePath\": \"image\"}}\r\n",
    position.latitude, position.longitude, arg_ossname, base64SecondBuf, arg_ossname);
    /*transfer to 4G module*/
    if(DEBUGMODE)printf("Second data size: %d bytes.\r\n", SecondSendLen);
    // printf("scd:\n%s\n",sendbuf_second);     //test
    Uart2Send(sprintf(send2_buf, "AT+HTTPDATA=%d\r\n", SecondSendLen));
    usleep(300000);
    Uart2BigSend(SecondSendLen, sendbuf_second);
    rcv2_flag = 0;
    while(rcv2_flag == 0)
    {
        usleep(200000);
        if(timeoutc > 25)  //5s
        {
            printf("transfer timeout!\r\n");
            break;
        }
        timeoutc++;
    }
    free(sendbuf_second);
    sendbuf_second = NULL;
    /*HTTPPOST*/
    if(rcv2_flag == 1)
    {
        CheckAtCommand("AT+HTTPACT=1\r\n", "HTTPRES");
        if(DEBUGMODE)printf("sending\r\n");
    }
    //end//
    free(arg_ossname);
    arg_ossname = NULL;
    free(base64Buf);
    base64Buf = NULL;
    if(DEBUGMODE)printf("transfer thread exit\r\n");
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    netBusy = 0;
    pthread_exit(NULL);
    return (void*)0;

TRD_ERR:
    free(arg_ossname);
    arg_ossname = NULL;
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    netBusy = 0;
    pthread_exit(NULL);
}

////////////////////////////////////////////////////////////////////////////////////////
void* thread_audio_transfer(void *arg)                                                                                                                             
{
    uint8_t timeoutc = 0;
    int aacLen = 0;
    int base64BufLen_a = 0;
    char *arg_folderName = (char *)arg;
    int rcvLen = 0, ret = 0;
    
    //open acc file
    FILE *aacFile = fopen("audio_chn0.aac", "rb");
    if (aacFile == NULL) 
    {
        printf("failed to open aac file\r\n");
        goto TRD_ERR;
    }

    //get file size
    fseek(aacFile, 0, SEEK_END);
    long filesize = ftell(aacFile);
    fseek(aacFile, 0, SEEK_SET);
    if (filesize <= 0) 
    {
        printf("failed to get file size\r\n");
        fclose(aacFile);
        goto TRD_ERR;
    }
    printf("aac file size: %d bytes.\r\n", filesize);

    //apply buf
    char *binaryBuf_a = (char*)malloc(filesize);
    if (binaryBuf_a == NULL) {
        printf("failed to allocate memory\r\n");
        fclose(aacFile);
        goto TRD_ERR;
    }

    //save binary file to buf
    size_t read_size = fread(binaryBuf_a, 1, filesize, aacFile);
    if (read_size != filesize) 
    {
        printf("failed to read file\r\n");
        free(binaryBuf_a);
        fclose(aacFile);
        goto TRD_ERR;
    }
    fclose(aacFile);
    aacFile = NULL;

    //binary convert to base64
    char *base64Buf_temp = (char*)malloc(filesize * 2);
    ret = base64_encode(binaryBuf_a, filesize, base64Buf_temp, &base64BufLen_a);
    free(binaryBuf_a);
    binaryBuf_a = NULL;
    // printf("\nbase64Buf:%s\n", base64Buf_a);   //test
    if(DEBUGMODE)printf("base64Len:%d, ret: %d.\n", base64BufLen_a, ret);
    /*copy base64*/
    char *base64Buf_a = (char*)malloc(base64BufLen_a + 1);
    if (base64Buf_a != NULL)
    {
        strncpy(base64Buf_a, base64Buf_temp, base64BufLen_a);
        base64Buf_a[base64BufLen_a] = '\0';
    }
    else
    {
        printf("Failed to allocate base64Buf_a.\n");
        goto TRD_ERR;
    }
    free(base64Buf_temp);
    base64Buf_temp = NULL;

    //set url
    CheckAtCommand("AT+HTTPSET=\"URL\",\"http://47.109.198.70:8080/Audio/getRouteByAudio\"\r\n", "OK");
    usleep(200000);

    //get position
    positionUpdate();

    //load json data
    char *sendBuf = (char *)malloc(base64BufLen_a + 256);
    if (sendBuf == NULL){
        printf("malloc fail\r\n");
        free(base64Buf_a);
        goto TRD_ERR;
    }

    aacLen = snprintf(sendBuf, base64BufLen_a + 256,
        "{\"audioBase64\": \"%s\",\"origin\": \"%.6f,%.6f\",\"time\": \"%s\"}\r\n", 
            base64Buf_a, position.longitude, position.latitude, arg_folderName);

    if(DEBUGMODE)printf("data size: %d bytes.\n", aacLen);
    // printf("base64Buf_a:%s\n\n", sendBuf);      //test
    // usleep(200000);
    Uart2Send(sprintf(send2_buf, "AT+HTTPDATA=%d\r\n", aacLen));
    usleep(300000);

    //transfer to 4G module
    Uart2BigSend(aacLen, sendBuf);
    rcv2_flag = 0;
    while(rcv2_flag == 0){
        usleep(200000);
        if(timeoutc > 25){  //5s
            timeoutc = 0;
            printf("transfer timeout!\n");
            free(base64Buf_a);
            free(sendBuf);
            goto TRD_ERR;
        }
        timeoutc++;
    }
    free(base64Buf_a);
    base64Buf_a = NULL;
    free(sendBuf);
    sendBuf = NULL;

    //HTTPPOST
    if(rcv2_flag == 1)
    {
        Uart2Send(sprintf(send2_buf, "AT+HTTPACT=1\r\n"));
        while(1)    //wait respond
        {
            usleep(500000);
            char* rcvPtr = strstr(uart2ReadBuff, "+HTTPRES: 1,200,");
            if (rcvPtr != NULL)
            {
                rcvPtr += strlen("+HTTPRES: 1,200,");
                ret = sscanf(rcvPtr, "%d", &rcvLen);
                break;
            }
            if(timeoutc > 40)  //20s
            {
                printf("get rcvLen timeout!\n");
                goto TRD_ERR;
            }
            timeoutc++;
        }
    }

    if(ret == 1 && rcvLen > 0)
    {
        memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
        Uart2Send(sprintf(send2_buf, "AT+HTTPREAD=0,%d\r\n", rcvLen));
        usleep(300000);

        char *pathNumPtr = strstr(uart2ReadBuff, "pathNum:");
        if (pathNumPtr != NULL)
        {
            pathNumPtr += strlen("pathNum:");
            sscanf(pathNumPtr, "%d", &pathNum);
            if(pathNum < 1)
            {
                printf("path number error.\n");
                goto TRD_ERR;
            }
            else
            {
                if(DEBUGMODE)printf("path number: %d\n", pathNum);
                memset(audioFolderName, 0, sizeof(audioFolderName));
                strncpy(audioFolderName, arg_folderName, 20);
                printf("audioFolderName: %s.\n", audioFolderName);
            }
        }
        else
        {
            printf("path return error.\n");
            goto TRD_ERR;
        }
    }
    else
    {
        printf("rcv error.\n");
        goto TRD_ERR;
    }

    free(arg_folderName);
    arg_folderName = NULL;
    if(DEBUGMODE)printf("transfer thread exit\n");
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    netBusy = 0;
    return (void*)0;

TRD_ERR:
    free(arg_folderName);
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    netBusy = 0;
    return (void*)1;
}
/*
/////////////////////////////////receive audio and polyline//////////////////////////////////
*/
void* thread_first_audio_receive(void)                                                                                                                           
{
    int ret = 0;
    int sendAudioDateLen = 0, receiveFileLen = 0;
    int base64BufLen = 0;
    int binaryBufLen = 0;
    uint8_t timeoutc = 0;

    //set url
    CheckAtCommand("AT+HTTPSET=\"URL\",\"http://47.109.198.70:8080/oss/exportFile\"\r\n", "OK");
    usleep(200000);
    //transfer to 4G module
    Uart2Send(sprintf(send2_buf, "AT+HTTPDATA=49\r\n"));
    usleep(300000);
    Uart2Send(sprintf(send2_buf, "{\"fileName\": \"route/%s/path0\"}\r\n", audioFolderName));
    printf("{\"fileName\": \"route/%s/path0\"}\r\n", audioFolderName);
    rcv2_flag = 0;
    while(rcv2_flag == 0)
    {
        usleep(200000);
        if(timeoutc > 25){  //5s
            printf("Transfer timeout!\n");
            goto TRD_R_ERR;
        }
    }
    //HTTPPOST
    if(rcv2_flag == 1)
    {
        Uart2Send(sprintf(send2_buf, "AT+HTTPACT=1\r\n"));
        while(1)
        {
            usleep(500000);
            char* rcvPtr = strstr(uart2ReadBuff, "+HTTPRES: 1,200,");
            if (rcvPtr != NULL)
            {
                rcvPtr += strlen("+HTTPRES: 1,200,");
                ret = sscanf(rcvPtr, "%d", &receiveFileLen);
                if(DEBUGMODE)printf("receiveLen: %d, ret:%d.\n", receiveFileLen, ret);
                break;
            }
            if(timeoutc > 20)  //10s
            {
                printf("get file timeout!\n");
                goto TRD_R_ERR;
            }
        }
    }
    //get file data
    if(ret == 1 && receiveFileLen > 0)
    {
        memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
        Uart2Send(sprintf(send2_buf, "AT+HTTPREAD=0,%d\r\n", receiveFileLen));
        usleep(receiveFileLen*50);
        char *getBase64LenPtr = strstr(uart2ReadBuff, "Content-Length: ");
        if (getBase64LenPtr != NULL)
        {
            sscanf(getBase64LenPtr, "Content-Length: %d", &base64BufLen);
            if(DEBUGMODE)printf("base64 length: %d bytes.\n", base64BufLen);
            //get bas64 code
            char *base64Buf = strstr(uart2ReadBuff, "//");
            if (base64Buf != NULL)
            {
                //base64 decoder and save
                char *binaryBuf = (char *)malloc(receiveFileLen);
                if (binaryBuf == NULL){
                    printf("malloc fail\n");
                    goto TRD_R_ERR;
                }
                base64_decode(base64Buf, base64BufLen, binaryBuf, &binaryBufLen);
                FILE *audiofile = fopen("audio_0.aac", "wb"); 
                if (audiofile == NULL){
                    printf("error opening file\n");
                    free(binaryBuf);
                    goto TRD_R_ERR;
                }
                fwrite(binaryBuf, 1, binaryBufLen, audiofile);
                if(DEBUGMODE)printf("audiofile length: %d bytes.\n", binaryBufLen);
                fclose(audiofile);
                free(binaryBuf);
            }
            else
            {
                printf("base64Buf not found.\n");
                goto TRD_R_ERR;
            }
        }
        else
        {
            printf("Content-Length not found.\n");
            goto TRD_R_ERR;
        }     
    }

    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    if(DEBUGMODE)printf("transfer thread exit\n");
    netBusy = 0;
    return (void*)0;

TRD_R_ERR:
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    netBusy = 0;
    return (void*)1;
}

////////////////////////////////////////////////////////////////////////////////////////////
void* thread_audio_receive(void *args)                                                                                                                             
{
    int ret = 0;
    int sendAudioDateLen = 0;
    int receiveFileLen = 0;
    int base64BufLen = 0;
    int binaryBufLen = 0;
    uint8_t timeoutc = 0;
    thread_args_receive *my_args = (thread_args_receive *)args;

    //set url
    CheckAtCommand("AT+HTTPSET=\"URL\",\"http://47.109.198.70:8080/route/getPolyLineAndPath\"\r\n", "OK");
    usleep(200000);

    //transfer to 4G module
    char *sendAudioDateBuf = (char *)malloc(80);
    if (sendAudioDateBuf == NULL) {
        printf("failed to allocate sendAudioDateBuf\r\n");
        goto TRD_R_ERR;
    }
    sendAudioDateLen = sprintf(sendAudioDateBuf, "{\"fileName\": \"route/%s/%s\"}\r\n", 
                                            audioFolderName, my_args->arg1_audioName);
    Uart2Send(sprintf(send2_buf, "AT+HTTPDATA=%d\r\n", sendAudioDateLen));
    usleep(300000);
    Uart2BigSend(sendAudioDateLen, sendAudioDateBuf);
    rcv2_flag = 0;
    while(rcv2_flag == 0)
    {
        usleep(200000);
        if(timeoutc > 25)  //5s
        {
            printf("Transfer timeout!\n");
            free(sendAudioDateBuf);
            timeoutc = 0;
            goto TRD_R_ERR;
        }
        timeoutc++;
    }
    free(sendAudioDateBuf);

    //HTTPPOST
    if(rcv2_flag == 1)
    {
        Uart2Send(sprintf(send2_buf, "AT+HTTPACT=1\r\n"));
        while(1)
        {
            usleep(500000);
            char* rcvPtr = strstr(uart2ReadBuff, "+HTTPRES: 1,200,");
            if (rcvPtr != NULL)
            {
                rcvPtr += strlen("+HTTPRES: 1,200,");
                ret = sscanf(rcvPtr, "%d", &receiveFileLen);
                if(DEBUGMODE)printf("receiveLen: %d.\n", receiveFileLen);
                break;
            }
            if(timeoutc > 20)  //10s
            {
                printf("get file timeout!\n");
                goto TRD_R_ERR;
            }
            timeoutc++;
        }
    }
    //get file data
    if(ret == 1 && receiveFileLen > 0)
    {
        memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
        Uart2Send(sprintf(send2_buf, "AT+HTTPREAD=0,%d\r\n", receiveFileLen));
        usleep(receiveFileLen*50);
        //get polyline
        char *polyPtr = strstr(uart2ReadBuff, "{\"code\":0,\"message\":\"success\",\"data\":{\"polyLines\":");
        if (polyPtr != NULL)
        {
            polyPtr += strlen("{\"code\":0,\"message\":\"success\",\"data\":{\"polyLines\":");
            sscanf(polyPtr, "\"%lf,%lf-%lf,%lf\",", &poly.longitude_begin, 
                &poly.latitude_begin, &poly.longitude_end, &poly.latitude_begin);

            if(DEBUGMODE)
            {
                printf("poly: %.6f, %.6f - %.6f, %.6f\n", poly.longitude_begin, 
                    poly.latitude_begin, poly.longitude_end, poly.latitude_begin);
            }
        }
        else
        {
            printf("polyline not found.\n");
            goto TRD_R_ERR;
        }
        //get base64
        char *base64PtrBegin = strstr(uart2ReadBuff, "//F");
        if (base64PtrBegin != NULL)
        {
            char *base64PtrEnd = strstr(uart2ReadBuff, "\"}}");
            if (base64PtrEnd != NULL){
                base64BufLen = base64PtrEnd - base64PtrBegin;
                if(DEBUGMODE)printf("base64 length: %d bytes.\n", base64BufLen);
            }else{
                printf("\"}} not found!\n");
                goto TRD_R_ERR;
            }
            //allocate buffer
            char *base64Buf_r = (char *)malloc(base64BufLen+1);
            if (base64Buf_r == NULL) {
                printf("failed to allocate base64Buf_r\r\n");
                goto TRD_R_ERR;
            }
            strncpy(base64Buf_r, base64PtrBegin, base64BufLen);
            // base64Buf_r[base64BufLen] = "\0";
            // printf("\n\n%s\n\n",base64Buf_r);   //test
            usleep(10000);
            /*base64 decoder and save*/
            char *binaryBuf = (char *)malloc(base64BufLen);
            if (binaryBuf == NULL){
                printf("malloc binaryBuf fail\n");
                free(base64Buf_r);
                goto TRD_R_ERR;
            }
            ret = base64_decode(base64Buf_r, base64BufLen, binaryBuf, &binaryBufLen);
            if(ret != 0){
                printf("decoder error: %d\n", ret);
                free(base64Buf_r);
                free(binaryBuf);
                goto TRD_R_ERR;
            }

            FILE *audiofile = fopen(my_args->arg2_fileName, "wb"); 
            if (audiofile == NULL){
                printf("error opening file\n");
                free(binaryBuf);
                free(base64Buf_r);
                goto TRD_R_ERR;
            }
            free(base64Buf_r);
            base64Buf_r = NULL;
            fwrite(binaryBuf, 1, binaryBufLen, audiofile);
            if(DEBUGMODE)printf("audiofile length: %d bytes.\n", binaryBufLen);
            usleep(1000);
            fclose(audiofile);
            audiofile = NULL;
            free(binaryBuf);
            binaryBuf = NULL;
        }
        else
        {
            printf("base64 file not found.\n");
            goto TRD_R_ERR;
        }
    }

    free(my_args->arg1_audioName);
    free(my_args->arg2_fileName);
    free(my_args);
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    if(DEBUGMODE)printf("transfer thread exit\n");
    netBusy = 0;
    return (void*)0;

TRD_R_ERR:
    free(my_args->arg1_audioName);
    free(my_args->arg2_fileName);
    free(my_args);
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    netBusy = 0;
    return (void*)1;
}

///////////////////////////////////////////MAIN/////////////////////////////////////////////
int main_net(void)
{
    char ch;
    int ret;
    pthread_t t1_id, t2_id, t3_id, t4_id, t5_id, t6_id, t7_id;
    void *status1, *status2, *status3, *status4;

    sdk_init();
    ret = allUartInit();
    printf("ret: %d\n",ret);

    pthread_create(&t1_id, NULL, thread_uart2, NULL);
    pthread_create(&t2_id, NULL, thread_uart1, NULL);
    GNSS_Init();
    usleep(600000);
    AudioInit();
    ret = net_Init();
    if(ret != 0)
    {
        printf("init error!\r\n");
        while(1)
        {
            ch = getchar();
            if (ch == '\n'){
                break;
            }
        }
        sdk_exit();
        run_status = 1;
        // pthread_join(t1_id, NULL);
        // pthread_join(t2_id, NULL);
        return 0;
    }

    utcTimeUpdate_Beijing();
    usleep(200000);
    positionUpdate();
    printf("Enter\r\n");
    while(1)
    {
        ch = getchar();
        if (ch == '\n'){
            break;
        }
    }

    statusTransferTrd(&t3_id);
    pthread_join(t3_id, NULL);
    printf("Enter\r\n");
    while(1)
    {
        ch = getchar();
        if (ch == '\n'){
            break;
        }
    }
    
    // audioTransferTrd(&t4_id);
    // pthread_join(t4_id, &status3);
    // printf("Enter\r\n");
    // while(1)
    // {
    //     ch = getchar();
    //     if (ch == '\n'){
    //         break;
    //     }
    // }
    // if(status3 != 0)
    // {
    //     goto ERR;
    // }

    firstAudioReceiveTrd(&t6_id);
    pthread_join(t6_id, &status2);
    printf("Enter\r\n");
    while(1)
    {
        ch = getchar();
        if (ch == '\n'){
            break;
        }
    }
    if(status2 == 0)Play_audioFile(0);

    audioReceiveTrd(&t5_id, 1, 1);
    pthread_join(t5_id, &status1);
    printf("Enter\r\n");
    while(1)
    {
        ch = getchar();
        if (ch == '\n'){
            break;
        }
    }
    if(status1 == 0)Play_audioFile(1);

    // audioReceiveTrd(&t6_id, 2, 2);
    // pthread_join(t6_id, &status4);
    // printf("Enter\r\n");
    // while(1)
    // {
    //     ch = getchar();
    //     if (ch == '\n'){
    //         break;
    //     }
    // }
    // if(status4 == 0)Play_audioFile(2);


ERR:
    sdk_exit();

    usleep(1000000);
    printf("main thread\r\n");

    return 0;
}
///////////////////////////////////////////MAIN/////////////////////////////////////////////
/*
transfer thread, jpg and position, transfer twice
*/
void statusTransferTrd(pthread_t *transID)
{
    netBusy = 1;

    char *ossJpgFileName = (char*)malloc(30);
    utcTimeUpdate_Beijing();
    strcpy(ossJpgFileName, BeijingTime.timeString);

    if(pthread_create(transID, NULL, thread_status_transfer, (void*)ossJpgFileName) != 0)
    {
        free(ossJpgFileName);
        printf("create fileTransferTrd fail\r\n");
        return;
    }
}

/*
transfer thread, aac and position, transfer once
*/
void audioTransferTrd(pthread_t *transID)
{
    netBusy = 1;

    char *ossAacFileName = (char*)malloc(30);
    utcTimeUpdate_Beijing();
    strcpy(ossAacFileName, BeijingTime.timeString);

    if(pthread_create(transID, NULL, thread_audio_transfer, (void*)ossAacFileName) != 0)
    {
        free(ossAacFileName);
        printf("create audioTransferTrd fail\r\n");
        return;
    }
}

/*
receive thread
*/
void firstAudioReceiveTrd(pthread_t *rcvID)
{
    netBusy = 1;

    if(pthread_create(rcvID, NULL, thread_first_audio_receive, NULL) != 0)
    {
        printf("create audioReceiveTrd fail\n");
        return;
    }
    return;
}


void audioReceiveTrd(pthread_t *rcvID, uint8_t audioName, uint8_t fileName)
{
    netBusy = 1;
    thread_args_receive *receiveArgs = (thread_args_receive*)malloc(sizeof(thread_args_receive));
    if (receiveArgs == NULL)
    {
        printf("malloc args fail\n");
        return;
    }
    receiveArgs->arg1_audioName = (char*)malloc(16);
    if (receiveArgs->arg1_audioName == NULL)
    {
        printf("malloc args1 fail\n");
        return;
    }
    receiveArgs->arg2_fileName = (char*)malloc(32);
    if (receiveArgs->arg2_fileName == NULL)
    {
        printf("malloc args2 fail\n");
        free(receiveArgs->arg1_audioName);
        return;
    }

    sprintf(receiveArgs->arg1_audioName, "path%d", audioName);   //oss name
    sprintf(receiveArgs->arg2_fileName, "audio_%d.aac", fileName);   //local name

    if(pthread_create(rcvID, NULL, thread_audio_receive, receiveArgs) != 0)
    {
        free(receiveArgs->arg1_audioName);
        free(receiveArgs->arg2_fileName);
        free(receiveArgs);
        printf("create audioReceiveTrd fail\n");
        return;
    }
    return;
}

/*
4G module initialization
*/
uint8_t net_Init(void)
{
    uint8_t retAT = 0;
    usleep(300000);     //wait uart stable
    retAT = CheckAtCommand("AT\r\n", "OK");
    if(retAT == 1)
    {
        printf("AT HardFualt!\r\n");
        return 1;
    }
    usleep(100000);
    // CheckAtCommand("AT+CBAUD?\r\n", "OK");
    // usleep(200000);
    CheckAtCommand("ATE0\r\n", "OK");
    usleep(200000);
    // CheckAtCommand("AT+CPIN?\r\n", "OK");
    // usleep(200000);
    // CheckAtCommand("AT+CSQ?\r\n", "OK");   //no need
    // usleep(200000);
    // CheckAtCommand("AT+CEREG?\r\n", "OK");
    // usleep(200000);
    // CheckAtCommand("AT+COPS?\r\n", "OK");
    // usleep(200000);
    CheckAtCommand("AT+MIPCALL=1\r\n", "null");
    usleep(500000);
    // CheckAtCommand("AT+HTTPSET=\"UAGENT\",\"guardian\"\r\n", "OK");
    // usleep(200000);
    CheckAtCommand("AT+HTTPSET=\"CONTYPE\",\"application/json\"\r\n", "OK");
    usleep(200000);
    // CheckAtCommand("AT+HTTPSET?\r\n", "null");
    // usleep(200000);
    if(DEBUGMODE)printf("4G module initialization complete!\r\n");
    return 0;
}

/*
time update
*/
uint8_t utcTimeUpdate_Beijing(void)
{
    // +CCLK: "24/06/18,14:58:16+32"
    int ret = 0;
    rcv2_flag = 0;

    if(GNSS.GnssTimeValid)
    {
        BeijingTime.utc_year = GNSS.year;
        BeijingTime.utc_month = GNSS.month;
        BeijingTime.utc_day = GNSS.day;
        BeijingTime.utc_hour = GNSS.hour;
        BeijingTime.utc_minute = GNSS.minute;
        BeijingTime.utc_second = GNSS.second;

        memset(BeijingTime.timeString, 0, sizeof(BeijingTime.timeString));
        sprintf(BeijingTime.timeString, "20%d-%02d-%02d %02d:%02d:%02d", BeijingTime.utc_year, 
                        BeijingTime.utc_month, BeijingTime.utc_day, BeijingTime.utc_hour,
                        BeijingTime.utc_minute, BeijingTime.utc_second);
        // if(DEBUGMODE)printf("UTC+8 time: %s ===GNSS\r\n", BeijingTime.timeString);
        return 0;
    }

    Uart2Send(sprintf(send2_buf, "AT+CCLK?\r\n"));
    usleep(50000);
    if(rcv2_flag == 1)
    {
        char* timePtr = strstr(uart2ReadBuff, "+CCLK: \"");
        if (timePtr != NULL)
        {
            timePtr += strlen("+CCLK: \"");
        }
        ret = sscanf(timePtr, "%d/%2d/%2d,%2d:%2d:%2d",
                    &BeijingTime.utc_year, &BeijingTime.utc_month, &BeijingTime.utc_day, 
                    &BeijingTime.utc_hour,&BeijingTime.utc_minute, &BeijingTime.utc_second);
        BeijingTime.utc_month = 7;
        if(ret == 6)
        {
            memset(BeijingTime.timeString, 0, sizeof(BeijingTime.timeString));
            sprintf(BeijingTime.timeString, "20%02d-%02d-%02d %02d:%02d:%02d", BeijingTime.utc_year, 
                        BeijingTime.utc_month, BeijingTime.utc_day, BeijingTime.utc_hour,
                        BeijingTime.utc_minute, BeijingTime.utc_second);     
            // if(DEBUGMODE)printf("UTC+8 time: %s\r\n", BeijingTime.timeString);
            memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
            return 0;
        }
        else
        {
            if(DEBUGMODE)printf("AT+CCLK? command error: %d!", ret);
            strcpy(BeijingTime.timeString, "0000-00-00 00:00:00");
        }
    }
    else
    {
        if(DEBUGMODE)printf("AT+CCLK? command error: %d!", ret);
        strcpy(BeijingTime.timeString, "0000-00-00 00:00:00");
    }
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    return 1;
}

/*
position update
*/
uint8_t positionUpdate(void)
{
    int ret = 0;
    rcv2_flag = 0;
    uint8_t timeoutc = 0;

    if(GNSS.GnssPosValid)
    {
        position.longitude = GNSS.Longitude;
        position.latitude = GNSS.Latitude;
        printf("use gnss position\r\n");      //success
        return 0;
    }
    //+GTGIS: "108.7489727,34.015118"
    Uart2Send(sprintf(send2_buf, "AT+GTGIS=6\r\n"));
    while(rcv2_flag == 0)
    {
        usleep(1000000);
        if(timeoutc > 4)  //5s
        {
            printf("AT+GTGIS=6 command no respond!");
            position.longitude = 0.0;
            position.latitude = 0.0;
            memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
            return 1;
        }
        timeoutc++;
    }
    timeoutc = 0;
    while(1)
    {
        usleep(1000000);
        char* posPtr = strstr(uart2ReadBuff, "+GTGIS: \"1");
        if (posPtr != NULL)
        {
            ret = sscanf(posPtr, "+GTGIS: \"%lf,%lf\"", &position.longitude, &position.latitude);
            if(ret == 2)
            {    
                if(DEBUGMODE)printf("pos: %.6f, %.6f\r\n", position.longitude, position.latitude);
                memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
                return 0;     //success
            }
            else
            {
                printf("AT+GTGIS=6 command error: %d!", ret);
                position.longitude = 0.0;
                position.latitude = 0.0;
                memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
                return 1;
            }
        }
        if(timeoutc > 4)  //5s
        {
            printf("LBS network error\r\n");
            position.longitude = 0.0;
            position.latitude = 0.0;
            memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
            return 1;
        }
        timeoutc++;
    }
    
    position.longitude = 0.0;
    position.latitude = 0.0;
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));
    return 1;
}

uint8_t allUartInit(void)
{
    uint8_t ret1 = 0, ret2 = 0;
    ret1 = Uart1Init();
    ret2 = Uart2Init(B460800);
    usleep(100000);
    return ret1 && ret2;
}

uint8_t Uart2Init(int baud)
{
    char *uart2 = "/dev/ttyAMA2";
    Uart2PinConfig();
    uart2Fd = open(uart2, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart2Fd  < 0) 
    {
        printf("open %s is failed\n", uart2);
        return 1;
    }
    else 
    {
        Uart2Config(uart2Fd, B115200);
    }
    /*set baudrate*/
    CheckAtCommand("AT+CBAUD=460800\r\n", "null");
    usleep(10000);
    Uart2Config(uart2Fd, B460800);
    memset(uart2ReadBuff, 0, sizeof(uart2ReadBuff));

    return 0;
}

void Uart2Close(void)
{
    close(uart2Fd);
}

void allUartClose(void)
{
    Uart1Close();
    Uart2Close();
}
