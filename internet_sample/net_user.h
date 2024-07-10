
#ifndef NET_USER_H
#define NET_USER_H

#include <stdint.h>
#include <pthread.h>

extern char uart2ReadBuff[98304];
extern char send2_buf[2048];
extern uint8_t rcv2_flag;
extern uint8_t netBusy;
int8_t Uart2Read(void);

typedef struct {
    char *arg1_ossFileName;
    char *arg2_fileName;
} thread_args_transfer;

typedef struct {
    char *arg1_audioName;
    char *arg2_fileName;
} thread_args_receive;

typedef struct {
    uint16_t utc_year;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_month;
    uint8_t utc_second;
    char timeString[21];
} UTCtime;

typedef struct {
    double latitude;
    double longitude;
} Position;

typedef struct {
    double latitude_begin;
    double longitude_begin;
    double latitude_end;
    double longitude_end;
} polyLine;

extern UTCtime BeijingTime;
extern polyLine poly;

uint8_t allUartInit(void);

uint8_t Uart2Init(int baud);

uint8_t utcTimeUpdate_Beijing(void);

uint8_t positionUpdate(void);

uint8_t net_Init(void);

void statusTransferTrd(pthread_t *transID);

void audioTransferTrd(pthread_t *transID);  //get route navi path

void firstAudioReceiveTrd(pthread_t *rcvID);

void audioReceiveTrd(pthread_t *rcvID, uint8_t audioName, uint8_t fileName);

void Uart2Close(void);

void allUartClose(void);

// uint32_t RegTarget = 0x114F005C;
// uint32_t RegWriteValue = 0x0433;
// RegTarget = 0x114F0058;
// RegWriteValue = 0x0533;

#endif
