
#ifndef USERMAIN_H
#define USERMAIN_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdint.h>
// #include "imu_user.h"
// #include "i2c_user.h"
// #include "gpio_user.h"
// #include "kalman.h"
// #include "lhpf.h"
// #include "gpio_user.h"
// #include "gnss_user.h"
// #include "dtof_user.h"
// #include "net_user.h"
// #include "base64_user.h"
// #include "audio_aac_adp.h"
// #include "audio_dl_adp.h"
// #include "sample_audio.h"
// #include "sample_media_ai.h"
// #include "obstacle_detect_classify.h"
// #include "hi_mipi_tx.h"
// #include "sdk.h"
// #include "sample_comm.h"
// #include "ai_infer_process.h"
// #include "vgs_img.h"
// #include "osd_img.h"
// #include "posix_help.h"

//////////////////////////////////////////////////////////////////////////////////////////
/*function*/
long long Now_Timestamp(void);
void getDirection(void);

/*thread function*/
void* thread_init1(void); //THREAD INIT1,2,3
void* thread_init2(void);
void* thread_init3(void);
void* thread_uart1Rcv(void); //THREAD UART1,2
void* thread_uart2Rcv(void);
void* thread_obstacleAvoidance(void); //THREAD ALWAYS RUNNING
void* walkingNavigation(void);
void* thread_shutdown(void);

//////////////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif