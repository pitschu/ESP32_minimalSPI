/*
 * appl.h
 *
 *  Created on: 03.03.2017
 *      Author: pit
 */

#ifndef MAIN_APPL_H_
#define MAIN_APPL_H_

#include "tcpip_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#ifndef DEBUG
#define DEBUG		2
#endif
#ifndef DEBUG_COLOR
#define DEBUG_COLOR	_ANSI_GREEN_
#endif

//#undef DEBUG
//#define DEBUG 0

#define _ANSI_NONE_
// color codes from:
#define _ANSI_NONE_
#define _ANSI_WHITE_	"\x1b[1;37m"
#define _ANSI_RED_		"\x1b[0;31m"
#define _ANSI_YELLOW_	"\x1b[1;33m"
#define _ANSI_GREEN_	"\x1b[0;32m"
#define _ANSI_BLUE_		"\x1b[0;34m"
#define _ANSI_MAGENTA_	"\x1b[0;35m"
#define _ANSI_CYAN_		"\x1b[0;36m"

#define _ANSI_BROWN_	"\x1b[0;33m"
#define _ANSI_GREY_		"\x1b[0;37m"
#define _ANSI_DGREY_	"\x1b[1;30m"
#define _ANSI_LBLUE_	"\x1b[1;34m"
#define _ANSI_LGREEN_	"\x1b[1;32m"
#define _ANSI_LCYAN_	"\x1b[1;36m"
#define _ANSI_LRED_		"\x1b[1;31m"
#define _ANSI_LPURPLE_	"\x1b[1;35m"

#define _ALERT_RED_		"\x1b[0;31;41m"
#define _ALERT_OFF_		"\x1b[39;49m"

char					logTimeBuf[32];

#if DEBUG > 0
#define PRT_DEBUG(a,...)		printf(DEBUG_COLOR"%s [%d]:%5d:%5d ", logTimeBuf, xPortGetCoreID(), xPortGetFreeHeapSize(), (int)uxTaskGetStackHighWaterMark(NULL)); printf((a), ##__VA_ARGS__)
#define ALERT_DEBUG(a,...)		printf(_ALERT_RED_"%s [%d]:%5d:%5d "_ALERT_OFF_ DEBUG_COLOR, logTimeBuf, xPortGetCoreID(), xPortGetFreeHeapSize(), (int)uxTaskGetStackHighWaterMark(NULL)); printf((a), ##__VA_ARGS__)
#define PRT_POS()				printf(DEBUG_COLOR"--> line %4d\n", (int)__LINE__)
#else
#define PRT_DEBUG(a,...)
#endif

#if DEBUG > 1
#define LOG_DEBUG(a,...)		{printf(DEBUG_COLOR"%s [%d]:%5d:%5d ", logTimeBuf, xPortGetCoreID(), xPortGetFreeHeapSize(), (int)uxTaskGetStackHighWaterMark(NULL)); printf((a), ##__VA_ARGS__);}
#define LOG_ASSERT(e)       ((e) ? (void)0 : LOG_DEBUG("%s()%d): %s\n", __FILE__, __LINE__, #e))
#else
#define LOG_DEBUG(a,...)
#define LOG_ASSERT(e)
#endif

#if DEBUG > 2
#define TRC_DEBUG(a,...)		printf(DEBUG_COLOR"%s %5d:%5d ", logTimeBuf, system_get_free_heap_size(), (int)uxTaskGetStackHighWaterMark(NULL)); printf((a), ##__VA_ARGS__)
#else
#define TRC_DEBUG(a,...)
#endif

#define	HTTPD_TASK_PRIO			3
#define	FLASH_TASK_PRIO			9		// scheduler for flash param store jobs
#define	BOOT_TASK_PRIO			9		// task to run post-boot jobs once (should have highest prio)
#define	MAIN_TASK_PRIO			6		// main display and event handler task
#define CAPT_DNS_TASK_PRIO		4
#define WIFIWATCH_TASK_PRIO		4
#define SNTP_TASK_PRIO			4
#define AMBIENT_TASK_PRIO		4

// pitschu: some very useful and often used macros
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#define constrain(x,low,high) (((x)<=(low))?(low):(((x)>=(high))?(high):(x)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define putInRange(V,VMIN,VMAX) 		max(VMIN,min(VMAX,V))
#define mapToRange(V,VMIN0,VMAX0,VMIN1,VMAX1) ((VMIN1) + (putInRange(V,VMIN0,VMAX0) - (VMIN0)) * ((VMAX1) - (VMIN1)) / ((VMAX0) - (VMIN0)))

#define IP4_CONV(a,b,c,d) (((u32_t)((d) & 0xff) << 24) | \
                           ((u32_t)((c) & 0xff) << 16) | \
                           ((u32_t)((b) & 0xff) << 8)  | \
                            (u32_t)((a) & 0xff))

#define ST_CONNECTED_BIT 	BIT0
#define  ST_GOTIP_BIT 		BIT1
#define  AP_RUNNING 		BIT2		// 0 = AP is stopped, 1 = AP is running
#define  AP_TARGETSTATE		BIT3		// 0 = stop AP, 1 = start AP
#define  AP_FASTSHUTDOWN	BIT4		// 1 = stop AP now

#define INDICATOR_NTP		0			// indicator LEDs
#define INDICATOR_AP		1
#define INDICATOR_OFFLINE	2
#define INDICATOR_3			3

// following externs are stored in PARARAM section
extern char     softapSSID[32];
extern char     softapPassword[32];
extern int      softapChannel;
extern int		softapTimeout;
extern tcpip_adapter_ip_info_t softapIPinfo;
extern short	stationReconfigure;


extern char     stationConnectSSID[32];
extern char     stationConnectPassword[32];
extern char     stationName[32];
extern uint8_t  stationUseDHCP;
extern tcpip_adapter_ip_info_t stationIPinfo;
extern char     deviceName[32];
extern char     ntpServer_1[64];
extern char     ntpServer_2[64];
extern char     ntpServer_3[64];

extern uint16_t 				nightModeFrom;	// 22:00 Uhr
extern uint16_t 				nightModeUntil;	// 07:00 Uhr
extern uint8_t					isNightMode;

extern uint8_t					mainMode;
#define	MODE_MINICLOCK			1
#define	MODE_MOODLIGHT			2
#define	MODE_CALIBRATE			3
#define	MODE_STANDBY			9

// ------------- end of PARARAM vars -----------------

extern EventGroupHandle_t wifi_event_group;

void hexDump(const uint8_t *p, int size);

#endif /* MAIN_APPL_H_ */
