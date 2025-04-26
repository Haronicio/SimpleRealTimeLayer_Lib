#ifndef SRTL_SYNC_H
#define SRTL_SYNC_H

#include "../core/core.h"
#include <WiFi.h>
#include <SD.h>
#include "time.h"

/*
    Sync is the communication with internet and/or storage, it manage synchronisation outside the local context of the platform
*/

// TODO sync struct with read and write function handler for typedef cloud and storage

#ifdef COMPILE_TIME
#define SYNC_SYSTIME_CONFIG COMPILE_TIME
#else
#define SYNC_SYSTIME_CONFIG 1738079979UL
#endif

// INFO : NEVER USED, setenv and tzset instead
// integer -12 -> +12
#define GMT_LOCAL 1
// 0-1 (De)activate auto daylight time 
#define DST_LOCAL 1

#define MAX_WAIT_TIME_WIFI 60000
#define MAX_ATTEMPT_WIFI 100
#define WAIT_TIME_ATTEMPT_WIFI (MAX_WAIT_TIME_WIFI / MAX_ATTEMPT_WIFI)

bool syncWifiInit(const char *ssid, const char *passphrase = (const char *)__null);

extern const char *ntpServer;
// const char *ntpServer = "pool.ntp.org";

// INFO : NEVER USED, setenv and tzset instead
const long gmtOffset_sec = GMT_LOCAL * 3600;
const int daylightOffset_sec = DST_LOCAL * 3600;

uint32_t getNTP(void);

#define SYNC_NTP_RESYNC_DELAY 86400000UL // (24h in ms)
// #define _K_MILLIS_TO_EPOCH ((uint64_t)(1ULL << 32) / 1000)
#define _K_MILLIS_TO_EPOCH 4294967ULL
#define MILLIS_TO_EPOCH(ms) (uint32_t)((uint64_t)(_K_MILLIS_TO_EPOCH * (ms)) >> 32)
#define CURRENT_MILLIS millis()

// pdMS_TO_TICK cannot handle more than 1h22 https://github.com/espressif/esp-idf/issues/4292
#define SAFE_MS_TO_TICKS(xTimeInMs) \
    ( ((xTimeInMs) < (0xFFFFFFFFU / configTICK_RATE_HZ) * 1000U) ? \
      (TickType_t)(((uint64_t)(xTimeInMs) * (uint64_t)configTICK_RATE_HZ) / 1000U) : \
      (TickType_t)0xFFFFFFFFU )


int32_t syncTimer(uint32_t targetTime);


#define CURRENT_EPOCH (-syncTimer(0))

// Event Timer system

#define NBITS_EVENTTIMER_ID 4
#define MAX_EVENT_TIMER_IN_MODULE (1 << NBITS_EVENTTIMER_ID)
#define BIT_EVENT_TIMER_ACTIVE 0
#define BIT_EVENT_TIMER_PERIOD 1
#define BIT_EVENT_TIMER_ISRECT 2
#define BIT_EVENT_TIMER_ISNEXT 3

#define GET_EVENT_TIMER_ACTIVE(eventFlags) (eventFlags & 1) 
#define GET_EVENT_TIMER_PERIOD(eventFlags) ((eventFlags >> BIT_EVENT_TIMER_PERIOD) & 1) 
#define GET_EVENT_TIMER_ISRECT(eventFlags) ((eventFlags >> BIT_EVENT_TIMER_ISRECT) & 1)
#define GET_EVENT_TIMER_ISNEXT(eventFlags) ((eventFlags >> BIT_EVENT_TIMER_ISNEXT) & 1)
#define GET_EVENT_TIMER_ID(eventFlags) (((eventFlags >> (8 - NBITS_EVENTTIMER_ID)) & (MAX_EVENT_TIMER_IN_MODULE - 1)))

#define SET_EVENT_TIMER_ACTIVE(eventFlags, value) (eventFlags = (eventFlags & ~(1 << BIT_EVENT_TIMER_ACTIVE)) | ((value & 1) << BIT_EVENT_TIMER_ACTIVE))
#define SET_EVENT_TIMER_PERIOD(eventFlags, value) (eventFlags = (eventFlags & ~(1 << BIT_EVENT_TIMER_PERIOD)) | ((value & 1) << BIT_EVENT_TIMER_PERIOD))
#define SET_EVENT_TIMER_ISRECT(eventFlags, value) (eventFlags = (eventFlags & ~(1 << BIT_EVENT_TIMER_ISRECT)) | ((value & 1) << BIT_EVENT_TIMER_ISRECT))
#define SET_EVENT_TIMER_ISNEXT(eventFlags, value) (eventFlags = (eventFlags & ~(1 << BIT_EVENT_TIMER_ISNEXT)) | ((value & 1) << BIT_EVENT_TIMER_ISNEXT))
#define SET_EVENT_TIMER_ID(eventFlags, value)     (eventFlags = (eventFlags & ~(((1 << NBITS_EVENTTIMER_ID) - 1) << (8 - NBITS_EVENTTIMER_ID))) \
                                                              | ((value & ((1 << NBITS_EVENTTIMER_ID) - 1)) << (8 - NBITS_EVENTTIMER_ID)))



/*
    Structure de communication intra-Tâche sur la base d'évènement temporel supérieur à une seconde
    Utile lorsqu'on veut qu'une tâche passe beaucoup de temps à dormir, et informer sa MAE
    (Les autres tâches peuvent accéder de manière non protégé, il est pas possible d'utiliser un mutex sur un eventTimer géré par autoTimer) 
    
    WARNING eventTimer are heavy memory load 21 octets !!!
*/


typedef struct eventTimer
{
    uint8_t flags; // bits : 0 isactiv, 1 isperiodic, 2 isrectified, 3 isnext , 4-7 id 
    uint32_t eventCount; // occurence de l'évenements effectué
    uint32_t missedCount; // occurence de l'évenements manqué
    uint32_t time; // in s
    uint32_t period; // in s
    uint32_t lastTimer; // in s
}eventTimer;


// TODO gestion eventTimer

/*
    compute Missed count
*/
inline uint32_t getMissedEventCounts(eventTimer *event)
{
    if (event->period == 0)
    {
        return 0;
    }

    return (CURRENT_EPOCH - event->time) / event->period;
}

inline eventTimer createEventTimer(bool isactive,bool isperiodic,bool isrectified,uint32_t time, uint32_t period){
    uint8_t flags = 0;
    SET_EVENT_TIMER_ACTIVE(flags,isactive);
    SET_EVENT_TIMER_PERIOD(flags,isperiodic);
    SET_EVENT_TIMER_ISRECT(flags,isrectified);
    // flags, eventCount, missedCount,time,period,lastTimer
    eventTimer ret = {flags,0,0,time,period,0};

    return ret;
}

void addEvent(eventTimer *eventList, uint8_t *nEvent,
              bool isActive, bool isRectified, uint32_t time, uint32_t period);


#define SDCARD_CHIPSELECT 2

bool syncSDCardInit(uint8_t chipSelectPin) ;

#endif // SRTL_SYNC_H
