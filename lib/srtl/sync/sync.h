#ifndef SYNC_H
#define SYNC_H

#include "../core/core.h"
#include <WiFi.h>
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

#define MAX_WAIT_TIME_WIFI 60000
#define MAX_ATTEMPT_WIFI 100
#define WAIT_TIME_ATTEMPT_WIFI (MAX_WAIT_TIME_WIFI / MAX_ATTEMPT_WIFI)

bool syncWifiInit(const char *ssid, const char *passphrase = (const char *)__null)
{
    WiFi.disconnect(true);

    WiFi.begin(ssid, passphrase);
    Serial.println("Connect to WiFi");

    // Wait for connection
    uint8_t i = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        vTaskDelay(pdMS_TO_TICKS(WAIT_TIME_ATTEMPT_WIFI));
        Serial.print(".");

        if (++i > MAX_ATTEMPT_WIFI)
        {
            Serial.println("Error Connection Timeout");
            return false;
        }
    }
    Serial.print("\n Connected to: \t ");
    Serial.println(ssid);
    Serial.print("IP address: \t");
    Serial.println(WiFi.localIP());

    return true;
}

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

uint32_t getNTP(void)
{
    

    if (WiFi.status() == WL_CONNECTED)
    {
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

        struct tm timeinfo;
        if (!getLocalTime(&timeinfo, 5000))
        {
            Serial.println("Failed to obtain NTP time");
            return 0;
        }
        return (uint32_t)mktime(&timeinfo);
    }

    Serial.println("WiFi not connected.");
    return 0;
}

#define SYNC_NTP_RESYNC_DELAY 86400000UL // (24h in ms)
// #define _K_MILLIS_TO_EPOCH ((uint64_t)(1ULL << 32) / 1000)
#define _K_MILLIS_TO_EPOCH 4294967ULL
#define MILLIS_TO_EPOCH(ms) (uint32_t)((uint64_t)(_K_MILLIS_TO_EPOCH * (ms)) >> 32)
#define CURRENT_MILLIS millis()



static int32_t syncTimer(uint32_t targetTime)
{
    // uint32_t currentMillis = millis();
    static uint32_t driftEpoch = SYNC_SYSTIME_CONFIG + MILLIS_TO_EPOCH(CURRENT_MILLIS) ; //time drift since build + launch

    // TODO WARNING pour éxécuter la première fois
    static uint32_t lastNtpSync = INT32_MAX;
    if ((CURRENT_MILLIS - ((int32_t)lastNtpSync)) >= SYNC_NTP_RESYNC_DELAY)
    {
        uint32_t ntpTime = getNTP(); // ajustement brutale (https://fr.wikipedia.org/wiki/Heure_Unix#Utilisation_d'une_r%C3%A9solution_inf%C3%A9rieure_%C3%A0_la_seconde)
        if (ntpTime > 0)
        {
            driftEpoch = ntpTime - MILLIS_TO_EPOCH(CURRENT_MILLIS); //time drift since last sync
            lastNtpSync = CURRENT_MILLIS;

            Serial.println("NTP success");
        }
    }

    uint32_t currentEpoch = driftEpoch + MILLIS_TO_EPOCH(CURRENT_MILLIS);

    return (int32_t)(targetTime - currentEpoch);
}


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
    
    WARNING eventTimer are heavy memory load
*/
typedef struct 
{
    uint8_t flags; // bits : 0 isactiv, 1 isperiodic, 2 isrectified, 3 isnext , 4-7 id 
    uint32_t eventCount; // occurence du Timer
    uint32_t time; // in s
    uint32_t period; // in s
    uint32_t lastExecution; // in s
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
    eventTimer ret = {flags,0,time,period,0};

    return ret;
}

void addEvent(eventTimer *eventList, uint8_t *eventCount, uint8_t maxSize,
              bool isActive, bool isRectified, uint32_t time, uint32_t period)
{

    if(eventList == nullptr){
        Serial.println("Erreur: Tableau d'événements n'est pas initialiser.");
        return;
    }

    if(maxSize >= MAX_EVENT_TIMER_IN_MODULE ){
        Serial.println("Erreur: Tableau d'événements dépasse la taille autorisé.");
        return;
    }

    if (*eventCount >= maxSize)
    {
        Serial.println("Erreur: Tableau d'événements plein.");
        return;
    }

    eventTimer newEvent = createEventTimer(isActive, (period > 0), isRectified, time, period);
    SET_EVENT_TIMER_ID(newEvent.flags, *eventCount);

    eventList[*eventCount] = newEvent;
    (*eventCount)++;

    Serial.printf("Événement ajouté avec ID %d, time: %d, period: %d\n",GET_EVENT_TIMER_ID(newEvent.flags), time, period);
}

#endif // SYNC_H
