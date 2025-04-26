#include "sync.h"

const char *ntpServer = "pool.ntp.org";

bool syncWifiInit(const char *ssid, const char *passphrase)
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

uint32_t getNTP(void)
{
    

    if (WiFi.status() == WL_CONNECTED)
    {
        // INFO : NEVER USED, setenv and tzset instead
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

        struct tm timeinfo; // local time
        if (!getLocalTime(&timeinfo, 5000))
        {
            Serial.println("Failed to obtain NTP time");
            return 0;
        }
        return (uint32_t)mktime(&timeinfo); //global epoch
    }

    Serial.println("WiFi not connected.");
    return 0;
}

int32_t syncTimer(uint32_t targetTime)
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

void addEvent(eventTimer *eventList, uint8_t *nEvent,
    bool isActive, bool isRectified, uint32_t time, uint32_t period)
{

if(eventList == nullptr){
Serial.println("Erreur: Tableau d'événements n'est pas initialiser.");
return;
}

if (*nEvent >= MAX_EVENT_TIMER_IN_MODULE)
{
Serial.println("Erreur: Tableau d'événements plein.");
return;
}

eventTimer newEvent = createEventTimer(isActive, (period > 0), isRectified, time, period);
SET_EVENT_TIMER_ID(newEvent.flags, *nEvent);

eventList[*nEvent] = newEvent;
(*nEvent)++;

Serial.printf("Événement ajouté avec ID %d, time: %d, period: %d\n",GET_EVENT_TIMER_ID(newEvent.flags), time, period);
}

bool syncSDCardInit(uint8_t chipSelectPin) {

    Serial.println("Access to SD card");

    pinMode(chipSelectPin, OUTPUT);

    if (!SD.begin(chipSelectPin)) {
        Serial.println("Error Cannot access to SD Card");
        return false;
    }

    if (SD.cardType() == CARD_NONE) {
        Serial.println("Error Wrong");
        return false;
    }
    Serial.print("\tCard type : \t");
    switch (SD.cardType()) {
        case CARD_MMC: Serial.println("MMC"); break;
        case CARD_SD: Serial.println("SDSC"); break;
        case CARD_SDHC: Serial.println("SDHC"); break;
        default: Serial.println("Unknown");
    }

    uint64_t cardSize = SD.cardSize() >> 20 ;
    Serial.print("\t Card capacity : \t");
    Serial.print(cardSize);
    Serial.println(" Mo");

    return true;
}