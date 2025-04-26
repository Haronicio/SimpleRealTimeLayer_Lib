#ifndef _SRTLLOGSYSTEM_H
#define _SRTLLOGSYSTEM_H

#include <Arduino.h>
#include <SD.h>
#include <Preferences.h>

#include "esp_log.h"
#include "esp_private/panic_internal.h"

// #include "../sync/sync.h"

#define SRTL_LOG_MSG_SIZE 1024
#define SRTL_LOG_KEY_SIZE 32

#define SYS_TAG "SRTL"

#undef configPRINTF
#define configPRINTF(X) vLoggingPrintf X


// #define LOG_VERBOSE(tag,format,__VA_ARGS__...) ESP_LOG_VERBOSE(tag,format,## __VA_ARGS__)

typedef enum {
    SRTL_LOG_ERROR = ESP_LOG_ERROR,
    SRTL_LOG_WARN = ESP_LOG_WARN,
    SRTL_LOG_INFO = ESP_LOG_INFO,
    SRTL_LOG_DEBUG = ESP_LOG_DEBUG,
    SRTL_LOG_VERBOSE = ESP_LOG_VERBOSE,
} SrtlLogLevel;

// Flags internes pour indiquer quel support est initialisé
static bool SRTL_useSerial = false;
static bool SRTL_useSD = false;
static bool SRTL_useFlash = false;

// Objet global pour stockage flash
static Preferences flashLog;

// Ajuste la sensibilité globale des logs ESP
inline void setGlobalLogLevel(esp_log_level_t level) {
    esp_log_level_set("*", level);
    esp_log_level_set("arduino", ESP_LOG_VERBOSE);
}

// Initialisation simple du système de logs
void logSystemInit(esp_log_level_t level = ESP_LOG_VERBOSE, bool serialEnable = true, bool sdEnable = true, bool flashEnable = false) ;

// Écriture générique vers les supports actifs
void writeLog(const char* buffer) ;


// Fonction vprintf personnalisée pour esp_log_set_vprintf
int custom_vprintf(const char* format, va_list args) ;

// Gestion des logs FreeRTOS (configPRINTF)
extern "C" void vLoggingPrintf(const char* format, ...);

// Gestion personnalisée en cas de Panic (backtrace)
// extern "C" void esp_panic_handler(panic_info_t *info) ;

void readFlashLogs();

inline const char* levelToString(esp_log_level_t level) {
    switch (level) {
        case ESP_LOG_ERROR:   return "ERROR";
        case ESP_LOG_WARN:    return "WARN";
        case ESP_LOG_INFO:    return "INFO";
        case ESP_LOG_DEBUG:   return "DEBUG";
        case ESP_LOG_VERBOSE: return "VERBOSE";
        default:              return "UNKNOWN";
    }
}

void srtl_log(SrtlLogLevel level, const char* tag, const char* format, ...) ;

#endif // _SRTLLOGSYSTEM_H
