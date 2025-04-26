#include "log.h"
#include "freertos/semphr.h"

SemaphoreHandle_t  xMutex_print = NULL;

void logSystemInit(esp_log_level_t level , bool serialEnable , bool sdEnable , bool flashEnable) {
    
    xMutex_print = xSemaphoreCreateMutex();
    if(xMutex_print == NULL){
        Serial.println("Error : Cannot initialize log system (Failed to allocate Mutex)");
    }

    setGlobalLogLevel(level);

    esp_log_set_vprintf(custom_vprintf);
    
    SRTL_useSerial = serialEnable && Serial;
    
    SRTL_useSD = sdEnable && SD.begin();
    if (SRTL_useSD) {
        File logFile = SD.open("/log.txt", FILE_APPEND);
        if (logFile) {
            // logFile.println("=== LOG SD START ===");
            logFile.close();
        } else {
            SRTL_useSD = false; // Échec ouverture fichier
        }
    }

    SRTL_useFlash = flashEnable && flashLog.begin("logflash", false);

    writeLog("============= LOG START ============\n");
}

// Écriture générique vers les supports actifs
void writeLog(const char* buffer) {
    if (SRTL_useSerial) {
        Serial.print(buffer);
    }

    if (SRTL_useSD) {
        File logFile = SD.open("/log.txt", FILE_APPEND);
        if (logFile) {
            logFile.print(buffer);
            logFile.close();
        }
    }

    if (SRTL_useFlash) {
        size_t logCount = flashLog.getUInt("count", 0);
        char key[SRTL_LOG_KEY_SIZE];
        snprintf(key, sizeof(key), "log%u", (unsigned)logCount);
        flashLog.putString(key, buffer);
        flashLog.putUInt("count", logCount + 1);
    }
}

// Fonction vprintf personnalisée pour esp_log_set_vprintf
int custom_vprintf(const char* format, va_list args) {
    int len = -1;
    // if(xSemaphoreTake(xMutex_print, portMAX_DELAY)){
    char buffer[SRTL_LOG_MSG_SIZE] = {'\0'};
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    writeLog(buffer);
    // xSemaphoreGive(xMutex_print);
    // }
    return len;
}

// Gestion des logs FreeRTOS (configPRINTF)
extern "C" void vLoggingPrintf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    custom_vprintf(format, args);
    va_end(args);
}

// Gestion personnalisée en cas de Panic (backtrace)
// extern "C" void esp_panic_handler(panic_info_t *info) {
//     char panicBuffer[SRTL_LOG_MSG_SIZE] = {'\0'};
//     snprintf(panicBuffer, sizeof(panicBuffer), "\n!!! PANIC : %s !!!\n", info->reason);
//     writeLog(panicBuffer);
    
//     // Écriture backtrace standard dans buffer
//     // char backtraceBuffer[512];
//     // esp_back
//     // esp_panic_print_backtrace(backtraceBuffer, sizeof(backtraceBuffer));
//     // writeLog(backtraceBuffer);

//     esp_restart();
// }

void readFlashLogs(){
    flashLog.begin("logflash", true);
    unsigned logCount = flashLog.getUInt("count", 0);
    for(unsigned i = 0; i < logCount; i++){
      char key[SRTL_LOG_KEY_SIZE];
      snprintf(key, sizeof(key), "log%u", i);
      String entry = flashLog.getString(key, "");
      Serial.print(entry);
    }
    flashLog.end();
  } 


  void srtl_log(SrtlLogLevel level, const char* tag, const char* format, ...) {
    char logBuffer[SRTL_LOG_MSG_SIZE];
    char formattedMessage[SRTL_LOG_MSG_SIZE];

    // Obtention du timestamp en millisecondes depuis le démarrage
    uint32_t timestamp_ms = esp_log_timestamp();

    // Traitement des arguments variadiques pour formater le message utilisateur
    va_list args;
    va_start(args, format);
    vsnprintf(formattedMessage, sizeof(formattedMessage), format, args);
    va_end(args);

    // Création de la chaîne complète avec timestamp, niveau et tag
    snprintf(logBuffer, sizeof(logBuffer), "( %10lu ms )\t [%s]\t [%s]:\t %s\n",
             (unsigned long)timestamp_ms, levelToString((esp_log_level_t)level), tag, formattedMessage);

    // Utilisation de votre fonction existante d'écriture vers les supports (serial, SD, flash)
    writeLog(logBuffer);

    // Envoi vers le log système interne d'ESP-IDF (facultatif, mais conseillé pour le debug via UART)
    esp_log_write((esp_log_level_t)level, tag, "%s", formattedMessage);
}