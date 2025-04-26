#ifndef SRTL_H
#define SRTL_H

#include "core/core.h"
#include "ihm/ihm.h"
#include "sync/sync.h"
// #include "util/log.h"

class SRTL
{
public:
    // Constructeur et destructeur
    SRTL();
    ~SRTL();

    // Initialisation
    SemaphoreHandle_t init();

    // Gestion des modules
    uint8_t registerModule(TaskFunction_t taskFunction, const char *name, uint32_t stackSize, uint8_t priority,
                           uint32_t ressourceOfInterest, uint32_t taskFrequency, void *moduleparameters);

    // Gestion des ressources partagées
    uint8_t registerSharedResource(protected_data_t& data);

    // Notifications
    uint8_t notifyModule(uint16_t destModuleIndex, uint16_t srcModuleBits, uint32_t sharedRessourceBits, eNotifyAction action);
    uint8_t notifyAll(uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action);

    // Gestion de l'IHM
    void registerMonitor(TaskFunction_t taskFunction, const char *name, uint32_t stackSize, uint8_t priority,
                         uint32_t taskFrequency, void *monitorparameters);
    uint8_t notifyMonitor(uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action);

    // Gestion des contrôleurs
    uint8_t registerController(void (*digitalHandler)(void *), PinControllerParam digitalPins[],
                               TimerCallbackFunction_t analogHandler, PinControllerParam analogPins[], uint8_t analogT, void *parameters);

    uint32_t join(uint8_t joinListIndex, uint8_t modulePtrIndex);
    uint32_t release(uint8_t joinListIndex, uint8_t srcModuleIndex = 0);
    uint32_t unjoin(uint8_t joinListIndex, uint8_t modulePtrIndex, uint8_t srcModuleIndex = 0);

    // Gestion du WiFi
    bool setupWiFi(const char *ssid, const char *passphrase = nullptr);

    // Gestion du Timer
    uint8_t autoTimer(Module *modulePtr, eventTimer *events, uint8_t nEvent);

    /* 
        If the task referenced by xTask was not in the Blocked state then pdFAIL is returned. Otherwise pdPASS is returned. 
        resetLast indicate to replace lastAwake
    */
    inline BaseType_t abortDelay(uint8_t moduleID){

        this->moduleList[moduleID].abort_requested.store(true);
        return xTaskAbortDelay(this->moduleList[moduleID].handle);
        
        // Note ne pas faire ca ici (condition de course) plutot utilisé un atomic pour anticiper
        // IMPORTANT : Si cette ligne n'est pas présente, lors du abort, lastawake n'est plus à jour et l'ordonnanceur de freertos bug
        // if(resetLast)
		//     this->moduleList[moduleID].lastAwake = xTaskGetTickCount();
    }

    // Gestion des accès aux ressources partagées
    bool lock(uint8_t protectedRessourceIndex, uint32_t delay = MAX_DELAY);
    bool unlock(uint8_t protectedRessourceIndex);

    // Récupération de l'instance
    static SRTL &GetCurrentInstance(Module *modulePtr);

    // Variables internes
    uint8_t nModule;
    Module moduleList[MAX_MODULES];
    SemaphoreHandle_t xMutex_NotifyAll;
    protected_data_t *sharedRessources[MAX_SHARED_RESOURCES];
    uint8_t nSharedResource;
    uint32_t joinList[MAX_JOIN_LIST] = {0};
    SemaphoreHandle_t xMutex_JoinList;
    Monitor sysMonitor;
    uint8_t nController;
    Controller controllerList[MAX_CONTROLLER];
    SemaphoreHandle_t xMutex_SyncTimer;
};

#endif // SRTL_H
