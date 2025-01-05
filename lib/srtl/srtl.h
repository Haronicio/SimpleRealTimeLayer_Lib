#ifndef SRTL_H
#define SRTL_H

#include "core/core.h"
#include "ihm/ihm.h"


// #ifdef __cplusplus
// extern "C" {
// #endif

#ifndef C_ONLY

class SRTL
{
// private:
public: //TODO getter and setter

    // CORE

    uint8_t nModule;
    Module moduleList[MAX_MODULES];
    SemaphoreHandle_t xMutex_NotifyAll;

    void * sharedRessources[MAX_SHARED_RESOURCES];
    uint8_t nSharedResource;

    uint32_t joinList[MAX_JOIN_LIST] = {0}; // TODO : add more module (use uint32_t but due to splitting 32 bit to Module and SharedResource ... see NOTIFY_MSG uses)
    SemaphoreHandle_t xMutex_JoinList;

    // IHM

    Monitor sysMonitor;

    uint8_t nController;
    Controller controllerList[MAX_CONTROLLER];

// public:
    // Constructeur
    SRTL()
    {
        nModule = 0;
        nSharedResource = 0;
        nController = 0;
        this->init();
    }

    // Destructeur
    ~SRTL() {
        // Détruire la sémaphore de notifyAll
        if (this->xMutex_NotifyAll != nullptr) {
            vSemaphoreDelete(xMutex_NotifyAll);
        }

        // Détruire les sémaphores des ressources partagées
        for (uint8_t i = 0; i < nSharedResource; ++i) {
            protected_data_t* data = reinterpret_cast<protected_data_t*>(sharedRessources[i]);
            if (data != nullptr && data->mutex != nullptr) {
                vSemaphoreDelete(*(data->mutex));
            }
        }

        // Supprimer toutes les tâches
        for (uint8_t i = 0; i < nModule; ++i) {
            if (moduleList[i].handle != nullptr) {
                vTaskDelete(moduleList[i].handle);
            }
        }
    }

    // Initialisation
    SemaphoreHandle_t init()
    {
        xMutex_JoinList = xSemaphoreCreateMutex();
        return xMutex_NotifyAll = xSemaphoreCreateMutex();
    }

    // CORE

    // Gestion des modules
    inline uint8_t registerModule(TaskFunction_t taskFunction, const char *name, uint32_t stackSize, uint8_t priority,
                                  uint32_t ressourceOfInterest, uint32_t taskFrequency,
                                  void *moduleparameters)
    {
        if (this->nModule >= MAX_MODULES)
        {
            // Serial.println("Error: No space left in moduleList array");
            while (1)
                ;
        }

        this->moduleList[this->nModule].parent = this;
        createModule(taskFunction, name, stackSize, moduleparameters, priority, ressourceOfInterest, taskFrequency, &this->moduleList[this->nModule]);
        
        return this->nModule++;
    }

    // Gestion des ressources partagées
    inline uint8_t registerSharedResource(protected_data_t data)
    {
        if (this->nSharedResource >= MAX_SHARED_RESOURCES)
        {
            // Serial.println("Error: No space left in sharedRessources array");
            while (1)
                ;
        }
        createSharedResource(&this->sharedRessources[this->nSharedResource], data.mutex, data.data);
        return this->nSharedResource++;
    }

    // Notifications
    uint8_t notifyModule(uint16_t destModuleIndex, uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action)
    {
        return xTaskNotify(this->moduleList[destModuleIndex].handle, NOTIFY_MSG(sharedRessourceBits, srcModuleIndex), action);
    }

    uint8_t notifyAll(uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action)
    {
        uint8_t success = pdFALSE;
        xSemaphoreTake(this->xMutex_NotifyAll, portMAX_DELAY);

        for (Module *m = this->moduleList; m->handle != nullptr || ARRAY_INDEX_FROM_PTR(moduleList,m) < MAX_MODULES; m++) // TODO : optimise
        {
            if (m->ressourceOfInterest & sharedRessourceBits)
            {
                success = xTaskNotify(m->handle, NOTIFY_MSG(sharedRessourceBits, srcModuleIndex), action);
            }
        }

        xSemaphoreGive(this->xMutex_NotifyAll);
        return success;
    }


    // IHM

    inline void registerMonitor(TaskFunction_t taskFunction, const char *name, uint32_t stackSize, uint8_t priority,
                                    uint32_t taskFrequency,
                                    void *monitorparameters)
    {
        createModule(taskFunction, name, stackSize, monitorparameters, priority, 0xFFFF, taskFrequency, &(this->sysMonitor));
    }

    inline u_int8_t notifyMonitor(uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action)
    {
        return xTaskNotify(this->sysMonitor.handle, NOTIFY_MSG(sharedRessourceBits, srcModuleIndex), action);
    }

    inline uint8_t registerController(void (*digitalHandler)(void*), PinControllerParam digitalPins[],
                                            TimerCallbackFunction_t analogHandler, PinControllerParam analogPins[], uint8_t analogT, void *parameters
                                            /*,uint8_t analogRes,uint8_t analogRef*/)
    {

        if (this->nController >= MAX_CONTROLLER)
        {
            // Serial.println("Error: No space left in controller array");
            while (1)
                ;
        }
        createController(digitalHandler, digitalPins, analogHandler, analogPins, analogT, parameters, &(this->controllerList[nController]));
        return this->nController++;
    }


// WARNING : Only handle can wait itself at a barrier using introspection

// Barriere join from a handler (No FRTOS thread pause) Awaiting release
// Bloc executing handler into this function, set the module idx into the specified joinList of awaiting module
// Return Released Modules
inline uint32_t join(uint8_t joinListIndex,uint8_t modulePtrIndex){
    xSemaphoreTake(xMutex_JoinList, portMAX_DELAY);

    joinList[joinListIndex] |= (1 << modulePtrIndex);

    xSemaphoreGive(xMutex_JoinList);

    uint32_t ret = 0;
    xTaskNotifyWait(0x0,0x0,&ret,portMAX_DELAY);

    return ret;
}

// Release Module blocked at the Barrier, use with join
// Clear the bit field
// Return Released Modules
inline uint32_t release(uint8_t joinListIndex){

    uint32_t releasedModule = joinList[joinListIndex];
    
    uint8_t i = 0;

    xSemaphoreTake(xMutex_JoinList, portMAX_DELAY);
    while (i < MAX_MODULES)
    {
        if(releasedModule << i) xTaskNotify(this->moduleList[i].handle,releasedModule,eNoAction);
        i++;
    }
    joinList[joinListIndex] = 0;
    xSemaphoreGive(xMutex_JoinList);
    
    return releasedModule;
}

// specific case of release when free a specific module from the barrier
inline uint32_t unjoin(uint8_t joinListIndex,uint8_t modulePtrIndex){
    xSemaphoreTake(xMutex_JoinList, portMAX_DELAY);

    joinList[joinListIndex] &= (0 << modulePtrIndex);

    xSemaphoreGive(xMutex_JoinList);

    uint32_t ret = 0;
    xTaskNotify(this->moduleList[modulePtrIndex].handle,ret,eNoAction);
    return ret;
}


};

#endif // !C_ONLY
// #ifdef __cplusplus
// }
// #endif

#endif // !SRTL_H