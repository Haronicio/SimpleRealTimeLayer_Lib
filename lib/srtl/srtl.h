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
    uint8_t nModule;
    Module moduleList[MAX_MODULES];
    SemaphoreHandle_t xMutex_NotifyAll;

    void * sharedRessources[MAX_SHARED_RESOURCES];
    uint8_t nSharedResource;

// public:
    // Constructeur
    SRTL()
    {
        nModule = 0;
        nSharedResource = 0;
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
        return xMutex_NotifyAll = xSemaphoreCreateMutex();
    }

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


};

#endif // !C_ONLY
// #ifdef __cplusplus
// }
// #endif

#endif // !SRTL_H