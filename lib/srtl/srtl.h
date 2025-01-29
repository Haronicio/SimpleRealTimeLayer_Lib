#ifndef SRTL_H
#define SRTL_H

#include "core/core.h"
#include "ihm/ihm.h"
#include "sync/sync.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

#ifndef C_ONLY

class SRTL
{
    // private:
public: // TODO getter and setter
    // CORE

    uint8_t nModule;
    Module moduleList[MAX_MODULES];
    SemaphoreHandle_t xMutex_NotifyAll;

    void *sharedRessources[MAX_SHARED_RESOURCES];
    uint8_t nSharedResource;

    uint32_t joinList[MAX_JOIN_LIST] = {0}; // TODO : add more module (use uint32_t but due to splitting 32 bit to Module and SharedResource ... see NOTIFY_MSG uses)
    SemaphoreHandle_t xMutex_JoinList;

    // IHM

    Monitor sysMonitor;

    uint8_t nController;
    Controller controllerList[MAX_CONTROLLER];

    // SYNC

    SemaphoreHandle_t xMutex_SyncTimer;

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
    ~SRTL()
    {
        // Détruire la sémaphore de notifyAll
        if (this->xMutex_NotifyAll != nullptr)
        {
            vSemaphoreDelete(xMutex_NotifyAll);
        }
        // Détruire la sémaphore de JoinList
        if (this->xMutex_JoinList != nullptr)
        {
            vSemaphoreDelete(xMutex_JoinList);
        }
        // Détruire la sémaphore de SyncTimer
        if (this->xMutex_SyncTimer != nullptr)
        {
            vSemaphoreDelete(xMutex_SyncTimer);
        }

        // Détruire les sémaphores des ressources partagées
        for (uint8_t i = 0; i < nSharedResource; ++i)
        {
            protected_data_t *data = reinterpret_cast<protected_data_t *>(sharedRessources[i]);
            if (data != nullptr && data->mutex != nullptr)
            {
                vSemaphoreDelete(*(data->mutex));
            }
        }

        // Supprimer toutes les tâches ATTENTION a la destruction d'une copie qui détruira les tâches
        for (uint8_t i = 0; i < nModule; ++i)
        {
            if (moduleList[i].handle != nullptr)
            {
                vTaskDelete(moduleList[i].handle);
            }
        }
    }

    // Initialisation
    SemaphoreHandle_t init()
    {
        xMutex_JoinList = xSemaphoreCreateMutex();
        xMutex_SyncTimer = xSemaphoreCreateMutex();
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
        if (this->moduleList[destModuleIndex].handle == NULL)
        {
            Serial.printf("WARNING %d NULL ( from %d)", destModuleIndex, srcModuleIndex);
        }
        return xTaskNotify(this->moduleList[destModuleIndex].handle, NOTIFY_MSG(sharedRessourceBits, srcModuleIndex), action);
    }

    inline uint8_t notifyAll(uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action)
    {
        uint8_t success = pdFALSE;
        xSemaphoreTake(this->xMutex_NotifyAll, portMAX_DELAY);
        // Serial.printf("notify%d\n", srcModuleIndex);

        for (Module *m = this->moduleList; m->handle != nullptr || ARRAY_INDEX_FROM_PTR(moduleList, m) < MAX_MODULES; m++) // TODO : optimise
        {
            if (m->ressourceOfInterest & sharedRessourceBits)
            {
                if (m->handle == NULL)
                {
                    Serial.printf("WARNING %d NULL ( from %d)", ARRAY_INDEX_FROM_PTR(this->moduleList, m), srcModuleIndex);
                }
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

    inline uint8_t registerController(void (*digitalHandler)(void *), PinControllerParam digitalPins[],
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

    // WARNING : Only handle can wait itself (join) at a barrier using introspection
    // Notify value (NOTIFY_MSG) contain  (actual state of bitfield's waiting queue)(index of source (0 if non specified))

    // Barriere join from a handler (No FRTOS thread pause) Awaiting release
    // Bloc executing handler into this function, set the module idx into the specified joinList of awaiting module
    // Return Released Modules
    inline uint32_t join(uint8_t joinListIndex, uint8_t modulePtrIndex)
    {
        xSemaphoreTake(xMutex_JoinList, portMAX_DELAY);
        joinList[joinListIndex] |= (1 << modulePtrIndex);

        // Serial.printf("%d join Barrier %d (%s)\n", modulePtrIndex, joinListIndex, TO_BIN(joinList[joinListIndex]));
        xSemaphoreGive(xMutex_JoinList);

        uint32_t ret = 0;
        xTaskNotifyWait(0x0, 0x0, &ret, portMAX_DELAY);
        // Serial.printf("free %d\n", modulePtrIndex);

        return ret;
    }

    // Release Module blocked at the Barrier, use with join
    // Clear the bit field
    // Return Released Modules Indexes
    inline uint32_t release(uint8_t joinListIndex, uint8_t srcModuleIndex = 0)
    {
        xSemaphoreTake(xMutex_JoinList, portMAX_DELAY);
        uint32_t releasedModule = joinList[joinListIndex];
        // Serial.printf("Released Barrier %d : %s\n", joinListIndex, TO_BIN(releasedModule));

        for (uint8_t i = 0; i < MAX_MODULES; i++)
        {
            if ((releasedModule & (1 << i)) != 0)
            {
                // TODO
                if (this->moduleList[i].handle == NULL)
                {
                    Serial.printf("WARNING: Module %d has a NULL handle (from join %d)\n", i, joinListIndex);
                }
                else
                {
                    xTaskNotify(this->moduleList[i].handle, NOTIFY_MSG(releasedModule, srcModuleIndex), eNoAction);
                }
            }
        }
        joinList[joinListIndex] = 0;
        xSemaphoreGive(xMutex_JoinList);

        return releasedModule;
    }

    // specific case of release when free a specific module from the barrier
    // return index of module released
    // UPDATE : same as other func
    inline uint32_t unjoin(uint8_t joinListIndex, uint8_t modulePtrIndex, uint8_t srcModuleIndex = 0)
    {
        xSemaphoreTake(xMutex_JoinList, portMAX_DELAY);
        joinList[joinListIndex] &= ~(1 << modulePtrIndex);
        uint32_t ret = joinList[joinListIndex];

        xSemaphoreGive(xMutex_JoinList);

        if (this->moduleList[modulePtrIndex].handle == NULL)
        {
            Serial.printf("WARNING %d NULL ( from unjoin %d)", modulePtrIndex, joinListIndex);
        }
        xTaskNotify(this->moduleList[modulePtrIndex].handle, NOTIFY_MSG(ret, srcModuleIndex), eNoAction);
        return ret;
    }

    // SYNC

    inline bool setupWiFi(const char *ssid, const char *passphrase = (const char *)__null)
    {
        return syncWifiInit(ssid, passphrase);
    }

    /*

        The main concept of the autoTimer is to sleep until a precise date will reached,
        Module expose the situation to other (I'm sleeping on this timer) and hold tick delay information
        But EventTimer should not be shared (even with a proper protected_data_t) because syncTimer will created starvation or deadlock
        return the event id if events in success or UINT8_MAX
    */

    inline uint8_t autoTimer(Module *modulePtr, eventTimer *events, uint8_t nEvent)
    {

        if (events == nullptr)
        {
            Serial.println("No timers specified");
            return UINT8_MAX;
        }

        if (modulePtr == nullptr)
        {
            Serial.println("No module for timer");
            return UINT8_MAX;
        }

        if (nEvent >= MAX_EVENTTIMER_IN_MODULE)
        {
            Serial.println("Too many timer");
            return UINT8_MAX;
        }



        int32_t minDelta = INT32_MAX;
        eventTimer *nextEvent = NULL;

        // SyncTimer is a static function, calibration must be protected
        xSemaphoreTake(xMutex_SyncTimer, portMAX_DELAY);
        for (uint8_t i = 0; i < nEvent; i++)
        {
            if (events[i].flags & 1)
            {

                int32_t delta = syncTimer(events[i].time); //( > 0) futur ( < 0) past

                if(delta < 1) //task is late
                {
                    /*
                        doit q'éxécuté toutes les 15 secondes, il ne s'est pas éxécuté pendant 48 seconde (donc -48 )
                        cela a équivaut à 48/15 ~ 3 fois manqué, donc la 4ème éxécution sera à l'heure, et la 3ème doit s'éxécuter maintenant
                        s'il avait fait ces 3 fois manqué il aurait un peu de retard (3secondes), c'est celui ci qui est éxécuté immédiatement
                    */
                    uint32_t missedCount = getMissedEventCounts(&events[i]);
                    events[i].time += events[i].period * missedCount;
                    events[i].eventCount += missedCount;
                    events[i].lastExecution =  events[i].time;
                    delta = syncTimer(events[i].time); //( > 0) futur ( < 0) past
                }

                int32_t tmp = min(minDelta, delta);
                if (tmp != minDelta)
                {
                    minDelta = tmp;
                    nextEvent = &events[i];
                }

                // periodicity
                if (GET_EVENT_TIMER_PERIOD(events[i].flags))
                {
                    events[i].time += events[i].period;
                }
               
            }
        }
        xSemaphoreGive(xMutex_SyncTimer);

        // TODO WARNING PROTECT THAT NOTE list head or nextactive ?
        // modulePtr->lastEvents = events;
        modulePtr->lastEvent = nextEvent;

        if (nextEvent == NULL)
        {
            Serial.println("No valid event found, skipping sleep.");
            return UINT8_MAX;
        }

        SET_EVENT_TIMER_ISNEXT(nextEvent->flags, 1);

        if (minDelta > 1) //task is early
        {
            Serial.printf("Module %d sleep for %ld , ET %d\n", ARRAY_INDEX_FROM_PTR(this->moduleList, modulePtr), minDelta, GET_EVENT_TIMER_ID(nextEvent->flags));

            // Sleep to reach next date (heavy time load measured ~ 1 sec)
            vTaskDelayUntil(&modulePtr->lastAwake, pdMS_TO_TICKS(minDelta - MILLIS_TO_EPOCH(modulePtr->taskFrequency) + 1));
        }
        else //task is late
            Serial.printf("Module %d execute immediatly  %ld , ET %d\n", ARRAY_INDEX_FROM_PTR(this->moduleList, modulePtr), minDelta, GET_EVENT_TIMER_ID(nextEvent->flags));

        // lastExecution and count
        nextEvent->lastExecution = nextEvent->time;
        nextEvent->eventCount++;

        return GET_EVENT_TIMER_ID(nextEvent->flags);
    }

    /*
       modulePtr->lastEvents->lastExecution ~~ pdTICKS_TO_MILLIS(modulePtr->lastAwake)
    */
};

#endif // !C_ONLY
// #ifdef __cplusplus
// }
// #endif

#endif // !SRTL_H