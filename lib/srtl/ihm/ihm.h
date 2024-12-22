#ifndef SRTL_IHM_H
#define SRTL_IHM_H

#include "../core/core.h"

// For a total of 64 digital input and 32 analog input (Xbox controller have 16 and 4 with maximum res of 16 bits)
// TODO : not enough for a standard PC keyboard (116), or Grand Piano (88)

#define MAX_CONTROLLER 4 // 8 ?
#define MAX_DIGITALPIN_IN_CONTROLLER 16
#define MAX_ANALOGPIN_IN_CONTROLLER 8

// Monitor have an identical as Module 
typedef Module Monitor;


typedef struct 
{
    uint8_t pin;
    //TODO: combine this ?
    uint8_t READmode;
    uint8_t ISRmode;
}PinControllerParam;

// A controller is a set of both digital and analog input
// due to the premptive and atomic nature of digital interrupt, states are implicitly protected 
// for analog I decide to use a periodic softwareTimer NOTE : pretty big struct for controller 😁
typedef struct 
{
    volatile uint16_t digitalPinStates;
    volatile uint16_t analogPinStates[MAX_ANALOGPIN_IN_CONTROLLER];
    void* parameters;    
    //TODO : No need to keep this ?
    void* digitalHandler;
    void* analogHandler;
    PinControllerParam digitalPins[MAX_DIGITALPIN_IN_CONTROLLER];
    PinControllerParam analogPins[MAX_ANALOGPIN_IN_CONTROLLER];
}Controller;

inline void createController(void * digitalHandler,PinControllerParam * digitalPins,void * analogHandler,PinControllerParam analogPins[],
                            uint8_t analogT, void* parameters,Controller * controller/*,uint8_t analogRes,uint8_t analogRef*/){ 
    

    controller->digitalPinStates = 0;
    controller->digitalHandler = digitalHandler;
    controller->analogHandler = analogHandler;
    controller->parameters = parameters;

    // memset(controller->analogPinStates, 0, sizeof(PinControllerParam)*MAX_ANALOGPIN_IN_CONTROLLER); //error due to the volatile nature of variable
    for (uint8_t i = 0; i < MAX_ANALOGPIN_IN_CONTROLLER; i++)
        controller->analogPinStates[i] = 0;
    

    for (uint8_t i = 0; i < MAX_DIGITALPIN_IN_CONTROLLER || digitalPins[i].pin != 0; i++)
    {
        pinMode(digitalPins[i].pin,digitalPins[i].READmode);
        attachInterruptArg(digitalPinToInterrupt(digitalPins[i].pin), digitalHandler, controller,digitalPins[i].ISRmode);
        memcpy(&controller->digitalPins[i],&digitalPins[i],sizeof(PinControllerParam)); //here ? //TODO : exception
    }

    // TODO : configure analog ref and res
    // TODO : Hardware Timer or FreeRTOSTimer ?

    memcpy(controller->analogPins,analogPins,sizeof(PinControllerParam)*MAX_ANALOGPIN_IN_CONTROLLER); //TODO : exception

    if (xTimerCreate(
        "Timer's Controller",           // Nom du timer
        pdMS_TO_TICKS(analogT),        // Période  (ms)
        pdTRUE,                        // Répétitif
        controller,                     // ID,param or Controller ptr, access with  TimerParams *params = (TimerParams *)pvTimerGetTimerID(xTimer);
        analogHandler                  // Fonction de rappel
    ) == NULL)
    {
        // Serial.println("Error: cannot create a Timer for Controller");
        while (1);
    }
}

// #ifdef C_ONLY

extern Monitor sysMonitor;

static inline void registerMonitor(TaskFunction_t taskFunction, const char *name, uint32_t stackSize, uint8_t priority,
                                     uint32_t ressourceOfInterest, uint32_t taskFrequency,
                                     void *monitorparameters)
{
    ressourceOfInterest = 0xFFFF;
    createModule(taskFunction, name, stackSize, monitorparameters, priority, ressourceOfInterest, taskFrequency, &sysMonitor);
}

inline u_int8_t notifyMonitor(uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action)
{
    return xTaskNotify(sysMonitor.handle, NOTIFY_MSG(sharedRessourceBits, srcModuleIndex), action);
}

extern uint8_t nController;
extern Controller controllerList[MAX_CONTROLLER];



inline uint8_t registerController( void * digitalHandler,PinControllerParam digitalPins[],
                                void * analogHandler,PinControllerParam analogPins[],uint8_t analogT,void * parameters
                                /*,uint8_t analogRes,uint8_t analogRef*/){ 

    if (nController >= MAX_CONTROLLER)
    {
        // Serial.println("Error: No space left in controller array");
        while (1);
    }
    createController(digitalHandler,digitalPins,analogHandler,analogPins, analogT, parameters,&controllerList[nController]);
    return nController++;
}


// #endif //C_ONLY

#endif //SRTL_IHM_H