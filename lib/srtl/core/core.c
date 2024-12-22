
// #define C_ONLY

#ifdef C_ONLY
#include "core.h"

// Listes des modules et ressources
uint8_t nModule;                        // Nombre de modules enregistrés
Module moduleList[MAX_MODULES];         // Liste des modules
// SemaphoreHandle_t xMutex_moduleList;    // Mutex pour la gestion des modules
SemaphoreHandle_t xMutex_NotifyAll;    

void *sharedRessources[MAX_SHARED_RESOURCES];   // Liste des ressources partagées
uint8_t nSharedResource;  

#endif // DEBUG





// BECAUSE inline function definition must be in header 

// inline void createModule(TaskFunction_t taskFunction, const char *name, uint32_t stackSize,void * moduleparameters,uint8_t priority,
// 						 uint32_t ressourceOfInterest, uint32_t taskFrequency, Module *module)
// {
// 	module->ressourceOfInterest = ressourceOfInterest;
// 	module->taskFrequency = taskFrequency;
// 	module->parameters = moduleparameters;

// 	// moduleparameters in module, module adress in taskhandler parameters
// 	if (xTaskCreate(taskFunction, name, stackSize, &module, priority, &module->handle) != pdPASS) // TODO : prefer Static Task alloc ?
// 	{
// 		// Serial.printf("Failed to create task %s\n", name);
// 		while (1)
// 			;
// 	}
// }

// inline uint8_t registerModule(TaskFunction_t taskFunction, const char *name, uint32_t stackSize,uint8_t priority,
// 							  uint32_t ressourceOfInterest, uint32_t taskFrequency,
// 							   void * moduleparameters)
// {
// 	if (nModule >= MAX_MODULES)
// 	{
// 		// Serial.println("Error: No space left in moduleList array");
// 		while (1)
// 			;
// 	}

// 	createModule(taskFunction, name, stackSize, moduleparameters,priority, ressourceOfInterest, taskFrequency, &moduleList[nModule]);
// 	return nModule++;
// }

// inline void createSharedResource(void **resource, SemaphoreHandle_t *mutex, void *valueLocation)
// {
// 	*resource = valueLocation;
// 	*mutex = xSemaphoreCreateMutex();

// 	if (mutex == NULL)
// 	{
// 		// Serial.println("Failed to create mutex for resource");
// 		while (1)
// 			; // Blocage en cas d'échec
// 	}
// }

// inline uint8_t registerSharedRessource(protected_data_t data)
// {
// 	if (nSharedResource >= MAX_SHARED_RESOURCES)
// 	{
// 		// Serial.println("Error: No space left in sharedRessources array");
// 		while (1)
// 			;
// 	}
// 	createSharedResource(&sharedRessources[nSharedResource], data.mutex,data.data);
// 	return nSharedResource++;
// }

// inline u_int8_t notifyModule(uint16_t destModuleIndex, uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action)
// {
// 	return xTaskNotify(moduleList[destModuleIndex].handle, NOTIFY_MSG(sharedRessourceBits, srcModuleIndex), action);
// }

// inline u_int8_t notifyAll(uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action)
// {
// 	u_int8_t success = pdFALSE;
// 	xSemaphoreTake(xMutex_NotifyAll, portMAX_DELAY); // TODO: if ? in ISR ? Atomic ?
// 	for (Module *m = moduleList; m->handle != NULL; m++)
// 	{
// 		if (m->ressourceOfInterest & sharedRessourceBits)
// 		{
// 			success = xTaskNotify(m->handle, NOTIFY_MSG(sharedRessourceBits, srcModuleIndex), action); 
// 		}				
// 	}
// 	xSemaphoreGive(xMutex_NotifyAll);
// 	return success;
// }

// inline SemaphoreHandle_t SRTLinit(){
//     return xMutex_NotifyAll = xSemaphoreCreateMutex();
// }   