#define C_ONLY


#include <Arduino.h>
#include <srtl.h>


#ifdef C_ONLY
// DEBUG

void monitorHeap()
{
	Serial.printf("Free Heap Size: %d bytes\n", xPortGetFreeHeapSize());
	Serial.printf("Minimum Ever Free Heap Size: %d bytes\n", xPortGetMinimumEverFreeHeapSize());
}

#undef INCLUDE_uxTaskGetStackHighWaterMark
#define INCLUDE_uxTaskGetStackHighWaterMark 1

void monitorStack()
{
	for (Module *m = moduleList; m->handle != NULL; m++)
	{
		if (m->handle != NULL)
		{
			// Mesure de la quantité de stack libre en mots
			UBaseType_t stackLeft = uxTaskGetStackHighWaterMark(m->handle);

			// Conversion en octets (si nécessaire, dépend de la taille du mot de l'architecture)
			size_t stackSizeBytes = stackLeft * sizeof(StackType_t);

			// Affichage
			Serial.printf("Task '%d': Stack remaining: %d bytes\n", ARRAY_INDEX_FROM_PTR(moduleList, m), stackSizeBytes);
		}
		else
		{
			Serial.printf("Task '%d': Invalid handle!\n", ARRAY_INDEX_FROM_PTR(moduleList, m));
		}
	}
}

void printMemoryStats() {
    Serial.println("=== Statistiques Mémoire ===");
    Serial.print("Heap totale : ");
    Serial.println(heap_caps_get_total_size(MALLOC_CAP_DEFAULT));
    
    Serial.print("Heap libre : ");
    Serial.println(heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    
    Serial.print("Plus gros bloc contigu libre : ");
    Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    
    Serial.print("Heap DRAM totale : ");
    Serial.println(heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
    
    Serial.print("Heap DRAM libre : ");
    Serial.println(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.println("============================");
}






/*
USER CASE TEST

*/






// DATA FOR TEST
struct custom_data_t
{
	char cvar = 0;
	int ivar = 0;
	float fvar = 0.0;
};
struct custom_data_t vardata;
float var1 = 0.0, var2 = 0.0;

SemaphoreHandle_t xMutex_var1, xMutex_var2, xMutex_vardata;

protected_data_t protected_var1 = {&xMutex_var1,(void *)&var1}; 
protected_data_t protected_var2 = {&xMutex_var2,(void *)&var2}; 
protected_data_t protected_vardata = {&xMutex_vardata,(void *)&vardata}; 



void producer1(void *pvParameters)
{
	// setup

	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_var1, portMAX_DELAY) == pdTRUE)
		{
			Serial.println(F("Producer1"));
			*(float *)sharedRessources[0] += 0.01;
			xSemaphoreGive(xMutex_var1);
		}
		notifyAll(0b1, 0b1,eSetBits);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}

void producer2(void *pvParameters)
{
	// uint8_t identity = whoami();
	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_var2, portMAX_DELAY) == pdTRUE)
		{
			Serial.println(F("Producer2"));
			*(float *)sharedRessources[1] += 0.02;
			xSemaphoreGive(xMutex_var2);
		}
		notifyAll(0b10, 0b10,eSetBits);
		// notifyAll(0b10, 0b10,eSetValueWithOverwrite);
		vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000ms
	}
	vTaskDelete(NULL);
}

void producer3(void *pvParameters)
{

	// Task implementation
	for (;;)
	{
		Serial.println(F("Producer3"));
		if (xSemaphoreTake(xMutex_var1, portMAX_DELAY) == pdTRUE)
		{
			*(float *)sharedRessources[0] -= 0.01;
			xSemaphoreGive(xMutex_var1);
		}
		if (xSemaphoreTake(xMutex_var2, portMAX_DELAY) == pdTRUE)
		{
			*(float *)sharedRessources[1] -= 0.02;
			xSemaphoreGive(xMutex_var2);
		}
		notifyAll(0b100, 0b11,eSetValueWithOverwrite);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void producer4(void *pvParameters)
{
	// uint8_t identity = whoami();
	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_vardata, portMAX_DELAY) == pdTRUE)
		{
			((custom_data_t *)sharedRessources[2])->cvar += 1;
			xSemaphoreGive(xMutex_vardata);
		}
		if (notifyAll(0b1000, 0b100, eSetValueWithoutOverwrite))
			Serial.println(F("Producer4"));
		vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 1000ms
	}
}

void consummer1(void *pvParameters)
{
	float local_var1 = 0.0;
	// Task implementation
	for (;;)
	{
		uint32_t ulNotifiedValue = 0;
		if (xTaskNotifyWait(
				0x00,
				0x01,
				&ulNotifiedValue,
				pdMS_TO_TICKS(250)))
		{

			if (xSemaphoreTake(xMutex_var1, portMAX_DELAY) == pdTRUE /*&&
				GET_BYTE(ulNotifiedValue, 0) & 0x01*/
			)
			{
				local_var1 = *(float *)sharedRessources[0];
				xSemaphoreGive(xMutex_var1);
			}
		}
		Serial.printf("consumer1 var1: %.2f\n", local_var1);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

void consummer2(void *pvParameters)
{

	float local_var2 = 0.0;
	// Task implementation
	for (;;)
	{

		uint32_t ulNotifiedValue = 0;
		if (xTaskNotifyWait(
				0x00,
				0x02,
				&ulNotifiedValue,
				pdMS_TO_TICKS(250)))
		{
			if (xSemaphoreTake(xMutex_var2, portMAX_DELAY) == pdTRUE)
			{
				local_var2 = *(float *)sharedRessources[1];
				xSemaphoreGive(xMutex_var2);
			}
		}
		Serial.printf("consumer2 var2: %.2f\n", local_var2);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

//wait 2 notification
void consummer3(void *modulePtr)
{
	Module currentModule = *((Module *)modulePtr);
	currentModule.notificationValue = 0;

	float local_var1 = 0.0, local_var2 = 0.0;
	// Task implementation
	for (;;)
	{
		if (xTaskNotifyWait(0xFFFFFFFF, 0x00, &currentModule.notificationValue, pdMS_TO_TICKS(250)) &&
			((EXTRACT_SHARED_RESOURCE(currentModule.notificationValue) & 0x03) == 0x03))
		{

			xSemaphoreTake(xMutex_var1, portMAX_DELAY);
			local_var1 = *(float *)sharedRessources[0];
			xSemaphoreGive(xMutex_var1);

			xSemaphoreTake(xMutex_var2, portMAX_DELAY);
			local_var2 = *(float *)sharedRessources[1];
			xSemaphoreGive(xMutex_var2);

			Serial.printf("consumer3 var1: %.2f var2: %.2f src: 0x%08X\n", local_var1, local_var2, currentModule.notificationValue);
		}
		vTaskDelay(pdMS_TO_TICKS(200)); 
	}
}

//read the first notification arrive
void consummer31(void *modulePtr)
{
	Module currentModule = *((Module *)modulePtr);
	currentModule.notificationValue = 0;
	float local_var1 = 0.0, local_var2 = 0.0;

	// Task implementation
	for (;;)
	{
		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, &currentModule.notificationValue, portMAX_DELAY))
		{
			// Serial.printf(" src: 0x%08X \t\n", currentModule.notificationValue);
			if ((EXTRACT_SHARED_RESOURCE(currentModule.notificationValue) & 0b1) == 0b1)
			{
				xSemaphoreTake(xMutex_var1, portMAX_DELAY);
				local_var1 = *(float *)sharedRessources[0];

				Serial.printf("consumer31 var1: %.2f var2: %.2f src: 0x%08X \t1\n", local_var1, local_var2, currentModule.notificationValue);
				xSemaphoreGive(xMutex_var1);
			}

			if ((EXTRACT_SHARED_RESOURCE(currentModule.notificationValue) & 0b10 )== 0b10)
			{

				xSemaphoreTake(xMutex_var2, portMAX_DELAY);
				local_var2 = *(float *)sharedRessources[1];

				Serial.printf("consumer31 var1: %.2f var2: %.2f src: 0x%08X \t2\n ",local_var1, local_var2,currentModule.notificationValue);
				xSemaphoreGive(xMutex_var2);
			}

			currentModule.notificationValue = 0;

		}
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

// notification send if previous notification consume (see producer4)
void consummer4(void *pvParameters)
{
	custom_data_t local_var;
	uint32_t ulNotifiedValue = 0;
	// Task implementation
	for (;;)
	{
		if (xTaskNotifyWait(
				0x04,
				0x00,
				&ulNotifiedValue,
				pdMS_TO_TICKS(250)))
		{

			if (xSemaphoreTake(xMutex_vardata, portMAX_DELAY) == pdTRUE /*&&
				GET_BYTE(ulNotifiedValue, 0) & 0x01*/
			)
			{
				local_var = *((custom_data_t *)sharedRessources[2]);
				xSemaphoreGive(xMutex_vardata);
			}
		}
		Serial.printf("consumer4 var: %d\n", local_var.cvar);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}




// Définition de la ressource partagée et des semaphores
SemaphoreHandle_t xMutex_replicatedVar;   // Mutex pour protéger l'accès à replicatedVar

struct replicated_custom_data_t{
  float replicatedVar = 0.0;               // Ressource partagée à répliquer
  int replicationCount = 0;                 // Compteur de répliques
} replicatedVar;




// Protection de la ressource partagée
protected_data_t protected_replicatedVar = {&xMutex_replicatedVar, (void *)&replicatedVar};

// Fonction de synchronisation des producteurs (replicatorA et replicatorB)
void replicatorA(void *pvParameters)
{
    // Processus de réplication pour replicatorA
    for (;;)
    {
        // Attendre que l'autre producteur (replicatorB) libère la ressource
        if (xSemaphoreTake(xMutex_replicatedVar, portMAX_DELAY) == pdTRUE)
        {
            // Mettre à jour replicatedVar
            ((replicated_custom_data_t*)protected_replicatedVar.data)->replicatedVar += 1.0;
            ((replicated_custom_data_t*)protected_replicatedVar.data)->replicationCount++;  // Incrémenter le compteur

            // Log l'état de la ressource répliquée
            Serial.printf("ReplicatorA: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t*)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t*)protected_replicatedVar.data)->replicationCount);
            
            // Libérer le mutex pour permettre à l'autre producteur d'accéder à la ressource
            xSemaphoreGive(xMutex_replicatedVar);
        }

        // Ajouter un délai avant la prochaine réplique
        vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
    }
}

void replicatorB(void *pvParameters)
{
    // Processus de réplication pour replicatorB
    for (;;)
    {
        // Attendre que l'autre producteur (replicatorA) libère la ressource
        if (xSemaphoreTake(xMutex_replicatedVar, portMAX_DELAY) == pdTRUE)
        {
            // Mettre à jour replicatedVar
            ((replicated_custom_data_t*)protected_replicatedVar.data)->replicatedVar += 1.0;
            ((replicated_custom_data_t*)protected_replicatedVar.data)->replicationCount++;  // Incrémenter le compteur

            // Log l'état de la ressource répliquée
            Serial.printf("ReplicatorB: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t*)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t*)protected_replicatedVar.data)->replicationCount);
            
            // Libérer le mutex pour permettre à l'autre producteur d'accéder à la ressource
            xSemaphoreGive(xMutex_replicatedVar);
        }

        // Ajouter un délai avant la prochaine réplique
        vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
    }
}






void setup()
{
	Serial.begin(9600);
	Serial.println(F("In Setup function"));

	// init
	SRTLinit();

	// Enregistrement des ressources partagées
	uint8_t var1Index = registerSharedRessource(protected_var1);
	uint8_t var2Index = registerSharedRessource(protected_var2);
	uint8_t vardataIndex = registerSharedRessource(protected_vardata);

	uint8_t varReplicatedIndex = registerSharedRessource(protected_replicatedVar);

	// Enregistrement des modules producteurs
	registerModule(producer1, "P1", MINIMAL_STACK_SIZE, 1, 0x00, 200,NULL);
	registerModule(producer2, "P2", MINIMAL_STACK_SIZE, 1, 0x00, 200,NULL);
	registerModule(producer3, "P3", MINIMAL_STACK_SIZE, 1, 0x00, 200,NULL);
	registerModule(producer4, "P4", MINIMAL_STACK_SIZE, 1, 0x00, 200,NULL);

	// Enregistrement des modules consommateurs
	registerModule(consummer1, "C1", MINIMAL_STACK_SIZE, 1, (1 << var1Index), 200,NULL);					 // Intérêt pour var1
	registerModule(consummer2, "C2", MINIMAL_STACK_SIZE, 1, (1 << var2Index), 200,NULL);					 // Intérêt pour var2
	registerModule(consummer3, "C3", MINIMAL_STACK_SIZE, 1, (1 << var1Index) | (1 << var2Index), 200,NULL); // Intérêt pour var1 et var2
	registerModule(consummer31, "consummer31", MINIMAL_STACK_SIZE, 2, (1 << var1Index) | (1 << var2Index), 200,NULL); // Intérêt pour var1 et var2
	registerModule(consummer4, "C4", MINIMAL_STACK_SIZE, 1, (1 << vardataIndex), 200,NULL);				 // Intérêt pour vardata

  // Enregistrer les modules producteurs
  registerModule(replicatorA, "ReplicatorA", MINIMAL_STACK_SIZE, 1, (1 << varReplicatedIndex), 200, NULL);
  registerModule(replicatorB, "ReplicatorB", MINIMAL_STACK_SIZE, 1, (1 << varReplicatedIndex), 200, NULL);

	// Affichage des modules enregistrés
	for (int i = 0; i < nModule; i++)
	{
		if (moduleList[i].handle != NULL)
		{
			Serial.printf("Module %d Handle: %p Interest: 0x%X\n", i, moduleList[i].handle, moduleList[i].ressourceOfInterest);
		}
	}


	delay(1000);
	monitorStack();
	monitorHeap();
	Serial.printf("Program size: %d \n",ESP.getSketchSize());
	printMemoryStats();
	
}

void loop()
{
	// Main loop code (optional, as tasks will run independently)
}

#endif










#ifndef C_ONLY










void printMemoryStats() {
    Serial.println("=== Statistiques Mémoire ===");
    Serial.print("Heap totale : ");
    Serial.println(heap_caps_get_total_size(MALLOC_CAP_DEFAULT));
    
    Serial.print("Heap libre : ");
    Serial.println(heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    
    Serial.print("Plus gros bloc contigu libre : ");
    Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    
    Serial.print("Heap DRAM totale : ");
    Serial.println(heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
    
    Serial.print("Heap DRAM libre : ");
    Serial.println(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.println("============================");
}






/*
USER CASE TEST

*/






// DATA FOR TEST
struct custom_data_t
{
	char cvar = 0;
	int ivar = 0;
	float fvar = 0.0;
};
struct custom_data_t vardata;
float var1 = 0.0, var2 = 0.0;

SemaphoreHandle_t xMutex_var1, xMutex_var2, xMutex_vardata;

protected_data_t protected_var1 = {&xMutex_var1,(void *)&var1}; 
protected_data_t protected_var2 = {&xMutex_var2,(void *)&var2}; 
protected_data_t protected_vardata = {&xMutex_vardata,(void *)&vardata}; 

#define GET_SRTL_INSTANCE ((*((Module*)modulePtr)->parent))

void producer1(void *modulePtr)
{
	// setup

	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_var1, portMAX_DELAY) == pdTRUE)
		{
			Serial.println(F("Producer1"));
			*(float *)GET_SRTL_INSTANCE.sharedRessources[0] += 0.01;
			xSemaphoreGive(xMutex_var1);
		}
		GET_SRTL_INSTANCE.notifyAll(0b1, 0b1,eSetBits);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}

void producer2(void *modulePtr)
{
	// uint8_t identity = whoami();
	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_var2, portMAX_DELAY) == pdTRUE)
		{
			Serial.println(F("Producer2"));
			*(float *)GET_SRTL_INSTANCE.sharedRessources[1] += 0.02;
			xSemaphoreGive(xMutex_var2);
		}
		GET_SRTL_INSTANCE.notifyAll(0b10, 0b10,eSetBits);
		// notifyAll(0b10, 0b10,eSetValueWithOverwrite);
		vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000ms
	}
	vTaskDelete(NULL);
}

void producer3(void *modulePtr)
{

	// Task implementation
	for (;;)
	{
		Serial.println(F("Producer3"));
		if (xSemaphoreTake(xMutex_var1, portMAX_DELAY) == pdTRUE)
		{
			*(float *)GET_SRTL_INSTANCE.sharedRessources[0] -= 0.01;
			xSemaphoreGive(xMutex_var1);
		}
		if (xSemaphoreTake(xMutex_var2, portMAX_DELAY) == pdTRUE)
		{
			*(float *)GET_SRTL_INSTANCE.sharedRessources[1] -= 0.02;
			xSemaphoreGive(xMutex_var2);
		}
		GET_SRTL_INSTANCE.notifyAll(0b100, 0b11,eSetValueWithOverwrite);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void producer4(void *modulePtr)
{
	// uint8_t identity = whoami();
	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_vardata, portMAX_DELAY) == pdTRUE)
		{
			((custom_data_t *)GET_SRTL_INSTANCE.sharedRessources[2])->cvar += 1;
			xSemaphoreGive(xMutex_vardata);
		}
		if (GET_SRTL_INSTANCE.notifyAll(0b1000, 0b100, eSetValueWithoutOverwrite))
			Serial.println(F("Producer4"));
		vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 1000ms
	}
}

void consummer1(void *modulePtr)
{
	float local_var1 = 0.0;
	// Task implementation
	for (;;)
	{
		uint32_t ulNotifiedValue = 0;
		if (xTaskNotifyWait(
				0x00,
				0x01,
				&ulNotifiedValue,
				pdMS_TO_TICKS(250)))
		{

			if (xSemaphoreTake(xMutex_var1, portMAX_DELAY) == pdTRUE /*&&
				GET_BYTE(ulNotifiedValue, 0) & 0x01*/
			)
			{
				local_var1 = *(float *)GET_SRTL_INSTANCE.sharedRessources[0];
				xSemaphoreGive(xMutex_var1);
			}
		}
		Serial.printf("consumer1 var1: %.2f\n", local_var1);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

void consummer2(void *modulePtr)
{

	float local_var2 = 0.0;
	// Task implementation
	for (;;)
	{

		uint32_t ulNotifiedValue = 0;
		if (xTaskNotifyWait(
				0x00,
				0x02,
				&ulNotifiedValue,
				pdMS_TO_TICKS(250)))
		{
			if (xSemaphoreTake(xMutex_var2, portMAX_DELAY) == pdTRUE)
			{
				local_var2 = *(float *)GET_SRTL_INSTANCE.sharedRessources[1];
				xSemaphoreGive(xMutex_var2);
			}
		}
		Serial.printf("consumer2 var2: %.2f\n", local_var2);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

//wait 2 notification
void consummer3(void *modulePtr)
{
	Module currentModule = *((Module *)modulePtr);
	currentModule.notificationValue = 0;

	float local_var1 = 0.0, local_var2 = 0.0;
	// Task implementation
	for (;;)
	{
		if (xTaskNotifyWait(0xFFFFFFFF, 0x00, &currentModule.notificationValue, pdMS_TO_TICKS(250)) &&
			((EXTRACT_SHARED_RESOURCE(currentModule.notificationValue) & 0x03) == 0x03))
		{

			xSemaphoreTake(xMutex_var1, portMAX_DELAY);
			local_var1 = *(float *)GET_SRTL_INSTANCE.sharedRessources[0];
			xSemaphoreGive(xMutex_var1);

			xSemaphoreTake(xMutex_var2, portMAX_DELAY);
			local_var2 = *(float *)GET_SRTL_INSTANCE.sharedRessources[1];
			xSemaphoreGive(xMutex_var2);

			Serial.printf("consumer3 var1: %.2f var2: %.2f src: 0x%08X\n", local_var1, local_var2, currentModule.notificationValue);
		}
		vTaskDelay(pdMS_TO_TICKS(200)); 
	}
}

//read the first notification arrive
void consummer31(void *modulePtr)
{
	Module currentModule = *((Module *)modulePtr);
	currentModule.notificationValue = 0;
	float local_var1 = 0.0, local_var2 = 0.0;

	// Task implementation
	for (;;)
	{
		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, &currentModule.notificationValue, portMAX_DELAY))
		{
			// Serial.printf(" src: 0x%08X \t\n", currentModule.notificationValue);
			if ((EXTRACT_SHARED_RESOURCE(currentModule.notificationValue) & 0b1) == 0b1)
			{
				xSemaphoreTake(xMutex_var1, portMAX_DELAY);
				local_var1 = *(float *)GET_SRTL_INSTANCE.sharedRessources[0];

				Serial.printf("consumer31 var1: %.2f var2: %.2f src: 0x%08X \t1\n", local_var1, local_var2, currentModule.notificationValue);
				xSemaphoreGive(xMutex_var1);
			}

			if ((EXTRACT_SHARED_RESOURCE(currentModule.notificationValue) & 0b10 )== 0b10)
			{

				xSemaphoreTake(xMutex_var2, portMAX_DELAY);
				local_var2 = *(float *)GET_SRTL_INSTANCE.sharedRessources[1];

				Serial.printf("consumer31 var1: %.2f var2: %.2f src: 0x%08X \t2\n ",local_var1, local_var2,currentModule.notificationValue);
				xSemaphoreGive(xMutex_var2);
			}

			currentModule.notificationValue = 0;

		}
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

// notification send if previous notification consume (see producer4)
void consummer4(void *modulePtr)
{
	custom_data_t local_var;
	uint32_t ulNotifiedValue = 0;
	// Task implementation
	for (;;)
	{
		if (xTaskNotifyWait(
				0x04,
				0x00,
				&ulNotifiedValue,
				pdMS_TO_TICKS(250)))
		{

			if (xSemaphoreTake(xMutex_vardata, portMAX_DELAY) == pdTRUE /*&&
				GET_BYTE(ulNotifiedValue, 0) & 0x01*/
			)
			{
				local_var = *((custom_data_t *)GET_SRTL_INSTANCE.sharedRessources[2]);
				xSemaphoreGive(xMutex_vardata);
			}
		}
		Serial.printf("consumer4 var: %d\n", local_var.cvar);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}




// Définition de la ressource partagée et des semaphores
SemaphoreHandle_t xMutex_replicatedVar;   // Mutex pour protéger l'accès à replicatedVar

struct replicated_custom_data_t{
  float replicatedVar = 0.0;               // Ressource partagée à répliquer
  int replicationCount = 0;                 // Compteur de répliques
} replicatedVar;




// Protection de la ressource partagée
protected_data_t protected_replicatedVar = {&xMutex_replicatedVar, (void *)&replicatedVar};

// Fonction de synchronisation des producteurs (replicatorA et replicatorB)
void replicatorA(void *pvParameters)
{
    // Processus de réplication pour replicatorA
    for (;;)
    {
        // Attendre que l'autre producteur (replicatorB) libère la ressource
        if (xSemaphoreTake(xMutex_replicatedVar, portMAX_DELAY) == pdTRUE)
        {
            // Mettre à jour replicatedVar
            ((replicated_custom_data_t*)protected_replicatedVar.data)->replicatedVar += 1.0;
            ((replicated_custom_data_t*)protected_replicatedVar.data)->replicationCount++;  // Incrémenter le compteur

            // Log l'état de la ressource répliquée
            Serial.printf("ReplicatorA: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t*)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t*)protected_replicatedVar.data)->replicationCount);
            
            // Libérer le mutex pour permettre à l'autre producteur d'accéder à la ressource
            xSemaphoreGive(xMutex_replicatedVar);
        }

        // Ajouter un délai avant la prochaine réplique
        vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
    }
}

void replicatorB(void *pvParameters)
{
    // Processus de réplication pour replicatorB
    for (;;)
    {
        // Attendre que l'autre producteur (replicatorA) libère la ressource
        if (xSemaphoreTake(xMutex_replicatedVar, portMAX_DELAY) == pdTRUE)
        {
            // Mettre à jour replicatedVar
            ((replicated_custom_data_t*)protected_replicatedVar.data)->replicatedVar += 1.0;
            ((replicated_custom_data_t*)protected_replicatedVar.data)->replicationCount++;  // Incrémenter le compteur

            // Log l'état de la ressource répliquée
            Serial.printf("ReplicatorB: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t*)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t*)protected_replicatedVar.data)->replicationCount);
            
            // Libérer le mutex pour permettre à l'autre producteur d'accéder à la ressource
            xSemaphoreGive(xMutex_replicatedVar);
        }

        // Ajouter un délai avant la prochaine réplique
        vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
    }
}


void test(void *modulePtr){

  Serial.printf("%d %d\n",((Module*)modulePtr)->parent->nSharedResource,((Module*)modulePtr)->parent->nModule);
  // Serial.printf("%p %p\n",((Module*)modulePtr),((Module*)modulePtr)->parent);

  Serial.println("ok");

  
  vTaskDelete(NULL);
}


// init
SRTL srtl;

void setup()
{
	Serial.begin(9600);
	Serial.println(F("In Setup function"));


	// Enregistrement des ressources partagées
	uint8_t var1Index = srtl.registerSharedResource(protected_var1);
	uint8_t var2Index = srtl.registerSharedResource(protected_var2);
	uint8_t vardataIndex = srtl.registerSharedResource(protected_vardata);

	uint8_t varReplicatedIndex = srtl.registerSharedResource(protected_replicatedVar);

	// Enregistrement des modules producteurs
	srtl.registerModule(test, "test", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL);
	srtl.registerModule(producer1, "P1", MINIMAL_STACK_SIZE, 1, 0x00, 200,NULL);
	srtl.registerModule(producer2, "P2", MINIMAL_STACK_SIZE, 1, 0x00, 200,NULL);
	srtl.registerModule(producer3, "P3", MINIMAL_STACK_SIZE, 1, 0x00, 200,NULL);
	srtl.registerModule(producer4, "P4", MINIMAL_STACK_SIZE, 1, 0x00, 200,NULL);

	// // Enregistrement des modules consommateurs
	srtl.registerModule(consummer1, "C1", MINIMAL_STACK_SIZE, 1, (1 << var1Index), 200,NULL);					 // Intérêt pour var1
	srtl.registerModule(consummer2, "C2", MINIMAL_STACK_SIZE, 1, (1 << var2Index), 200,NULL);					 // Intérêt pour var2
	srtl.registerModule(consummer3, "C3", MINIMAL_STACK_SIZE, 1, (1 << var1Index) | (1 << var2Index), 200,NULL); // Intérêt pour var1 et var2
	srtl.registerModule(consummer31, "consummer31", MINIMAL_STACK_SIZE, 2, (1 << var1Index) | (1 << var2Index), 200,NULL); // Intérêt pour var1 et var2
	srtl.registerModule(consummer4, "C4", MINIMAL_STACK_SIZE, 1, (1 << vardataIndex), 200,NULL);				 // Intérêt pour vardata

  // // Enregistrer les modules producteurs
  srtl.registerModule(replicatorA, "ReplicatorA", MINIMAL_STACK_SIZE, 1, (1 << varReplicatedIndex), 200, NULL);
  srtl.registerModule(replicatorB, "ReplicatorB", MINIMAL_STACK_SIZE, 1, (1 << varReplicatedIndex), 200, NULL);

	// // Affichage des modules enregistrés
	// for (int i = 0; i < nModule; i++)
	// {
	// 	if (moduleList[i].handle != NULL)
	// 	{
	// 		Serial.printf("Module %d Handle: %p Interest: 0x%X\n", i, moduleList[i].handle, moduleList[i].ressourceOfInterest);
	// 	}
	// }


	delay(1000);
	// monitorStack();
	// monitorHeap();
	Serial.printf("Program size: %d \n",ESP.getSketchSize());
	printMemoryStats();
	
}

void loop()
{
	// Main loop code (optional, as tasks will run independently)
}

#endif