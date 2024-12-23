#include <Arduino.h>
#include <srtl.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

void printMemoryStats()
{
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
	u_int32_t ivar = 0;
	float fvar = 0.0;
};
struct custom_data_t vardata;
float var1 = 0.0, var2 = 0.0;

SemaphoreHandle_t xMutex_var1, xMutex_var2, xMutex_vardata;

protected_data_t protected_var1 = {&xMutex_var1, (void *)&var1};
protected_data_t protected_var2 = {&xMutex_var2, (void *)&var2};
protected_data_t protected_vardata = {&xMutex_vardata, (void *)&vardata};

void producer1(void *pvParameters)
{
	// setup
	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);
	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_var1, portMAX_DELAY) == pdTRUE)
		{
			Serial.println(F("Producer1"));
			*(float *)sharedRessources[0] += 0.01;
			xSemaphoreGive(xMutex_var1);
		}
		notifyAll(0b1, 0b1, eSetBits);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}

void producer2(void *pvParameters)
{
	// uint8_t identity = whoami();
	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);

	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_var2, portMAX_DELAY) == pdTRUE)
		{
			Serial.println(F("Producer2"));
			*(float *)sharedRessources[1] += 0.02;
			xSemaphoreGive(xMutex_var2);
		}
		notifyAll(0b10, 0b10, eSetBits);
		// notifyAll(0b10, 0b10,eSetValueWithOverwrite);
		vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000ms
	}
	vTaskDelete(NULL);
}

void producer3(void *pvParameters)
{
	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);

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
		notifyAll(0b100, 0b11, eSetValueWithOverwrite);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void producer4(void *pvParameters)
{
	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);

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
	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);
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
	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);

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

// wait 2 notification
void consummer3(void *modulePtr)
{
	Module currentModule = *((Module *)modulePtr);
	currentModule.notificationValue = 0;

	float local_var1 = 0.0, local_var2 = 0.0;
	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);

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

// read the first notification arrive
void consummer31(void *modulePtr)
{
	Module currentModule = *((Module *)modulePtr);
	currentModule.notificationValue = 0;
	float local_var1 = 0.0, local_var2 = 0.0;


	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);
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

			if ((EXTRACT_SHARED_RESOURCE(currentModule.notificationValue) & 0b10) == 0b10)
			{

				xSemaphoreTake(xMutex_var2, portMAX_DELAY);
				local_var2 = *(float *)sharedRessources[1];

				Serial.printf("consumer31 var1: %.2f var2: %.2f src: 0x%08X \t2\n ", local_var1, local_var2, currentModule.notificationValue);
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

	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);
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
SemaphoreHandle_t xMutex_replicatedVar; // Mutex pour protéger l'accès à replicatedVar

struct replicated_custom_data_t
{
	float replicatedVar = 0.0; // Ressource partagée à répliquer
	int replicationCount = 0;  // Compteur de répliques
} replicatedVar;

// Protection de la ressource partagée
protected_data_t protected_replicatedVar = {&xMutex_replicatedVar, (void *)&replicatedVar};

// Fonction de synchronisation des producteurs (replicatorA et replicatorB)
void replicatorA(void *pvParameters)
{
	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);

	// Processus de réplication pour replicatorA
	for (;;)
	{
		// Attendre que l'autre producteur (replicatorB) libère la ressource
		if (xSemaphoreTake(xMutex_replicatedVar, portMAX_DELAY) == pdTRUE)
		{
			// Mettre à jour replicatedVar
			((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar += 1.0;
			((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount++; // Incrémenter le compteur

			// Log l'état de la ressource répliquée
			Serial.printf("ReplicatorA: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount);

			// Libérer le mutex pour permettre à l'autre producteur d'accéder à la ressource
			xSemaphoreGive(xMutex_replicatedVar);
		}

		// Ajouter un délai avant la prochaine réplique
		vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
	}
}

void replicatorB(void *pvParameters)
{
	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);
	// Processus de réplication pour replicatorB
	for (;;)
	{
		// Attendre que l'autre producteur (replicatorA) libère la ressource
		if (xSemaphoreTake(xMutex_replicatedVar, portMAX_DELAY) == pdTRUE)
		{
			// Mettre à jour replicatedVar
			((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar += 1.0;
			((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount++; // Incrémenter le compteur

			// Log l'état de la ressource répliquée
			Serial.printf("ReplicatorB: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount);

			// Libérer le mutex pour permettre à l'autre producteur d'accéder à la ressource
			xSemaphoreGive(xMutex_replicatedVar);
		}

		// Ajouter un délai avant la prochaine réplique
		vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
	}
}

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // Reset pin (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Screen center and scale factor
const int centerX = SCREEN_WIDTH / 2;
const int centerY = SCREEN_HEIGHT / 2;
const float scale = 48;

// Cube properties
struct cube_custom_data_t
{
	float angleX = 0;
	float angleY = 0;
	float angleZ = 0;
	float rotationSpeed = 1;

	float cubeVertices[8][3] = {
		{-1, -1, -1},
		{1, -1, -1},
		{1, 1, -1},
		{-1, 1, -1},
		{-1, -1, 1},
		{1, -1, 1},
		{1, 1, 1},
		{-1, 1, 1}};

	int cubeEdges[12][2] = {
		{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
		
} cubeVar;

// not used for know
SemaphoreHandle_t xMutex_CubeVar;
protected_data_t protected_cube = {&xMutex_CubeVar, (void *)&cubeVar};

struct cube_projection_custom_data_t
{
	int projectedVertcices[8][2];
} cubeProjected;
SemaphoreHandle_t xMutex_CubeProj;
protected_data_t protected_cube_projection = {&xMutex_CubeProj, (void *)&cubeProjected};

uint8_t cubeProjectSharedID = 0;

inline void rotateVertex(float vertex[3], float out[3])
{
	// Copy the vertex for transformation
	float x = vertex[0];
	float y = vertex[1];
	float z = vertex[2];

	float cosX = cos(cubeVar.angleX);
    float sinX = sin(cubeVar.angleX);
    float cosY = cos(cubeVar.angleY);
    float sinY = sin(cubeVar.angleY);
    float cosZ = cos(cubeVar.angleZ);
    float sinZ = sin(cubeVar.angleZ);

	// Rotation around X axis
	float tempY = y * cosX - z * sinX;
	float tempZ = y *sinX + z * cosX;
	y = tempY;
	z = tempZ;

	// Rotation around Y axis
	float tempX = x * cosY + z * sinY;
	tempZ = -x * sinY + z * cosY;
	x = tempX;
	z = tempZ;

	// Rotation around Z axis
	tempX = x * cosZ - y * sinZ;
	tempY = x * sinZ + y * cosZ;
	x = tempX;
	y = tempY;

	// Store transformed vertex
	out[0] = x;
	out[1] = y;
	out[2] = z;
}

inline void projectVertex(float vertex[3], int &screenX, int &screenY)
{
	// Apply perspective projection
	float zOffset = 3; // Prevent division by zero
	float perspectiveFactor = 1 / (zOffset - vertex[2]);
	screenX = centerX + (int)(vertex[0] * perspectiveFactor * scale);
	screenY = centerY - (int)(vertex[1] * perspectiveFactor * scale);
}

inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cubeTask(void *modulePtr)
{
	// local variable
	float transformedVertices[8][3];
	int projectedVertices[8][2];

	xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);
	Serial.println("Cube Begin");

	for (;;)
	{
		// no need to protect cube properties for know (nobodie lse use it)
		// Serial.printf("vertice 1 coor %f %f %f ",cubeVar.cubeVertices[0][0],cubeVar.cubeVertices[0][1],cubeVar.cubeVertices[0][2]);
		//  Rotate and project each vertex
		for (int i = 0; i < 8; i++)
		{
			rotateVertex(cubeVar.cubeVertices[i], transformedVertices[i]);
			projectVertex(transformedVertices[i], projectedVertices[i][0], projectedVertices[i][1]);
		}

		// Increment rotation angles depending on controller 0 (joystick)
		if (xSemaphoreTake(xMutex_CubeVar, portMAX_DELAY) == pdTRUE)
		{
			cubeVar.angleX = mapFloat(controllerList[0].analogPinStates[0], 0, 4095, PI/2, -PI/2) * cubeVar.rotationSpeed;
			cubeVar.angleY = mapFloat(controllerList[0].analogPinStates[1], 0, 4095, -PI/2, PI/2) * cubeVar.rotationSpeed;
			// cubeVar.angleX += cubeVar.rotationSpeed;
			// cubeVar.angleY += cubeVar.rotationSpeed;
			// cubeVar.angleZ += cubeVar.rotationSpeed;
			xSemaphoreGive(xMutex_CubeVar);
		}

		// store new Value of projection and send it to Monitor
		if (xSemaphoreTake(xMutex_CubeProj, portMAX_DELAY) == pdTRUE)
		{
			memcpy(cubeProjected.projectedVertcices, projectedVertices, sizeof(int) * 8 * 2);
			notifyMonitor(ARRAY_INDEX_FROM_PTR(moduleList, (Module *)modulePtr), *(int *)(((Module *)modulePtr)->parameters), eSetBits);
			xSemaphoreGive(xMutex_CubeProj);
		}
	vTaskDelay(pdMS_TO_TICKS(25));
	}
}

void monitorCube(void *monitorPtr)
{
	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
	{
		Serial.println(F("SSD1306 allocation failed"));
		for (;;)
			;
	}
	display.clearDisplay();
	display.display();

	// local var
	Monitor currentModule = *((Monitor *)monitorPtr);
	currentModule.notificationValue = 0;
	int projectedVertices[8][2];

	Serial.println("Monitor Begin");

	for (;;)
	{
		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, &currentModule.notificationValue, portMAX_DELAY))
		{
			// Draw the cube :
			display.clearDisplay();
			// load vertices
			if (xSemaphoreTake(xMutex_CubeProj, portMAX_DELAY) == pdTRUE)
			{
				memcpy(projectedVertices, cubeProjected.projectedVertcices, sizeof(int) * 8 * 2);
				xSemaphoreGive(xMutex_CubeProj);
			}

			// Draw edges
			for (int i = 0; i < 12; i++)
			{
				int start = cubeVar.cubeEdges[i][0];
				int end = cubeVar.cubeEdges[i][1];
				display.drawLine(
					projectedVertices[start][0], projectedVertices[start][1],
					projectedVertices[end][0], projectedVertices[end][1],
					SSD1306_WHITE);
			}

			// Update the screen
			display.display();
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void IRAM_ATTR customDigitalController(void * controllerPtr){
	
	Controller *controller = (Controller *)controllerPtr;

	//update first switch
	uint8_t pinState = digitalRead(controller->digitalPins[0].pin);
    controller->digitalPinStates = (controller->digitalPinStates & ~(1 << 0)) | (pinState << 0);

	//update second switch
    pinState = digitalRead(controller->digitalPins[1].pin);
    controller->digitalPinStates = (controller->digitalPinStates & ~(1 << 1)) | (pinState << 1);

}

void IRAM_ATTR joystickSW(void * controllerPtr){
	
	Controller *controller = (Controller *)controllerPtr;

	uint8_t pinState = digitalRead(controller->digitalPins[0].pin);
    controller->digitalPinStates = (controller->digitalPinStates & ~(1 << 0)) | (pinState << 0);


}

// VRx(0) pin 34 VRy(1) pin 39
void IRAM_ATTR joystickVR(TimerHandle_t xTimer)
{
	Controller *controllerPtr = (Controller *)pvTimerGetTimerID(xTimer);

	uint16_t pinState = analogRead(controllerPtr->analogPins[0].pin);
	controllerPtr->analogPinStates[0] = pinState;

	pinState = analogRead(controllerPtr->analogPins[1].pin);
	controllerPtr->analogPinStates[1] = pinState;
}

void setup()
{
	Serial.begin(9600);
	Serial.println(F("\n Setup"));

	// init
	SRTLinit();

	// Enregistrement des ressources partagées
	uint8_t var1Index = registerSharedRessource(protected_var1);
	uint8_t var2Index = registerSharedRessource(protected_var2);
	uint8_t vardataIndex = registerSharedRessource(protected_vardata);

	uint8_t varReplicatedIndex = registerSharedRessource(protected_replicatedVar);

	uint8_t varCubeIndex = registerSharedRessource(protected_cube);
	uint8_t varCubeProjIndex = registerSharedRessource(protected_cube_projection);

	// Enregistrement des modules producteurs
	registerModule(producer1, "P1", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL);
	registerModule(producer2, "P2", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL);
	registerModule(producer3, "P3", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL);
	registerModule(producer4, "P4", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL);

	// // Enregistrement des modules consommateurs
	registerModule(consummer1, "C1", MINIMAL_STACK_SIZE, 1, (1 << var1Index), 200, NULL);							   // Intérêt pour var1
	registerModule(consummer2, "C2", MINIMAL_STACK_SIZE, 1, (1 << var2Index), 200, NULL);							   // Intérêt pour var2
	registerModule(consummer3, "C3", MINIMAL_STACK_SIZE, 1, (1 << var1Index) | (1 << var2Index), 200, NULL);		   // Intérêt pour var1 et var2
	registerModule(consummer31, "consummer31", MINIMAL_STACK_SIZE, 2, (1 << var1Index) | (1 << var2Index), 200, NULL); // Intérêt pour var1 et var2
	registerModule(consummer4, "C4", MINIMAL_STACK_SIZE, 1, (1 << vardataIndex), 200, NULL);						   // Intérêt pour vardata

	// Enregistrer les modules producteurs
	registerModule(replicatorA, "ReplicatorA", MINIMAL_STACK_SIZE , 1, (1 << varReplicatedIndex), 200, NULL);
	registerModule(replicatorB, "ReplicatorB", MINIMAL_STACK_SIZE, 1, (1 << varReplicatedIndex), 200, NULL);

	cubeProjectSharedID = varCubeProjIndex;
	registerModule(cubeTask, "Cube_compute", MINIMAL_STACK_SIZE, 3, 0x00, 50, &cubeProjectSharedID);

	registerMonitor(monitorCube, "OLED_Sreen", MINIMAL_STACK_SIZE, 3, 50, NULL);
	if (sysMonitor.handle != NULL)
	{
		Serial.printf("Monitor Handle: %p Interest: 0x%X\n", sysMonitor.handle, sysMonitor.ressourceOfInterest);
	}

	PinControllerParam joysticksw_param[MAX_DIGITALPIN_IN_CONTROLLER] = {{32,INPUT_PULLDOWN,CHANGE},{0,0,0}};
	PinControllerParam joystickvr_param[MAX_ANALOGPIN_IN_CONTROLLER] = {{34,0,0},{39,0,0}};

	registerController(joystickSW,joysticksw_param,joystickVR,joystickvr_param,20,NULL);

	PinControllerParam customController_param[MAX_DIGITALPIN_IN_CONTROLLER] = {{15,INPUT_PULLDOWN,CHANGE},{27,INPUT_PULLDOWN,CHANGE},{0,0,0}};
	registerController(customDigitalController,customController_param,NULL,NULL,0,NULL);

	// Affichage des modules enregistrés
	for (int i = 0; i < nModule; i++)
	{
		if (moduleList[i].handle != NULL)
		{
			Serial.printf("Module %d Handle: %p Interest: 0x%X\n", i, moduleList[i].handle, moduleList[i].ressourceOfInterest);
		}
	}

	for (int i = 0; i < nController; i++)
	{
		if ((controllerList[i].digitalHandler != NULL))
		{
			Serial.printf("Controller digital %d Handle: %p Pin initialized ===> : \n", i, controllerList[i].digitalHandler);

			for (uint8_t j = 0; j < MAX_DIGITALPIN_IN_CONTROLLER; j++)
			{
				Serial.printf("\t\t\t Pin digital %d num: %d \n", i, controllerList[i].digitalPins[j].pin);
			}
			
		}

		if ((controllerList[i].analogHandler != NULL))
		{
			Serial.printf("Controller analog %d Handle: %p \tPin initialized ===> : \n", i,controllerList[i].analogHandler);

			for (uint8_t j = 0; j < MAX_ANALOGPIN_IN_CONTROLLER; j++)
			{
				Serial.printf("\t\t\t\t\t Pin digital %d num: %d \n", i, controllerList[i].analogPins[j].pin);
			}
			
		}
	}

	// Wait initialization of all task
	delay(3000);

	monitorStack();
	monitorHeap();
	Serial.printf("Program size: %d \n", ESP.getSketchSize());
	printMemoryStats();

	// Join all module and begin
	for (int i = 0; i < nModule; i++)
	{
		if (moduleList[i].handle != NULL)
		{
			notifyModule(i,0,0xFFFF,eNoAction);
		}
	}
}

// Input states
uint8_t swlastVal = 0;
uint16_t vrxlastVal = 0;
uint16_t vrylastVal = 0;

void loop()
{
	// Main loop code (optional, as tasks will run independently)
	// if (swlastVal != controllerList[0].digitalPinStates) {
    //     swlastVal = controllerList[0].digitalPinStates;
    //     Serial.printf("Bouton change ! %d \n",swlastVal);
    // }
	// if (vrxlastVal != controllerList[0].analogPinStates[0]) {
    //     vrxlastVal = controllerList[0].analogPinStates[0];
    //     Serial.printf("Joy X change ! %d ",vrxlastVal);
    // }
	// if (vrylastVal != controllerList[0].analogPinStates[1]) {
    //     vrylastVal = controllerList[0].analogPinStates[1];
    //     Serial.printf("Joy Y change ! %d \n",vrylastVal);
    // }



	// vTaskDelay(pdMS_TO_TICKS(100));
}

#endif

#ifndef C_ONLY

void printMemoryStats()
{
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

protected_data_t protected_var1 = {&xMutex_var1, (void *)&var1};
protected_data_t protected_var2 = {&xMutex_var2, (void *)&var2};
protected_data_t protected_vardata = {&xMutex_vardata, (void *)&vardata};

#define GET_SRTL_INSTANCE ((*((Module *)modulePtr)->parent))

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
		GET_SRTL_INSTANCE.notifyAll(0b1, 0b1, eSetBits);
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
		GET_SRTL_INSTANCE.notifyAll(0b10, 0b10, eSetBits);
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
		GET_SRTL_INSTANCE.notifyAll(0b100, 0b11, eSetValueWithOverwrite);
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

// wait 2 notification
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

// read the first notification arrive
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

			if ((EXTRACT_SHARED_RESOURCE(currentModule.notificationValue) & 0b10) == 0b10)
			{

				xSemaphoreTake(xMutex_var2, portMAX_DELAY);
				local_var2 = *(float *)GET_SRTL_INSTANCE.sharedRessources[1];

				Serial.printf("consumer31 var1: %.2f var2: %.2f src: 0x%08X \t2\n ", local_var1, local_var2, currentModule.notificationValue);
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
SemaphoreHandle_t xMutex_replicatedVar; // Mutex pour protéger l'accès à replicatedVar

struct replicated_custom_data_t
{
	float replicatedVar = 0.0; // Ressource partagée à répliquer
	int replicationCount = 0;  // Compteur de répliques
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
			((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar += 1.0;
			((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount++; // Incrémenter le compteur

			// Log l'état de la ressource répliquée
			Serial.printf("ReplicatorA: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount);

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
			((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar += 1.0;
			((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount++; // Incrémenter le compteur

			// Log l'état de la ressource répliquée
			Serial.printf("ReplicatorB: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount);

			// Libérer le mutex pour permettre à l'autre producteur d'accéder à la ressource
			xSemaphoreGive(xMutex_replicatedVar);
		}

		// Ajouter un délai avant la prochaine réplique
		vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
	}
}

void test(void *modulePtr)
{

	Serial.printf("%d %d\n", ((Module *)modulePtr)->parent->nSharedResource, ((Module *)modulePtr)->parent->nModule);
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
	srtl.registerModule(producer1, "P1", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL);
	srtl.registerModule(producer2, "P2", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL);
	srtl.registerModule(producer3, "P3", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL);
	srtl.registerModule(producer4, "P4", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL);

	// // Enregistrement des modules consommateurs
	srtl.registerModule(consummer1, "C1", MINIMAL_STACK_SIZE, 1, (1 << var1Index), 200, NULL);								// Intérêt pour var1
	srtl.registerModule(consummer2, "C2", MINIMAL_STACK_SIZE, 1, (1 << var2Index), 200, NULL);								// Intérêt pour var2
	srtl.registerModule(consummer3, "C3", MINIMAL_STACK_SIZE, 1, (1 << var1Index) | (1 << var2Index), 200, NULL);			// Intérêt pour var1 et var2
	srtl.registerModule(consummer31, "consummer31", MINIMAL_STACK_SIZE, 2, (1 << var1Index) | (1 << var2Index), 200, NULL); // Intérêt pour var1 et var2
	srtl.registerModule(consummer4, "C4", MINIMAL_STACK_SIZE, 1, (1 << vardataIndex), 200, NULL);							// Intérêt pour vardata

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
	Serial.printf("Program size: %d \n", ESP.getSketchSize());
	printMemoryStats();
}

void loop()
{
	// Main loop code (optional, as tasks will run independently)
}

#endif