#define configCHECK_FOR_STACK_OVERFLOW 2

#undef configUSE_MALLOC_FAILED_HOOK
#define configUSE_MALLOC_FAILED_HOOK 1

#include <Arduino.h>
#include <srtl.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <esp_log.h>

/*
Debug
cd C:\Users\haron\.platformio\packages\toolchain-xtensa-esp32\bin
.\xtensa-esp32-elf-addr2line -pfiaC -e "C:\Users\haron\Documents\PlatformIO\Projects\SimpleRealTimeLayer_Lib\.pio\build\esp32doit-devkit-v1\firmware.elf" backtrace
*/

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	log_e("Stack overflow in task: %s\n", pcTaskName);
	while (1)
		;
}

void vApplicationMallocFailedHook(void)
{
	log_e("Heap allocation failed!");
	while (1)
		;
}

// #define configTOTAL_HEAP_SIZE ((1024) * (128))

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
	float tempZ = y * sinX + z * cosX;
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

inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
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
			cubeVar.angleX = mapFloat(controllerList[0].analogPinStates[0], 0, 4095, -PI / 2, PI / 2) * cubeVar.rotationSpeed;
			cubeVar.angleY = mapFloat(controllerList[0].analogPinStates[1], 0, 4095, PI / 2, -PI / 2) * cubeVar.rotationSpeed;
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

void IRAM_ATTR customDigitalController(void *controllerPtr)
{

	Controller *controller = (Controller *)controllerPtr;

	// update first switch
	uint8_t pinState = digitalRead(controller->digitalPins[0].pin);
	controller->digitalPinStates = (controller->digitalPinStates & ~(1 << 0)) | (pinState << 0);

	// update second switch
	pinState = digitalRead(controller->digitalPins[1].pin);
	controller->digitalPinStates = (controller->digitalPinStates & ~(1 << 1)) | (pinState << 1);
}

void IRAM_ATTR joystickSW(void *controllerPtr)
{

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
	registerModule(replicatorA, "ReplicatorA", MINIMAL_STACK_SIZE, 1, (1 << varReplicatedIndex), 200, NULL);
	registerModule(replicatorB, "ReplicatorB", MINIMAL_STACK_SIZE, 1, (1 << varReplicatedIndex), 200, NULL);

	cubeProjectSharedID = varCubeProjIndex;
	registerModule(cubeTask, "Cube_compute", MINIMAL_STACK_SIZE, 3, 0x00, 50, &cubeProjectSharedID);

	registerMonitor(monitorCube, "OLED_Sreen", MINIMAL_STACK_SIZE, 3, 50, NULL);
	if (sysMonitor.handle != NULL)
	{
		Serial.printf("Monitor Handle: %p Interest: 0x%X\n", sysMonitor.handle, sysMonitor.ressourceOfInterest);
	}

	PinControllerParam joysticksw_param[MAX_DIGITALPIN_IN_CONTROLLER] = {{32, INPUT_PULLDOWN, CHANGE}, {0, 0, 0}};
	PinControllerParam joystickvr_param[MAX_ANALOGPIN_IN_CONTROLLER] = {{34, 0, 0}, {39, 0, 0}};

	registerController(joystickSW, joysticksw_param, joystickVR, joystickvr_param, 20, NULL);

	PinControllerParam customController_param[MAX_DIGITALPIN_IN_CONTROLLER] = {{15, INPUT_PULLDOWN, CHANGE}, {27, INPUT_PULLDOWN, CHANGE}, {0, 0, 0}};
	registerController(customDigitalController, customController_param, NULL, NULL, 0, NULL);

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
			Serial.printf("Controller analog %d Handle: %p \tPin initialized ===> : \n", i, controllerList[i].analogHandler);

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
			notifyModule(i, 0, 0xFFFF, eNoAction);
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

#undef INCLUDE_uxTaskGetStackHighWaterMark
#define INCLUDE_uxTaskGetStackHighWaterMark 1
/*
	// #undef configGENERATE_RUN_TIME_STATS
	// #define configGENERATE_RUN_TIME_STATS 1
	// #undef configUSE_STATS_FORMATTING_FUNCTIONS
	// #define configUSE_STATS_FORMATTING_FUNCTIONS 1
	// #undef configSUPPORT_DYNAMIC_ALLOCATION
	// #define configSUPPORT_DYNAMIC_ALLOCATION 1
	// #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() configureTimerForRunTimeStats()
	// #define portGET_RUN_TIME_COUNTER_VALUE() getRunTimeCounterValue()

	// // Déclaration globale pour le timer
	// hw_timer_t *timer = NULL;
	// volatile uint32_t runTimeCounter = 0;

	// // Fonction pour configurer le timer pour les statistiques FreeRTOS
	// void configureTimerForRunTimeStats() {
	//     // Initialiser le timer matériel
	//     timer = timerBegin(0, 80, true); // Timer 0, diviseur d'horloge (80 pour µs), comptage ascendant
	//     timerAttachInterrupt(timer, []() {
	//         runTimeCounter++;
	//     }, true); // Lier une ISR pour incrémenter le compteur
	//     timerAlarmWrite(timer, 1000, true); // Déclenchement toutes les 1000 µs (1 ms)
	//     timerAlarmEnable(timer); // Activer l'alarme
	// }

	// // Fonction pour obtenir la valeur du compteur d'exécution
	// unsigned long getRunTimeCounterValue() {
	//     return runTimeCounter; // Retourne la valeur du compteur global
	// }


	// void runTimeStats(){
	// 	Serial.println("=== FreeRTOS run TIME ===");
	// 	char buffer[1024];
	// 	vTaskGetRunTimeStats(buffer);
	// 	Serial.println(buffer);
	// 	Serial.println("=== FreeRTOS run TIME ===");
	// }
*/

/*
État des piles (stacks) allouées pour les tâches.
Détection d'un éventuel débordement de pile pour un module.
*/
void monitorStack(SRTL &instance)
{
	// cas très intéressant : ne jamais passer un objet par copie si son destructeur agit sur des variables partagés et handler de tâche (comme srtl)
	Module *m;
	Serial.println("=== Statistiques Modules ===");
	for (uint8_t i = 1; i < instance.nModule; i++)
	{
		m = &instance.moduleList[i];
		if (m->handle != NULL && eTaskGetState(m->handle) != eDeleted)
		{
			// Mesure de la quantité de stack libre en mots
			UBaseType_t stackLeft = uxTaskGetStackHighWaterMark(m->handle);

			// Conversion en octets (si nécessaire, dépend de la taille du mot de l'architecture)
			size_t stackSizeBytes = stackLeft * sizeof(StackType_t);
			Serial.printf("Task '%d': Stack remaining: %d bytes\n", ARRAY_INDEX_FROM_PTR(instance.moduleList, m), stackSizeBytes);
		}
		else
		{
			Serial.printf("Task '%d': Invalid handle!\n", ARRAY_INDEX_FROM_PTR(instance.moduleList, m));
		}
	}

	if (instance.sysMonitor.handle != NULL)
	{
		UBaseType_t stackLeft = uxTaskGetStackHighWaterMark(instance.sysMonitor.handle);
		size_t stackSizeBytes = stackLeft * sizeof(StackType_t);
		Serial.printf("Task 'sysMonitor': Stack remaining: %d bytes\n", stackSizeBytes);
	}
	else
	{
		Serial.printf("Task 'sysMonitor': Invalid handle!\n");
	}

	Serial.println("============================");
}

/*
* Affiche la taille du programme (section TEXT). Inclut les instructions machine et les constantes en ROM.
* Renvoie la taille totale du tas global (somme de tous les blocs disponibles pour l'allocation dynamique).
* Renvoie la mémoire disponible dans le tas global.
* Affiche la taille du plus grand bloc contigu libre dans le tas global. Mesure utile pour déterminer si une allocation de mémoire importante peut réussir.
* Affiche la taille totale de la mémoire interne (DRAM uniquement, sans PSRAM).
* Affiche la mémoire DRAM libre.
* Renvoie la quantité de mémoire libre actuellement disponible dans le tas global.
	Exprime la fragmentation potentielle si le tas est très divisé en petits blocs.
* Renvoie la plus petite quantité de mémoire libre jamais enregistrée depuis le démarrage du système.
	Indique si  l'application s’approche d’un épuisement de mémoire.
*/
void printMemoryStats()
{
	Serial.println("=== Statistiques Mémoire ===");
	Serial.print("Program size : ");
	Serial.println(ESP.getSketchSize());

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

	Serial.printf("\n\tFree Heap Size: %d bytes\n", xPortGetFreeHeapSize());
	Serial.printf("\tMinimum Ever Free Heap Size: %d bytes\n\n", xPortGetMinimumEverFreeHeapSize());

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

void producer1(void *modulePtr)
{
	// setup
	GET_SRTL_INSTANCE.join(1, GET_CURRENT_MODULE_INDEX);
	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_var1, portMAX_DELAY) == pdTRUE)
		{
			// Serial.println(F("Producer1"));
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
	uint32_t ret = GET_SRTL_INSTANCE.join(1, GET_CURRENT_MODULE_INDEX);
	// Serial.printf("RETV %d\n", ret);

	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_var2, portMAX_DELAY) == pdTRUE)
		{
			// Serial.println(F("Producer2"));
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
	GET_SRTL_INSTANCE.join(1, GET_CURRENT_MODULE_INDEX);

	// Task implementation
	for (;;)
	{
		// Serial.println(F("Producer3"));
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
	GET_SRTL_INSTANCE.join(1, GET_CURRENT_MODULE_INDEX);

	// Task implementation
	for (;;)
	{
		if (xSemaphoreTake(xMutex_vardata, portMAX_DELAY) == pdTRUE)
		{
			((custom_data_t *)GET_SRTL_INSTANCE.sharedRessources[2])->cvar += 1;
			xSemaphoreGive(xMutex_vardata);
		}

		// Serial.printf("test %d\n",xMutex_var2->uxItemSize);

		if (GET_SRTL_INSTANCE.notifyAll(0b1000, 0b100, eSetValueWithoutOverwrite))
			// Serial.println(F("Producer4"));

		vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 1000ms
	}
}

void consummer1(void *modulePtr)
{
	float local_var1 = 0.0;

	GET_SRTL_INSTANCE.join(0, GET_CURRENT_MODULE_INDEX);

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
		// Serial.printf("consumer1 var1: %.2f\n", local_var1);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

void consummer2(void *modulePtr)
{

	float local_var2 = 0.0;
	GET_SRTL_INSTANCE.join(0, GET_CURRENT_MODULE_INDEX);
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
		// Serial.printf("consumer2 var2: %.2f\n", local_var2);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

// wait 2 notification
void consummer3(void *modulePtr)
{
	Module * currentModule = ((Module *)modulePtr);
	currentModule->notificationValue = 0;

	float local_var1 = 0.0, local_var2 = 0.0;

	GET_SRTL_INSTANCE.join(0, GET_CURRENT_MODULE_INDEX);
	// Task implementation
	for (;;)
	{
		if (xTaskNotifyWait(0xFFFFFFFF, 0x00, &currentModule->notificationValue, pdMS_TO_TICKS(250)) &&
			((EXTRACT_SHARED_RESOURCE(currentModule->notificationValue) & 0x03) == 0x03))
		{

			xSemaphoreTake(xMutex_var1, portMAX_DELAY);
			local_var1 = *(float *)GET_SRTL_INSTANCE.sharedRessources[0];
			xSemaphoreGive(xMutex_var1);

			xSemaphoreTake(xMutex_var2, portMAX_DELAY);
			local_var2 = *(float *)GET_SRTL_INSTANCE.sharedRessources[1];
			xSemaphoreGive(xMutex_var2);

			// Serial.printf("consumer3 var1: %.2f var2: %.2f src: 0x%08X\n", local_var1, local_var2, currentModule.notificationValue);
		}
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

// read the first notification arrive
void consummer31(void *modulePtr)
{
	Module * currentModule = ((Module *)modulePtr);
	currentModule->notificationValue = 0;
	float local_var1 = 0.0, local_var2 = 0.0;

	GET_SRTL_INSTANCE.join(0, GET_CURRENT_MODULE_INDEX);
	// Task implementation
	for (;;)
	{
		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, &currentModule->notificationValue, portMAX_DELAY))
		{
			// Serial.printf(" src: 0x%08X \t\n", currentModule.notificationValue);
			if ((EXTRACT_SHARED_RESOURCE(currentModule->notificationValue) & 0b1) == 0b1)
			{
				xSemaphoreTake(xMutex_var1, portMAX_DELAY);
				local_var1 = *(float *)GET_SRTL_INSTANCE.sharedRessources[0];

				// Serial.printf("consumer31 var1: %.2f var2: %.2f src: 0x%08X \t1\n", local_var1, local_var2, currentModule.notificationValue);
				xSemaphoreGive(xMutex_var1);
			}

			if ((EXTRACT_SHARED_RESOURCE(currentModule->notificationValue) & 0b10) == 0b10)
			{

				xSemaphoreTake(xMutex_var2, portMAX_DELAY);
				local_var2 = *(float *)GET_SRTL_INSTANCE.sharedRessources[1];

				// Serial.printf("consumer31 var1: %.2f var2: %.2f src: 0x%08X \t2\n ", local_var1, local_var2, currentModule.notificationValue);
				xSemaphoreGive(xMutex_var2);
			}

			currentModule->notificationValue = 0;
		}
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

// notification send if previous notification consume (see producer4)
void consummer4(void *modulePtr)
{
	custom_data_t local_var;
	uint32_t ulNotifiedValue = 0;
	GET_SRTL_INSTANCE.join(0, GET_CURRENT_MODULE_INDEX);
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
		// Serial.printf("consumer4 var: %d\n", local_var.cvar);
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
			// Serial.printf("ReplicatorA: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount);

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
			// Serial.printf("ReplicatorB: replicatedVar = %.2f, replicationCount = %d\n", ((replicated_custom_data_t *)protected_replicatedVar.data)->replicatedVar, ((replicated_custom_data_t *)protected_replicatedVar.data)->replicationCount);

			// Libérer le mutex pour permettre à l'autre producteur d'accéder à la ressource
			xSemaphoreGive(xMutex_replicatedVar);
		}

		// Ajouter un délai avant la prochaine réplique
		vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
	}
}

void test(void *modulePtr)
{
	GET_SRTL_INSTANCE.join(2, GET_CURRENT_MODULE_INDEX);
	// Serial.printf("TEST ===> nShared ressource : %d , nModule : %d\n", ((Module *)modulePtr)->parent->nSharedResource, ((Module *)modulePtr)->parent->nModule);
	vTaskDelay(pdMS_TO_TICKS(10000));
	// Serial.printf("TEST ===> task late of %d ms\n", millis() - 10000);
	vTaskDelete(NULL);
}

// TEST CUBE : MONITOR & CONTROLLER

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
	float tempZ = y * sinX + z * cosX;
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

inline void projectVertex(float vertex[3], int &screenX, int &screenY, uint8_t drawScale = scale)
{
	// Apply perspective projection
	float zOffset = 3; // Prevent division by zero
	float perspectiveFactor = 1 / (zOffset - vertex[2]);
	screenX = centerX + (int)(vertex[0] * perspectiveFactor * drawScale);
	screenY = centerY - (int)(vertex[1] * perspectiveFactor * drawScale);
}

inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cubeTask(void *modulePtr)
{
	// local variable
	float transformedVertices[8][3];
	int projectedVertices[8][2];

	// xTaskNotifyWait(0, 0, &vardata.ivar, portMAX_DELAY);
	Serial.println("Cube Begin");


	GET_SRTL_INSTANCE.join(2, GET_CURRENT_MODULE_INDEX);
	for (;;)
	{

		//  Rotate and project each vertex
		for (int i = 0; i < 8; i++)
		{
			rotateVertex(cubeVar.cubeVertices[i], transformedVertices[i]);
			projectVertex(transformedVertices[i], projectedVertices[i][0], projectedVertices[i][1]);
		}

		// Increment rotation angles depending on controller 0 (joystick)
		if (xSemaphoreTake(xMutex_CubeVar, portMAX_DELAY) == pdTRUE)
		{
			cubeVar.angleX = mapFloat(GET_SRTL_INSTANCE.controllerList[0].analogPinStates[0], 0, 4095, -PI / 2, PI / 2) * cubeVar.rotationSpeed;
			cubeVar.angleY = mapFloat(GET_SRTL_INSTANCE.controllerList[0].analogPinStates[1], 0, 4095, PI / 2, -PI / 2) * cubeVar.rotationSpeed;
			// cubeVar.angleX += cubeVar.rotationSpeed;
			// cubeVar.angleY += cubeVar.rotationSpeed;
			// cubeVar.angleZ += cubeVar.rotationSpeed;
			xSemaphoreGive(xMutex_CubeVar);
		}

		// store new Value of projection and send it to Monitor
		if (xSemaphoreTake(xMutex_CubeProj, portMAX_DELAY) == pdTRUE)
		{
			memcpy(cubeProjected.projectedVertcices, projectedVertices, sizeof(int) * 8 * 2);
			// Utilisation bancale de notify, mettre directement l'index n'est pas génant mais peut override les bits des autres modules qui notifie
			// params contient l'id de la ressource partagé
			GET_SRTL_INSTANCE.notifyMonitor(GET_CURRENT_MODULE_INDEX, GET_PARAMS_INSTANCE(int), eSetBits);
			xSemaphoreGive(xMutex_CubeProj);
		}
		vTaskDelay(pdMS_TO_TICKS(25));
	}
}

void IRAM_ATTR customDigitalController(void *controllerPtr)
{

	Controller *controller = (Controller *)controllerPtr;

	// update first switch
	uint8_t pinState = digitalRead(controller->digitalPins[0].pin);
	controller->digitalPinStates = (controller->digitalPinStates & ~(1 << 0)) | (pinState << 0);

	// update second switch
	pinState = digitalRead(controller->digitalPins[1].pin);
	controller->digitalPinStates = (controller->digitalPinStates & ~(1 << 1)) | (pinState << 1);
}

void IRAM_ATTR joystickSW(void *controllerPtr)
{

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

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void monitorCube(void *monitorPtr)
{
	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
	{
		Serial.println(F("SSD1306 allocation failed"));
		for (;;)
			;
	}

	Serial.print("Stack size monitor: ");
	Serial.println(uxTaskGetStackHighWaterMark(NULL));
	display.clearDisplay();
	vTaskDelay(pdMS_TO_TICKS(1000));
	Serial.println("Monitor Begin");

	// portENTER_CRITICAL(&mux);
	display.display();
	// portEXIT_CRITICAL(&mux);

	display.setTextColor(WHITE, BLACK);
	display.setTextSize(0);

	// local var
	Monitor * currentModule = ((Monitor *)monitorPtr);
	currentModule->notificationValue = 0;
	int cubeProjectedVertices[8][2];

	Serial.println("Monitor Begin");

	for (;;)
	{
		// from any task
		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, &currentModule->notificationValue, portMAX_DELAY))
		{
			// Draw the cube :
			display.clearDisplay();
			// load vertices
			if (xSemaphoreTake(xMutex_CubeProj, portMAX_DELAY) == pdTRUE)
			{
				memcpy(cubeProjectedVertices, cubeProjected.projectedVertcices, sizeof(int) * 8 * 2);
				xSemaphoreGive(xMutex_CubeProj);
			}

			// Draw edges
			for (int i = 0; i < 12; i++)
			{
				int start = cubeVar.cubeEdges[i][0];
				int end = cubeVar.cubeEdges[i][1];
				display.drawLine(
					cubeProjectedVertices[start][0], cubeProjectedVertices[start][1],
					cubeProjectedVertices[end][0], cubeProjectedVertices[end][1],
					SSD1306_WHITE);
			}

			// Update the screen
			display.display();
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

// Test event

void eventTest(void *modulePtr)
{
	// ERROR  NON copie une structure dans la variable locale  : Module currentModule = *((Module *)modulePtr);
	// 				
	Module * currentModule = ((Module *)modulePtr);


	eventTimer eventList[MAX_EVENT_TIMER_IN_MODULE]; // Tableau de test
	uint8_t eventCount = 0;

	vTaskDelay(pdMS_TO_TICKS(2000));

	uint32_t now = CURRENT_EPOCH;

	// TODO  essayéavec commutation intra-tâche notify inter-tâche 
	// compter les missed pour que l'utilisateur gère les famines

	// 0 Minuteur il s'éxécute forcment au bout de 1000 secondes
	addEvent(eventList, &eventCount, true, true, now + 1000, 0);

	// 1 Minuteur dans 500 secondes il s'éxécutera toutes les 1000 secondes pour sur
	addEvent(eventList, &eventCount, true, true, now + 500, 1000);

	// 2 Minuteur  il s'éxécute en retard
	addEvent(eventList, &eventCount,  true, true, now - 100, 0);

	// 3 Minuteur Très en retard, il rattrape toutes ses éxécutions puis toutes les 1000 secondes pour sur
	addEvent(eventList, &eventCount,  true, true, now - 20010, 1000);

	// 4 Minuteur s'éxécute dans 500 secondes sauf en cas de retard
	addEvent(eventList, &eventCount,  true, false, now + 500, 0);

	// 5 Minuteur s'éxécute dans 1000 secondes toutes les 1000 secondes sauf en cas de retard
	// la mae lui permet d'être éxécuté de force tous les 5 évènement ratés
	addEvent(eventList, &eventCount,  true, false, now + 1000, 1000);

	// 6 Minuteur à beaucoup de retard et ne s'éxécutera jamais
	addEvent(eventList, &eventCount,  true, false, now - 20010 , 0);

	// 7 Minuteur à beaucoup de retard et se reporte à 1000 secondes sauf en cas de retard
	// la mae lui permet d'être éxécuté de force tous les 200 secondes de retard accumulé
	addEvent(eventList, &eventCount,  true, false, now - 20010 , 1000);

	// 8 Minuteur désactivé (ne doit pas être pris en compte)
	addEvent(eventList, &eventCount,  false, false, now , 0);

	// 9 Minuteur flood sans correction
	addEvent(eventList, &eventCount,  true, false, now + 50, 10);

	// 10 Minuteur flood 
	addEvent(eventList, &eventCount,  true, true, now + 50, 10);

	// 11 Minuteur de fin
	addEvent(eventList, &eventCount,  true, true, now + 10000, 0);

	// Debug
	Serial.println("Tableau d'événements créé:");
	Serial.printf("Actual Time : %d\n", now);
	for (uint8_t i = 0; i < eventCount; i++)
	{
		Serial.printf("ID: %d | Actif: %d | Périodique: %d | Rectifié: %d | Time: %d | Period: %d\n",
					  GET_EVENT_TIMER_ID(eventList[i].flags),
					  GET_EVENT_TIMER_ACTIVE(eventList[i].flags),
					  GET_EVENT_TIMER_PERIOD(eventList[i].flags),
					  GET_EVENT_TIMER_ISRECT(eventList[i].flags),
					  eventList[i].time,
					  eventList[i].period);
	}

	for (;;)
	{
		uint8_t eventID = GET_SRTL_INSTANCE.autoTimer(currentModule, eventList, eventCount);

		if(eventID < eventCount){
			Serial.printf("Epoch: %d | ID: %d | Actif: %d | Périodique: %d | Rectifié: %d | Time: %d | Period: %d | Count: %d | Miss: %d | Last: %d\n",
					  CURRENT_EPOCH,
					  eventID,
					  GET_EVENT_TIMER_ACTIVE(eventList[eventID].flags),
					  GET_EVENT_TIMER_PERIOD(eventList[eventID].flags),
					  GET_EVENT_TIMER_ISRECT(eventList[eventID].flags),
					  eventList[eventID].time,
					  eventList[eventID].period,
					  eventList[eventID].eventCount,
					  eventList[eventID].missedCount,
					  eventList[eventID].lastTimer
					  );


		// if(CURRENT_EPOCH > SYNC_SYSTIME_CONFIG + 10000)// ~3h
		if(eventID == 11)// ~3h
			break;


		if(eventID == 5)
			SET_EVENT_TIMER_ISRECT(eventList[eventID].flags,0);

		if(eventID == 7)
			SET_EVENT_TIMER_ISRECT(eventList[eventID].flags,0);


		if(eventID == 10)
			SRTL::GetCurrentInstance(currentModule).notifyModule(GET_CURRENT_MODULE_INDEX + 1,INDEX_TO_BITSET(GET_CURRENT_MODULE_INDEX),0,eSetBits);
			// GET_SRTL_INSTANCE.notifyModule(GET_CURRENT_MODULE_INDEX + 1,INDEX_TO_BITSET(GET_CURRENT_MODULE_INDEX),0,eSetBits);

		
		}
		else
			Serial.println("Error in EventList");

		if(eventList[5].missedCount > 5){
			SET_EVENT_TIMER_ISRECT(eventList[5].flags,1);
			eventList[5].missedCount = 0;
		}

		if(syncTimer(eventList[7].time) < -200){
			SET_EVENT_TIMER_ISRECT(eventList[7].flags,1);
		}

		vTaskDelay(pdMS_TO_TICKS(currentModule->taskFrequency));

	}


	Serial.printf("\n\nend\n\n");

	for (uint8_t i = 0; i < eventCount; i++)
	{
		if(i < eventCount)
			Serial.printf("Epoch: %d | ID: %d | Actif: %d | Périodique: %d | Rectifié: %d | Time: %d | Period: %d | Count: %d | Last: %d\n",
					  CURRENT_EPOCH,
					  i,
					  GET_EVENT_TIMER_ACTIVE(eventList[i].flags),
					  GET_EVENT_TIMER_PERIOD(eventList[i].flags),
					  GET_EVENT_TIMER_ISRECT(eventList[i].flags),
					  eventList[i].time,
					  eventList[i].period,
					  eventList[i].eventCount,
					  eventList[i].lastTimer);
	}

	vTaskDelete(NULL);
}

void eventTestReceiver(void *modulePtr){
	Module * currentModule = ((Module *)modulePtr);
	currentModule->notificationValue = 0;

	for(;;){

		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, &currentModule->notificationValue, portMAX_DELAY)){
			if(EXTRACT_SOURCE_MODULE(currentModule->notificationValue) == INDEX_TO_BITSET((GET_CURRENT_MODULE_INDEX - 1))){
				Serial.println("\nEvent Receive\n");
			}
		}

	moduleSleep(currentModule);
	}

}

void pwmTask(void *modulePtr){
	Module * currentModule = ((Module *)modulePtr);

	const int pwmChannel = 0;  // Canal PWM (ESP32 supporte plusieurs canaux)
	const int pwmFreq = 300;  // Fréquence du signal PWM en Hz
	const int pwmResolution = 6;  // Résolution du signal PWM (6 bits = 64 niveaux)
	const int pwmPin = 4;  // GPIO sur lequel on génère le signal PWM
	int duty = 63;

	ledcSetup(pwmChannel, pwmFreq, pwmResolution);
	ledcAttachPin(pwmPin, pwmChannel);
	ledcWrite(pwmChannel, duty); 


	for(;;){
	// duty = (duty+4)% ((1<<pwmResolution)+4);
	// ledcWrite(pwmChannel, duty); 
	// Serial.println(duty);
	vTaskDelay(pdMS_TO_TICKS(currentModule->taskFrequency));
	}

}

// TEST
struct Digit{

};

struct DigitalClock
{
	struct Digit digits[6];
} digitalClock;

SemaphoreHandle_t xMutex_digitalClockVar;
protected_data_t protected_clock = {&xMutex_digitalClockVar, (void *)&digitalClock};

int clockProjected[6][6][2];
SemaphoreHandle_t xMutex_ClockProj;
protected_data_t protected_clock_projection = {&xMutex_ClockProj, (void *)&clockProjected};

// main loop :

// init
SRTL srtl;

SET_LOOP_TASK_STACK_SIZE(MINIMAL_STACK_SIZE * 10);

void setup()
{
	Serial.begin(115200);
	Serial.println(F("In Setup function"));

	esp_log_level_set("*", ESP_LOG_ERROR); // Activer tous les logs
	esp_log_level_set("*", ESP_LOG_WARN);  // Activer tous les logs
	// esp_log_level_set("I2C", ESP_LOG_ERROR);  // Log uniquement les erreurs pour I2C

	log_e("test");

	if (!syncWifiInit("Freebox-022439", "SebastienSexy7"))
	{
		Serial.println("Failed to connect to WiFi.");
		return;
	}
	Serial.print(SYNC_SYSTIME_CONFIG);

	// Enregistrement des ressources partagées
	uint8_t var1Index = srtl.registerSharedResource(protected_var1);
	uint8_t var2Index = srtl.registerSharedResource(protected_var2);
	uint8_t vardataIndex = srtl.registerSharedResource(protected_vardata);

	uint8_t varReplicatedIndex = srtl.registerSharedResource(protected_replicatedVar);

	uint8_t varCubeIndex = srtl.registerSharedResource(protected_cube);
	uint8_t varCubeProjIndex = srtl.registerSharedResource(protected_cube_projection);

	uint8_t varClockIndex = srtl.registerSharedResource(protected_clock);
	uint8_t varClockProjIndex = srtl.registerSharedResource(protected_clock_projection);

	uint32_t consumerBits = 0;
	uint32_t producerBits = 0;
	uint32_t otherBits = 0;

	otherBits |= INDEX_TO_BITSET(srtl.registerModule(test, "test", MINIMAL_STACK_SIZE, 5, 0x00, 200, NULL));

	// Enregistrement des modules producteurs
	producerBits |= INDEX_TO_BITSET(srtl.registerModule(producer1, "P1", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL));
	producerBits |= INDEX_TO_BITSET(srtl.registerModule(producer2, "P2", MINIMAL_STACK_SIZE, 1, 0x00, 200, NULL));
	producerBits |= INDEX_TO_BITSET(srtl.registerModule(producer3, "P3", MINIMAL_STACK_SIZE * 2, 1, 0x00, 200, NULL));
	producerBits |= INDEX_TO_BITSET(srtl.registerModule(producer4, "P4", MINIMAL_STACK_SIZE * 2, 1, 0x00, 200, NULL));

	// Enregistrement des modules consommateurs
	consumerBits |= INDEX_TO_BITSET(srtl.registerModule(consummer1, "C1", MINIMAL_STACK_SIZE, 1, (1 << var1Index), 200, NULL));									 // Intérêt pour var1
	consumerBits |= INDEX_TO_BITSET(srtl.registerModule(consummer2, "C2", MINIMAL_STACK_SIZE, 1, (1 << var2Index), 200, NULL));									 // Intérêt pour var2
	consumerBits |= INDEX_TO_BITSET(srtl.registerModule(consummer3, "C3", MINIMAL_STACK_SIZE * 2, 1, (1 << var1Index) | (1 << var2Index), 200, NULL));			 // Intérêt pour var1 et var2
	consumerBits |= INDEX_TO_BITSET(srtl.registerModule(consummer31, "consummer31", MINIMAL_STACK_SIZE * 3, 2, (1 << var1Index) | (1 << var2Index), 200, NULL)); // Intérêt pour var1 et var2
	consumerBits |= INDEX_TO_BITSET(srtl.registerModule(consummer4, "C4", MINIMAL_STACK_SIZE, 1, (1 << vardataIndex), 200, NULL));								 // Intérêt pour vardata

	// // Enregistrer les modules producteurs
	srtl.registerModule(replicatorA, "ReplicatorA", MINIMAL_STACK_SIZE * 2, 1, (1 << varReplicatedIndex), 200, NULL);
	srtl.registerModule(replicatorB, "ReplicatorB", MINIMAL_STACK_SIZE * 2, 1, (1 << varReplicatedIndex), 200, NULL);

	// TEST CUBE

	cubeProjectSharedID = varCubeProjIndex;
	otherBits |= INDEX_TO_BITSET(srtl.registerModule(cubeTask, "Cube_compute", MINIMAL_STACK_SIZE * 2, 3, 0x00, 50, &cubeProjectSharedID));

	// MONITOR

	srtl.registerMonitor(monitorCube, "OLED_Sreen", MINIMAL_STACK_SIZE * 3, (configMAX_PRIORITIES - 1), 50, NULL);
	if (srtl.sysMonitor.handle != NULL)
	{
		Serial.printf("Monitor Handle: %p Interest: 0x%X\n", srtl.sysMonitor.handle, srtl.sysMonitor.ressourceOfInterest);
	}

	// CONTROLLER

	PinControllerParam joysticksw_param[MAX_DIGITALPIN_IN_CONTROLLER] = {{32, INPUT_PULLDOWN, CHANGE}, {0, 0, 0}};
	PinControllerParam joystickvr_param[MAX_ANALOGPIN_IN_CONTROLLER] = {{34, 0, 0}, {39, 0, 0}};

	srtl.registerController(joystickSW, joysticksw_param, joystickVR, joystickvr_param, 20, NULL);

	// PinControllerParam customController_param[MAX_DIGITALPIN_IN_CONTROLLER] = {{15, INPUT_PULLDOWN, CHANGE}, {27, INPUT_PULLDOWN, CHANGE}, {0, 0, 0}};
	// srtl.registerController(customDigitalController, customController_param, NULL, NULL, 0, NULL);

	// TEST EVENT
	srtl.registerModule(eventTest, "Event", MINIMAL_STACK_SIZE * 2, 5, 0, 1, NULL);
	srtl.registerModule(eventTestReceiver, "Event Receiver",MINIMAL_STACK_SIZE * 2, 5, 0, 999, NULL);


	// PWM
	srtl.registerModule(pwmTask, "PWM",MINIMAL_STACK_SIZE, 5, 0, 500, NULL);

	// JOIN

	assert(consumerBits == 0b1111100000);
	assert(producerBits == 0b11110);

	// Wait initialization of all task with join barriere consummers(11 1110 0000) : 0, producers(1 1110) : 1, cubetask : 2 (1 0000 0000 0000)
	while (srtl.joinList[0] != consumerBits)
	{
		Serial.printf("wait 0 : %s\n", TO_BIN(srtl.joinList[0]));
	} // ALL consummers are waiting
	uint32_t released = srtl.release(0);
	Serial.printf("Consummer Released %s\n", TO_BIN(released));
	while ((srtl.joinList[1] != producerBits) && (released != consumerBits))
	{
		Serial.printf("wait 0 : %s, wait 1 :%s\n", TO_BIN(released), TO_BIN(srtl.joinList[1]));
	} // ALL producer are ready and consummer are waiting
	released = srtl.unjoin(1, 4);
	Serial.printf("Producer  %d leave barrier 1\n", released);

	delay(1000);
	Serial.print("Stack size setup(): ");
	Serial.println(uxTaskGetStackHighWaterMark(NULL));

	released = srtl.release(1);
	Serial.printf("Producer Released %s\n", TO_BIN(released));

	while (srtl.sysMonitor.handle == NULL)
	{
		Serial.printf("wait 2 %s\n", TO_BIN(srtl.joinList[2]));
	} // Handler has begin
	released = srtl.release(2);
	Serial.printf("Cube Released %s\n", TO_BIN(released));

	// DEBUG

	// Affichage des modules enregistrés
	Serial.println("================ MODULE LIST ===================");
	for (int i = 0; i < srtl.nModule; i++)
	{
		if (srtl.moduleList[i].handle != NULL)
		{
			Serial.printf("Module %d Handle: %p Interest: 0x%X\n", i, srtl.moduleList[i].handle, srtl.moduleList[i].ressourceOfInterest);
		}
	}
	Serial.println("================ ============== ===================");
	Serial.println("\n\n\n");

	// Affichage des controllers
	Serial.println("================ CONTROLLER LIST ===================");
	for (int i = 0; i < srtl.nController; i++)
	{
		if ((srtl.controllerList[i].digitalHandler != NULL))
		{
			Serial.printf("Controller digital %d Handle: %p Pin initialized ===> : \n", i, srtl.controllerList[i].digitalHandler);

			for (uint8_t j = 0; j < MAX_DIGITALPIN_IN_CONTROLLER; j++)
			{
				Serial.printf("\t\t\t Pin digital %d num: %d \n", i, srtl.controllerList[i].digitalPins[j].pin);
			}
		}

		Serial.println("\n");

		if ((srtl.controllerList[i].analogHandler != NULL))
		{
			Serial.printf("Controller analog %d Handle: %p \tPin initialized ===> : \n", i, srtl.controllerList[i].analogHandler);

			for (uint8_t j = 0; j < MAX_ANALOGPIN_IN_CONTROLLER; j++)
			{
				Serial.printf("\t\t\t\t\t Pin digital %d num: %d \n", i, srtl.controllerList[i].analogPins[j].pin);
			}
		}
	}
	Serial.println("================ ================= ===================");

	Serial.println("\n\n\n");

	monitorStack(srtl);
	printMemoryStats();
}

void loop()
{
	Serial.print("EPOCH : ");
	Serial.println(-syncTimer(0));
	vTaskDelay(pdMS_TO_TICKS(100 * 1000 )); //every 100 seconde
}

#endif