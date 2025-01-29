#ifndef SRTL_CORE_H
#define SRTL_CORE_H

/**
 * @file SRTL_Core.h
 * @brief Implémentation de la partie CORE de SRTL (Simple RealTime Layer)
 *
 * Cette bibliothèque gère les modules et les ressources partagées dans un système embarqué utilisant FreeRTOS.
 * Elle permet de créer et de gérer des tâches concurrentes avec une communication sécurisée via des ressources partagées.
 * L'objectif est de fournir une gestion optimisée des tâches et des ressources tout en restant léger et rapide.
 *
 * ## Fonctionnalités principales :
 * - Création de tâches (modules) avec gestion des ressources partagées.
 * - Synchronisation des tâches via des sémaphores et notifications.
 * - Notifications intelligentes pour la communication entre modules.
 * - Abstraction des tâches et ressources dans une structure simple pour l'utilisateur.
 *
 * @note Ce fichier est destiné à être utilisé avec le Framework Arduino et FreeRTOS.
 *
 * @version 1.0
 * @author Haron
 * @date 2024
 */

// Bibliothèques nécessaires
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "sync/sync.h"

//forward declaration of struct SRTL
struct SRTL;

// Configuration de base

#define MAX_MODULES 16          // Nombre maximum de modules
#define MAX_SHARED_RESOURCES 16 // Nombre maximum de ressources partagées
#define N_FREE_BITS 0           // Nombre de bits libres pour des utilisations personnalisées
#define MAX_JOIN_LIST 4         // Nombre maximum de file d'attente pour les barrières

#define NBITS_SRC MAX_MODULES          // Nombre de bits pour coder les modules dans une notification
#define NBITS_WHO MAX_SHARED_RESOURCES // Nombre de bits pour coder les ressources partagées dans une notification
#define NBITS_FREE N_FREE_BITS         // Nombre de bits libres pour des notifications personnalisées

#define MINIMAL_STACK_SIZE 2048 // Taille minimale de la pile pour les tâches

// Vérification de la somme des bits

#if (NBITS_SRC + NBITS_WHO + NBITS_FREE > 32)
#error "La somme des bits pour les modules, les ressources et les bits libres dépasse 32 bits"
#endif

// Macro pour manipulation des bits
#define GET_BYTE(value, index) ((uint8_t)(((value) >> ((index) * 8)) & 0xFF))
#define SET_BYTE(value, index, byte) \
    ((value) = ((value) & ~(0xFF << ((index) * 8))) | (((uint32_t)(byte) & 0xFF) << ((index) * 8)))

#define MAKE_UINT32(byte0, byte1, byte2, byte3) \
    (((uint32_t)(byte0) & 0xFF) | (((uint32_t)(byte1) & 0xFF) << 8) | (((uint32_t)(byte2) & 0xFF) << 16) | (((uint32_t)(byte3) & 0xFF) << 24))

#define MERGE_24_8(sharedRessourceBits, srcModuleIndex) \
    (((sharedRessourceBits) & 0xFFFFFF) | (((srcModuleIndex) & 0xFF) << 24))

// Notification système
#define NOTIFY_MSG(sharedResourceBits, srcModuleIndex) \
    (((sharedResourceBits) & ((1 << NBITS_WHO) - 1)) | (((srcModuleIndex) & ((1 << NBITS_SRC) - 1)) << NBITS_WHO))

#define EXTRACT_SHARED_RESOURCE(notificationValue) \
    ((notificationValue) & ((1 << NBITS_WHO) - 1))

#define EXTRACT_JOIN_LIST_QUEUE(notificationValue) \
    ((notificationValue) & ((1 << NBITS_WHO) - 1))

#define EXTRACT_SOURCE_MODULE(notificationValue) \
    (((notificationValue) >> NBITS_WHO) & ((1 << NBITS_SRC) - 1))

// MACRO Divers
#define ARRAY_INDEX_FROM_PTR(array, element_ptr) ((element_ptr) - (&array[0]))

char binaryBuffer[33];
#define TO_BIN(value) (itoa(value, binaryBuffer, 2))

#define INDEX_TO_BITSET(index) ((1 << index))

// MACRO INSIDE MODULE HANDLER

#define GET_SRTL_INSTANCE ((*((Module *)modulePtr)->parent))
#define GET_PARAMS_INSTANCE(type) (*(type *)(((Module *)modulePtr)->parameters))
#define GET_CURRENT_MODULE_INSTANCE (*((Module *)modulePtr))
#define GET_CURRENT_MODULE_INDEX (ARRAY_INDEX_FROM_PTR(GET_SRTL_INSTANCE.moduleList, (Module *)modulePtr))

// #ifdef __cplusplus
// extern "C" {
// #endif

// Module struct (task with some information) (compatibles C et C++)
typedef struct
{
    TaskHandle_t handle;          // Handle de la tâche
    uint32_t ressourceOfInterest; // Ressource d'intérêt (bitmask)
    uint32_t taskFrequency;       // Fréquence d'exécution de la tâche (< 1000 ms)
    uint32_t notificationValue;   // Valeur de notification
    TickType_t lastAwake;         // Pour la gestion de réveil précis (autoTimer voir sync.h)
    eventTimer *lastEvent;        // Guarder la ref du dernier évènements
    void *parameters;             // Paramètres de la tâche
#ifndef C_ONLY
    struct SRTL *parent; // Pointeur vers l'objet SRTL parent, attention forward declaration
#endif                   // !C_ONLY

} Module;

// #ifdef __cplusplus
// }
// #endif

// Définition de la structure d'une ressource partagée
typedef struct
{
    SemaphoreHandle_t *mutex; // Mutex pour la ressource
    void *data;               // Données de la ressource
} protected_data_t;

#ifdef C_ONLY
// Listes des modules et ressources
extern uint8_t nModule;                // Nombre de modules enregistrés
extern Module moduleList[MAX_MODULES]; // Liste des modules
// extern SemaphoreHandle_t xMutex_moduleList; // Mutex pour la gestion des modules
extern SemaphoreHandle_t xMutex_NotifyAll; // Mutex pour protéger les notifications (une tâche à la fois)

extern void *sharedRessources[MAX_SHARED_RESOURCES]; // Liste des ressources partagées
extern uint8_t nSharedResource;                      // Nombre de ressources partagées
#endif

// Fonctions principales

/**
 * @brief Crée un module (une tâche) dans le système. STATIC FOR C VERSION
 *
 * @param taskFunction Fonction de la tâche à exécuter.
 * @param name Nom du module.
 * @param stackSize Taille de la pile pour la tâche.
 * @param moduleparameters Paramètres pour la tâche.
 * @param priority Priorité de la tâche.
 * @param ressourceOfInterest Ressource d'intérêt pour la tâche.
 * @param taskFrequency Fréquence d'exécution de la tâche.
 * @param module Pointeur vers l'objet Module.
 */
static inline void createModule(TaskFunction_t taskFunction, const char *name, uint32_t stackSize, void *moduleparameters, uint8_t priority,
                                uint32_t ressourceOfInterest, uint32_t taskFrequency, Module *module)
{
    module->ressourceOfInterest = ressourceOfInterest;
    module->taskFrequency = taskFrequency;
    module->parameters = moduleparameters;
    module->lastAwake = xTaskGetTickCount();
    module->lastEvent = NULL;

    // moduleparameters in module, module adress in taskhandler parameters
    if (xTaskCreate(taskFunction, name, stackSize, module, priority, &module->handle) != pdPASS) // TODO : prefer Static Task alloc ?
    // if (xTaskCreate(taskFunction, name, stackSize, &module, priority, &module->handle) != pdPASS) // TODO :  &module work in C doesn't work in c++ prefer Static Task alloc ?
    {
        // Serial.printf("Failed to create task %s\n", name);
        while (1)
            ;
    }
}

#ifdef C_ONLY
/** ***@brief STATIC FOR C VERSION
    *

**@param taskFunction Fonction de la tâche à exécuter.
*@param name Nom du module.
    *@param stackSize Taille de la pile pour la tâche.
                *@param priority Priorité de la tâche.
                    *@param ressourceOfInterest Ressource d'intérêt pour la tâche.
            *@param taskFrequency Fréquence d'exécution de la tâche.
                            *@param moduleparameters Paramètres pour la tâche.
                                *
                                    *@ return Index du module dans la liste.*
    **/

static inline uint8_t registerModule(TaskFunction_t taskFunction, const char *name, uint32_t stackSize, uint8_t priority,
                                     uint32_t ressourceOfInterest, uint32_t taskFrequency,
                                     void *moduleparameters)
{
    if (nModule >= MAX_MODULES)
    {
        // Serial.println("Error: No space left in moduleList array");
        while (1)
            ;
    }

    createModule(taskFunction, name, stackSize, moduleparameters, priority, ressourceOfInterest, taskFrequency, &moduleList[nModule]);
    return nModule++;
}

#endif

#define MAX_SEMAPHORE_MUTEX 16
uint8_t nSemaphoreMutexBuffer = 0;
StaticSemaphore_t semaphoreMutexBufferList[MAX_SEMAPHORE_MUTEX] = {NULL};

/**
 * @brief Crée une ressource partagée dans le système.
 * @param resource Pointeur vers la ressource partagée.
 * @param mutex Mutex pour la ressource.
 * @param valueLocation Adresse de la ressource.
 */
inline void createSharedResource(void **resource, SemaphoreHandle_t *mutex, void *valueLocation)
{
    if (resource == NULL || mutex == NULL)
    {
        Serial.println("Invalid arguments to createSharedResource");
        while (1)
            ;
    }
    if (nSemaphoreMutexBuffer >= MAX_SEMAPHORE_MUTEX)
    {
        Serial.println("Semaphore buffer exhausted");
        while (1)
            ;
    }

    *resource = valueLocation;
    // version statique
    *mutex = xSemaphoreCreateMutexStatic(&semaphoreMutexBufferList[nSemaphoreMutexBuffer++]);
    // *mutex = xSemaphoreCreateMutex(); // version dynamique

    if (*mutex == NULL)
    {
        Serial.println("Failed to create mutex for resource");
        while (1)
            ;
    }

}
#ifdef C_ONLY
/**
 * @brief Enregistre une ressource partagée dans la liste des ressources.
 *
 * @param data Données de la ressource protégée (mutex et donnée).
 *
 * @return Index de la ressource partagée.
 */
inline uint8_t registerSharedRessource(protected_data_t data)
{
    if (nSharedResource >= MAX_SHARED_RESOURCES)
    {
        // Serial.println("Error: No space left in sharedRessources array");
        while (1)
            ;
    }
    createSharedResource(&sharedRessources[nSharedResource], data.mutex, data.data);
    return nSharedResource++;
}
/**
 * @brief Notifie un module spécifique d'un changement de ressource.
 *
 * @param destModuleIndex Index du module destinataire.
 * @param srcModuleIndex Index du module source.
 * @param sharedRessourceBits Bits de la ressource partagée.
 * @param action Action de notification (par défaut, SET).
 *
 * @return Succès de la notification.
 */
inline u_int8_t notifyModule(uint16_t destModuleIndex, uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action)
{
    return xTaskNotify(moduleList[destModuleIndex].handle, NOTIFY_MSG(sharedRessourceBits, srcModuleIndex), action);
}
/**
 * @brief Notifie tous les modules intéressés par une ressource partagée. Note : ne notifie pas les Monitor
 *
 * @param srcModuleIndex Index du module source.
 * @param sharedRessourceBits Bits de la ressource partagée.
 * @param action Action de notification (par défaut, SET).
 *
 * @return Succès de la notification.
 */
inline u_int8_t notifyAll(uint16_t srcModuleIndex, uint32_t sharedRessourceBits, eNotifyAction action)
{
    u_int8_t success = pdFALSE;
    xSemaphoreTake(xMutex_NotifyAll, portMAX_DELAY); // TODO: if ? in ISR ? Atomic ?
    for (Module *m = moduleList; (m->handle != NULL) && (m < &moduleList[MAX_MODULES]); m++)
    // for (Module *m = moduleList; m->handle != NULL; m++)
    {
        if (m->ressourceOfInterest & sharedRessourceBits)
        {
            success = xTaskNotify(m->handle, NOTIFY_MSG(sharedRessourceBits, srcModuleIndex), action);
        }
    }
    xSemaphoreGive(xMutex_NotifyAll);
    return success;
}
/**
 * @brief initialise le système srtl
 *
 * @return Succès de l'initialisation.
 */
inline SemaphoreHandle_t SRTLinit()
{
    return xMutex_NotifyAll = xSemaphoreCreateMutex();
}

// JOIN LIST : pseudo barrier for intermodule sync
// TODO : add more module (use uint32_t but due to splitting 32 bit to Module and SharedResource ... see NOTIFY_MSG uses)

// extern uint16_t joinList[MAX_JOIN_LIST];
extern SemaphoreHandle_t xMutex_JoinList = xSemaphoreCreateMutex();
extern uint32_t joinList[MAX_JOIN_LIST] = {0};

// Barriere join from a handler (No FRTOS thread pause) Awaiting release
// Bloc executing handler into this function, set the module idx into the specified joinList of awaiting module
// Return Released Modules
inline uint32_t join(uint8_t joinListIndex, uint8_t modulePtrIndex)
{
    xSemaphoreTake(xMutex_JoinList, portMAX_DELAY);

    joinList[joinListIndex] |= (1 << modulePtrIndex);

    xSemaphoreGive(xMutex_JoinList);

    uint32_t ret;
    xTaskNotifyWait(0x0, 0x0, &ret, portMAX_DELAY);

    return ret;
}

// Release Module blocked at the Barrier, use with join
// Return Released Modules
inline uint32_t release(uint8_t joinListIndex)
{

    uint32_t releasedModule = joinList[joinListIndex];

    uint8_t i = 0;

    xSemaphoreTake(xMutex_JoinList, portMAX_DELAY);
    while (i < MAX_MODULES)
    {
        if (releasedModule << i)
            xTaskNotify(moduleList[i].handle, releasedModule, eNoAction);
        i++;
    }
    joinList[joinListIndex] = 0;
    xSemaphoreGive(xMutex_JoinList);

    return releasedModule;
}

#endif

#endif // SRTL_CORE_H
