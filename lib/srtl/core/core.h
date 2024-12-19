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

#include "../srtl.h"

// Configuration de base

#define MAX_MODULES 16          // Nombre maximum de modules
#define MAX_SHARED_RESOURCES 16 // Nombre maximum de ressources partagées
#define N_FREE_BITS 0           // Nombre de bits libres pour des utilisations personnalisées

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

#define EXTRACT_SOURCE_MODULE(notificationValue) \
    (((notificationValue) >> NBITS_WHO) & ((1 << NBITS_SRC) - 1))

// MACRO Divers
#define ARRAY_INDEX_FROM_PTR(array, element_ptr) ((element_ptr) - (&array[0]))

// #ifdef __cplusplus
// extern "C" {
// #endif

// Déclarations C-only (compatibles C et C++)
typedef struct {
    TaskHandle_t handle;          // Handle de la tâche
    uint32_t ressourceOfInterest; // Ressource d'intérêt (bitmask)
    uint32_t taskFrequency;       // Fréquence d'exécution de la tâche
    uint32_t notificationValue;   // Valeur de notification
    void* parameters;             // Paramètres de la tâche
#ifndef C_ONLY
    struct SRTL* parent;          // Pointeur vers l'objet SRTL parent
#endif // !C_ONLY
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

/**
 * @brief Crée une ressource partagée dans le système.
 *
 * @param resource Pointeur vers la ressource partagée.
 * @param mutex Mutex pour la ressource.
 * @param valueLocation Adresse de la ressource.
 */
inline void createSharedResource(void **resource, SemaphoreHandle_t *mutex, void *valueLocation)
{
    *resource = valueLocation;
    *mutex = xSemaphoreCreateMutex();

    if (mutex == NULL)
    {
        // Serial.println("Failed to create mutex for resource");
        while (1)
            ; // Blocage en cas d'échec
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
 * @brief Notifie tous les modules intéressés par une ressource partagée.
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
    for (Module *m = moduleList; m->handle != NULL; m++)
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
#endif

#endif // SRTL_CORE_H
