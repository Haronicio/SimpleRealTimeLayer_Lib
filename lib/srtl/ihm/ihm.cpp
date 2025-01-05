#include "ihm.h"

#ifdef C_ONLY

Monitor sysMonitor;

uint8_t nController;
Controller controllerList[MAX_CONTROLLER];
#endif //C_ONLY