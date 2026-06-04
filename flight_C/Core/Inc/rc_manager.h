#ifndef RC_MANAGER_H
#define RC_MANAGER_H

#include "main.h"
#include "receiver.h"
#include <stdbool.h>

void    RC_Update(void);
void    RC_PrintTelemetry(void);
bool    RC_GetLatestPacket(dataPacket *out);
bool    RC_IsKillActive(void);

#endif