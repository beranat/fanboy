#pragma once

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

void fatalError(esp_err_t error, const char *info, ...);	// crash-error, message + deep sleep
void ledEnable(bool on);

#ifdef __cplusplus
}
#endif
