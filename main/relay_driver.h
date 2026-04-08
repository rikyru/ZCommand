#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void relay_driver_init(bool state);
void relay_driver_set_state(bool state);

#define RELAY_DEFAULT_ON  1
#define RELAY_DEFAULT_OFF 0

#ifdef __cplusplus
}
#endif
