#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>



#ifdef __cplusplus
extern "C" {
#endif


void ControlApp_Init(void);


void ControlApp_Drive(int8_t throttle, int8_t steering);


void ControlApp_Stop(void);

#ifdef __cplusplus
}
#endif

#endif