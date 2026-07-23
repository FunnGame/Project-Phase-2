#ifndef CONTROL_APP_H_
#define CONTROL_APP_H_

#include <stdint.h>
#include "car_types.h"

void ControlApp_Init(void);

void ControlApp_Update(void);

/* 
Ðây là hàm mà ban code RF hay CAN sau khi lay xong speed hoac Direction se ném hai gia tri ay vào ham nay 
de update. Ban nho dung ham nay nhe
*/
void ControlApp_Set(Car_Direction_t new_dir, uint8_t speed_percent);

CarData_t ControlApp_GetData(void);

#endif