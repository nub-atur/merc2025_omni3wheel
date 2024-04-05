
#ifndef RPM_ENCODER_H_
#define RPM_ENCODER_H_

#include "main.h"
#include "delay.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "stdbool.h"

//extern float rpm;
//extern uint16_t cnt;
//extern uint16_t preCnt;
//extern int loop;

extern double get_rpm(int motor);
extern void CountLoop(uint16_t cnt, bool flag);
extern void ResetLoop(bool flag);


#endif /* RPM_ENCODER_H_ */
