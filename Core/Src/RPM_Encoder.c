
#include "RPM_Encoder.h"
	double rpm;
	uint16_t cnt=0;
	uint16_t preCnt=0;
	int loop=0;
	bool flag_rot;
//	const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
double get_rpm(int motor){
//	const float timeout = (float)(1/osKernelGetSysTimerFreq())*160000;
////	reset_tick();
	switch (motor){
		case MOTOR_1:
			__HAL_TIM_SET_COUNTER(&htim1, (flag_rot_1)?65535:0);
			flag_rot=flag_rot_1;
			break;
		case MOTOR_2:
			__HAL_TIM_SET_COUNTER(&htim3, (flag_rot_2)?65535:0);
			flag_rot=flag_rot_2;
			break;
		case MOTOR_3:
			__HAL_TIM_SET_COUNTER(&htim4, (flag_rot_3)?65535:0);
			flag_rot=flag_rot_3;
			break;
	}
	ResetLoop(flag_rot);
//	uint32_t a=osKernelGetTickCount();
	uint32_t a=xTaskGetTickCount();
//	TickType_t a = pdTICKS_TO_MS( xTaskGetTickCount());
//	uint32_t a= osKernelSysTick();
//	UARTprintf("tick: %d \n",a);

	while(xTaskGetTickCount()- a <= 20){
		switch (motor){
		case MOTOR_1:
			cnt = __HAL_TIM_GET_COUNTER(&htim1);
			break;

		case MOTOR_2:
			cnt = __HAL_TIM_GET_COUNTER(&htim3);
			break;

		case MOTOR_3:
			cnt = __HAL_TIM_GET_COUNTER(&htim4);
			break;
		}
		CountLoop(cnt, flag_rot);
	}
//	UARTprintf("loop: %d, \t cnt: %d \r\n",loop,cnt);
	rpm= ((double)loop*65000+((flag_rot)?(double)(65535-cnt):(double)cnt))/47000*50*60;

	return rpm;
}
void CountLoop(uint16_t cnt, bool flag){
	if (!flag){
		if(cnt<preCnt){ //working as MOTOR1=RESET, MOTOR2=RESET, MOTOR3=RESET
			loop++;
		}
		preCnt=cnt;
	}else{
		if(cnt>preCnt){	//
			loop++;
		}
		preCnt=cnt;
	}
}
void ResetLoop(bool flag){
	loop=0;
	preCnt=(flag)?65535:0;
//	if(!flag){
//		preCnt=0;
//	}else{
//		preCnt=65535;
//	}
}
