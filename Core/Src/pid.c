#include "pid.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
double Kp,Ki,Kd
,target_val_1
,target_val_2
,target_val_3
,target_val
,prev_actual_val;
double err
,err_last_1
,err_last_2
,err_last_3
,err_last
,err_next_1
,err_next_2
,err_next_3
,err_next
,actual_val
,pre_actual_val_1=0
,pre_actual_val_2=0
,pre_actual_val_3=0;
void PID_init(PID_Param_t *par){
	Kp=par->Kp;
	Ki=par->Ki;
	Kd=par->Kd;
	target_val_1=par->target_val_1;
	target_val_2=par->target_val_2;
	target_val_3=par->target_val_3;
}

double update_motor_values(double target_val, double actual_val, double prev_actual_val,double err_last, double err_next) {

	if(target_val==0){
		actual_val=-10;
		return actual_val;
	}
	err=target_val-actual_val;
	actual_val = prev_actual_val + pid.Kp*(err - err_next)
				  + pid.Ki*err
				  + pid.Kd*(err - 2 * err_next + err_last);
	err_last = err_next;
	err_next = err;
	actual_val=(actual_val>10)?10:actual_val;
	actual_val=(actual_val<-10)?-10:actual_val;


	return actual_val;
}
void save_Err(int motor){
	switch(motor){
	case MOTOR_1:
		pre_actual_val_1=actual_val;
		err_last_1=err_last;
		err_next_1=err_next;
		break;
	case MOTOR_2:
		pre_actual_val_2=actual_val;
		err_last_2=err_last;
		err_next_2=err_next;
		break;
	case MOTOR_3:
		pre_actual_val_3=actual_val;
		err_last_3=err_last;
		err_next_3=err_next;
		break;
	}
}
double PID_Calculation(int motor, double actual_val){

	switch(motor) {
	    case MOTOR_1:
			actual_val=update_motor_values(target_val_1, actual_val, pre_actual_val_1, err_last_1, err_next_1);
			save_Err(MOTOR_1);

	        break;
	    case MOTOR_2:
			actual_val=update_motor_values(target_val_2, actual_val, pre_actual_val_2, err_last_2, err_next_2);
			save_Err(MOTOR_2);

	        break;
	    case MOTOR_3:
			actual_val=update_motor_values(target_val_3, actual_val, pre_actual_val_3, err_last_3, err_next_3);
			save_Err(MOTOR_3);

	        break;
	}
	return actual_val;
}

void PID(void){

	rpm_1=get_rpm(MOTOR_1);

	rpm_2=get_rpm(MOTOR_2);
	rpm_3=get_rpm(MOTOR_3);

	out_1=PID_Calculation(MOTOR_1, rpm_1);
	out_2=PID_Calculation(MOTOR_2, rpm_2);
	out_3=PID_Calculation(MOTOR_3, rpm_3);

	set_duty_cycle(MOTOR_1, out_1);
	set_duty_cycle(MOTOR_2, out_2);
	set_duty_cycle(MOTOR_3, out_3);

//	UARTprintf("SET_POINT >>>>> V1= %.2f rpm \t V2= %.2f rpm \t V3= %.2f rpm \r\n \t out_1: %f \t out_2: %f \t out_3: %f \r\n \t rpm_1: %.2f \t rpm_2: %.2f \t rpm_3: %.2f \r\n\n", V1,V2,V3,out_1,out_2,out_3, rpm_1, rpm_2, rpm_3);
//	UARTprintf("V1: %d \t V2: %d \t V3: %d \r\n",(uint32_t)V1,(uint32_t)V2,(uint32_t)V3);
//	UARTprintf("out_1: %d \t out_2: %d \t out_3: %d \r \n",(uint32_t)out_1,(uint32_t)out_2,(uint32_t)out_3);
//	UARTprintf("rpm_1: %d \t rpm_2: %d \t rpm_3: %d \r \n \r\n", (uint32_t)rpm_1, (uint32_t)rpm_2, (uint32_t)rpm_3);
}
