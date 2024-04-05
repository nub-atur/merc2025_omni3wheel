#ifndef PID_H_
#define PID_H_


typedef enum
	{
	Anti_windup_disabled=0,
	Anti_windup_enabled
	}Anti_windup_t;


typedef struct
	{
	double Kp;
	double Ki;
	double Kd;
	double Ts;
	double target_val_1;
	double target_val_2;
	double target_val_3;
//	double err_last_1;
//	double err_last_2;
//	double err_last_3;
//	double err_next_1;
//	double err_next_2;
//	double err_next_3;
	}PID_Param_t;


extern void PID_init(PID_Param_t *par);
extern double PID_Calculation(int motor, double actual_val);
extern void PID(void);
#endif /* PID_H_ */
