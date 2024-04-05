/*
 * motor.h
 *
 *  Created on: Mar 1, 2024
 *      Author: quocv
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "main.h"
#include "math.h"
#include "pid.h"
#include "delay.h"
#include <stdio.h>
#include"stdbool.h"

#define MOTOR_1	1
#define MOTOR_2	2
#define MOTOR_3	3
#define CLOCK_WISE 1
#define PI 3.141592654
#define diameter 120/1000

extern void set_duty_cycle(int motor, double out);
extern void Robot_Move(double Vd, double Theta, double Vtheta);
double rpm_to_duty(double rpm);
void Rotation(int motor, int rotation);
double v2rpm (double vantoc);


extern double V1, V2, V3;
extern double duty_V1, duty_V2, duty_V3;

extern bool flag_rot_1,flag_rot_2,flag_rot_3;
extern double prev_duty_1, prev_duty_2, prev_duty_3;
extern double duty;
#endif /* INC_MOTOR_H_ */
