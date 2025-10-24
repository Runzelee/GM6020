#include <stdint.h>
#include <stdbool.h>

extern float velocity;
extern float angle;                /* 实时角度（多圈展开 ticks） */

/* Debug / PID 输出变量 */
extern float error;
extern float derivative;
extern float pid_output;
extern int16_t send_vol;

/* 函数原型 */
void Double_PID_Handler(uint16_t raw_angle, float velocity);
void Double_PID_ToAngle(float setpoint_angle);
