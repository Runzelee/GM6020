#include <stdint.h>
#include <stdbool.h>

extern float velocity;

/* Debug / PID 输出变量 */
extern float error;
extern float derivative;
extern float pid_output;
extern int16_t send_vol;

/* 函数原型 */

void Single_PID_Handler(uint16_t raw_angle, float _velocity);
void Single_PID_ToVelocity(float setpoint_velocity);
