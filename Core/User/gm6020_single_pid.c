#include <stdint.h>
#include <stdbool.h>
#include "gm6020_single_pid.h"
#include "gm6020_can.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

static float velocity;

/* ---------------- PID parameters & state ---------------- */
static const float Kp = 15.0f;     // 比例
static const float Ki = 5.0f;    // 积分
static const float Kd = 0.0f;     // 微分（最简单可以先设为0）
static const float SAMPLE_DT = 0.05f; // 主循环延时 50 ms

static float pid_integral = 0.0f;
static float pid_prev_error = 0.0f;
static const int16_t MAX_VOLTAGE = 20000; // 输出电压上限，可按需调整

/* ---------------- For Debug ---------------- */
static float error, derivative, pid_output;
static int16_t send_vol;

void Single_PID_Handler(uint16_t raw_angle, float _velocity){
    velocity = _velocity;
}

/**
 * @brief 单闭环速度 PID 控制函数
 * @param setpoint 目标速度（单位：RPM）
 */
void Single_PID_ToVelocity(float setpoint)
{
    // 在循环开始或结束保持与 SAMPLE_DT 对齐
    HAL_Delay((uint32_t)(SAMPLE_DT * 1000)); // 50 ms

    // PID 控制
    error = setpoint - velocity;
    pid_integral += error * SAMPLE_DT;
    derivative = (error - pid_prev_error) / SAMPLE_DT;

    pid_output = Kp * error + Ki * pid_integral + Kd * derivative;

    pid_prev_error = error;
    uint8_t TxData[50];
    sprintf((char *)TxData, "%.2f, %.2f, %.2f\r\n", velocity, error, pid_output);
    HAL_UART_Transmit(&huart3, TxData, strlen((char *)TxData), 1000);

    // 限幅并转为 int16_t（驱动期望的电压范围）
    if (pid_output > MAX_VOLTAGE) pid_output = MAX_VOLTAGE;
    if (pid_output < -MAX_VOLTAGE) pid_output = -MAX_VOLTAGE;

    send_vol = (int16_t)roundf(pid_output);

    Set_GM6020_Voltage(send_vol);
    //Set_GM6020_Voltage(8000);
}
