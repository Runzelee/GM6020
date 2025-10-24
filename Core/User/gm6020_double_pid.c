#include <stdint.h>
#include <stdbool.h>
#include "gm6020_double_pid.h"
#include "gm6020_can.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

static float velocity;
static float angle;                // 实时角度（现在为多圈展开的 ticks，单位仍是 0..8191 的基础刻度）

// 新增：多圈展开相关状态
static int32_t angle_wrap_count = 0;    // 环绕计数（每增减一次代表 +/-8192 ticks）
static uint16_t prev_raw_angle = 0;     // 上一次收到的 raw 13-bit 角度
static bool prev_raw_valid = false;     // 是否已有有效的 prev_raw_angle

/* Deadband / hysteresis for angle loop */
static const float ANG_DEAD_ENTER = 500.0f;  // 进入死区阈值 (ticks)
static const float ANG_DEAD_EXIT  = 600.0f;  // 退出死区阈值 (ticks)
static bool hold_position = false;         // 在死区内则保持不输出，直到误差超过退出阈值

/* ---------------- PID parameters & state ---------------- */
static const float Kp = 15.0f;     // 比例 (速度环)
static const float Ki = 5.0f;    // 积分 (速度环)
static const float Kd = 0.0f;     // 微分（速度环）
static const float SAMPLE_DT = 0.05f; // 主循环延时 50 ms

static float pid_integral = 0.0f;
static float pid_prev_error = 0.0f;
static const int16_t MAX_VOLTAGE = 20000; // 输出电压上限，可按需调整

/* --------- 角度外环 PID 参数与状态（产生速度设定） --------- */
static const float Kp_ang = 15.0f;    // 比例 (角度环)
static const float Ki_ang = 5.0f;    // 积分 (角度环)
static const float Kd_ang = 0.0f;    // 微分 (角度环)

static float ang_integral = 0.0f;
static float ang_prev_error = 0.0f;
static const float MAX_TARGET_VEL = 200.0f; // 角度环输出速度上限（rpm），按需调整

/* ---------------- For Debug ---------------- */
static float error, derivative, pid_output;
static int16_t send_vol;

void Double_PID_Handler(uint16_t raw_angle, float velocity){
    // 如果之前没有有效值，初始化 prev_raw_angle
    if (!prev_raw_valid) {
        prev_raw_angle = raw_angle;
        prev_raw_valid = true;
        angle_wrap_count = 0;
    } else {
        // 计算 raw 的差值以检测环绕（unwrap）
        int32_t diff = (int32_t)raw_angle - (int32_t)prev_raw_angle;
        // 若差值超过半圈（>4096），说明发生了回绕
        if (diff > 4096) {
            // 例如 prev=8000 new=50 -> diff 正很大，实际应为向负方向跨过 8192
            angle_wrap_count -= 1;
        } else if (diff < -4096) {
            // 例如 prev=50 new=8000 -> diff 大负，实际应为向正方向跨过 8192
            angle_wrap_count += 1;
        }
        prev_raw_angle = raw_angle;
    }

    // 多圈展开角度 = raw + wrap_count * 8192
    angle = (float)raw_angle + (float)angle_wrap_count * 8192.0f;
}

/**
 * @brief 双环 PID 控制到目标角度，写在while(1)里
 * @param setpoint_angle 目标角度，单位为 ticks（可为多圈展开值，例如：3000 + 8192*turns）
 */
void Double_PID_ToAngle(float setpoint_angle)
{
  // 在循环开始或结束保持与 SAMPLE_DT 对齐
  HAL_Delay((uint32_t)(SAMPLE_DT * 1000)); // 50 ms

  /* -------- 外环：角度 PID（多圈绝对角度），产生速度设定（rpm） -------- */
  // 这里不要再做 fmod 环绕；angle 已经是展开的多圈绝对值，setpoint_angle 也应是相同单位
  float sp = setpoint_angle; // 如果想目标为多圈，可设置为：angle + 3*8192.0f + 3000.0f
  float ang_error = sp - angle;
  // 不再做单圈最短路径的调整

  /* 死区 + 滞回逻辑：误差小于 ANG_DEAD_ENTER 进入保持（停止输出），
      保持状态下误差超过 ANG_DEAD_EXIT 才退出保持 */
  if (hold_position) {
      if (fabsf(ang_error) > ANG_DEAD_EXIT) {
          hold_position = false; // 恢复控制
      }
  } else {
      if (fabsf(ang_error) < ANG_DEAD_ENTER) {
          hold_position = true; // 进入保持
      }
  }

  if (hold_position) {
      // 清零外/内环积分以防止积分风up，并保持输出为 0
      ang_integral = 0.0f;
      ang_prev_error = ang_error;
      pid_integral = 0.0f;
      pid_prev_error = 0.0f;

      float vel_setpoint = 0.0f;
      /* 直接发送 0 输出（位置保持）——后面分支会处理发送 0 电压 */
      pid_output = 0.0f;
  } else {
      ang_integral += ang_error * SAMPLE_DT;
      float ang_derivative = (ang_error - ang_prev_error) / SAMPLE_DT;
      float vel_setpoint = Kp_ang * ang_error + Ki_ang * ang_integral + Kd_ang * ang_derivative;
      ang_prev_error = ang_error;

      // 限幅外环输出到合理的速度范围
      if (vel_setpoint > MAX_TARGET_VEL) vel_setpoint = MAX_TARGET_VEL;
      if (vel_setpoint < -MAX_TARGET_VEL) vel_setpoint = -MAX_TARGET_VEL;

      /* -------- 内环：速度 PID（原有逻辑），使用 vel_setpoint -------- */
      error = vel_setpoint - velocity;
      pid_integral += error * SAMPLE_DT;
      derivative = (error - pid_prev_error) / SAMPLE_DT;

      pid_output = Kp * error + Ki * pid_integral + Kd * derivative;

      pid_prev_error = error;
  }

  /* 无论是否保持，最终输出限幅并发送；保持时 pid_output 已为 0 */
  uint8_t TxData[80];
  /* 发送额外的单圈绝对角度（0..8191） */
  float abs_angle = fmodf(angle, 8192.0f);
  if (abs_angle < 0.0f) abs_angle += 8192.0f;
  sprintf((char *)TxData, "%.2f, %.2f, %.2f, %.2f, %.2f, %d\r\n",
      angle, abs_angle, ang_error, velocity, pid_output, hold_position ? 1 : 0);
  HAL_UART_Transmit(&huart3, TxData, strlen((char *)TxData), 1000);

  // 限幅并转为 int16_t（驱动期望的电压范围）
  if (pid_output > MAX_VOLTAGE) pid_output = MAX_VOLTAGE;
  if (pid_output < -MAX_VOLTAGE) pid_output = -MAX_VOLTAGE;

  send_vol = (int16_t)roundf(pid_output);

  Set_GM6020_Voltage(send_vol);
  //Set_GM6020_Voltage(8000);
}
