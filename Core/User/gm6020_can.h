#include <stdint.h>

void Enable_CAN1(void);
void Set_GM6020_Voltage(int16_t vol);

typedef void (*GM6020_CAN_AngleVelocityCallback_t)(uint16_t raw_angle, float velocity);

void GM6020_CAN_RegisterAngleVelocityCallback(GM6020_CAN_AngleVelocityCallback_t callback);
