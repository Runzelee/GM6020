#include "gm6020_can.h"
#include "can.h"

const uint32_t CAN_CMD_ID = 0x1ff;

static GM6020_CAN_AngleVelocityCallback_t GM6020_CAN_AngleVelocityCallback = NULL;

/**
 * @brief 注册回调函数以接收 GM6020 电机的角度和速度数据
 * @param callback 回调函数指针，Single_PID_Handler 或 Double_PID_Handler 二选一
 */
void GM6020_CAN_RegisterAngleVelocityCallback(GM6020_CAN_AngleVelocityCallback_t callback)
{
    GM6020_CAN_AngleVelocityCallback = callback;
}

/* ------------------------------ 初始化（配置过滤器）4------------------------------ */
void Enable_CAN1(void)
{
    CAN_FilterTypeDef CAN_Filter;
    CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter.FilterBank = 0;
    CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter.SlaveStartFilterBank = 0;
    CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
    CAN_Filter.FilterIdHigh = 0x0000;
    CAN_Filter.FilterIdLow = 0x0000;
    CAN_Filter.FilterMaskIdHigh= 0x0000;
    CAN_Filter.FilterMaskIdLow = 0x0000;
    if (HAL_CAN_ConfigFilter(&hcan1,&CAN_Filter)!= HAL_OK){
        //HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);
        Error_Handler();
    }
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // CAN1 -> FIFO0
}

/* ------------------------------ 发送函数 ------------------------------ */
void Set_GM6020_Voltage(int16_t vol)
{
    uint8_t TxData[8] = {0}; // 清零
    TxData[2] = (uint8_t)(vol>>8);
    TxData[3] = (uint8_t)vol;
    CAN_TxHeaderTypeDef TxHeader = {
            .DLC = 8,
            .IDE = CAN_ID_STD,    // 标准帧
            .RTR = CAN_RTR_DATA,  // 数据帧
            .StdId = CAN_CMD_ID
    };
    uint32_t TxBox = CAN_TX_MAILBOX0;
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxBox) != HAL_OK){
        //HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//错误处理
    }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  if (hcan == &hcan1)
  {
      CAN_RxHeaderTypeDef RxHeader;
      uint8_t RxData[8];
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)  // 获得接收到的数据头和数据
      {
          // 角度为无符号 0..8191（13-bit），高字节在 RxData[0], 低字节在 RxData[1]
          uint16_t raw_angle = (uint16_t)(((uint16_t)RxData[0] << 8) | (uint16_t)RxData[1]);
          raw_angle &= 0x1FFF; // 保证 13-bit（0..8191）

          // 速度为有符号 int16，高字节放在 RxData[2], 低字节在 RxData[3]
          int16_t raw_vel = (int16_t)(((uint16_t)RxData[2] << 8) | (uint16_t)RxData[3]);
          float velocity = (float)raw_vel;

          
        if (GM6020_CAN_AngleVelocityCallback != NULL)
        {
            GM6020_CAN_AngleVelocityCallback(raw_angle, velocity);
        }
          
      }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 再次使能FIFO0接收中断
}
