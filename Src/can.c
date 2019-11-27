#include "stm32f4xx_hal.h"

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_FilterTypeDef fcan;

static void User_CAN1_Config(void){
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void User_CAN2_Config(void){
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void User_CAN_Filter_Config(){
    fcan.FilterActivation = ENABLE;
    fcan.FilterMode = CAN_FILTERMODE_IDMASK;
    fcan.FilterScale = CAN_FILTERSCALE_32BIT;
    //ÆÁ±ÎÂë0x00000110£» ÑéÖ¤Âë0x00000200
    fcan.FilterMaskIdHigh = 0x0000;
    fcan.FilterMaskIdLow = 0x0110;
    fcan.FilterIdHigh = 0x0000;
    fcan.FilterIdLow = 0x0200;
    fcan.FilterBank = 0;
    fcan.FilterFIFOAssignment = CAN_RX_FIFO0;
    
    HAL_CAN_ConfigFilter(&hcan1, &fcan);
    HAL_CAN_ConfigFilter(&hcan2, &fcan);
}

static void User_CAN_Init(){
    User_CAN1_Config();
    User_CAN2_Config();
    User_CAN_Filter_Config();
}

uint32_t CAN_RxMessage(){
    ;
}

void CAN_TxMessage(){
    ;
}