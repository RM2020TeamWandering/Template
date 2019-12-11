/**
  ******************************************************************************
  *  用户自定义 CAN API 实现
  ******************************************************************************
  */

#include "main.h"

/***私有变量***/
CAN_HandleTypeDef       hcan1;
CAN_HandleTypeDef       hcan2;
CAN_FilterTypeDef       fcan;
CAN_TxHeaderTypeDef*    TxHeader;
CAN_RxHeaderTypeDef*    RxHeader;
uint8_t                 TxData[8];
uint8_t                 RxData[8];
uint32_t                TxMailbox;

/**
  ******************************************************************************
  *  函数功能实现
  ******************************************************************************
  */
static void MX_CAN1_Config(void){
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_CAN2_Config(void){
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  ******************************************************************************
  *  静态函数 My_CAN_Filter_Config
  *  功能：配置 CAN 过滤器
  ******************************************************************************
  */
static void My_CAN_Filter_Config(){
    fcan.FilterActivation = ENABLE;
    fcan.FilterMode = CAN_FILTERMODE_IDMASK;
    fcan.FilterScale = CAN_FILTERSCALE_32BIT;
    //屏蔽码0x00000110； 验证码0x00000200
    fcan.FilterMaskIdHigh = 0x0000;
    fcan.FilterMaskIdLow = 0x0110;
    fcan.FilterIdHigh = 0x0000;
    fcan.FilterIdLow = 0x0200;
    fcan.FilterBank = 14;
    fcan.FilterFIFOAssignment = CAN_RX_FIFO0;
    
    HAL_CAN_ConfigFilter(&hcan1, &fcan);
    HAL_CAN_ConfigFilter(&hcan2, &fcan);
}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

static void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PD1     ------> CAN1_TX
    PD0     ------> CAN1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration    
    PB13     ------> CAN2_TX
    PB12     ------> CAN2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }

}

static void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PD1     ------> CAN1_TX
    PD0     ------> CAN1_RX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN2 GPIO Configuration    
    PB13     ------> CAN2_TX
    PB12     ------> CAN2_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }

}
/**
  ******************************************************************************
  *  API接口函数 My_CAN_Init
  *  功能：初始化 CAB
  ******************************************************************************
  */
void My_CAN_Init(){
    MX_CAN1_Config();
    MX_CAN2_Config();
    My_CAN_Filter_Config();
    HAL_CAN_MspInit(&hcan1);
    HAL_CAN_MspInit(&hcan2);
    HAL_CAN_MspDeInit(&hcan1);
    HAL_CAN_MspDeInit(&hcan2);
    if (HAL_CAN_Start(&hcan1) != HAL_OK || HAL_CAN_Start(&hcan2) != HAL_OK)
    {
      /* Start Error */
      Error_Handler();
    }
}


/**
  ******************************************************************************
  *  API接口函数 Set_Motor_Current
  *  功能：设置电机的电流值
  ******************************************************************************
  */
void Set_Motor_Current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4){
    TxHeader->StdId = 0x200;
    TxHeader->IDE = CAN_ID_STD;
    TxHeader->RTR = CAN_RTR_DATA;
    TxHeader->DLC = 8;
    TxHeader->TransmitGlobalTime = ENABLE;
    
    //将电流值数据装入数组
    //电流值用 2 个字节的有符号整型数表示
    //一个完整的电流值需要使用 2 个数组元素表示
    TxData[0] = iq1 >> 8;
    TxData[1] = iq1;
    TxData[2] = iq2 >> 8;
    TxData[3] = iq2;
    TxData[4] = iq3 >> 8;
    TxData[5] = iq3;
    TxData[6] = iq4 >> 8;
    TxData[7] = iq4;
  
    //请求发送报文
    if(HAL_CAN_AddTxMessage(&hcan1, TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
    
    //等待发送完成
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
}


/**
  ******************************************************************************
  *  API接口函数 Get_Motor_Info
  *  功能：获取电机的反馈信息，包括：
  *      转子机械角度大小
  *      转子转速大小
  *      实际转矩电流值
  *      电机温度
  ******************************************************************************
  */
void Get_Motor_Info(Motor_Info* info){
    //开始接收报文
    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, RxHeader, RxData) != HAL_OK){
        Error_Handler();
    }
    
    //将报文装入结构体
    if( (RxHeader->IDE == CAN_ID_STD)    && 
        (RxHeader->RTR == CAN_RTR_DATA)  && 
        (RxHeader->DLC == 8) )
    {
        switch(RxHeader->StdId){
            case M3508_Motor1_ID:
                info->angle[0]       = RxData[0] << 8 | RxData[1];
                info->current[0]     = RxData[2] << 8 | RxData[3];
                info->speed[0]       = RxData[4] << 8 | RxData[5];
                info->temperature[0] = RxData[6];
                break;
            case M3508_Motor2_ID:
                info->angle[1]       = RxData[0] << 8 | RxData[1];
                info->current[1]     = RxData[2] << 8 | RxData[3];
                info->speed[1]       = RxData[4] << 8 | RxData[5];
                info->temperature[1] = RxData[6];
                break;
            case M3508_Motor3_ID:
                info->angle[2]       = RxData[0] << 8 | RxData[1];
                info->current[2]     = RxData[2] << 8 | RxData[3];
                info->speed[2]       = RxData[4] << 8 | RxData[5];
                info->temperature[2] = RxData[6];
                break;
            case M3508_Motor4_ID:
                info->angle[3]       = RxData[0] << 8 | RxData[1];
                info->current[3]     = RxData[2] << 8 | RxData[3];
                info->speed[3]       = RxData[4] << 8 | RxData[5];
                info->temperature[3] = RxData[6];
                break;
        }
    }
}