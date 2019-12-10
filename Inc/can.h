/**
  ******************************************************************************
  *  用户自定义 CAN API
  ******************************************************************************
  */

#ifndef __CAN_H
#define __CAN_H

/***CAN 发送或接收的 ID***/
typedef enum{
    //底盘电机
    M3508_Motor1_ID = 0x201,
    M3508_Motor2_ID = 0x202,
    M3508_Motor3_ID = 0x203,
    M3508_Motor4_ID = 0x204,
}CAN_Message_ID;

/***电机反馈参数结构体***/
typedef struct{
	uint16_t angle;
    uint16_t speed;
    uint16_t current;
    uint8_t  temperature;
}Motor_Info;

/***API接口***/
extern void My_CAN_Init();                                                          //初始化CAN
extern void Set_Motor_Current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);  //设置电机电流值
extern void Get_Motor_Info(Motor_Info* info);                                       //获取电机反馈信息

#endif