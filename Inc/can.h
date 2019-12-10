/**
  ******************************************************************************
  *  �û��Զ��� CAN API
  ******************************************************************************
  */

#ifndef __CAN_H
#define __CAN_H

/***CAN ���ͻ���յ� ID***/
typedef enum{
    //���̵��
    M3508_Motor1_ID = 0x201,
    M3508_Motor2_ID = 0x202,
    M3508_Motor3_ID = 0x203,
    M3508_Motor4_ID = 0x204,
}CAN_Message_ID;

/***������������ṹ��***/
typedef struct{
	uint16_t angle;
    uint16_t speed;
    uint16_t current;
    uint8_t  temperature;
}Motor_Info;

/***API�ӿ�***/
extern void My_CAN_Init();                                                          //��ʼ��CAN
extern void Set_Motor_Current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);  //���õ������ֵ
extern void Get_Motor_Info(Motor_Info* info);                                       //��ȡ���������Ϣ

#endif