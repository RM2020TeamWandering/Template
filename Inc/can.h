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
#define FILTER_BUF_LEN 5
typedef struct{
	int16_t	 	speed_rpm;
    int16_t  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	uint8_t		buf_idx;
	uint16_t	angle_buf[FILTER_BUF_LEN];
	uint16_t	fited_angle;
	uint32_t	msg_cnt;
}Motor_Info;

/***API接口***/
extern void Set_Motor_Current(CAN_HandleTypeDef* hcan);                        //设置电机电流值
extern void Get_Motor_Info(CAN_HandleTypeDef* hcan, Motor_Info* info);         //获取电机反馈信息

#endif