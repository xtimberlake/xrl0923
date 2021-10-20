#ifndef XSENS_H
#define XSENS_H

#include "main.h"
#include "usart.h"
#include "bsp_usart.h"
#include "stdio.h"

//xsens Message ID
typedef enum
{
	xsens_msg_ID_MTData2		  =		0x36,		//Motion Tracker Data
	xsens_msg_ID_Err          =   0x42    //Error Message
}xsens_message_ID;


//MTData2 Type
typedef enum
{
	MTData_Type_PacketCounter		  =	0x1020,		//数据包序号
	MTData_Type_EulerAngles       = 0x2030,   //欧拉角
	MTData_Type_Acceleration      = 0x4020,   //加速度
	MTData_Type_RateOfTurn        = 0x8020,   //角速度
	MTData_Type_StatusWord        = 0xE020    //状态字

}MTData2_Type;



typedef struct
{
	uint16_t packetCounter;	//数据序号
	float roll;
	float pitch;
	float yaw;
	float accX;
	float accY;
	float accZ;
	float gyrX;
	float gyrY;
	float gyrZ;
	uint32_t status_word;
} xsens_dataTypeDef_t;

extern xsens_dataTypeDef_t xsens_data;

float hex32_to_float(uint8_t HH, uint8_t HL, uint8_t LH, uint8_t LL);
extern void xsens_callback_handle(uint8_t *rx_buff, uint16_t length);
extern int xsens_checkSum(uint8_t *buff, uint16_t len);
void assemble_data_process(float buff_imuRoll,float buff_imuPitch,float buff_imuYaw,	\
						                        float buff_gyroX,float buff_gyroY,float buff_gyroZ,	\
												float buff_accX,float buff_accY,float buff_accZ);

#endif

