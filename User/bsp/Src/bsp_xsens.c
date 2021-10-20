/** 
  * @file xsens_bsp.c 
  * @version 1.0
  * @date April 1st 2021
	*
  * @brief  xsens数据解码、数据转换位float类型
  * 		MTi-630 Series
	* @author Haoyun Yan
	*
  * @note   不含配置输出函数(setOutputConfiguration)
	*         重新配置方法：用串口调式助手发信息给xsens
	*					配置参考链接http://mtidocs.xsens.com/home
  */
	
#include "bsp_xsens.h"

xsens_dataTypeDef_t xsens_data;

/**
  * @brief     xsens数据解码
  * @param     串口接收到的数据和数据长度
  * @return    none
  */
void xsens_callback_handle(uint8_t *rx_buff, uint16_t length)
{
	uint8_t xsens_MegID_buff;
	uint8_t data_consumption; 
	uint16_t MTData_Type_buff;
	float buff_imuRoll, buff_imuPitch, buff_imuYaw;
	float buff_accX, buff_accY, buff_accZ;
	float buff_gyroX, buff_gyroY, buff_gyroZ;
	
	
	if(rx_buff[0] == 0xFA && rx_buff[1] == 0xFF) // 条件1：帧头
	{
		if(xsens_checkSum(rx_buff, length)) //条件2：校验和
		{
			xsens_MegID_buff = rx_buff[2];
			switch(xsens_MegID_buff)
			{
				case xsens_msg_ID_Err:
				{	Error_Handler(); printf("xsens receive error message!\n"); break;}
				case xsens_msg_ID_MTData2:
				{
					
				  for(int i=4; i<length-1 ; i+=data_consumption) //第一个Type索引为4,循环检索到最后一字节checkSum
					{
						MTData_Type_buff = rx_buff[i] << 8 | rx_buff[i+1];
						data_consumption = 2 + 1 + rx_buff[i+2] ; //Type + Length + Data
						switch(MTData_Type_buff)
						{
							case MTData_Type_PacketCounter: 
							{//2个数据
								if(rx_buff[i+2] == 0x02) {xsens_data.packetCounter = rx_buff[i+3] << 8 | rx_buff[i+4];} break;
							}
							case MTData_Type_EulerAngles:
							{//12个数据
								if(rx_buff[i+2] == 0x0C) 
								{
									buff_imuRoll  = hex32_to_float(rx_buff[i+3], rx_buff[i+4], rx_buff[i+5], rx_buff[i+6]);
								  buff_imuPitch = hex32_to_float(rx_buff[i+7], rx_buff[i+8], rx_buff[i+9], rx_buff[i+10]);
									buff_imuYaw   = hex32_to_float(rx_buff[i+11], rx_buff[i+12], rx_buff[i+13], rx_buff[i+14]);	
								}
							break;
							}
							case MTData_Type_Acceleration:
							{//12个数据
								if(rx_buff[i+2] == 0x0C) 
								{
									buff_accX = hex32_to_float(rx_buff[i+3], rx_buff[i+4], rx_buff[i+5], rx_buff[i+6]);
									buff_accY = hex32_to_float(rx_buff[i+7], rx_buff[i+8], rx_buff[i+9], rx_buff[i+10]);
									buff_accZ = hex32_to_float(rx_buff[i+11], rx_buff[i+12], rx_buff[i+13], rx_buff[i+14]);
								}
							break;
							}
							case MTData_Type_RateOfTurn:
							{//12个数据
								if(rx_buff[i+2] == 0x0C) 
								{
									buff_gyroX = hex32_to_float(rx_buff[i+3], rx_buff[i+4], rx_buff[i+5], rx_buff[i+6]);
								  buff_gyroY= hex32_to_float(rx_buff[i+7], rx_buff[i+8], rx_buff[i+9], rx_buff[i+10]);
									buff_gyroZ = hex32_to_float(rx_buff[i+11], rx_buff[i+12], rx_buff[i+13], rx_buff[i+14]);	
								}
							break;
							}
							case MTData_Type_StatusWord:
							{//4个数据
							if(rx_buff[i+2] == 0x04) 
								{
									xsens_data.status_word = rx_buff[i+3] << 24 |  rx_buff[i+4] << 16 |  rx_buff[i+5] << 8 |  rx_buff[i+6];
								}
							break;
							}
						
						}
						
							assemble_data_process(buff_imuRoll,buff_imuPitch,buff_imuYaw,	\
						                        buff_gyroX,buff_gyroY,buff_gyroZ,	\
												buff_accX, buff_accY, buff_accZ);
							
						
					}
					break;
				}
			
			}
		 
		}
	}



}


/**
  * @brief     因安装位置，改变数据极性
  * 		   具体坐标方向参考./xrl_0807/doc/coordinate_of_IMU.pdf
  * @param     
  * @return    none
  */
void assemble_data_process(float buff_imuRoll,float buff_imuPitch,float buff_imuYaw,	\
						                        float buff_gyroX,float buff_gyroY,float buff_gyroZ,	\
												float buff_accX,float buff_accY,float buff_accZ)
{
			if(buff_imuRoll > 0) {xsens_data.roll = buff_imuRoll - 180; }
			else if (buff_imuRoll < 0) {xsens_data.roll = buff_imuRoll + 180; }
		  else {xsens_data.roll = 0.0;}

			xsens_data.pitch = 	buff_imuPitch;
			xsens_data.yaw = buff_imuYaw;
			
			xsens_data.gyrX = buff_gyroX;
			xsens_data.gyrY = -buff_gyroY;
			xsens_data.gyrZ = -buff_gyroZ;
																		
			xsens_data.accX = -buff_accX;
			xsens_data.accY = buff_accY;
			xsens_data.accZ = buff_accZ;

																		
}


/**
  * @brief     将IMU 4bytes 的数据转换位float类型
  * @param     顺序data[0]-data[3] 十六进制
  * @return    转换后float数据
  */
float hex32_to_float(uint8_t HH, uint8_t HL, uint8_t LH, uint8_t LL)
{
	unsigned char pxMem[4] = {LL, LH, HL, HH};
	float result;
	float *p;
	
	p = (float*)pxMem;
	result = (float)*p;
	
	return result;
}

/**
  * @brief     串口数据校验和
  *            除帧头0xFF,所有数据加起来,最低的byte = 0x00
  * @param     接受数据
  * @return    1：校验成功
  *            0: 校验未通过
  */
int xsens_checkSum(uint8_t *buff, uint16_t len)
{
	uint8_t sum = 0;
	
	for(int i=1; i<len; i++)
	{sum+= buff[i];}

	if(sum == 0x00) return 1;
	else return 0;
}
