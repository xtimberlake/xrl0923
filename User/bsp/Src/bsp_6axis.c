#include <bsp_6axis.h>

FORCE force;
uint8_t aRxBuffer[60];
uint8_t bRxBuffer[60];
uint8_t RI = 0;
uint8_t RJ = 0;
uint8_t ru = 0;
uint16_t Reciver[31];
uint8_t Rstatus = 0;

void sixaxis_callback_handle(uint8_t *rx_buff, uint16_t length)
{
	for(RI=0;RI<=60;RI++)
	{
		if ((rx_buff[RI] == 0xAA)&&(rx_buff[RI+1] == 0x55))
		{
			Rstatus = 1;
		}
		if (Rstatus == 1)
		{
			Reciver[RJ] = rx_buff[RI];
			RJ++;
		}
		if (RJ >= 31 )
		{
			RJ = 0;
			RI = 0;
			Rstatus = 0;
			break;
		}
	}

		for (ru = 0;ru<4;ru++)
		{
			force.Fx.buf[ru] = Reciver[ru+6];
			force.Fy.buf[ru] = Reciver[ru+10];
			force.Fz.buf[ru] = Reciver[ru+14];
		}
		for (ru = 0;ru<4;ru++)
		{
			force.MX.buf[ru] = Reciver[ru+18];
			force.MY.buf[ru] = Reciver[ru+22];
			force.MZ.buf[ru] = Reciver[ru+26];
		}
		force.FX1 = force.Fx.f;
		force.FY1 = force.Fy.f;
		force.FZ1 = force.Fz.f;
		force.mx1 = force.MX.f;
		force.my1 = force.MY.f;
		force.mz1 = force.MZ.f;
//		printf("%f,%f, %f ,%f ,%f ,%f\r\n",motor.FX1,motor.FY1,motor.FZ1,motor.mx1,motor.my1,motor.mz1);
//		HAL_UART_Transmit_DMA(&huart8,rx_buff,60);
//		HAL_UART_Receive_DMA(&huart8,rx_buff,60);
}
