#ifndef BSP_6AXIS_H
#define BSP_6AXIS_H

#include "main.h"
#include "usart.h"
#include "bsp_usart.h"
#include "stdio.h"
#include "stdarg.h"


typedef union {
	float f;
	uint8_t  buf[4];
}HexFloat;

typedef struct{
	HexFloat Fx,Fy,Fz;
    HexFloat MX,MY,MZ;
	float FX1,FY1,FZ1;
	float mx1,my1,mz1;
	float defaultForce;
	float k;
	float deltaF;
} FORCE;

extern FORCE force;

#endif
