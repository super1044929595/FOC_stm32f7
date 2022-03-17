#ifndef __JW_ALGORITHEM_H
#define __JW_ALGORITHEM_H

#include "math.h"
#include "jwMPU6050.h"
#include "jwMPUI2CSoftWare.h"


#ifdef __cplusplus
	exetrn "C"{
#endif


// 【扩展】坐标倾角计算\USER\src\main.c 
/* 坐标角度结构体 */
typedef struct Angle
{
    double X_Angle;
    double Y_Angle;
    double Z_Angle;
    
} MPU6050_Angle;

void jw_MPU6050_init(void);
void MPU6050_Get_Angle(MPU6050_Angle *data);
void MPU6050_Display_First(void);













#ifdef __cplusplus
		}
#endif


#endif

