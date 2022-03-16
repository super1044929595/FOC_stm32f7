#ifndef __JW_MPU_SOFTWARE_I2C_H
#define __JW_MPU_SOFTWARE_I2C_H

#include "main.h"
#include "jwUnite.h"
#include "stm32f7xx_hal_gpio.h"

#ifdef __cplusplus
	exetrn "C"{
#endif

u8 MPU_Write_Byte(u8 reg,u8 data);		
u8 MPU_Read_Byte(u8 reg);
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);


#ifdef __cplusplus
		}
#endif

#endif
