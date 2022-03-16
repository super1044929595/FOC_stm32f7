#ifndef __JW_MPU_6050_h
#define __JW_MPU_6050_h

#ifdef __cplusplus
	extern "C"{
#endif

#include "main.h"
#include "jwUnite.h"
#include "stm32f7xx_hal.h"
#include "stm32f767xx.h"

bool JW_MPU6050_Init(void);


uint8_t MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
uint8_t MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
uint8_t MPU_Write_Byte(u8 reg,u8 data);			
uint8_t MPU_Read_Byte(u8 reg);
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);



struct _float{
	      float x;
				float y;
				float z;};

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};	

struct _int32{
				int32_t x;
				int32_t y;
				int32_t z;};

struct _trans{
     struct _int16 origin;  //原始值
	   struct _float averag;  //平均值
	   struct _float histor;  //历史值
	   struct _int16 quiet;   //静态值
	   struct _float offset;  //去零偏值
          };

struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };

extern struct _sensor sensor;	

void MPU_Getdata(void);
void MPU_Offset(void);
void Get_Angle(void);
int balance_x(float Angle,float gyro);//倾角PD控制 入口参数：角度 返回	值：倾角控制PWM
int velocity_x(int encoder);   //位置式PID控制器 入口参数：编码器测量位置信息，目标位置  返回  值：电机PWM

#ifdef __cplusplus
		}
#endif

#endif


