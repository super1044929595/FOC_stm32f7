#include "jwAlgorothem.h"

//----------------------------------------- 
#define SMPLRT_DIV 0x19 //陀螺仪采样率，典型值：0x07(125Hz) 
#define CONFIG 0x1A //低通滤波频率，典型值：0x06(5Hz) 
#define GYRO_CONFIG 0x1B //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) 
#define ACCEL_CONFIG 0x1C //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) 

/* 加速度相关寄存器地址 */
#define ACCEL_XOUT_H 0x3B 
#define ACCEL_XOUT_L 0x3C 
#define ACCEL_YOUT_H 0x3D 
#define ACCEL_YOUT_L 0x3E 
#define ACCEL_ZOUT_H 0x3F 
#define ACCEL_ZOUT_L 0x40 

/* 温度相关寄存器地址 */
#define TEMP_OUT_H 0x41 
#define TEMP_OUT_L 0x42 

/* 陀螺仪相关寄存器地址 */
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44 
#define GYRO_YOUT_H 0x45 
#define GYRO_YOUT_L 0x46 
#define GYRO_ZOUT_H 0x47 
#define GYRO_ZOUT_L 0x48 

#define PWR_MGMT_1 0x6B  //电源管理，典型值：0x00(正常启用) 
#define WHO_AM_I 0x75 //IIC地址寄存器(默认数值0x68，只读) 
#define SlaveAddress 0xD0 //IIC写入时的地址字节数据，+1为读取 

void jw_MPU6050_init(void)
{

    MPU_Write_Byte(PWR_MGMT_1, 0x00);    //解除休眠状态     
    MPU_Write_Byte(SMPLRT_DIV, 0x07);    //陀螺仪采样率，典型值：0x07(125Hz)     
    MPU_Write_Byte(CONFIG, 0x06);        //低通滤波频率，典型值：0x06(5Hz)     
    MPU_Write_Byte(GYRO_CONFIG, 0x18);   //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)     
    MPU_Write_Byte(ACCEL_CONFIG, 0x01);  //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) 


	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	

}


// 【基础】基本数据读取\USER\src\mpu6050.c 
/****************************************************************************** 
* 函数介绍： 连续读两个寄存器并合成 16 位数据 
* 输入参数： regAddr：数据低位寄存器地址 
* 输出参数： 无 
* 返回值 ： data：2 个寄存器合成的 16 位数据 
******************************************************************************/
int16_t MPU6050_Get_Data(uint8_t regAddr)
{
    uint8_t Data_H, Data_L;
    uint16_t data;
    
    Data_H = MPU_Read_Byte(regAddr);
    Data_L = MPU_Read_Byte(regAddr + 1);
    data = (Data_H << 8) | Data_L;  // 合成数据 
    return data;
}

// 【基础】基本数据读取\USER\src\main.c 
/****************************************************************************** 
* 函数介绍： 串口打印 6050 传感器读取的三轴加速度、陀螺仪及温度数据 
* 输入参数： 无 
* 输出参数： 无 
* 返回值 ： 无 
******************************************************************************/


typedef struct
{
	uint16_t	x_accel_offset;
	uint16_t	y_accel_offset;
	uint16_t	z_accel_offset;
	uint16_t    gyro_x_offset;
	uint16_t    gyro_y_offset;
	uint16_t    gyro_z_offset;
}mpu6050_jw_offset;

mpu6050_jw_offset jw_ooferset;

void MPU6050_Display_First(void)
{
    /* 打印 x, y, z 轴加速度 */
	jw_ooferset.x_accel_offset=0;
	jw_ooferset.y_accel_offset=0;
	jw_ooferset.z_accel_offset=0;
	jw_ooferset.gyro_x_offset=0;
	jw_ooferset.gyro_y_offset=0;
	jw_ooferset.gyro_z_offset=0;

    printf("ACCEL_X: %d\t", MPU6050_Get_Data(ACCEL_XOUT_H));
    printf("ACCEL_Y: %d\t", MPU6050_Get_Data(ACCEL_YOUT_H));
    printf("ACCEL_Z: %d\t", MPU6050_Get_Data(ACCEL_ZOUT_H));
    
    /* 打印温度，需要根据手册的公式换算为摄氏度 */
    printf("TEMP: %0.2f\t", MPU6050_Get_Data(TEMP_OUT_H) / 340.0 + 36.53);
    
    /* 打印 x, y, z 轴角速度 */
    printf("GYRO_X: %d\t", MPU6050_Get_Data(GYRO_XOUT_H));
    printf("GYRO_Y: %d\t", MPU6050_Get_Data(GYRO_YOUT_H));
    printf("GYRO_Z: %d\t", MPU6050_Get_Data(ACCEL_ZOUT_H));
    
    printf("\r\n");

	jw_ooferset.x_accel_offset=MPU6050_Get_Data(ACCEL_XOUT_H);	
	jw_ooferset.y_accel_offset=MPU6050_Get_Data(ACCEL_YOUT_H);
	jw_ooferset.z_accel_offset=MPU6050_Get_Data(ACCEL_ZOUT_H);


	jw_ooferset.gyro_x_offset=MPU6050_Get_Data(GYRO_XOUT_H);	
	jw_ooferset.gyro_y_offset=MPU6050_Get_Data(GYRO_YOUT_H);
	jw_ooferset.gyro_z_offset=MPU6050_Get_Data(ACCEL_ZOUT_H);

}

// 基本数据读取\USER\src\main.c 
/* 传感器数据修正值（消除芯片固定误差，根据硬件进行调整） */
//#define X_ACCEL_OFFSET -600 
//#define Y_ACCEL_OFFSET -100 
//#define Z_ACCEL_OFFSET 2900 
//#define X_GYRO_OFFSET 32 
//#define Y_GYRO_OFFSET -11 
//#define Z_GYRO_OFFSET 1 
/****************************************************************************** 
* 函数介绍： 串口打印 6050 传感器读取的三轴加速度、陀螺仪及温度数据 
* 输入参数： 无 
* 输出参数： 无 
* 返回值 ： 无 
把开发板/模块的放置在水平位置（或其他参考位置）保持静止，观察并记录输出的原始数据，不断调整各个补偿值 OFFSET 的大小，确保原始数据在 X、Y、Z 轴加速度为 0，0，16384（对应 1 g） 左右，且三轴角速度值为 0。
扩展操作 —— 坐标倾角计算

******************************************************************************/
void MPU6050_Display_JW(void)
{
    /* 打印 x, y, z 轴加速度 */
    printf("ACCEL_X: %d\t", MPU6050_Get_Data(ACCEL_XOUT_H) + jw_ooferset.x_accel_offset);
    printf("ACCEL_Y: %d\t", MPU6050_Get_Data(ACCEL_YOUT_H) + jw_ooferset.y_accel_offset);
    printf("ACCEL_Z: %d\t", MPU6050_Get_Data(ACCEL_ZOUT_H) + jw_ooferset.z_accel_offset);
    
    /* 打印温度 */
    printf("TEMP: %0.2f\t", MPU6050_Get_Data(TEMP_OUT_H) / 340.0 + 36.53);
    
    /* 打印 x, y, z 轴角速度 */
    printf("GYRO_X: %d\t", MPU6050_Get_Data(GYRO_XOUT_H) + jw_ooferset.gyro_x_offset);
    printf("GYRO_Y: %d\t", MPU6050_Get_Data(GYRO_YOUT_H) + jw_ooferset.gyro_y_offset);
    printf("GYRO_Z: %d\t", MPU6050_Get_Data(GYRO_ZOUT_H) + jw_ooferset.gyro_z_offset);
    
    printf("\r\n");
}



/****************************************************************************** 
* 函数介绍： 计算 x, y, z 三轴的倾角 
* 输入参数： 无 
* 输出参数： data：角度结构体 
* 返回值 ： 无 
******************************************************************************/
void MPU6050_Get_Angle(MPU6050_Angle *data)
{   
	jw_ooferset.x_accel_offset=0;
	jw_ooferset.y_accel_offset=0;
	jw_ooferset.z_accel_offset=0;
	jw_ooferset.gyro_x_offset=0;
	jw_ooferset.gyro_y_offset=0;
	jw_ooferset.gyro_z_offset=0;

    /* 计算x, y, z 轴倾角，返回弧度值*/
    data->X_Angle = acos((MPU6050_Get_Data(ACCEL_XOUT_H) + jw_ooferset.x_accel_offset) / 16384.0);
    data->Y_Angle = acos((MPU6050_Get_Data(ACCEL_YOUT_H) + jw_ooferset.y_accel_offset) / 16384.0);
    data->Z_Angle = acos((MPU6050_Get_Data(ACCEL_ZOUT_H) + jw_ooferset.z_accel_offset) / 16384.0);

    /* 弧度值转换为角度值 */
    data->X_Angle = data->X_Angle * 57.29577;
    data->Y_Angle = data->Y_Angle * 57.29577;
    data->Z_Angle = data->Z_Angle * 57.29577;
	
	

	if((data->X_Angle>0)||(data->X_Angle<=180)||(data->Y_Angle>0)||(data->Y_Angle<=180)||(data->Z_Angle>0)||(data->Z_Angle<=180)){
		printf("\r\n jw-----------> X_Angle:[%0.3f],y_Angle:[%0.3f],z_Angle:[%0.3f]",data->X_Angle,data->Y_Angle,data->Z_Angle);
	}
	
} 


