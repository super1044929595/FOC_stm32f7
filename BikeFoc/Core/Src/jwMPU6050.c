#include "jwMPU6050.h"
#include "jwMPUI2CSoftWare.h"




#define MOPU6050_ADDRESS  0x68

#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
 
//Èç¹ûAD0½Å(9½Å)½ÓµØ,IICµØÖ·Îª0X68(²»°üº¬×îµÍÎ»).
//Èç¹û½ÓV3.3,ÔòIICµØÖ·Îª0X69(²»°üº¬×îµÍÎ»).
#define MPU_ADDR				0X68



//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}


#if 0 //hardware driver
uint8_t MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	//HAL_I2C_Mem_Write(&hi2c2,0x68,reg,I2C_MEMADD_SIZE_8BIT, buf, len, 100);

}
uint8_t MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	HAL_I2C_Mem_Read(&hi2c2, 0x68,reg,I2C_MEMADD_SIZE_8BIT,buf,len,1000);
	return 0;
}
uint8_t MPU_Write_Byte(u8 reg,u8 data)
{
	//if(HAL_I2C_Mem_Write(&hi2c2,0x68,reg,I2C_MEMADD_SIZE_8BIT, &data, 1, 100)!=HAL_OK){
		//printf("\r\n  i2c error");
	}
}
uint8_t MPU_Read_Byte(u8 reg)
{
	uint8_t data;

	if(HAL_I2C_Mem_Read(&hi2c2, 0x68,reg,I2C_MEMADD_SIZE_8BIT,&data,1,1000)!=HAL_OK){
		printf("\r\n  i2c error");

	}
	return data;
}
#endif

bool JW_MPU6050_Init(void)
{
	u8 res; 

	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    osDelay(500);
	#if 0
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	#else
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);//唤醒MPU6050 
	MPU_Write_Byte(MPU_SAMPLE_RATE_REG,0x01);//设置采样率为500Hz
	MPU_Write_Byte(MPU_CFG_REG,0x04);	//设置20Hz低通滤波
	MPU_Write_Byte(MPU_GYRO_CFG_REG,0x10);//设置陀螺仪量程+-1000
	MPU_Write_Byte(MPU_ACCEL_CFG_REG,0x09);//设置加速度计量程+-4g
	#endif
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG); 
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);
		//设置采样率为50Hz
		return 0;
 	}else return 1;

}



/***********************************************************************/

struct _sensor sensor;	


void MPU_Getdata(void)
{
	u8 bufa[6],bufg[6];
	
	MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,bufa);
	MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,bufg);
	
	sensor.acc.origin.x = ((((int16_t)bufa[0]) << 8) | bufa[1]);
	sensor.acc.origin.y = ((((int16_t)bufa[2]) << 8) | bufa[3]);
	sensor.acc.origin.z = ((((int16_t)bufa[4]) << 8) | bufa[5]);

	sensor.gyro.origin.x = ((((int16_t)bufg[0]) << 8)| bufg[1]);
	sensor.gyro.origin.y = ((((int16_t)bufg[2]) << 8)| bufg[3]);
	sensor.gyro.origin.z = ((((int16_t)bufg[4]) << 8)| bufg[5]);
	
	sensor.acc.offset.x=sensor.acc.origin.x-sensor.acc.quiet.x;
	sensor.acc.offset.y=sensor.acc.origin.y-sensor.acc.quiet.y;
  
	sensor.gyro.offset.x = sensor.gyro.origin.x - sensor.gyro.quiet.x;
	sensor.gyro.offset.y = sensor.gyro.origin.y - sensor.gyro.quiet.y;
	sensor.gyro.offset.z = sensor.gyro.origin.z - sensor.gyro.quiet.z;
}

void MPU_Offset(void)
{
	static u8 over_flag=0;
	u8  i,cnt_g = 0;
	struct _int16 last;
	int Integral[3] = {0,0,0};
	s32 tempg[3]={0,0,0},tempa[3]={0,0,0};
	//s16 gx_last=0,gy_last=0,gz_last=0;
	while(!over_flag)	//此循环是确保MPU6050芯片处于完全静止状态
	{
		if(cnt_g < 200)
		{
			MPU_Getdata();

			tempg[0] += sensor.gyro.origin.x;
			tempg[1] += sensor.gyro.origin.y;
			tempg[2] += sensor.gyro.origin.z;
			
			tempa[0] += sensor.acc.origin.x;
			tempa[1] += sensor.acc.origin.y;
			tempa[2] += sensor.acc.origin.z;
			
			Integral[0] += abs(last.x - sensor.gyro.origin.x);
			Integral[1] += abs(last.y - sensor.gyro.origin.y);
			Integral[2] += abs(last.z - sensor.gyro.origin.z);

			last.x = sensor.gyro.origin.x;
			last.y = sensor.gyro.origin.y;
			last.z = sensor.gyro.origin.z;
		}
		else 
			{			
			if(Integral[0]>=60|| Integral[1]>=60|| Integral[2]>= 60) // 未校准成功
				{
				 cnt_g = 0;
				 for(i=0;i<3;i++)tempa[i]=tempg[i]=Integral[i]=0;
			  }
			else // 校准成功 
				{				
				  sensor.acc.quiet.x = tempa[0]/200;
					sensor.acc.quiet.y = tempa[1]/200;
					sensor.acc.quiet.z = tempa[2]/200;
				
				  sensor.gyro.quiet.x = tempg[0]/200;
	        sensor.gyro.quiet.y = tempg[1]/200;
	        sensor.gyro.quiet.z = tempg[2]/200;
				  over_flag = 1;
			  }
		}
		cnt_g++;
	}
}


float Angle_Balance_x; //横向角度
float Gyro_Balance_x=0;     //横向角加速度
float Center_Gravity;               //机械中值x轴方向机械中值  可以将小车PWM关闭然后观察小车的x轴平衡时屏幕显示的角度（即为机械中值），每个小车由于电池和惯量轮安装位置不同会有些许差异
float Center_Gra_Sart=90.0;         //开机设定机械中值

#define PI 3.14159265

void Yijielvbo_X(float angle_m, float gyro_m) //一阶互补滤波  入口参数：加速度、角速度
{
	Angle_Balance_x = 0.1 * angle_m+ (1-0.1) * (Angle_Balance_x + gyro_m * 0.005);
}


float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_X,Gyro_Z;
void Get_Angle(void)
{ 
  u8 bufa[6],bufg[6];	
	
	MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,bufa);
	MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,bufg);
	
//	sensor.acc.origin.y = ((((int16_t)bufa[0]) << 8) | bufa[1]);
//	sensor.acc.origin.x = ((((int16_t)bufa[2]) << 8) | bufa[3]);
//	sensor.acc.origin.z = ((((int16_t)bufa[4]) << 8) | bufa[5]);

//	sensor.gyro.origin.y = ((((int16_t)bufg[0]) << 8)| bufg[1]);
//	sensor.gyro.origin.x = ((((int16_t)bufg[2]) << 8)| bufg[3]);
//	sensor.gyro.origin.z = ((((int16_t)bufg[4]) << 8)| bufg[5]);
	
	sensor.acc.origin.x = ((((int16_t)bufa[0]) << 8) | bufa[1]);
	sensor.acc.origin.y = ((((int16_t)bufa[2]) << 8) | bufa[3]);
	sensor.acc.origin.z = ((((int16_t)bufa[4]) << 8) | bufa[5]);

	sensor.gyro.origin.x = ((((int16_t)bufg[0]) << 8)| bufg[1]);
	sensor.gyro.origin.y = ((((int16_t)bufg[2]) << 8)| bufg[3]);
	sensor.gyro.origin.z = ((((int16_t)bufg[4]) << 8)| bufg[5]);
		  
	Gyro_Y=sensor.gyro.origin.y;    //读取X轴陀螺仪
	Gyro_Z=sensor.gyro.origin.z;    //读取Z轴陀螺仪
	Accel_X=sensor.acc.origin.x;    //读取Y轴加速度计
	Accel_Z=sensor.acc.origin.z;    //读取Z轴加速度计
	if(Gyro_Y>32768)  Gyro_Y-=65536;                       //数据类型转换  也可通过short强制类型转换
	if(Gyro_Z>32768)  Gyro_Z-=65536;                       //数据类型转换
	if(Accel_X>32768) Accel_X-=65536;                      //数据类型转换
	if(Accel_Z>32768) Accel_Z-=65536;                      //数据类型转换	
//	Gyro_Balance_y=-Gyro_Y;                //更新平衡角速度		
	Accel_Y=atan2(Accel_X,Accel_Z)*180/PI; //计算倾角	
  Gyro_Y=Gyro_Y/16.4;                    //陀螺仪量程转换

			
	Gyro_X=sensor.gyro.origin.x;       //读取X轴陀螺仪
	Accel_Y=sensor.acc.origin.y;       //读取Y轴加速度计
	if(Gyro_X>32768)  Gyro_X-=65536;   //数据类型转换  也可通过short强制类型转换
	if(Accel_Y>32768) Accel_Y-=65536;  //数据类型转换
	Gyro_Balance_x=-Gyro_X;            //更新平衡角速度
	Accel_X= (atan2(Accel_Z , Accel_Y)) * 180 / PI; //计算倾角	
  Gyro_X=Gyro_X/16.4;                //陀螺仪量程转换			
	Yijielvbo_X(Accel_X,-Gyro_X);
}


int balance_x(float Angle,float gyro)//倾角PD控制 入口参数：角度 返回  值：倾角控制PWM
{  
	 float Balance_KP=350,Balance_KI=0,Balance_KD=2;
   float Bias;                                        //倾角偏差
	 static float D_Bias,Integral_bias;                 //PID相关变量
	 int balance;                                       //PWM返回值 
	 Bias=Angle-Center_Gravity;                                   //求出平衡的角度中值 和机械相关
	 Integral_bias+=Bias;	
	 if(Integral_bias>30000)Integral_bias=30000;
	 if(Integral_bias<-30000)Integral_bias=-30000;
   D_Bias=gyro;	                                      //求出偏差的微分 进行微分控制
	 balance=Balance_KP*Bias+Balance_KI*Integral_bias+D_Bias*Balance_KD;   //===计算倾角控制的电机PWM  PD控制
	 return balance;
}
int velocity_x(int encoder)   //位置式PID控制器 入口参数：编码器测量位置信息，目标位置  返回  值：电机PWM
{ 	
	 float Position_KP=50,Position_KI=0.02,Position_KD=0;
	 static float Pwm,Integral_bias,Last_Bias,Encoder;
	 Encoder *= 0.65;		                                       //一阶低通滤波器       
	 Encoder += encoder*0.35;	                                 //一阶低通滤波器    
	 Integral_bias+=Encoder;	                                 //求出偏差的积分
	 if(Integral_bias>20000)Integral_bias=20000;
	 if(Integral_bias<-20000)Integral_bias=-20000;
	 Pwm=Position_KP*Encoder+Position_KI*Integral_bias+Position_KD*(Encoder-Last_Bias);       //位置式PID控制器
	 Last_Bias=Encoder;                                       //保存上一次偏差 
	 return Pwm;                                              //增量输出
}


/***********************************************************************/






