//IO操作函数	 

#include "jwMPUI2CSoftware.h"
#include "stm32f7xx_hal.h"


#define I2C_SDA_PIN  GPIO_PIN_0
#define I2C_SCL_PIN  GPIO_PIN_1  
#define MPU_ADDR 0x68


#define MPU_IIC_SDA_1()  do{HAL_GPIO_WritePin(GPIOF, I2C_SDA_PIN, GPIO_PIN_SET);}while(0)
#define MPU_IIC_SDA_0()  do{HAL_GPIO_WritePin(GPIOF, I2C_SDA_PIN, GPIO_PIN_RESET);}while(0)
 
#define MPU_IIC_SCL_1()  do{HAL_GPIO_WritePin(GPIOF, I2C_SCL_PIN, GPIO_PIN_SET);}while(0)
#define MPU_IIC_SCL_0()  do{HAL_GPIO_WritePin(GPIOF, I2C_SCL_PIN, GPIO_PIN_RESET);}while(0)
 
#define MPU_IIC_SDA_READ()    HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0)


static void MPU_IIC_Ack(void);
static void MPU_IIC_NAck(void);

void MPU_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOF_CLK_ENABLE();
	
	/*Configure GPIO pins : PF0 PF1 */
	GPIO_InitStruct.Pin = I2C_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}
void MPU_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOF_CLK_ENABLE();
	
	/*Configure GPIO pins : PF0 PF1 */
	
	GPIO_InitStruct.Pin = I2C_SDA_PIN;
	GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

void MPU_IIC_Delay(void)
{ 
	#if 1
	for(int i=0;i<100;i++){
		__nop();
	}
	#endif
	//osDelay(200);
}

void MPU_IIC_Start(void)
{
		MPU_SDA_OUT();     //sda线输出
		MPU_IIC_SDA_1();	  	  
		MPU_IIC_SCL_1();
		MPU_IIC_Delay();
		MPU_IIC_SDA_0();//START:when CLK is high,DATA change form high to low 
		MPU_IIC_Delay();
		MPU_IIC_SCL_0();//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void MPU_IIC_Stop(void)
{
		MPU_SDA_OUT();//sda线输出
		MPU_IIC_SCL_0();
		MPU_IIC_SDA_0();//STOP:when CLK is high DATA change form low to high
		MPU_IIC_Delay();
		MPU_IIC_SCL_1();  
		MPU_IIC_SDA_1();//发送I2C总线结束信号
		MPU_IIC_Delay();							   	
}


//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL_0();//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
				if(((txd &0x80)>>7)==1){
					MPU_IIC_SDA_1();
				}else{
					MPU_IIC_SDA_0();
				}
				txd<<=1;
				MPU_IIC_SCL_1();
				MPU_IIC_Delay(); 
				MPU_IIC_SCL_0();	
				MPU_IIC_Delay();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL_0(); 
        MPU_IIC_Delay();
		MPU_IIC_SCL_1();
        receive<<=1;
        if(MPU_IIC_SDA_READ())
			receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//发送nACK
    else
        MPU_IIC_Ack(); //发送ACK   
    return receive;
}


//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	
	MPU_SDA_IN();      //SDA设置为输入  
	MPU_IIC_SDA_1();
	MPU_IIC_Delay();	   
	MPU_IIC_SCL_1();
	MPU_IIC_Delay();	 
	while(MPU_IIC_SDA_READ())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL_0();//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL_0();
	MPU_SDA_OUT();
	MPU_IIC_SDA_0();
	MPU_IIC_Delay();
	MPU_IIC_SCL_1();
	MPU_IIC_Delay();
	MPU_IIC_SCL_0();
}
//不产生ACK应答		    
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL_0();
	MPU_SDA_OUT();
	MPU_IIC_SDA_1();
	MPU_IIC_Delay();
	MPU_IIC_SCL_1();
	MPU_IIC_Delay();
	MPU_IIC_SCL_0();
}


//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答 
	MPU_IIC_Send_Byte(data);//发送数据
	if(MPU_IIC_Wait_Ack())	//等待ACK
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
    MPU_IIC_Stop();	 
	return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	MPU_IIC_Wait_Ack();		//等待应答 
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	res=MPU_IIC_Read_Byte(0);//读取数据,发送nACK 
    MPU_IIC_Stop();			//产生一个停止条件 
	return res;		
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)
			*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
		else 
			*buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//产生一个停止条件 
	return 0;	
}

