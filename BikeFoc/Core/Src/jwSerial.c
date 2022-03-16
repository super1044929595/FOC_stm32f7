#include "jwSerial.h"
#include "jwUnite.h"
#include "stdio.h"

//private method 
static bool jwSerilaPortInit(void);
static bool jwSerilaPortIntterrupt(void);


Communicatopn_InitTypedef xCommunication_Typedef={

.name ="st_serial_1",	
.communication_Init=jwSerilaPortInit,
.communication_interrupt_register=NULL,

};




 

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  Noned
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

bool jwSerilaPortInit(void)
{
	//printf("\r\n jwSerilaPortInit");
	
	return false;
}


bool jw_communication_Init(void)
{
	xCommunication_Typedef.communication_Init();
	return false;
}

