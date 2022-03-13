#include "jwSerial.h"
#include "jwUnite.h"

Communicatopn_InitTypedef xCommunication_Typedef={

.name ="st_serial_1",	
.communication_Init=jwSerilaPortInit,
.communication_interrupt_register=NULL,

};


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

