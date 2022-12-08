//TODO: add code for your component here
#include "global_variables.h"

void Init_global_variables(void)
{
	sem_adc = xSemaphoreCreateBinary();
	
	
	voltage = 0;
	
	global_status = 0;
}