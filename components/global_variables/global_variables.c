//TODO: add code for your component here
#include "global_variables.h"

void Init_global_variables(void)
{
	sem_adc = xSemaphoreCreateBinary();
	
	
	voltage = 0;
	
	global_status = 0;
	
	gps_start_flag = 0;
	// Init Operator Position
	g_OperatorLatitude = 37.5728265;
	g_OperatorLongitude = 126.9768827;
}