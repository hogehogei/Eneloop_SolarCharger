
#include "stm32f0xx_hal.h"
#include "UARTOut.hpp"

#ifdef __cplusplus
extern "C" {
#endif

extern UART_HandleTypeDef huart1;

void OutputFunc( unsigned char c )
{
	static unsigned char buf[4] = {};
	buf[0] = c;
	HAL_UART_Transmit( &huart1, buf, 1, 2 );
}

#ifdef __cplusplus
}
#endif

