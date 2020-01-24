/* Include Headers */
#include "trace.h"


/* ITM Variables */
volatile int32_t ITM_RxBuffer;		/*!< External variable to receive characters. */


/* Write to Host for Trace output */
long ITM_printf( void *src, uint32_t len )
{
	/* Local Variables */
	uint8_t *xdata = (uint8_t *) src;
	uint32_t xlen = len;
	uint32_t xcount = 0;

	/* Print Data for debugging */
	for ( xcount = 0; xcount < xlen; ++xcount ){
		ITM_SendChar( xdata[xcount] );
	}
	/* Return length of data received */
	return ((long) xcount);
}


/* Read from Host for Debugging */
long ITM_scanf( void *dst, uint32_t len )
{
	/* Local Variables */
	uint8_t *xdata = (uint8_t *)dst;
	uint32_t xlen = len;
	uint32_t xcount = 0;

	for( xcount = 0; xcount < xlen; ){
		/* Update buffer until a valid character is received */
		if( ( xdata[xcount] = ITM_ReceiveChar() ) != -1 ){
			++xcount;
		}
	}
	/* Return length of data received */
	return ((long) xcount);
}

