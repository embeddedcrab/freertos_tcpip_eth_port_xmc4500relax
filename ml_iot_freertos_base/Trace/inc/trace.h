#ifndef TRACE_H_
#define TRACE_H_


/* include Headers */
#include <core_cm4.h>


#ifdef __cplusplus
extern "C"{
#endif


/* Check Trace data present or not? */
static inline int32_t ITM_isAvailable( void ){
	return (int32_t) ITM_CheckChar();
}

/*------- Function Declarations -------*/

/* Write to Host for Trace output */
long ITM_printf( void *src, uint32_t len );

/* Read from Host for Debugging */
long ITM_scanf( void *dst, uint32_t len );


#ifdef __cplusplus
}
#endif

#endif	/* TRACE_H_ */
