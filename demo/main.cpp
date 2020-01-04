/*
 * main.cpp
 *
 *  Author: heman
 */


/* xmc libraries inclusion */
#include <xmc_common.h>
/* Peripheral libraries inclusion */
#include <xmc_gpio.h>


#define LED_PIN			0


extern "C"{

/* Routine to handle the SysTick interrupt */
void SysTick_Handler( void );


#ifdef HAVE_INITFINI_ARRAY

#ifdef HAVE_INIT_FINI
extern void _init (void);
#endif

extern void (*__preinit_array_start []) (void) __attribute__((weak));
extern void (*__preinit_array_end []) (void) __attribute__((weak));
extern void (*__init_array_start []) (void) __attribute__((weak));
extern void (*__init_array_end []) (void) __attribute__((weak));
extern void (*__fini_array_start []) (void) __attribute__((weak));
extern void (*__fini_array_end []) (void) __attribute__((weak));

void __libc_init_array (void)
{
	size_t count;
	size_t i;

	count = __preinit_array_end - __preinit_array_start;
	for (i = 0; i < count; i++)
	{
		__preinit_array_start[i] ();
	}

	#ifdef HAVE_INIT_FINI
	  _init ();
	#endif

	count = __init_array_end - __init_array_start;
	for (i = 0; i < count; i++)
	{
		__init_array_start[i] ();
	}
}

#endif


#ifdef HAVE_INITFINI_ARRAY
extern void (*__fini_array_start []) (void) __attribute__((weak));
extern void (*__fini_array_end []) (void) __attribute__((weak));

#ifdef HAVE_INIT_FINI
extern void _fini (void);
#endif


/* Run all the cleanup routines. */
void __libc_fini_array (void)
{
	size_t count;
	size_t i;
	
	count = __fini_array_end - __fini_array_start;
	for (i = count; i > 0; i--)
	{
		__fini_array_start[i-1] ();
	}

	#ifdef HAVE_INIT_FINI
	  _fini ();
	#endif
}
#endif


}


XMC_GPIO_PORT_t volatile * LedPort;


/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. 
 */

int main(void)
{
	/* Local variable */
	uint32_t status = 0;
	/* GPIO functionality for testing */
	XMC_GPIO_CONFIG_t LedPortConfig;

	/* Initialize GPIO port */
	LedPort = XMC_GPIO_PORT1;

	/* Configure pin specifications */
	LedPortConfig.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
	LedPortConfig.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW;
	LedPortConfig.output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_MEDIUM_EDGE;

	/* Update Clock  settings */
	SystemCoreClockUpdate();

	/* Intialize LED port with led pin */
	XMC_GPIO_Init( (XMC_GPIO_PORT_t *)LedPort, LED_PIN, &LedPortConfig);

	/* Configure SysTick */
	status = SysTick_Config(SystemCoreClock / 1000);	// Generate tick for every 1ms

	if( 1U == status )
	{
		XMC_GPIO_SetOutputLevel( (XMC_GPIO_PORT_t *)LedPort, LED_PIN, XMC_GPIO_OUTPUT_LEVEL_LOW );
		while(1U);
	}
	else{ ; }

	/* Placeholder for user application code. The while loop below can be replaced with user application code. */
	while(1U)
	{
		;
	}
}

/*********** End ***********/


/* Routine to handle the SysTick interrupt */
void SysTick_Handler( void )
{
	/* Routine functioning
	 * Toggle Output of Led
	 * */
	static uint32_t count = 500;
	if( 0 == --count )
	{
		XMC_GPIO_ToggleOutput( (XMC_GPIO_PORT_t *)LedPort, LED_PIN );
		count = 500;
	}
}


