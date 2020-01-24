/*
 * led_task.cpp
 *
 *  Created on: 19-Feb-2019
 *      Author: hp
 */

/* Include Required headers */
#include <led_task.hpp>


/* Task Initialization Function */
void LedTask_ :: xvinit( void )
{
	/* Return Status of Task */
	BaseType_t xReturned = pdFAIL;
	/* Initialize parameters */
	xHandle = NULL;

	/* Create task */
	xReturned = xTaskCreate( LedTask_::xsvTaskEntryPoint,
							xLedTaskParams.p_name, xLedTaskParams.stackDepth,
							this, xLedTaskParams.priority, &xHandle
							);

	/* Delete task if not created successfully */
	if( xReturned != pdPASS )
	{
		vTaskDelete( xHandle );
	}
	else
	{
		/* Can do some initial processing if needed */
	}
}


/* LED Task Entry function */
void LedTask_ :: xvTaskEntry( void *pvParams )
{
	/* Not using */
	(void)pvParams;

	/* Local Variables */
    TickType_t xLastWakeTime = 0;
    const TickType_t xFrequency = 500;

    /* Initialize the xLastWakeTime variable with the current time. */
    xLastWakeTime = xTaskGetTickCount();

    /* Task execution entry */
    for( ; ; )
    {
		GPIO__::fp_toggle( (XMC_GPIO_PORT_t *)xLedPort, xLedPin );
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}

	/* Should never reach here, if so the delete the task and free memory */
	vTaskDelete( NULL );
}


/********************************** End of File *******************************/
