/*
 * led_task.hpp
 *
 *  Created on: 19-Feb-2019
 *      Author: hp
 */

#ifndef APP_INC_LED_TASK_HPP_
#define APP_INC_LED_TASK_HPP_


/* Include FreeRTOS Headers */
#include <FreeRTOS.h>
#include <task.h>

/* Include type headers */
#include "app_types.h"

/* Include class headers */
#include <dri_gpio.h>


/* Define a class for LED task */
class LedTask_
{
public:
	/* Constructor */
	LedTask_( XMC_GPIO_PORT_t * port, uint8_t pin, TaskCreationParams *param )
	{
		xLedPort = port;
		xLedPin = pin;
		xLedTaskParams.p_name = param->p_name;
		xLedTaskParams.priority = param->priority;
		xLedTaskParams.stackDepth = param->stackDepth;
	}

	/* Destructor for Led task class */
	~LedTask_()
	{
		;/* Do Nothing for now
		or Free Dynamic Allocated Memory (if any) */
		vTaskDelete( xHandle );
	}

	/* Task Initialization Function */
	void xvinit( void );

private:
	/* Port Info */
	volatile XMC_GPIO_PORT_t * xLedPort;
	uint8_t xLedPin;

	/* Task Info */
	TaskCreationParams xLedTaskParams;
	TaskHandle_t xHandle;

	/* Will work as a linker to the actual task function */
	static void xsvTaskEntryPoint( void *params )
	{
		((LedTask_ *) params)->xvTaskEntry(params);
	}

	/* This function contains the actual body of task */
	void xvTaskEntry( void *pvParams );
};


#endif /* APP_INC_LED_TASK_HPP_ */
