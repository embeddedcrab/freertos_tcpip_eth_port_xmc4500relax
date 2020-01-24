/******************************************************************************
* Copyright (c) 2019 - Hemant Sharma - All Rights Reserved
*
* Feel free to use this Code at your own risk for your own purposes.
*
*******************************************************************************/
/******************************************************************************
* Title:		Client Task Header
* Filename:		client_task.h
* Author:		HS
* Origin Date:	01/18/2020
* Version:		1.0.0
* Notes:
*******************************************************************************/

/** @file:	client_task.h
 *  @brief:	This file contains function declarations
 *  		for Client Task
 */
#ifndef CLIENT_TASK_H_
#define CLIENT_TASK_H_


/******************************************************************************
* Includes
*******************************************************************************/
#include <rtos_wrappers.h>
#include <dri_mem.h>

/* Include FreeRTOS-Plus TCP/IP files */
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
 *
 * @brief	Class for ClientTask__ Instance
 *
 ******************************************************************************/

class ClientTask__
{
/* public members */
public:
	/** Constructors and Destructors */
	ClientTask__() = default;
	ClientTask__( const UBaseType_t& uxPriority, const configSTACK_DEPTH_TYPE& usStackDepth )
	{
		(void) RTOS_WRAPPER__::create( ClientTask__::client_entry,
										xtask_name, usStackDepth, NULL,
										uxPriority, xhandle );
	}

	virtual ~ClientTask__()
	{
		/* Add definition for Destructor */
		RTOS_WRAPPER__::destroy( xhandle );
	}

	/******* API Member functions *******/

	/**
	 * @function
	 *
	 * @brief
	 *
	 * @param[in]
	 *
	 * @param[out]
	 *
	 * @return
	 *
	 * \par<b>Description:</b><br>
	 *
	 * <i>Imp Note:</i>
	 *
	 */


/* private members */
private:
	/** Data Members **/
	const char * const xtask_name = "ClientTask";
	TaskHandle_t * xhandle = nullptr;

	/******* Private Member Functions *******/
	static void client_entry( void * pvParameters )
	{
		/* Pass the control to server body */
		((ClientTask__ *) pvParameters)->client_body( pvParameters );
	}

	/**
	 * @function	client_body
	 *
	 * @brief		This function contains client body which communicate with
	 * 				connection
	 *
	 * <i>Imp Note:</i>
	 *
	 */
	void client_body( void *pvParameters );


/* protected members, if any */
protected:

};



/******************************************************************************
* Variables
*******************************************************************************/


#endif /* CLIENT_TASK_H_ */

/********************************** End of File *******************************/
