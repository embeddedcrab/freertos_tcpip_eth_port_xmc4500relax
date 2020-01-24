/******************************************************************************
* Copyright (c) 2019 - Hemant Sharma - All Rights Reserved
*
* Feel free to use this Code at your own risk for your own purposes.
*
*******************************************************************************/
/******************************************************************************
* Title:		Server Task Header
* Filename:		server_task.h
* Author:		HS
* Origin Date:	01/18/2020
* Version:		1.0.0
* Notes:
*******************************************************************************/

/** @file:	server_task.h
 *  @brief:	This file contains function declarations
 *  		for Server Task
 */
#ifndef SERVER_TASK_H_
#define SERVER_TASK_H_


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

/* The maximum time to wait for a closing socket to close. */
#define tcpechoSHUTDOWN_DELAY	( pdMS_TO_TICKS( 5000 ) )


/******************************************************************************
* Typedefs
*******************************************************************************/

typedef struct tStServerTaskData_
{
	Socket_t socket;
	TaskHandle_t task_handle;
} tStServerTaskData;


/******************************************************************************
* Function Prototypes
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
 *
 * @brief	Class for ServerTask__ Instance
 *
 ******************************************************************************/

class ServerTask__
{
/* public members */
public:
	/** Constructors and Destructors */
	ServerTask__( const UBaseType_t& uxPriority, const configSTACK_DEPTH_TYPE& usStackDepth )
	{
		(void) RTOS_WRAPPER__::create( ServerTask__::server_entry,
										xtask_name, usStackDepth, NULL,
										uxPriority, xhandle );
	}

	virtual ~ServerTask__()
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
	const char * const xtask_name = "ServerTask";
	TaskHandle_t * xhandle = nullptr;

	/******* Private Member Functions *******/
	static void server_entry( void * pvParameters )
	{
		/* Pass the control to server body */
		((ServerTask__ *) pvParameters)->server_body( pvParameters );
	}

	/**
	 * @function	server_body
	 *
	 * @brief		This function contains server body which waits for connection
	 * 				and create a handler task for particular client
	 *
	 * <i>Imp Note:</i>
	 *
	 */
	void server_body( void *pvParameters );

	static void server_handler_entry( void * pvParameters )
	{
		/* Pass the control to server handler body */
		((ServerTask__ *) pvParameters)->server_handler_body( pvParameters );
	}

	void server_handler_body( void * pvParameters );

/* protected members, if any */
protected:

};



/******************************************************************************
* Variables
*******************************************************************************/


#endif /* SERVER_TASK_H_ */

/********************************** End of File *******************************/
