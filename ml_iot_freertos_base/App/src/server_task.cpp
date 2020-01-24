/******************************************************************************
* Copyright (c) 2019 - Hemant Sharma - All Rights Reserved
*
* Feel free to use this Code at your own risk for your own purposes.
*
*******************************************************************************/
/******************************************************************************
* Title:		Server Task Source
* Filename:		server_task.cpp
* Author:		HS
* Origin Date:	01/18/2020
* Version:		1.0.0
* Notes:
*
* Change History
* --------------
*
*******************************************************************************/

/** @file:	server_task.cpp
 *  @brief:	This source file contains function definitions for
 *  		server task class.
 */


/******************************************************************************
* Includes
*******************************************************************************/
#include <server_task.h>


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Configuration Constants
*******************************************************************************/

#define MAX_CLIENTS		2


/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/

/* Declare static data member of class
 *
 * Note: Tell linker to take variable from this source file
 * */
static const char * const server_handler_name = "ServerHandler";
static const UBaseType_t server_handler_priority = ipconfigIP_TASK_PRIORITY + 1;
static const configSTACK_DEPTH_TYPE stack_depth = configMINIMAL_STACK_SIZE * 2;


/******************************************************************************
* Function Declarations
*******************************************************************************/


/******************************************************************************
* Function Definitions
*******************************************************************************/


/******* Public Member function for ServerTask__ class *******/

/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 * 				Example taken from FreeRTOS-Plus TCP demo
 */
void ServerTask__ :: server_body( void *pvParameters )
{
	/* Local variables */
	struct freertos_sockaddr xClient, xBindAddress;
	Socket_t xListeningSocket = NULL;
	Socket_t xConnectedSocket = NULL;

	socklen_t xSize = sizeof( xClient );
	TickType_t xReceiveTimeOut = portMAX_DELAY;
	const BaseType_t xBacklog = MAX_CLIENTS;

	while( FreeRTOS_IsNetworkUp() != pdTRUE )
	{
		/* Sleep, then check again */
		vTaskDelay( 500 );
	}

	/* Attempt to open the socket. */
	xListeningSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP );
	configASSERT( xListeningSocket != FREERTOS_INVALID_SOCKET );

	/* Set a time out so accept() will just wait for a connection. */
	FreeRTOS_setsockopt( xListeningSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof( xReceiveTimeOut ) );

	/* Bind the socket to the port that the client task will send to,
	 * then listen for incoming connections. */
	xBindAddress.sin_port = 5900;
	xBindAddress.sin_port = FreeRTOS_htons( xBindAddress.sin_port );
	FreeRTOS_bind( xListeningSocket, &xBindAddress, sizeof( xBindAddress ) );
	FreeRTOS_listen( xListeningSocket, xBacklog );

	/* Create task to receive data from client */
	for( ; ; )
	{
		/* Wait for a client to connect. */
		xConnectedSocket = FreeRTOS_accept( xListeningSocket, &xClient, &xSize );
		configASSERT( xConnectedSocket != FREERTOS_INVALID_SOCKET );

		(void) RTOS_WRAPPER__::create( ServerTask__::server_handler_entry, server_handler_name, stack_depth,
										xConnectedSocket, server_handler_priority, NULL );
	}
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 */
void ServerTask__ :: server_handler_body( void * pvParameters )
{
	/* Local variables */
	Socket_t xConnectedSocket = NULL;
	uint8_t *pucRxBuffer = NULL;
	BaseType_t lBytes = 0;
	BaseType_t lSent = 0;
	BaseType_t lTotalSent = 0;
	constexpr TickType_t xReceiveTimeOut = pdMS_TO_TICKS( portMAX_DELAY );
	constexpr TickType_t xSendTimeOut = pdMS_TO_TICKS( 5000 );
	TickType_t xTimeOnShutdown = 0;

	/* Get the client details */
	xConnectedSocket = (Socket_t) pvParameters;

	/* Send Accept confirmation to client */
	FreeRTOS_send( xConnectedSocket, "Hello\r\n", sizeof("Hello\r\n"), 0 );

	/* Attempt to create the buffer used to receive the string to be echoed
	 * back.  This could be avoided using a zero copy interface that just
	 * returned the same buffer. */
	pucRxBuffer = new uint8_t[ipconfigTCP_MSS];
	/* Check memory allocation for buffer validity */
	if( pucRxBuffer != NULL )
	{
		/* Set socket options for client */
		FreeRTOS_setsockopt( xConnectedSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof( xReceiveTimeOut ) );
		FreeRTOS_setsockopt( xConnectedSocket, 0, FREERTOS_SO_SNDTIMEO, &xSendTimeOut, sizeof( xReceiveTimeOut ) );

		/* Receive data from client */
		for( ; ; )
		{
			/* Zero out the receive array so there is NULL at the end of the string
				when it is printed out. */
			memset( pucRxBuffer, 0x00, ipconfigTCP_MSS );

			/* Receive data on the socket. */
			lBytes = FreeRTOS_recv( xConnectedSocket, pucRxBuffer, ipconfigTCP_MSS, 0 );

			/* If data was received, echo it back. */
			if( lBytes >= 0 )
			{
				lSent = 0;
				lTotalSent = 0;

				/* Call send() until all the data has been sent. */
				while( ( lSent >= 0 ) && ( lTotalSent < lBytes ) )
				{
					lSent = FreeRTOS_send( xConnectedSocket, pucRxBuffer, lBytes - lTotalSent, 0 );
					lTotalSent += lSent;
				}

				if( lSent < 0 )
				{
					/* Socket closed? */
					break;
				}
			}
			else
			{
				/* Socket closed? */
				break;
			}
		}
	}

	/* Initiate a shutdown in case it has not already been initiated. */
	FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );

	/* Wait for the shutdown to take effect, indicated by FreeRTOS_recv()
	 * returning an error. */
	xTimeOnShutdown = xTaskGetTickCount();
	do
	{
		if( FreeRTOS_recv( xConnectedSocket, pucRxBuffer, ipconfigTCP_MSS, 0 ) < 0 )
		{
			break;
		}
	} while( ( xTaskGetTickCount() - xTimeOnShutdown ) < tcpechoSHUTDOWN_DELAY );

	/* Finished with the socket, buffer, the task. */
	delete[] pucRxBuffer;
	FreeRTOS_closesocket( xConnectedSocket );

	vTaskDelete( NULL );
}


/********************************** End of File *******************************/
