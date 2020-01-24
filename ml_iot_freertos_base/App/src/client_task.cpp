/******************************************************************************
* Copyright (c) 2019 - Hemant Sharma - All Rights Reserved
*
* Feel free to use this Code at your own risk for your own purposes.
*
*******************************************************************************/
/******************************************************************************
* Title:		Client Task Source
* Filename:		client_task.cpp
* Author:		HS
* Origin Date:	01/18/2020
* Version:		1.0.0
* Notes:
*
* Change History
* --------------
*
*******************************************************************************/

/** @file:	client_task.cpp
 *  @brief:	This source file contains function definitions for
 *  		client task class.
 */


/******************************************************************************
* Includes
*******************************************************************************/
#include <client_task.h>

#include <cstdio>


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
* Variables
*******************************************************************************/


/******************************************************************************
* Function Definitions
*******************************************************************************/


/******* Public Member function for ClientTask__ class *******/

/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 */
void ClientTask__ :: client_body( void *pvParameters )
{
    (void) pvParameters;
	/* Local variables */
	struct freertos_sockaddr xServerAddress;
	char p_buffer_l[15] = "Hello Server\r\n";
	Socket_t xClientSocket;
	constexpr TickType_t xSendTimeOut = pdMS_TO_TICKS( 5000 );
    TickType_t xTimeOnShutdown = 0;
	long bytes_l = 0;

	while( FreeRTOS_IsNetworkUp() != pdTRUE )
	{
		/* Sleep, then check again */
		vTaskDelay( 500 );
	}

	/* Configure server details */
	xServerAddress.sin_port = FreeRTOS_htons( 5901 );
	xServerAddress.sin_addr = FreeRTOS_inet_addr( "192.168.1.3" );

	/* Connect to a server and create a task for client */
	for(  ; ; )
	{
		/* Attempt to open the socket. */
		xClientSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP );
		configASSERT( xClientSocket != FREERTOS_INVALID_SOCKET );

		FreeRTOS_setsockopt( xClientSocket, 0, FREERTOS_SO_SNDTIMEO, &xSendTimeOut, sizeof( xSendTimeOut ) );

		/* Start processing */
		for( ; ; )
		{
            if( 0 == FreeRTOS_connect( xClientSocket, &xServerAddress, sizeof(xServerAddress) ) )
            {
                /* Client processing task */
                for( ; ; )
                {
                    bytes_l = FreeRTOS_send( xClientSocket, p_buffer_l, 14, 0 );
                    if( bytes_l < 0 )
                    {
                        /* Socket closed */
                        break;
                    }
                    else
                    {
                        vTaskDelay( pdMS_TO_TICKS(50) );
                    }
                }

                /* Initiate a shutdown in case it has not already been initiated. */
                FreeRTOS_shutdown( xClientSocket, FREERTOS_SHUT_RDWR );

                /* Wait for the shutdown to take effect, indicated by FreeRTOS_recv()
                 * returning an error. */
                xTimeOnShutdown = xTaskGetTickCount();
                do
                {
                    if( FreeRTOS_recv( xClientSocket, p_buffer_l, sizeof(p_buffer_l), 0 ) < 0 )
                    {
                        break;
                    }
                } while( ( xTaskGetTickCount() - xTimeOnShutdown ) < pdMS_TO_TICKS(1000) );

                /* Close socket */
                FreeRTOS_closesocket( xClientSocket );
                /* Break loop to recreate the client socket */
                break;
            }
            else
            {
                vTaskDelay( pdMS_TO_TICKS(1000) );
            }
        }
	}

	vTaskDelete( NULL );
}


/********************************** End of File *******************************/
