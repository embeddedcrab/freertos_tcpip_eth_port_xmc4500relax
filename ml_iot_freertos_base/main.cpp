/******************************************************************************
* Copyright (c) 2019 - Hemant Sharma - All Rights Reserved
*
* Feel free to use this Code at your own risk for your own purposes.
*
*******************************************************************************/
/******************************************************************************
* Title:		Main Source
* Filename:		main.cpp
* Author:		HS
* Origin Date:	18/02/2019
* Version:		1.0.1
* Notes:
*
* Change History
* --------------
*
* 30/09/2019	-	Hemant Sharma
* Updated file formatting
*
*******************************************************************************/

/** @file:	main.cpp
 *  @brief:	This source file contains main function entry with
 *  		Exception handlers and callback functions
 */


/******************************************************************************
* Includes
*******************************************************************************/

/* Include Application Headers */
#include <led_task.hpp>

/* Include peripheral device files */
#include <dri_gpio_pins.h>
#include <dri_uart_extern.h>

/* Include RTOS files */
#include <dri_mem.h>
#include <rtos_wrappers.h>

/* Include server task headers */
#include <server_task.h>
#include <client_task.h>


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/* ARM GRU Test, Test Example from Official Repository */
#define GRU_TEST

/* ARM NN Test, Test Example from Official Repository */
#define ARM_NN_TEST

#define FREERTOS_TCP_CLIENT
#undef FREERTOS_TCP_CLIENT


/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Functions
*******************************************************************************/

/* Extern "C" block for C variables
 *
 * C Linkage in CPP Application
 * */
#ifdef __cplusplus
extern "C"
{

#define EXCEPTION_HANDLER_DEBUG_ON
//#undef EXCEPTION_HANDLER_DEBUG_ON

/******* User defined initialization functions *******/
void init_user_before_main( void );
extern void initialize_ethernet_mac( void );

#ifdef EXCEPTION_HANDLER_DEBUG_ON

void get_registers_from_stack( unsigned long *pulFaultStackAddress );
void BusFault_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void UsageFault_Handler(void);

#endif

}
#endif	/* __cplusplus */


/******************************************************************************
* ARM DSP NN related functions
*******************************************************************************/

#if (defined(GRU_TEST) || defined(ARM_NN_TEST))

/* ARM DSP NN Math Library */
#include "arm_nn_example_nn_test.h"


/* ----------------------------------------------------------------------
* Copyright (C) 2010-2018 Arm Limited. All rights reserved.
*
*
* Project:       CMSIS NN Library
* Title:         arm_nnexamples_nn_test.cpp
*
* Description:   Example code for NN kernel testing.
*
* Target Processor: Cortex-M cores
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

#ifdef ARM_NN_TEST


#define TEST_NNMULT

int test_index = 0;
q7_t test_flags[50];
bool test_pass;


void arm_fully_connected_q7_ref(const q7_t * pV, const q7_t * pM, const uint16_t dim_vec,
		const uint16_t num_of_rows, const uint16_t bias_shift, const uint16_t out_shift,
		const q7_t * bias, q7_t * pOut, q15_t * vec_buffer
		);

void arm_fully_connected_q7_ref(const q7_t * pV,    // pointer to vector
		const q7_t * pM,    // pointer to matrix
		const uint16_t dim_vec, // length of the vector
		const uint16_t num_of_rows, // numCol of A
		const uint16_t bias_shift,  // amount of left-shift for bias
		const uint16_t out_shift,   // amount of right-shift for output
		const q7_t * bias, q7_t * pOut, // output operand
		q15_t * vec_buffer)
{
	for (int i = 0; i < num_of_rows; i++)
	{
#ifndef ARM_NN_TRUNCATE
		int       ip_out = (bias[i] << bias_shift) + (0x1 << (out_shift - 1));
#else
		int       ip_out = bias[i] << bias_shift;
#endif
		for (int j = 0; j < dim_vec; j++)
		{
			ip_out += pV[j] * pM[i * dim_vec + j];
		}
		pOut[i] = (q7_t) __SSAT((ip_out >> out_shift), 8);
	}
}

void arm_nn_mult_q7_ref(q7_t * pSrcA, q7_t * pSrcB, q7_t * pDst,
		const uint16_t out_shift, uint32_t blockSize
		);

void arm_nn_mult_q7_ref(q7_t * pSrcA, q7_t * pSrcB, q7_t * pDst,
		const uint16_t out_shift, uint32_t blockSize
		)
{
    uint16_t  i;
    for (i = 0; i < blockSize; i++)
    {
		q31_t product = pSrcA[i] * pSrcB[i];
#ifndef ARM_NN_TRUNCATE
        pDst[i] = (q7_t)__SSAT((product + (0x1 << (out_shift - 1)))>>out_shift, 8);
#else
        pDst[i] = (q7_t)__SSAT(product >> out_shift, 8);
#endif
    }
}

void arm_nn_mult_q15_ref(q15_t * pSrcA, q15_t * pSrcB, q15_t * pDst,
		const uint16_t out_shift, uint32_t blockSize
		);

void arm_nn_mult_q15_ref(q15_t * pSrcA, q15_t * pSrcB, q15_t * pDst,
		const uint16_t out_shift, uint32_t blockSize
		)
{
	uint16_t  i;
	for (i = 0; i < blockSize; i++)
    {
		q31_t product = pSrcA[i] * pSrcB[i];
#ifndef ARM_NN_TRUNCATE
        pDst[i] = (q15_t)__SSAT((product + (0x1 << (out_shift - 1)))>>out_shift, 16);
#else
        pDst[i] = (q15_t)__SSAT(product >> out_shift, 16);
#endif
    }
}

#endif


/** GRU Test functions **/
#ifdef GRU_TEST

/* ----------------------------------------------------------------------
* Copyright (C) 2010-2018 Arm Limited. All rights reserved.
*
*
* Project:       CMSIS NN Library
* Title:         arm_nnexamples_gru.cpp
*
* Description:   Gated Recurrent Unit Example
*
* Target Processor: Cortex-M4/Cortex-M7
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of Arm LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

/**
 * @ingroup Examples
 */

/**
 * @defgroup GRUExample Gated Recurrent Unit Example
 *
 * \par Description:
 * \par
 * Demonstrates a gated recurrent unit (GRU) example with the use of fully-connected,
 * Tanh/Sigmoid activation functions.
 *
 * \par Model definition:
 * \par
 * GRU is a type of recurrent neural network (RNN). It contains two sigmoid gates and one hidden
 * state.
 * \par
 * The computation can be summarized as:
 * <pre>z[t] = sigmoid( W_z &sdot; {h[t-1],x[t]} )
 * r[t] = sigmoid( W_r &sdot; {h[t-1],x[t]} )
 * n[t] = tanh( W_n &sdot; [r[t] &times; {h[t-1], x[t]} )
 * h[t] = (1 - z[t]) &times; h[t-1] + z[t] &times; n[t] </pre>
 * \image html GRU.gif "Gate Recurrent Unit Diagram"
 *
 * \par Variables Description:
 * \par
 * \li \c update_gate_weights, \c reset_gate_weights, \c hidden_state_weights are weights corresponding to update gate (W_z), reset gate (W_r), and hidden state (W_n).
 * \li \c update_gate_bias, \c reset_gate_bias, \c hidden_state_bias are layer bias arrays
 * \li \c test_input1, \c test_input2, \c test_history are the inputs and initial history
 *
 * \par
 * The buffer is allocated as:
 * \par
 * | reset | input | history | update | hidden_state |
 * \par
 * In this way, the concatination is automatically done since (reset, input) and (input, history)
 * are physically concatinated in memory.
 * \par
 *  The ordering of the weight matrix should be adjusted accordingly.
 *
  *
 *
 * \par CMSIS DSP Software Library Functions Used:
 * \par
 * - arm_fully_connected_mat_q7_vec_q15_opt()
 * - arm_nn_activations_direct_q15()
 * - arm_mult_q15()
 * - arm_offset_q15()
 * - arm_sub_q15()
 * - arm_copy_q15()
 *
 * <b> Refer  </b>
 * \link arm_nnexamples_gru.cpp \endlink
 *
 */


/* ARM DSP NN Math Library */
#include "arm_nnexamples_gru_test_data.h"
#include "arm_math.h"
#include "arm_nnfunctions.h"


#define DIM_HISTORY 32
#define DIM_INPUT 32
#define DIM_VEC 64

#define USE_X4

#ifndef USE_X4
static q7_t update_gate_weights[DIM_VEC * DIM_HISTORY] = UPDATE_GATE_WEIGHT_X2;
static q7_t reset_gate_weights[DIM_VEC * DIM_HISTORY] = RESET_GATE_WEIGHT_X2;
static q7_t hidden_state_weights[DIM_VEC * DIM_HISTORY] = HIDDEN_STATE_WEIGHT_X2;
#else
static q7_t update_gate_weights[DIM_VEC * DIM_HISTORY] = UPDATE_GATE_WEIGHT_X4;
static q7_t reset_gate_weights[DIM_VEC * DIM_HISTORY] = RESET_GATE_WEIGHT_X4;
static q7_t hidden_state_weights[DIM_VEC * DIM_HISTORY] = HIDDEN_STATE_WEIGHT_X4;
#endif

static q7_t update_gate_bias[DIM_HISTORY] = UPDATE_GATE_BIAS;
static q7_t reset_gate_bias[DIM_HISTORY] = RESET_GATE_BIAS;
static q7_t hidden_state_bias[DIM_HISTORY] = HIDDEN_STATE_BIAS;

static q15_t test_input1[DIM_INPUT] = INPUT_DATA1;
static q15_t test_input2[DIM_INPUT] = INPUT_DATA2;
static q15_t test_history[DIM_HISTORY] = HISTORY_DATA;

q15_t     scratch_buffer[DIM_HISTORY * 4 + DIM_INPUT];

void gru_example(q15_t * scratch_input, uint16_t input_size, uint16_t history_size,
                 q7_t * weights_update, q7_t * weights_reset, q7_t * weights_hidden_state,
                 q7_t * bias_update, q7_t * bias_reset, q7_t * bias_hidden_state
				 );
void gru_example(q15_t * scratch_input, uint16_t input_size, uint16_t history_size,
                 q7_t * weights_update, q7_t * weights_reset, q7_t * weights_hidden_state,
                 q7_t * bias_update, q7_t * bias_reset, q7_t * bias_hidden_state)
{
  q15_t    *reset = scratch_input;
  q15_t    *input = scratch_input + history_size;
  q15_t    *history = scratch_input + history_size + input_size;
  q15_t    *update = scratch_input + 2 * history_size + input_size;
  q15_t    *hidden_state = scratch_input + 3 * history_size + input_size;

  // reset gate calculation
  // the range of the output can be adjusted with bias_shift and output_shift
#ifndef USE_X4
  arm_fully_connected_mat_q7_vec_q15(input, weights_reset, input_size + history_size, history_size, 0, 15, bias_reset,
                                     reset, NULL);
#else
  arm_fully_connected_mat_q7_vec_q15_opt(input, weights_reset, input_size + history_size, history_size, 0, 15,
                                         bias_reset, reset, NULL);
#endif
  // sigmoid function, the size of the integer bit-width should be consistent with out_shift
  arm_nn_activations_direct_q15(reset, history_size, 0, ARM_SIGMOID);
  arm_mult_q15(history, reset, reset, history_size);

  // update gate calculation
  // the range of the output can be adjusted with bias_shift and output_shift
#ifndef USE_X4
  arm_fully_connected_mat_q7_vec_q15(input, weights_update, input_size + history_size, history_size, 0, 15,
                                     bias_update, update, NULL);
#else
  arm_fully_connected_mat_q7_vec_q15_opt(input, weights_update, input_size + history_size, history_size, 0, 15,
                                         bias_update, update, NULL);
#endif

  // sigmoid function, the size of the integer bit-width should be consistent with out_shift
  arm_nn_activations_direct_q15(update, history_size, 0, ARM_SIGMOID);

  // hidden state calculation
#ifndef USE_X4
  arm_fully_connected_mat_q7_vec_q15(reset, weights_hidden_state, input_size + history_size, history_size, 0, 15,
                                     bias_hidden_state, hidden_state, NULL);
#else
  arm_fully_connected_mat_q7_vec_q15_opt(reset, weights_hidden_state, input_size + history_size, history_size, 0, 15,
                                         bias_hidden_state, hidden_state, NULL);
#endif

  // tanh function, the size of the integer bit-width should be consistent with out_shift
  arm_nn_activations_direct_q15(hidden_state, history_size, 0, ARM_TANH);
  arm_mult_q15(update, hidden_state, hidden_state, history_size);

  // we calculate z - 1 here
  // so final addition becomes subtraction
  arm_offset_q15(update, 0x8000, update, history_size);
  // multiply history
  arm_mult_q15(history, update, update, history_size);
  // calculate history_out
  arm_sub_q15(hidden_state, update, history, history_size);

  return;
}

#endif	/* GRU_TEST */

#endif	/* (GRU_TEST || ARM_NN_TEST) */





/**
 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point.
 * It is invoked by the device startup code.
 */
int main(void)
{
	/* Task Parameters Variable */
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wwrite-strings"
	TaskCreationParams TaskParams = {"LED_TASK", configMINIMAL_STACK_SIZE, 1 };
	#pragma GCC diagnostic pop

	/* The MAC address array is not declared const as the MAC address will
	 * normally be read from an EEPROM and not hard coded (in real deployed
	 * applications) or read using HAL API from ETH MAC */
	static uint8_t ucMACAddress[ 6 ] = { 0x00U, 0x03U, 0x19U, 0x45U, 0x00, 0x00 };

	/* Define the network addressing.  These parameters will be used if either
	 * ipconfigUDE_DHCP is 0 or if ipconfigUSE_DHCP is 1 but DHCP auto configuration failed. */
	static const uint8_t ucIPAddress[ 4 ] = { 192, 168, 1, 9 };
	static const uint8_t ucNetMask[ 4 ] = { 255, 255, 255, 0 };
	static const uint8_t ucGatewayAddress[ 4 ] = { 192, 168, 1, 1 };
	/* The following is the address of an OpenDNS server. */
	static const uint8_t ucDNSServerAddress[ 4 ] = { 208, 67, 222, 222 };

	/* Initialize peripherals */
	/* Call UART Object functions */
	g_p_uart_obj[0]->init();
//	g_p_uart_obj[0]->transmit( (const unsigned char *)"Hello World\r\n", 13 );

	/* Initialize Ethernet MAC */
	initialize_ethernet_mac();


	/* ARM NN Test */
#ifdef ARM_NN_TEST
	/* common pointers for testing data */
	q7_t     *test1;
	q15_t    *test2;
	q7_t     *test3;
	q15_t    *test4;

	for(test_index = 0; test_index < 50; ++test_index)
	{
		test_flags[test_index] = -1;
	}
	test_index = 0;

#ifdef TEST_NNMULT
#define NNMULT_DIM 128
	test1 = new q7_t[NNMULT_DIM*2];
	test2 = new q15_t[NNMULT_DIM*2];
	test3 = new q7_t[NNMULT_DIM*2];
	test4 = new q15_t[NNMULT_DIM*2];

	q7_t * mult_out_q7 = test3;
	q7_t * mult_ref_q7 = test3 + NNMULT_DIM;
	q15_t * mult_out_q15 = test4;
	q15_t * mult_ref_q15 = test4 + NNMULT_DIM;

	for(int i = 0; i < NNMULT_DIM*2; ++i)
	{
		test1[i] = (rand() % 256 - 128);
		test2[i] = (rand() % 65536 - 32768);
	}

	/* Test q7 */
	arm_nn_mult_q7(test1, test1+NNMULT_DIM, mult_out_q7, 5, NNMULT_DIM);
	arm_nn_mult_q7_ref(test1, test1+NNMULT_DIM, mult_ref_q7, 5, NNMULT_DIM);
	verify_results_q7(mult_out_q7, mult_ref_q7, NNMULT_DIM);
	arm_nn_mult_q7(test1, test1+NNMULT_DIM, mult_out_q7, 9, NNMULT_DIM);
	arm_nn_mult_q7_ref(test1, test1+NNMULT_DIM, mult_ref_q7, 9, NNMULT_DIM);
	verify_results_q7(mult_out_q7, mult_ref_q7, NNMULT_DIM);

	/* Test q15 */
	arm_nn_mult_q15(test2, test2+NNMULT_DIM, mult_out_q15, 13, NNMULT_DIM);
	arm_nn_mult_q15_ref(test2, test2+NNMULT_DIM, mult_ref_q15, 13, NNMULT_DIM);
	verify_results_q15(mult_out_q15, mult_ref_q15, NNMULT_DIM);
	arm_nn_mult_q15(test2, test2+NNMULT_DIM, mult_out_q15, 18, NNMULT_DIM);
	arm_nn_mult_q15_ref(test2, test2+NNMULT_DIM, mult_ref_q15, 18, NNMULT_DIM);
	verify_results_q15(mult_out_q15, mult_ref_q15, NNMULT_DIM);

	delete[] test1;
	delete[] test2;
	delete[] test3;
	delete[] test4;

#endif	/* TEST_NNMULT */

#endif	/* ARM_NN_TEST */


#ifdef GRU_TEST

#ifdef RTE_Compiler_EventRecorder
	EventRecorderInitialize (EventRecordAll, 1);  // initialize and start Event Recorder
#endif

	printf("Start GRU execution\n");
	int       input_size = DIM_INPUT;
	int       history_size = DIM_HISTORY;

	// copy over the input data
	arm_copy_q15(test_input1, scratch_buffer + history_size, input_size);
	arm_copy_q15(test_history, scratch_buffer + history_size + input_size, history_size);

	gru_example(scratch_buffer, input_size, history_size,
			update_gate_weights, reset_gate_weights, hidden_state_weights,
			update_gate_bias, reset_gate_bias, hidden_state_bias);
	printf("Complete first iteration on GRU\n");

	arm_copy_q15(test_input2, scratch_buffer + history_size, input_size);
	gru_example(scratch_buffer, input_size, history_size,
			update_gate_weights, reset_gate_weights, hidden_state_weights,
			update_gate_bias, reset_gate_bias, hidden_state_bias);
	printf("Complete second iteration on GRU\n");

#endif


	/* Create LED tasks
	 *
	 * <i>Imp Note:</i>
	 * Application task creation shall be done using new while creating objects
	 * otherwise Bus Fault Exception will come!!
	 * */
	LedTask_ * LedTask = new LedTask_( XMC_GPIO_PORT1 , 0, &TaskParams );
	LedTask->xvinit();
	
	/* Initialize the RTOS’s TCP/IP stack.  The tasks that use the network
	 * are created in the vApplicationIPNetworkEventHook() hook function below.
	 * The hook function is called when the network connects. */
	BaseType_t xReturn = FreeRTOS_IPInit( ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, ucMACAddress );

	/* Check IP Init status */
	if( xReturn == pdPASS )
	{
		/* Create Server Handler task */
		ServerTask__ server( ipconfigIP_TASK_PRIORITY, configMINIMAL_STACK_SIZE );//-1

		#if defined(FREERTOS_TCP_CLIENT)
		ClientTask__ client( ipconfigIP_TASK_PRIORITY - 1, configMINIMAL_STACK_SIZE );
		#endif

		/* Start Task Scheduler */
		vTaskStartScheduler();

		for( ; ; )
		{
			GPIO__::fp_ctrl( XMC_GPIO_PORT1 , 0, XMC_GPIO_OUTPUT_LEVEL_HIGH );
		}
	}
	else
	{
		GPIO__::fp_ctrl( XMC_GPIO_PORT1 , 0, XMC_GPIO_OUTPUT_LEVEL_HIGH );
		/* Could not initialize Network, shall hang N/W related tasks */
		while(1U)
		{
			;
		}
	}

	return 0;
}


/******************************************************************************/


/** Exception Handler functions for debugging and tracing **/

#ifdef EXCEPTION_HANDLER_DEBUG_ON

void get_registers_from_stack( unsigned long *pulFaultStackAddress )
{
	/* These are volatile to try and prevent the compiler/linker optimizing them
	 away as the variables never actually get used.  If the debugger won't show the
	 values of the variables, make them global my moving their declaration outside
	 of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr;	/**< Link register. */
	volatile uint32_t pc;	/**< Program counter. */
	volatile uint32_t psr;	/**< Program status register. */

	/* Assign Register values into local variables */
	r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];

	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];

	/* When the following line is hit, the variables contain the register values. */
	for( ; ; );

	/* Avoid compiler warnings */
	(void) r0; (void) r1; (void) r2; (void) r3; (void) r12;
	(void) lr; (void) pc; (void) psr;
}

#endif	/* EXCEPTION_HANDLER_DEBUG_ON */

void BusFault_Handler(void)
{
#ifdef EXCEPTION_HANDLER_DEBUG_ON
	__asm volatile
	(
			" tst lr, #4                                                \n"
			" ite eq                                                    \n"
			" mrseq r0, msp                                             \n"
			" mrsne r0, psp                                             \n"
			" ldr r1, [r0, #24]                                         \n"
			" ldr r2, bus_fault_handler_address_const                   \n"
			" bx r2                                                     \n"
			" bus_fault_handler_address_const: .word get_registers_from_stack    \n"
	);
#endif
	for( ; ; );
}

void HardFault_Handler(void)
{
#ifdef EXCEPTION_HANDLER_DEBUG_ON
	__asm volatile
	(
			" tst lr, #4                                                \n"
			" ite eq                                                    \n"
			" mrseq r0, msp                                             \n"
			" mrsne r0, psp                                             \n"
			" ldr r1, [r0, #24]                                         \n"
			" ldr r2, hard_fault_handler_address_const                  \n"
			" bx r2                                                     \n"
			" hard_fault_handler_address_const: .word get_registers_from_stack    \n"
	);
	for( ; ; );
#endif	/* EXCEPTION_HANDLER_DEBUG_ON */
}

void MemManage_Handler(void)
{
#ifdef EXCEPTION_HANDLER_DEBUG_ON
    __asm volatile
        (
         " tst lr, #4                                                \n"
         " ite eq                                                    \n"
         " mrseq r0, msp                                             \n"
         " mrsne r0, psp                                             \n"
         " ldr r1, [r0, #24]                                         \n"
         " ldr r2, mem_manage_handler_address_const                  \n"
         " bx r2                                                     \n"
         " mem_manage_handler_address_const: .word get_registers_from_stack    \n"
        );
#endif	/* EXCEPTION_HANDLER_DEBUG_ON */
    for( ; ; );
}


void UsageFault_Handler(void)
{
#ifdef EXCEPTION_HANDLER_DEBUG_ON
    __asm volatile
        (
         " tst lr, #4                                                \n"
         " ite eq                                                    \n"
         " mrseq r0, msp                                             \n"
         " mrsne r0, psp                                             \n"
         " ldr r1, [r0, #24]                                         \n"
         " ldr r2, usage_fault_handler_address_const                  \n"
         " bx r2                                                     \n"
         " usage_fault_handler_address_const: .word get_registers_from_stack    \n"
        );
#endif	/* EXCEPTION_HANDLER_DEBUG_ON */
    for( ; ; );
}


/* Extern "C" block for C Functions */
/** User defined functions **/
extern "C" void init_user_before_main( void )
		{
			extern void gpio_init_c( void );
			gpio_init_c();
			extern void initialize_uart_objects_c( void );
			initialize_uart_objects_c();
		}


/* UART__ class callback handler function for User */
void uart_callback_handler(eUART_Channel_& channel, unsigned char *p_data, eUART_Event_ event)
{
	/* Can check the events for callback and do processing */
	(void) p_data;

	/* Check events */
	switch( event )
	{
		default:
		case eUART_Event_::UART_EVENT_TX_COMPLETE:
			if( eUART_Channel_::UART_CHANNEL_0 == channel )
			{
				/* Transmission successful! */
				__asm("NOP");
			}
			break;

		case eUART_Event_::UART_EVENT_RX_COMPLETE:
			if( eUART_Channel_::UART_CHANNEL_0 == channel )
			{
				/* Reception successful! */
				__asm("NOP");
			}
			break;
	}
}



/** RTOS related functions, shall be compiled in C **/

extern "C"
{

/*-----------------------------------------------------------*/

#if(  configCHECK_FOR_STACK_OVERFLOW > 0 )
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName );
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    (void) xTask;
    (void) pcTaskName;
    /* Check for Stack Overflow cause */
}

#endif

/*-----------------------------------------------------------*/

#if( configUSE_TICK_HOOK > 0 )
void vApplicationTickHook( void );
void vApplicationTickHook( void )
{
    /* Dummy Implementation */
	return;
}

#endif

/*-----------------------------------------------------------*/

#if ( configUSE_IDLE_HOOK == 1 )
void vApplicationIdleHook( void );
void vApplicationIdleHook( void )
{
const uint32_t ulMSToSleep = pdMS_TO_TICKS(500);

	/* This is just a trivial example of an idle hook.  It is called on each
	cycle of the idle task if configUSE_IDLE_HOOK is set to 1 in
	FreeRTOSConfig.h.  It must *NOT* attempt to block.  In this case the
	idle task just sleeps to lower the CPU usage. */
	vTaskDelay( ulMSToSleep );
}

#endif

/*-----------------------------------------------------------*/

#if( configUSE_MALLOC_FAILED_HOOK == 1 )
void vApplicationMallocFailedHook( void );
void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Report malloc failed hook */

}

#endif  /* #if( configUSE_MALLOC_FAILED_HOOK == 1 ) */

/*-----------------------------------------------------------*/

/* Use by the pseudo random number generator. */
static UBaseType_t ulNextRand;

UBaseType_t uxRand( void );
UBaseType_t uxRand( void )
{
const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;

	/* Utility function to generate a pseudo random number. */

	ulNextRand = ( ulMultiplier * ulNextRand ) + ulIncrement;
	return( ( int ) ( ulNextRand >> 16UL ) & 0x7fffUL );
}

/*-----------------------------------------------------------*/
void vLoggingPrintf( const char *pcFormat, ... );
void vLoggingPrintf( const char *pcFormat, ... )
{
    /* Dummy implementation */
    (void) pcFormat;
}


/*
* Callback that provides the inputs necessary to generate a randomized TCP
* Initial Sequence Number per RFC 6528.  In this case just a psuedo random
* number is used so THIS IS NOT RECOMMENDED FOR PRODUCTION SYSTEMS.
*/
uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress,
uint16_t usSourcePort, uint32_t ulDestinationAddress, uint16_t usDestinationPort );
uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress, uint16_t usSourcePort,
						uint32_t ulDestinationAddress, uint16_t usDestinationPort
						)
{
     ( void ) ulSourceAddress;
     ( void ) usSourcePort;
     ( void ) ulDestinationAddress;
     ( void ) usDestinationPort;

     return uxRand();
}

}   /* extern "C"{ end for rtos related functions } */


/********************************** End of File *******************************/


