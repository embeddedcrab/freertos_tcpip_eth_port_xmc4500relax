
#ifndef _ARM_NN_EXAMPLE_NN_TEST_H_
#define _ARM_NN_EXAMPLE_NN_TEST_H_


/** @file:	arm_nn_example_nn_test.h
 *  @brief:	This file contains ARM NN test functions
 */


/******************************************************************************
* Includes
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ARM CMSIS NN headers */
#include <arm_math.h>
#include <arm_nnfunctions.h>


/******************************************************************************
* Variables
*******************************************************************************/
extern int test_index;
extern q7_t test_flags[50];


/******************************************************************************
* Function Prototypes/Definitions
*******************************************************************************/

void initialize_results_q7(q7_t * ref, q7_t * opt, int length);
void initialize_results_q7(q7_t * ref, q7_t * opt, int length)
{
    arm_fill_q7(0, ref, length);
    arm_fill_q7(37, opt, length);
}


void initialize_results_q15(q15_t * ref, q15_t * opt, int length);
void initialize_results_q15(q15_t * ref, q15_t * opt, int length)
{
    arm_fill_q15(0, ref, length);
    arm_fill_q15(0x5F5, opt, length);
}


void verify_results_q7(q7_t * ref, q7_t * opt, int length);
void verify_results_q7(q7_t * ref, q7_t * opt, int length)
{
    bool if_match = true;

    for (int i = 0; i < length; i++)
    {
        if (ref[i] != opt[i])
        {
            printf("Output mismatch at %d, expected %d, actual %d\r\n", i, ref[i], opt[i]);
            if_match = false;
        }
    }

    if (if_match == true)
    {
        printf("Outputs match.\r\n\r\n");
        test_flags[test_index++] = 0;
    }
    else
    {
        test_flags[test_index++] = 1;
    }
}


void verify_results_q15(q15_t * ref, q15_t * opt, int length);
void verify_results_q15(q15_t * ref, q15_t * opt, int length)
{
    bool if_match = true;

    for (int i = 0; i < length; i++)
    {
        if (ref[i] != opt[i])
        {
            printf("Output mismatch at %d, expected %d, actual %d\r\n", i, ref[i], opt[i]);
            if_match = false;
        }
    }

    if (if_match == true)
    {
        printf("Outputs match.\r\n\r\n");
        test_flags[test_index++] = 0;
    }
    else
    {
        test_flags[test_index++] = 1;
    }
}


#endif	/* _ARM_NN_EXAMPLE_NN_TEST_H_ */
